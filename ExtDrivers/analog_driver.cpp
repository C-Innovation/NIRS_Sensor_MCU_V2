/*
 * analog_driver.cpp
 *
 *  Created on: 8 июл. 2026 г.
 *      Author: xwest
 *
 *  ONE-SHOT версия: один запуск = ровно ADC_RAW_DATA_COUNT срабатываний
 *  TIM3_TRGO -> ADC_RAW_MASS_SIZE слов в RawDataMass -> DMA TC ->
 *  обработка + смена светодиода + перезапуск в AdcConvCompleteDMA().
 *  Наложение циклов измерения на смену светодиода исключено.
 *
 *  Сценарий запускается с частотой NIRS_SAMPLE_RATE_HZ (100 Гц):
 *
 *   TIM5 (период ADC_CYCLE_US) --update--> CycleStart():
 *        LED_740 вкл, ADC+DMA взведены, TIM3 запущен
 *   TIM3 (период ADC_PHASE_US) --TRGO--> АЦП (скан 4 каналов) --DMA TC-->
 *        AdcConvCompleteDMA(): сохранить, включить следующий светодиод
 *   ... после 4-го этапа TIM3 останавливается, светодиоды погашены,
 *        до следующего update TIM5 (9 мс при 100 Гц) ничего не происходит.
 *
 *   Сон MP3320A (ANALOG_MP3320_SLEEP_IN_IDLE): после 4-го этапа EN=0 (ток покоя
 *   ~1 мА -> ~1.5 мкА), за ANALOG_MP3320_WAKE_LEAD_US до следующего сценария
 *   compare-прерывание TIM5 CC1 пишет EN=1 (ChipWake). Между EN=0 и любой
 *   следующей I2C-операцией должно пройти >1.5 мс (даташит Rev 1.1).
 *
 *   t, мкс   0      250     500     750     1000              10000
 *   светодиод 740  ->  темно ->  850  ->  темно -> (всё выкл до следующего цикла)
 *   отсчёт АЦП  -    740     фон-740   850     фон-850
 *   (светодиод переключается сразу ПОСЛЕ отсчёта АЦП, поэтому его импульс
 *    сдвинут на время I2C-записи ~70-100 мкс, но каждый отсчёт берётся
 *    ровно через ADC_PHASE_US после предыдущего).
 */

#include "analog_driver.h"
#include "mp3320a.h"
#include "adc.h"
#include "tim.h"
#include "gpdma.h"
#include "linked_list.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "semphr.h"
#include "projdefs.h"

/* Шаблон настроек канала GPDMA. В one-shot режиме связного списка нет
   (CLLR = 0), поэтому CSAR/CDAR/CBR1 инкрементируются железом по ходу
   передачи и их нужно перезаписывать перед каждым запуском. */
typedef struct {
    uint32_t CTR1;
    uint32_t CTR2;
    uint32_t CBR1;
    uint32_t CSAR;
    uint32_t CDAR;
    uint32_t CCR;
} GPDMA_ChCfg_Type;

static GPDMA_ChCfg_Type AdcDmaCfg;

MesurementData_t TotalData;
ActiveLed _ActiveLed;
MP3320A_HandleTypeDef hmp3320;

volatile uint32_t RawDataMass[ADC_RAW_MASS_SIZE];

/* Состояние сценария. Пишутся только из прерываний (TIM5 / GPDMA1_Ch2,
   один приоритет -> взаимно не вытесняются), читаются и из задач. */
static volatile bool     s_inCycle = false;         /* сценарий выполняется         */
static volatile bool     s_currentPending = false;  /* есть отложенная смена тока   */
static volatile uint8_t  s_currentCode = 0;

/* Диагностика (смотреть отладчиком / выводить по USB) */
volatile uint32_t g_analog_cycles        = 0;  /* завершённых сценариев            */
volatile uint32_t g_analog_cycle_aborts  = 0;  /* сценарий не успел / сбой DMA     */
volatile uint32_t g_analog_i2c_errors    = 0;  /* неудачных записей в MP3320A      */
volatile uint32_t g_analog_sleep_active  = 0;  /* 1 - MP3320A засыпает между сценариями */

/* Сон MP3320A между сценариями (см. ANALOG_MP3320_SLEEP_IN_IDLE).
   Значения REG01h с EN=1 / EN=0 запоминаются в Init(): остальные биты
   регистра (DMBLK, CH4MD, BLK_PWM, EN_CP) в прерывании не читаются. */
static volatile bool     s_sleepEnabled = false;
static uint8_t           s_modeOn  = 0;
static uint8_t           s_modeOff = 0;

extern DMA_QListTypeDef MainAdcQueue;
extern DMA_HandleTypeDef handle_GPDMA1_Channel10;
extern I2C_HandleTypeDef hi2c2;
extern uint32_t TimeSeconds;
extern osMessageQueueId_t dataRawQueueHandle;
extern SemaphoreHandle_t iic2RxTxMutex;

/* -------------------------------------------------------------------------
 *  Управление светодиодами
 *
 *  REG03h: [CLED4][CLED3][CLED2][CLED1][CH4EN][CH3EN][CH2EN][CH1EN]
 *  CLEDx = 1 -> канал работает как чистый источник тока (без ШИМ-модуляции)
 *
 *  0xF3 -> включены CH1+CH2  (740 нм)
 *  0xFC -> включены CH3+CH4  (850 нм)
 *  0xF0 -> все каналы выключены (измерение фона)
 * ------------------------------------------------------------------------- */
/* Запись регистра MP3320A из прерывания. Единственное место, где выбирается
   способ доступа к I2C (ADC_LED_SWITCH_IN_ISR). Мьютекс здесь НЕ берётся:
   xSemaphoreTakeFromISR() для мьютексов не допускается (нет наследования
   приоритета, владелец не запоминается), а результат раньше игнорировался,
   то есть защиты не было вовсе. Взаимное исключение обеспечено иначе:
   после Init() I2C2 использует только ISR. */
static void WriteRegIrq(uint8_t reg, uint8_t val)
{
	HAL_StatusTypeDef ret;
#if ADC_LED_SWITCH_IN_ISR
	/* Регистровая запись с ограниченным спин-таймаутом: без HAL и HAL_GetTick(). */
	ret = MP3320A_WriteReg_ISR(&hmp3320, reg, val);
#else
	ret = MP3320A_WriteReg(&hmp3320, reg, val);
#endif
	if (ret != HAL_OK)
		g_analog_i2c_errors++;
}

static void SetLedsState(ActiveLed activeLed)
{
	uint8_t chSet;

	switch (activeLed)
	{
	case Led_740:      chSet = 0xF3; break;
	case Led_850:      chSet = 0xFC; break;
	case Led_740_Bgd:
	case Led_850_Bgd:  chSet = 0xF0; break;
	default:           return;
	}

	WriteRegIrq(MP3320A_REG_CH_SET, chSet);
}

analog_driver::analog_driver()
{
	AdcDmaChannel = GPDMA1_Channel2_NS;
	AdcInstance = ADC1;
	_ActiveLed = Led_740;
}

analog_driver::~analog_driver()
{

}

void analog_driver::Init()
{
  /* Тайминги сценария (TIM3/TIM5) и шкала времени TIM2 рассчитаны от
     NIRS_SYSCLK_HZ. Если тактирование или предделитель TIM2 не совпали
     (например, CubeMX перегенерировал tim.c / main.cpp), измерения шли бы
     с неверными интервалами - лучше остановиться сразу. */
  if (SystemCoreClock != NIRS_SYSCLK_HZ ||
      TIM2->PSC != (NIRS_SYSCLK_HZ / 1000000UL - 1UL))
    Error_Handler();

  bool res = RCC_Init();
  if(!res)
  	Error_Handler();

  res = DMA_LLI_Init();
  if(!res)
  	Error_Handler();

  res = DMA_Init();
  if(!res)
  	Error_Handler();

  res = ADC_Init();
  if(!res)
  	Error_Handler();

  res = TIM3_Init();
  if(!res)
  	Error_Handler();

  res = TIM5_Init();
  if(!res)
  	Error_Handler();

  xSemaphoreTake(iic2RxTxMutex, pdMS_TO_TICKS(1000));

  if (MP3320A_Init(&hmp3320, &hi2c2, MP3320A_I2C_ADDR_DEFAULT) != HAL_OK) {
		Error_Handler(); // Ошибка I2C
	}

	/* Сначала конфигурируем (каналы выключены, ток задан), только потом EN=1.
	   Раньше MP3320A_Enable() вызывался ДО конфигурации -> кратковременная
	   вспышка всех 4 каналов током по умолчанию. */
	if (MP3320A_Config_Analog_PureCurrent(&hmp3320, 25.0f, 25.0f, 25.0f, 25.0f) != HAL_OK) {
		Error_Handler();
	}

#if ANALOG_MP3320_SLEEP_IN_IDLE
	ChipSleepSelfTest();
#endif

	xSemaphoreGive(iic2RxTxMutex);

	/* Между сценариями светодиоды погашены. Раньше здесь включался LED_740 и
	   горел до первого отсчёта. */
	_ActiveLed = Led_740;
	SetLedsState(Led_740_Bgd);
}

/* -------------------------------------------------------------------------
 *  Перезарядка канала GPDMA под ОДНУ передачу
 * ------------------------------------------------------------------------- */
void analog_driver::DMA_Rearm(void)
{
  uint32_t guard;

  /* В one-shot режиме бит EN снимается железом по завершении передачи.
     Если канал всё ещё активен (прерывание по ошибке / принудительный
     останов) - корректно гасим его через SUSP + RESET. */
  if (AdcDmaChannel->CCR & DMA_CCR_EN) {
    AdcDmaChannel->CCR |= DMA_CCR_SUSP;
    guard = ADC_STOP_GUARD;
    while (!(AdcDmaChannel->CSR & DMA_CSR_SUSPF) && --guard) { }
    AdcDmaChannel->CCR |= DMA_CCR_RESET;
    guard = ADC_STOP_GUARD;
    while ((AdcDmaChannel->CCR & DMA_CCR_EN) && --guard) { }
    AdcDmaChannel->CCR &= ~DMA_CCR_SUSP;
  }

  /* Сброс всех флагов канала (CFCR - write-1-to-clear, "|=" здесь бессмысленен) */
  AdcDmaChannel->CFCR = DMA_CFCR_TCF  | DMA_CFCR_HTF   | DMA_CFCR_DTEF |
                        DMA_CFCR_ULEF | DMA_CFCR_USEF  | DMA_CFCR_SUSPF |
                        DMA_CFCR_TOF;

  /* Полная перезагрузка дескриптора передачи */
  AdcDmaChannel->CCR  = AdcDmaCfg.CCR;
  AdcDmaChannel->CTR1 = AdcDmaCfg.CTR1;
  AdcDmaChannel->CTR2 = AdcDmaCfg.CTR2;
  AdcDmaChannel->CBR1 = AdcDmaCfg.CBR1;
  AdcDmaChannel->CSAR = AdcDmaCfg.CSAR;
  AdcDmaChannel->CDAR = AdcDmaCfg.CDAR;
  AdcDmaChannel->CLLR = 0U;              /* конец списка -> ОДНОКРАТНАЯ передача */

  AdcDmaChannel->CCR |= DMA_CCR_EN;
}

/* -------------------------------------------------------------------------
 *  Останов текущего блока измерения
 * ------------------------------------------------------------------------- */
void analog_driver::StopSequence(void)
{
  uint32_t guard;

  /* 1. Останавливаем регулярные преобразования */
  if (AdcInstance->CR & ADC_CR_ADSTART) {
    AdcInstance->CR |= ADC_CR_ADSTP;
    guard = ADC_STOP_GUARD;
    while ((AdcInstance->CR & ADC_CR_ADSTP) && --guard) { }
  }

  /* 2. Чистим статусные флаги ADC (в т.ч. OVR, который неизбежно
        взводится, если триггер пришёл после исчерпания счётчика DMA) */
  AdcInstance->ISR = ADC_ISR_EOC | ADC_ISR_EOS | ADC_ISR_EOSMP | ADC_ISR_OVR;
}

/* -------------------------------------------------------------------------
 *  Запуск ОДНОГО блока измерения (один этап сценария)
 *
 *  Только взводит DMA и АЦП. Триггер (TRGO) приходит от TIM3, который во
 *  время сценария считает свободно, поэтому отсчёты идут строго через
 *  ADC_PHASE_US независимо от времени работы обработчиков и I2C.
 * ------------------------------------------------------------------------- */
void analog_driver::StartSequence(void)
{
  /* Порядок важен: сперва готов приёмник (DMA), потом источник (ADC). */
  DMA_Rearm();

  AdcInstance->ISR = ADC_ISR_EOC | ADC_ISR_EOS | ADC_ISR_EOSMP | ADC_ISR_OVR;
  AdcInstance->CR |= ADC_CR_ADSTART;
}

/* Погасить канал GPDMA (общая часть Stop() и AbortCycle()) */
void analog_driver::DMA_Halt(void)
{
  if (AdcDmaChannel->CCR & DMA_CCR_EN) {
    uint32_t guard = ADC_STOP_GUARD;
    AdcDmaChannel->CCR |= DMA_CCR_SUSP;
    while (!(AdcDmaChannel->CSR & DMA_CSR_SUSPF) && --guard) { }
    AdcDmaChannel->CCR |= DMA_CCR_RESET;
    AdcDmaChannel->CCR &= ~DMA_CCR_SUSP;
  }

  AdcDmaChannel->CFCR = DMA_CFCR_TCF  | DMA_CFCR_HTF  | DMA_CFCR_DTEF |
                        DMA_CFCR_ULEF | DMA_CFCR_USEF | DMA_CFCR_SUSPF |
                        DMA_CFCR_TOF;
}

/* Прервать сценарий: остановить TIM3/АЦП/DMA и погасить светодиоды.
   Вызывается из прерываний (сбой DMA, наложение сценариев). */
void analog_driver::AbortCycle(void)
{
  TIM3->CR1 &= ~TIM_CR1_CEN;
  StopSequence();
  DMA_Halt();
  SetLedsState(Led_740_Bgd);
  _ActiveLed = Led_740;
  s_inCycle = false;
  g_analog_cycle_aborts++;
}

/* -------------------------------------------------------------------------
 *  Старт сценария: прерывание update таймера TIM5 (раз в ADC_CYCLE_US)
 * ------------------------------------------------------------------------- */
void analog_driver::CycleStart(void)
{
  /* Прошлый сценарий не завершился за весь период (потерян TRGO / DMA):
     гасим всё и начинаем заново с чистого состояния. */
  if (s_inCycle)
    AbortCycle();

  s_inCycle  = true;
  _ActiveLed = Led_740;

  StartSequence();                 /* DMA + АЦП ждут первый TRGO            */

  /* Первый TRGO придёт через полный период TIM3 (= этап 1). Таймер стартует
     ДО I2C-записи, чтобы все этапы были привязаны к нему одинаково. */
  TIM3->CNT = 0;
  TIM3->SR  = 0;
  TIM3->CR1 |= TIM_CR1_CEN;

  SetLedsState(Led_740);
}

void analog_driver::Start()
{
  Stop();

  /* Первый сценарий начнётся по первому update TIM5 (через ADC_CYCLE_US). */
  TIM5->CNT = 0;
  TIM5->SR  = 0;
  TIM5->DIER |= TIM_DIER_UIE | TIM_DIER_CC1IE;
  TIM5->CR1  |= TIM_CR1_CEN;
}

void analog_driver::Stop()
{
  /* Сначала прекращаем запуск новых сценариев, потом гасим текущий. */
  TIM5->CR1  &= ~TIM_CR1_CEN;
  TIM5->DIER &= ~TIM_DIER_UIE;
  TIM3->CR1  &= ~TIM_CR1_CEN;

  StopSequence();
  DMA_Halt();

  TIM5->DIER &= ~TIM_DIER_CC1IE;

  s_inCycle  = false;
  _ActiveLed = Led_740;
  SetLedsState(Led_740_Bgd);
  ChipSleep();           /* после этого I2C не трогаем >1.5 мс: следующее обращение - ChipWake() */
}

/* Запись отложенного тока светодиодов. Вызывается из прерывания сразу после
   последнего этапа сценария: светодиоды погашены, до следующего сценария
   остаётся почти весь период (при 100 Гц - 9 мс), 4 записи по ~80 мкс не мешают. */
void analog_driver::ApplyPendingCurrent(void)
{
  if (!s_currentPending)
    return;

  const uint8_t code = s_currentCode;
  s_currentPending = false;

  for (uint8_t reg = MP3320A_REG_ICH1; reg <= MP3320A_REG_ICH4; reg++)
    WriteRegIrq(reg, code);
}

/* EN=0: MP3320A в standby до следующего пробуждения. Вызывать после ВСЕХ
   I2C-операций сценария: следующие допустимы не раньше чем через 1.5 мс. */
void analog_driver::ChipSleep(void)
{
  if (s_sleepEnabled)
    WriteRegIrq(MP3320A_REG_MODE, s_modeOff);
}

/* EN=1: прерывание compare TIM5 за ANALOG_MP3320_WAKE_LEAD_US до сценария */
void analog_driver::ChipWake(void)
{
  if (s_sleepEnabled)
    WriteRegIrq(MP3320A_REG_MODE, s_modeOn);
}

/* Проверка, что MP3320A сохраняет регистры в standby (даташит этого прямо не
   гарантирует). Если после EN=0 регистры не совпали - сон отключается, а чип
   конфигурируется заново и остаётся включённым. Task-контекст, из Init(). */
void analog_driver::ChipSleepSelfTest(void)
{
  static const uint8_t regs[] = { MP3320A_REG_CH_SET,
                                  MP3320A_REG_ICH1, MP3320A_REG_ICH2,
                                  MP3320A_REG_ICH3, MP3320A_REG_ICH4 };
  uint8_t before[sizeof(regs)], after[sizeof(regs)], mode;
  bool ok = true;

  s_sleepEnabled = false;
  g_analog_sleep_active = 0;

  if (MP3320A_ReadReg(&hmp3320, MP3320A_REG_MODE, &mode) != HAL_OK)
    return;
  s_modeOn  = (uint8_t)(mode |  MP3320A_MODE_EN);
  s_modeOff = (uint8_t)(mode & ~MP3320A_MODE_EN);

  for (size_t i = 0; i < sizeof(regs); i++)
    ok = ok && (MP3320A_ReadReg(&hmp3320, regs[i], &before[i]) == HAL_OK);

  /* Enable(DISABLE) выдерживает паузу >1.5 мс после EN=0 */
  ok = ok && (MP3320A_Enable(&hmp3320, DISABLE) == HAL_OK);

  for (size_t i = 0; i < sizeof(regs); i++)
    ok = ok && (MP3320A_ReadReg(&hmp3320, regs[i], &after[i]) == HAL_OK)
            && (after[i] == before[i]);
  ok = ok && (MP3320A_ReadReg(&hmp3320, MP3320A_REG_MODE, &mode) == HAL_OK)
          && (mode == s_modeOff);

  if (ok) {
    s_sleepEnabled = true;          /* чип остаётся в standby до первого пробуждения */
    g_analog_sleep_active = 1;
  } else {
    /* регистры потеряны или обмен не удался: возвращаем чип в рабочее состояние */
    (void)MP3320A_Config_Analog_PureCurrent(&hmp3320, 25.0f, 25.0f, 25.0f, 25.0f);
  }
}

void analog_driver::SetIrLedsPower(float proc)
{
	/* Вызывается из контекста задачи. С I2C2 здесь НЕ работаем: шиной владеют
	   прерывания сценария, и запись из задачи могла бы вклиниться посреди
	   переключения светодиода (HAL вернул бы HAL_BUSY, и этап был бы
	   измерен при неверном состоянии светодиодов). Просто запоминаем код;
	   ApplyPendingCurrent() запишет его в конце ближайшего сценария. */
	if (proc < 0.0f)   proc = 0.0f;
	if (proc > 100.0f) proc = 100.0f;

	float mA = (MAX_IR_LEDS_CURRENT / 100.0f) * proc / 2.0f;   /* ток на канал */
	uint32_t code = (uint32_t)(mA / 0.2f + 0.5f);
	if (code > MP3320A_CURRENT_CODE_MAX) code = MP3320A_CURRENT_CODE_MAX;

	s_currentCode    = (uint8_t)code;
	s_currentPending = true;   /* флаг - после кода: ISR читает их в этом порядке */
}

bool analog_driver::RCC_Init()
{
  uint32_t delay = ADC_INITD_ELAY_VAL;

  // GPIO
  RCC->AHB2ENR1 |= RCC_AHB2ENR1_GPIOAEN | RCC_AHB2ENR1_GPIOCEN;

  // GPDMA
  RCC->AHB1ENR |= RCC_AHB1ENR_GPDMA1EN;
  while(!(RCC->AHB1ENR & RCC_AHB1ENR_GPDMA1EN)) {
    delay--;
    if(!delay) return false;
  }

  // Timer 3
  delay = ADC_INITD_ELAY_VAL;
  RCC->APB1ENR1 |= RCC_APB1ENR1_TIM3EN;
  while(!(RCC->APB1ENR1 & RCC_APB1ENR1_TIM3EN)){
    delay--;
    if(!delay) return false;
  }

  // Timer 5 (период сценариев)
  delay = ADC_INITD_ELAY_VAL;
  RCC->APB1ENR1 |= RCC_APB1ENR1_TIM5EN;
  while(!(RCC->APB1ENR1 & RCC_APB1ENR1_TIM5EN)){
    delay--;
    if(!delay) return false;
  }

  // ADC
  delay = ADC_INITD_ELAY_VAL;
  if(AdcInstance == ADC1 || AdcInstance == ADC4){
    /* Было: RCC->CCIPR3 |= (0 << ...) - операция "ИЛИ с нулём" ничего не делает.
       Явно выбираем источник (000 = HCLK, значение по умолчанию). */
    MODIFY_REG(RCC->CCIPR3, RCC_CCIPR3_ADCDACSEL, 0U);

    RCC->AHB2ENR1 |= RCC_AHB2ENR1_ADC12EN;
    while(!(RCC->AHB2ENR1 & RCC_AHB2ENR1_ADC12EN)){
      delay--;
      if(!delay) return false;
    }
  }
  else
    return false;

  return true;
}

/* -------------------------------------------------------------------------
 *  Подготовка шаблона канала GPDMA (связного списка больше нет)
 * ------------------------------------------------------------------------- */
bool analog_driver::DMA_LLI_Init()
{
  AdcDmaCfg.CTR1 = (0x0 << DMA_CTR1_DSEC_Pos)      |  // DSEC: nonsecure
                   (0x0 << DMA_CTR1_DAP_Pos)       |  // DAP: port 0
                   (0x0 << DMA_CTR1_DHX_Pos)       |
                   (0x0 << DMA_CTR1_DBX_Pos)       |
                   (0x0 << DMA_CTR1_DBL_1_Pos)     |  // burst = 1
                   (0x1 << DMA_CTR1_DINC_Pos)      |  // инкремент приёмника
                   (0x2 << DMA_CTR1_DDW_LOG2_Pos)  |  // 10 = word (32 бита!)
                   (0x0 << DMA_CTR1_SSEC_Pos)      |
                   (0x0 << DMA_CTR1_SAP_Pos)       |
                   (0x0 << DMA_CTR1_SBX_Pos)       |
                   (0x0 << DMA_CTR1_PAM_Pos)       |
                   (0x0 << DMA_CTR1_SBL_1_Pos)     |
                   (0x0 << DMA_CTR1_SINC_Pos)      |  // источник без инкремента
                   (0x2 << DMA_CTR1_SDW_LOG2_Pos);    // 10 = word (32 бита!)
  /* Примечание: DDW/SDW_LOG2 = 0b10 это WORD, а не half-word, как было
     написано в комментариях. Значение верное - RawDataMass[] это uint32_t. */

  AdcDmaCfg.CTR2 = (0x0 << DMA_CTR2_TCEM_Pos)    |  // TC на уровне блока
                   (0x0 << DMA_CTR2_TRIGPOL_Pos) |
                   (0x0 << DMA_CTR2_TRIGSEL_Pos) |
                   (0x0 << DMA_CTR2_TRIGM_Pos)   |
                   (0x0 << DMA_CTR2_BREQ_Pos)    |
                   (0x0 << DMA_CTR2_DREQ_Pos)    |  // источник = периферия
                   (0x0 << DMA_CTR2_SWREQ_Pos)   |  // аппаратный запрос
                   (0x0 << DMA_CTR2_REQSEL_Pos);    // 0 = GPDMA1_REQUEST_ADC1

  AdcDmaCfg.CBR1 = ADC_RAW_MASS_SIZE * sizeof(RawDataMass[0]); /* BNDT в БАЙТАХ */
  AdcDmaCfg.CSAR = (uint32_t)&AdcInstance->DR;
  AdcDmaCfg.CDAR = (uint32_t)RawDataMass;

  AdcDmaCfg.CCR  = (0x3 << DMA_CCR_PRIO_Pos)   |  // высокий приоритет
                   (0x0 << DMA_CCR_LAP_Pos)    |
                   (0x0 << DMA_CCR_LSM_Pos)    |
                   (0x1 << DMA_CCR_TOIE_Pos)   |
                   (0x1 << DMA_CCR_TCIE_Pos)   |  // transfer complete
                   (0x0 << DMA_CCR_SUSPIE_Pos) |
                   (0x1 << DMA_CCR_USEIE_Pos)  |
                   (0x1 << DMA_CCR_ULEIE_Pos)  |
                   (0x1 << DMA_CCR_DTEIE_Pos)  |
                   (0x0 << DMA_CCR_HTIE_Pos);     // HT больше не нужен

  return true;
}

bool analog_driver::DMA_Init()
{
  uint32_t timeout;

  AdcDmaChannel->CCR &= ~DMA_CCR_EN;
  timeout = ADC_STOP_GUARD;
  while((AdcDmaChannel->CCR & DMA_CCR_EN) && --timeout) {}

  AdcDmaChannel->CCR |= DMA_CCR_RESET;
  for(volatile uint32_t i = 0; i < 100; i++);

  /* Канал 2 -> бит 2 в SECCFGR/PRIVCFGR */
  GPDMA1->SECCFGR  &= ~(1UL << 2);
  GPDMA1->PRIVCFGR |=  (1UL << 2);

  /* CLBAR используется только при работе со связным списком.
     В one-shot он не задействован, но оставляем корректное значение. */
  AdcDmaChannel->CLBAR = ((uint32_t)&AdcDmaCfg) & 0xFFFF0000UL;

  DMA_Rearm();
  AdcDmaChannel->CCR &= ~DMA_CCR_EN;   /* запуск делает StartSequence() */

  NVIC_SetPriority(GPDMA1_Channel2_IRQn, 5);
  NVIC_EnableIRQ(GPDMA1_Channel2_IRQn);

  return true;
}

bool analog_driver::ADC_Init()
{
  uint32_t delay = ADC_INITD_ELAY_VAL;

  // --- 1. GPIO (аналоговый режим) ---
  // PA1 (ADC1_IN6)
  GPIOA->PUPDR &= ~(3UL << (1*2));
  GPIOA->MODER |=  (3UL << (1*2));
  // PA7 (ADC1_IN12)
  GPIOA->PUPDR &= ~(3UL << (7*2));
  GPIOA->MODER |=  (3UL << (7*2));
  // PC4 (ADC1_IN13)
  GPIOC->PUPDR &= ~(3UL << (4*2));
  GPIOC->MODER |=  (3UL << (4*2));
  // PC5 (ADC1_IN14)
  GPIOC->PUPDR &= ~(3UL << (5*2));
  GPIOC->MODER |=  (3UL << (5*2));

  if(AdcInstance->CR & ADC_CR_ADEN)
    return false;

  // PRESC = 0001 -> деление на 2: HCLK 80 МГц -> тактовая АЦП 40 МГц
  // (при 160 МГц было 80 МГц - выше допустимой fADC микроконтроллера)
  MODIFY_REG(ADC12_COMMON->CCR, ADC_CCR_PRESC, (1UL << ADC_CCR_PRESC_Pos));

  // --- 2. Питание ---
  AdcInstance->CR = 0;                    // сбрасывает в т.ч. DEEPPWD
  AdcInstance->CR &= ~ADC_CR_DEEPPWD;
  AdcInstance->CR |= ADC_CR_ADVREGEN;
  for(volatile int i = 0; i < 10000; i++);   // задержка запуска регулятора

  // --- 3. Преселекция каналов ---
  /* ВАЖНО: PCSEL должен быть записан ПРИ ADEN = 0 и ДО калибровки.
     В исходнике он выставлялся уже после включения ADC. */
  AdcInstance->PCSEL = ADC_PCSEL_PCSEL_6  | ADC_PCSEL_PCSEL_12 |
                       ADC_PCSEL_PCSEL_13 | ADC_PCSEL_PCSEL_14;

  // --- 4. Калибровка ---
  AdcInstance->CR |= ADC_CR_ADCAL;
  while(AdcInstance->CR & ADC_CR_ADCAL){
    delay--;
    if(!delay) return false;
  }
  for(volatile int i = 0; i < 100; i++);   // >4 такта ADC перед ADEN

  // --- 5. Включение ---
  delay = ADC_INITD_ELAY_VAL;
  AdcInstance->ISR = ADC_ISR_ADRDY;
  AdcInstance->CR |= ADC_CR_ADEN;
  while(!(AdcInstance->ISR & ADC_ISR_ADRDY)){
    delay--;
    if(!delay) return false;
  }

  // --- 6. Последовательность сканирования (4 канала) ---
  AdcInstance->SQR1 = (3UL << ADC_SQR1_L_Pos)   |   // L = 3 -> 4 преобразования
                      (6UL  << ADC_SQR1_SQ1_Pos) |
                      (12UL << ADC_SQR1_SQ2_Pos) |
                      (13UL << ADC_SQR1_SQ3_Pos) |
                      (14UL << ADC_SQR1_SQ4_Pos);

  /* SMP = 1 это 6.5 такта. Для высокоомных источников (в названиях каналов
     фигурируют 1/5.1/20/100 МОм) этого заведомо мало - см. замечания. */
  AdcInstance->SMPR1 = (1UL << ADC_SMPR1_SMP6_Pos);
  AdcInstance->SMPR2 = (1UL << ADC_SMPR2_SMP12_Pos) |
                       (1UL << ADC_SMPR2_SMP13_Pos) |
                       (1UL << ADC_SMPR2_SMP14_Pos);

  // --- 7. Конфигурация ---
  AdcInstance->CFGR1 = (1UL << ADC_CFGR1_DMNGT_Pos) | // 01 = DMA ONE SHOT (было 11 = circular)
                       (1UL << ADC_CFGR1_RES_Pos)   | // 12 бит
                       ADC_CFGR1_OVRMOD             |
                       (4UL << ADC_CFGR1_EXTSEL_Pos)| // TIM3_TRGO
                       (1UL << ADC_CFGR1_EXTEN_Pos);  // по фронту

  AdcInstance->IER |= ADC_IER_OVRIE;

  return true;
}

bool analog_driver::TIM3_Init(void) {
    TIM3->CR1 = 0;
    TIM3->CR2 = 0;
    TIM3->DIER = 0;              /* прерываний TIM3 нет: только TRGO для АЦП */

    TIM3->PSC = ADC_TIM_Prescaler;
    TIM3->ARR = ADC_TIM_Period;  /* 9999 -> ровно ADC_PHASE_US (было 10000 = +0.01 %) */

    /* TRGO = Update event */
    TIM3->CR2 &= ~TIM_CR2_MMS;
    TIM3->CR2 |= (0x2UL << TIM_CR2_MMS_Pos);

    /* URS = 1: событие Update генерируется только по переполнению,
       чтобы программная перезагрузка счётчика не породила лишний TRGO. */
    TIM3->CR1 |= TIM_CR1_URS;

    TIM3->EGR = TIM_EGR_UG;      /* загрузить PSC/ARR (АЦП ещё не взведён - TRGO безвреден) */
    TIM3->CNT = 0;
    TIM3->SR  = 0;

    /* Счёт НЕ запускается: TIM3 работает только во время сценария
       (CycleStart() -> CEN=1, последний этап -> CEN=0). */
    return true;
}

/* TIM5 (32 бита) - задатчик периода сценариев. Прерывание update приходит в
   HAL_TIM_PeriodElapsedCallback() (main.cpp) и вызывает CycleStart(). */
bool analog_driver::TIM5_Init(void) {
    TIM5->CR1 &= ~TIM_CR1_CEN;
    TIM5->DIER = 0;

    TIM5->PSC = 0;
    TIM5->ARR = ADC_CYCLE_TIM_Period;
    TIM5->CCMR1 = 0;                     /* CC1 - выход, замороженный: только событие compare */
    TIM5->CCR1  = ADC_WAKE_TIM_Compare;  /* пробуждение MP3320A */

    TIM5->EGR = TIM_EGR_UG;      /* загрузить ARR; флаг UIF сбрасываем ниже, */
    TIM5->CNT = 0;               /* иначе первое прерывание придёт сразу     */
    TIM5->SR  = 0;

    /* Приоритет и разрешение линии уже выставлены HAL_TIM_Base_MspInit()
       (MX_TIM5_Init): приоритет 5, как у DMA - прерывания друг друга не вытесняют. */
    return true;
}

void analog_driver::Adjust_Brightness(uint8_t channel, float target_mA) {
    if (channel < 1 || channel > 4) return;
    uint32_t code = (uint32_t)(target_mA / 0.2f + 0.5f);
    if (code > MP3320A_CURRENT_CODE_MAX) code = MP3320A_CURRENT_CODE_MAX;
    uint8_t reg = MP3320A_REG_ICH1 + (channel - 1);
    MP3320A_WriteReg(&hmp3320, reg, (uint8_t)code);
}

/* -------------------------------------------------------------------------
 *  Обработчик прерывания GPDMA1 Channel 2
 * ------------------------------------------------------------------------- */
void analog_driver::AdcConvCompleteDMA(void) {
  static MesurementRawData_t MesData;
  bool isMesCycleComplete = false;
  uint32_t status = AdcDmaChannel->CSR;

  /* ---- Ошибки: сценарий прерываем, следующий начнётся по TIM5 ---- */
  if (status & (DMA_CSR_DTEF | DMA_CSR_ULEF | DMA_CSR_USEF | DMA_CSR_TOF)) {
    AdcDmaChannel->CFCR = DMA_CFCR_DTEF | DMA_CFCR_ULEF |
                          DMA_CFCR_USEF | DMA_CFCR_TOF  |
                          DMA_CFCR_HTF  | DMA_CFCR_TCF  | DMA_CFCR_SUSPF;
    AbortCycle();
    return;
  }

  /* ---- Не завершение передачи (HT / SUSP) - просто чистим ---- */
  if (!(status & DMA_CSR_TCF)) {
    AdcDmaChannel->CFCR = DMA_CFCR_HTF | DMA_CFCR_SUSPF;
    return;
  }

  AdcDmaChannel->CFCR = DMA_CFCR_TCF | DMA_CFCR_HTF;

  /* Завершение вне сценария (запоздавший флаг после AbortCycle/Stop) - игнорируем */
  if (!s_inCycle)
    return;

  /* Блок завершён. В one-shot режиме канал DMA уже выключен железом,
     но ADC и таймер надо остановить ЯВНО - иначе следующая серия
     преобразований начнётся во время переключения светодиода по I2C. */
  StopSequence();

  HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);

#if ADC_DEBUG_PRINT
  printf("%4lu;\t%4lu;\t%4lu;\t%4lu;\n",
         RawDataMass[0], RawDataMass[1], RawDataMass[2], RawDataMass[3]);
#endif

  MesData.Channel_1_MOhm   = 0;
  MesData.Channel_5_1_MOhm = 0;
  MesData.Channel_20_MOhm  = 0;
  MesData.Channel_100_MOhm = 0;

  for (int i = 0; i < ADC_RAW_MASS_SIZE; i += ADC_RAW_DATA_CHANNELS)
  {
    MesData.Channel_1_MOhm   += RawDataMass[i];
    MesData.Channel_5_1_MOhm += RawDataMass[1 + i];
    MesData.Channel_20_MOhm  += RawDataMass[2 + i];
    MesData.Channel_100_MOhm += RawDataMass[3 + i];
  }
  MesData.Channel_1_MOhm   /= ADC_RAW_DATA_COUNT;
  MesData.Channel_5_1_MOhm /= ADC_RAW_DATA_COUNT;
  MesData.Channel_20_MOhm  /= ADC_RAW_DATA_COUNT;
  MesData.Channel_100_MOhm /= ADC_RAW_DATA_COUNT;

  switch (_ActiveLed)
  {
  case Led_740:
    TotalData.Led740_1 = (uint16_t)MesData.Channel_1_MOhm;
    TotalData.Led740_2 = (uint16_t)MesData.Channel_5_1_MOhm;
    TotalData.Led740_3 = (uint16_t)MesData.Channel_20_MOhm;
    TotalData.Led740_4 = (uint16_t)MesData.Channel_100_MOhm;
    _ActiveLed = Led_740_Bgd;
    break;

  case Led_740_Bgd:
    TotalData.Led740_Bgd_1 = (uint16_t)MesData.Channel_1_MOhm;
    TotalData.Led740_Bgd_2 = (uint16_t)MesData.Channel_5_1_MOhm;
    TotalData.Led740_Bgd_3 = (uint16_t)MesData.Channel_20_MOhm;
    TotalData.Led740_Bgd_4 = (uint16_t)MesData.Channel_100_MOhm;
    _ActiveLed = Led_850;
    break;

  case Led_850:
    TotalData.Led850_1 = (uint16_t)MesData.Channel_1_MOhm;
    TotalData.Led850_2 = (uint16_t)MesData.Channel_5_1_MOhm;
    TotalData.Led850_3 = (uint16_t)MesData.Channel_20_MOhm;
    TotalData.Led850_4 = (uint16_t)MesData.Channel_100_MOhm;
    _ActiveLed = Led_850_Bgd;
    break;

  case Led_850_Bgd:
    TotalData.Led850_Bgd_1 = (uint16_t)MesData.Channel_1_MOhm;
    TotalData.Led850_Bgd_2 = (uint16_t)MesData.Channel_5_1_MOhm;
    TotalData.Led850_Bgd_3 = (uint16_t)MesData.Channel_20_MOhm;
    TotalData.Led850_Bgd_4 = (uint16_t)MesData.Channel_100_MOhm;
    isMesCycleComplete = true;
    _ActiveLed = Led_740;
    break;

  default:
    _ActiveLed = Led_740;
    break;
  }

  if (isMesCycleComplete)
  {
    /* Сценарий закончен: 4-й отсчёт (фон 850) снят при погашенных светодиодах,
       переключать нечего. TIM3 останавливаем СРАЗУ (следующий TRGO придёт
       через ADC_PHASE_US), АЦП/DMA уже остановлены выше. */
    TIM3->CR1 &= ~TIM_CR1_CEN;
    s_inCycle = false;
    g_analog_cycles++;

    /* Полный цикл (740 / фон / 850 / фон) собран - отдаём в очередь */
    TotalData.Time = (uint64_t)TimeSeconds * 1000000ULL + (uint64_t)TIM2->CNT;
    /* Было: (uint64_t)(TimeSeconds * 1000000) - умножение в 32 битах,
       переполнение примерно через 4295 с работы. */
    (void)osMessageQueuePut(dataRawQueueHandle, &TotalData, 0, 0);

    /* Светодиоды погашены, шина свободна - применяем отложенную смену тока */
    ApplyPendingCurrent();

    /* Последней I2C-операцией сценария усыпляем MP3320A (EN=0) */
    ChipSleep();

    HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
    return;
  }

  /* Переключение светодиода выполняется при ОСТАНОВЛЕННЫХ ADC/DMA
     (TIM3 продолжает считать - следующий отсчёт ровно через ADC_PHASE_US
     после предыдущего) */
  SetLedsState(_ActiveLed);

  HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);

  /* Взвод следующего этапа. */
  StartSequence();
}

void analog_driver::AdcConvCompleteIT(void) {
  uint32_t isr = ADC1->ISR;

  if (isr & ADC_ISR_EOSMP) ADC1->ISR = ADC_ISR_EOSMP;
  if (isr & ADC_ISR_OVR)   ADC1->ISR = ADC_ISR_OVR;
  if (isr & ADC_ISR_EOS)   ADC1->ISR = ADC_ISR_EOS;
}
