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

#if ADC_LED_SWITCH_IN_ISR
	/* НЕ используем HAL_I2C_* и НЕ трогаем мьютекс:
	   - xSemaphoreTakeFromISR() запрещён для мьютексов (только для бинарных
	     семафоров/счётчиков), исходный код полагался на UB;
	   - HAL_I2C_Mem_Write() в ISR может зависнуть навсегда (SysTick не тикает).
	   Регистровая запись с ограниченным спин-таймаутом безопасна.
	   ВАЖНО: I2C2 не должен использоваться из задач одновременно с этим
	   вызовом. Все задачные обращения к I2C2 обязаны брать iic2RxTxMutex,
	   а лучше - вынести переключение светодиодов в отдельную задачу. */
	(void)MP3320A_WriteReg_ISR(&hmp3320, MP3320A_REG_CH_SET, chSet);
#else
//	if (xSemaphoreTake(iic2RxTxMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
//		(void)MP3320A_WriteReg(&hmp3320, MP3320A_REG_CH_SET, chSet);
//		xSemaphoreGive(iic2RxTxMutex);
//	}
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreTakeFromISR(iic2RxTxMutex, &xHigherPriorityTaskWoken);
	(void)MP3320A_WriteReg(&hmp3320, MP3320A_REG_CH_SET, chSet);
	xSemaphoreGiveFromISR(iic2RxTxMutex, &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
#endif
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

	xSemaphoreGive(iic2RxTxMutex);

	_ActiveLed = Led_740;
	SetLedsState(_ActiveLed);
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

  /* 1. Прекращаем генерацию триггеров */
//  TIM3->CR1 &= ~TIM_CR1_CEN;

  /* 2. Останавливаем регулярные преобразования */
  if (AdcInstance->CR & ADC_CR_ADSTART) {
    AdcInstance->CR |= ADC_CR_ADSTP;
    guard = ADC_STOP_GUARD;
    while ((AdcInstance->CR & ADC_CR_ADSTP) && --guard) { }
  }

  /* 3. Чистим статусные флаги ADC (в т.ч. OVR, который неизбежно
        взводится, если триггер пришёл после исчерпания счётчика DMA) */
  AdcInstance->ISR = ADC_ISR_EOC | ADC_ISR_EOS | ADC_ISR_EOSMP | ADC_ISR_OVR;
}

/* -------------------------------------------------------------------------
 *  Запуск ОДНОГО блока измерения
 * ------------------------------------------------------------------------- */
void analog_driver::StartSequence(void)
{
  /* Порядок важен: сперва готов приёмник (DMA), потом источник (ADC),
     и только затем разрешаем триггеры (TIM3). */
  DMA_Rearm();

  AdcInstance->ISR = ADC_ISR_EOC | ADC_ISR_EOS | ADC_ISR_EOSMP | ADC_ISR_OVR;
  AdcInstance->CR |= ADC_CR_ADSTART;

  /* CNT = 0 -> первый TRGO придёт через полный период таймера.
     Этот интервал одновременно служит временем установления светодиода
     после переключения MP3320A. */
//  TIM3->CNT = 0;
//  TIM3->SR  = 0;
//  TIM3->CR1 |= TIM_CR1_CEN;
}

void analog_driver::Start()
{
  StopSequence();
  StartSequence();
}

void analog_driver::Stop()
{
  StopSequence();

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

void analog_driver::SetIrLedsPower(float proc)
{
	/* Вызывается из контекста задачи -> обычные (не FromISR) примитивы. */
	if (proc < 0.0f)   proc = 0.0f;
	if (proc > 100.0f) proc = 100.0f;

	float mA = (MAX_IR_LEDS_CURRENT / 100.0f) * proc / 2.0f;   /* ток на канал */
	uint32_t code = (uint32_t)(mA / 0.2f + 0.5f);
	if (code > MP3320A_CURRENT_CODE_MAX) code = MP3320A_CURRENT_CODE_MAX;

//	if (xSemaphoreTake(iic2RxTxMutex, pdMS_TO_TICKS(100)) != pdTRUE)
//		return;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreTakeFromISR(iic2RxTxMutex, &xHigherPriorityTaskWoken);
	HAL_StatusTypeDef ret;
	for (uint8_t reg = MP3320A_REG_ICH1; reg <= MP3320A_REG_ICH4; reg++)
		(void)MP3320A_WriteReg(&hmp3320, reg, (uint8_t)code);

//	xSemaphoreGive(iic2RxTxMutex);
	xSemaphoreGiveFromISR(iic2RxTxMutex, &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
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

  // PRESC = 0001 -> деление на 2 (комментарий "/1" в исходнике был неверен)
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

    TIM3->PSC = ADC_TIM_Prescaler;
    TIM3->ARR = ADC_TIM_Period;

    /* TRGO = Update event */
    TIM3->CR2 &= ~TIM_CR2_MMS;
    TIM3->CR2 |= (0x2UL << TIM_CR2_MMS_Pos);

    /* URS = 1: событие Update генерируется только по переполнению,
       чтобы программная перезагрузка счётчика не породила лишний TRGO. */
    TIM3->CR1 |= TIM_CR1_URS;

    TIM3->EGR = TIM_EGR_UG;      /* загрузить PSC/ARR (при URS=1 TRGO не будет) */
    TIM3->SR  = 0;

    TIM3->CR1 |= TIM_CR1_CEN;
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

  /* ---- Ошибки: гасим всё и перезапускаем блок ---- */
  if (status & (DMA_CSR_DTEF | DMA_CSR_ULEF | DMA_CSR_USEF | DMA_CSR_TOF)) {
    AdcDmaChannel->CFCR = DMA_CFCR_DTEF | DMA_CFCR_ULEF |
                          DMA_CFCR_USEF | DMA_CFCR_TOF  |
                          DMA_CFCR_HTF  | DMA_CFCR_TCF  | DMA_CFCR_SUSPF;
    StopSequence();
    StartSequence();
    return;
  }

  /* ---- Не завершение передачи (HT / SUSP) - просто чистим ---- */
  if (!(status & DMA_CSR_TCF)) {
    AdcDmaChannel->CFCR = DMA_CFCR_HTF | DMA_CFCR_SUSPF;
    return;
  }

  AdcDmaChannel->CFCR = DMA_CFCR_TCF | DMA_CFCR_HTF;

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

  /* Полный цикл (740 / фон / 850 / фон) собран - отдаём в очередь */
  if (isMesCycleComplete)
  {
    TotalData.Time = (uint64_t)TimeSeconds * 1000000ULL + (uint64_t)TIM2->CNT;
    /* Было: (uint64_t)(TimeSeconds * 1000000) - умножение в 32 битах,
       переполнение примерно через 4295 с работы. */
    (void)osMessageQueuePut(dataRawQueueHandle, &TotalData, 0, 0);
  }

  /* Переключение светодиода выполняется при ОСТАНОВЛЕННЫХ ADC/TIM/DMA */
  SetLedsState(_ActiveLed);

  HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);

  /* Перезапуск однократного измерения. Первый триггер придёт через
     полный период TIM3 - это время установления светодиода. */
  StartSequence();
}

void analog_driver::AdcConvCompleteIT(void) {
  uint32_t isr = ADC1->ISR;

  if (isr & ADC_ISR_EOSMP) ADC1->ISR = ADC_ISR_EOSMP;
  if (isr & ADC_ISR_OVR)   ADC1->ISR = ADC_ISR_OVR;
  if (isr & ADC_ISR_EOS)   ADC1->ISR = ADC_ISR_EOS;
}
