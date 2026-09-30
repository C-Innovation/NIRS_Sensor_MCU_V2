/*
 * analog_driver.h
 *
 *  Created on: Jun 10, 2026
 *      Author: ivanov.y
 *
 *  ИЗМЕНЕНО: ADC+DMA+TIM переведены из циклического (circular) режима
 *            в однократный (one-shot). Перезапуск блока измерения
 *            выполняется в analog_driver::AdcConvCompleteDMA().
 */

#ifndef ANALOG_DRIVER_H_
#define ANALOG_DRIVER_H_

#include "main.h"
#include "data_structs.h"

#define ADC_RAW_DATA_COUNT (int)1
#define ADC_RAW_DATA_CHANNELS (int)4
#define ADC_RAW_MASS_SIZE (int)(ADC_RAW_DATA_COUNT * ADC_RAW_DATA_CHANNELS)

#define ADC_DEBUG_PRINT 0
#define ADC_INITD_ELAY_VAL (uint32_t)0xffffffffUL

/* Ограниченный спин-таймаут для ожиданий внутри ISR (без HAL_GetTick) */
#define ADC_STOP_GUARD (uint32_t)100000UL

/* ---- Тайминги сценария ------------------------------------------------
 *  Сценарий (4 этапа по ADC_PHASE_US): LED_740 -> темнота -> LED_850 -> темнота.
 *  Этапы задаёт TIM3 (свободный счёт, TRGO на каждом переполнении = запуск АЦП),
 *  но только на время сценария: TIM3 стартует из TIM5 и останавливается по
 *  окончании 4-го этапа. Период следования сценариев задаёт TIM5.
 *  Остальное время цикла (ADC_CYCLE_US - 4*ADC_PHASE_US) светодиоды погашены,
 *  АЦП/DMA/TIM3 не работают, ядро спит в WFI (FreeRTOS idle).
 * ---------------------------------------------------------------------- */
#define ADC_TIM_CLK_HZ      (uint32_t)160000000UL   /* тактовая TIM3/TIM5 (APB1 = HCLK) */
#define ADC_PHASE_US        (uint32_t)250U          /* длительность одного этапа        */
#define ADC_PHASES_PER_CYCLE (uint32_t)4U
#define ADC_CYCLE_US        (uint32_t)(1000000UL / NIRS_SAMPLE_RATE_HZ)

/* TIM3: 160 МГц / 4 = 40 МГц; ARR = отсчёты за этап - 1 (ARR+1 = период) */
#define ADC_TIM_Prescaler   (uint32_t)(4-1)
#define ADC_TIM_Period      (uint32_t)(ADC_PHASE_US * (ADC_TIM_CLK_HZ / (ADC_TIM_Prescaler + 1U) / 1000000UL) - 1U)
/* TIM5 (32 бита): без предделителя, ARR = отсчёты за цикл - 1 */
#define ADC_CYCLE_TIM_Period (uint32_t)(ADC_CYCLE_US * (ADC_TIM_CLK_HZ / 1000000UL) - 1U)

static_assert((ADC_PHASE_US * ADC_PHASES_PER_CYCLE) < ADC_CYCLE_US,
              "Сценарий (4 этапа) не помещается в период следования: уменьшите NIRS_SAMPLE_RATE_HZ");
static_assert(ADC_TIM_Period <= 0xFFFFU, "ADC_TIM_Period не помещается в 16-битный TIM3");

#define MAX_IR_LEDS_CURRENT	100.0f

/* Способ записи в MP3320A из прерываний (переключение этапов, смена тока).
 * После Init() I2C2 принадлежит ТОЛЬКО прерываниям TIM5/GPDMA (одного приоритета,
 * друг друга не вытесняют); из задач I2C2 не трогать - SetIrLedsPower() лишь
 * откладывает запись до конца текущего сценария.
 * 1 - регистровая запись с ограниченным спин-таймаутом (MP3320A_WriteReg_ISR):
 *     не зависит от HAL_GetTick(). SysTick под FreeRTOS имеет НИЗШИЙ приоритет,
 *     поэтому при зависшей шине HAL-таймаут внутри ISR не сработает никогда.
 * 0 - HAL_I2C_Mem_Write (как было). Работает, пока шина исправна. */
#define ADC_LED_SWITCH_IN_ISR 0

typedef enum
{
  Led_740 = 0,
  Led_740_Bgd = 2,
  Led_850 = 1,
  Led_850_Bgd = 3,
}ActiveLed;

typedef struct
{
  uint32_t Channel_1_MOhm;
  uint32_t Channel_5_1_MOhm;
  uint32_t Channel_20_MOhm;
  uint32_t Channel_100_MOhm;
}MesurementRawData_t;

class analog_driver
{
public:
	analog_driver();
	~analog_driver();
	void Init();
  void Start();
  void Stop();
  void SetIrLedsPower(float proc);

  /* Старт очередного сценария. Вызывать ТОЛЬКО из прерывания TIM5 (update):
     см. HAL_TIM_PeriodElapsedCallback в main.cpp. */
  static void CycleStart(void);

private:

  bool RCC_Init();
  bool DMA_Init();
  bool DMA_LLI_Init();
  bool ADC_Init();
  bool TIM3_Init(void);
  bool TIM5_Init(void);
  void Adjust_Brightness(uint8_t channel, float target_mA);

  /* Однократный запуск/останов одного блока измерения.
     static - потому что вызываются из статического обработчика прерывания. */
  static void StartSequence(void);
  static void StopSequence(void);
  static void DMA_Rearm(void);
  static void DMA_Halt(void);
  static void AbortCycle(void);
  static void ApplyPendingCurrent(void);

  static inline void AdcConvCompleteDMA() asm("GPDMA1_Channel2_IRQHandler") __attribute__((used));
  static inline void AdcConvCompleteIT() asm("ADC1_2_IRQHandler") __attribute__((used));

  static inline DMA_Channel_TypeDef * AdcDmaChannel;
  static inline ADC_TypeDef * AdcInstance;
};

#endif /* ANALOG_DRIVER_H_ */
