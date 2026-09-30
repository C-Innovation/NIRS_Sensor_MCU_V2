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

#define ADC_TIM_Prescaler (uint32_t)(4-1)
#define ADC_TIM_Period (uint32_t)(10000)

#define MAX_IR_LEDS_CURRENT	100.0f

/* 1 - переключение светодиодов делается прямо в DMA ISR регистровой
 *     (не-HAL) записью по I2C: HAL_I2C_* в прерывании опирается на
 *     HAL_GetTick(), а SysTick под FreeRTOS имеет НИЗШИЙ приоритет,
 *     поэтому таймаут никогда не сработает -> потенциальный вечный цикл.
 * 0 - переключение вынесено в задачу (нужно реализовать снаружи). */
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

private:

  bool RCC_Init();
  bool DMA_Init();
  bool DMA_LLI_Init();
  bool ADC_Init();
  bool TIM3_Init(void);
  void Adjust_Brightness(uint8_t channel, float target_mA);

  /* Однократный запуск/останов одного блока измерения.
     static - потому что вызываются из статического обработчика прерывания. */
  static void StartSequence(void);
  static void StopSequence(void);
  static void DMA_Rearm(void);

  static inline void AdcConvCompleteDMA() asm("GPDMA1_Channel2_IRQHandler") __attribute__((used));
  static inline void AdcConvCompleteIT() asm("ADC1_2_IRQHandler") __attribute__((used));

  static inline DMA_Channel_TypeDef * AdcDmaChannel;
  static inline ADC_TypeDef * AdcInstance;
};

#endif /* ANALOG_DRIVER_H_ */
