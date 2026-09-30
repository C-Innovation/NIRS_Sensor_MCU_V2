/*
 * power_mgmt.cpp
 */
#include "power_mgmt.h"
#include "stm32u5xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"

extern TIM_HandleTypeDef htim1;      /* HAL time base (stm32u5xx_hal_timebase_tim.c) */
extern __IO uint32_t uwTick;

static volatile bool     s_tickFromRtos = false;
static volatile uint32_t s_tickBase = 0;

/* Замена weak-функции HAL. До старта планировщика (HAL_Init, настройка
   тактирования) работает прежний uwTick от TIM1. После Power_HalTickToRtos()
   время берётся у FreeRTOS: TIM1 остановлен и не будит ядро каждую миллисекунду,
   а счётчик тиков FreeRTOS корректно наверстывается после tickless-сна.
   Смещение выбрано так, чтобы значение не прыгало в момент переключения. */
extern "C" uint32_t HAL_GetTick(void)
{
  if (s_tickFromRtos)
    return s_tickBase + (uint32_t)xTaskGetTickCount();
  return uwTick;
}

extern "C" void Power_HalTickToRtos(void)
{
  taskENTER_CRITICAL();
  s_tickBase     = uwTick - (uint32_t)xTaskGetTickCount();
  s_tickFromRtos = true;
  HAL_SuspendTick();                 /* TIM1: запретить прерывание update */
  __HAL_TIM_DISABLE(&htim1);         /* и остановить счётчик */
  taskEXIT_CRITICAL();
}

/* ---------------------------------------------------------------------------
 *  Про Stop-режим (следующий этап, здесь НЕ реализован)
 *
 *  Sleep оставляет тактирование включённым: ядро не исполняет команды, но PLL
 *  160 МГц, шины и периферия потребляют ток. Stop 1/2 даёт на порядки меньше,
 *  но требует:
 *   - заменить TIM3/TIM5 (сценарий измерения и период 10 мс) на LPTIM или RTC
 *     wake-up: в Stop обычные таймеры стоят; TRGO для АЦП на время сценария
 *     (1 мс) можно оставить активным, просыпаясь на весь сценарий;
 *   - заменить источник тика FreeRTOS на LPTIM (SysTick в Stop стоит);
 *   - восстанавливать тактирование (HSI, PLL, HSI48) после каждого выхода;
 *   - решить вопрос с USB: при подключённом хосте USB-CDC требует HSI48 и
 *     регулярной обработки, Stop допустим только без USB (VBUS отсутствует)
 *     либо в USB suspend.
 * ------------------------------------------------------------------------- */
