/*
 * power_mgmt.h
 *
 *  Управление энергопотреблением ядра: tickless idle FreeRTOS и отвязка
 *  HAL_GetTick() от аппаратного таймера TIM1.
 *
 *  Зачем. Без этого ядро просыпалось ~3 тысячи раз в секунду: прерывание
 *  SysTick FreeRTOS (1 кГц), прерывание TIM1 - HAL-тик (1 кГц) и опрос задач
 *  через osDelay(1). При сценарии измерения 100 Гц полезных пробуждений ~200/с.
 *
 *  Что сделано.
 *   1. configUSE_TICKLESS_IDLE = 1 (FreeRTOSConfig.h): пока все задачи ждут,
 *      SysTick останавливается и перепрограммируется на ближайший таймаут, ядро
 *      спит в WFI (режим Sleep) до первого прерывания.
 *   2. HAL_GetTick() после старта планировщика считает по тикам FreeRTOS
 *      (они корректно компенсируются после tickless-сна), а TIM1 останавливается.
 *   3. Задачи не опрашивают по 1 мс, а ждут событие (очередь / уведомление).
 *
 *  Режим Sleep, не Stop: в Stop останавливаются TIM3/TIM5 (сценарий измерения),
 *  SysTick и тактирование USB; см. заметки в power_mgmt.cpp.
 */
#ifndef POWER_MGMT_H_
#define POWER_MGMT_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Вызвать один раз из задачи после старта планировщика, ДО первой паузы/сна. */
void Power_HalTickToRtos(void);

#ifdef __cplusplus
}
#endif

#endif /* POWER_MGMT_H_ */
