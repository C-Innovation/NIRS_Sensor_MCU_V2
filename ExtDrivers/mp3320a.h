#ifndef __MP3320A_H
#define __MP3320A_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32u5xx_hal.h"

/* ================= I2C Address ================= */
/* 7-bit адрес зависит от резистора на пину ADR (0x60 ... 0x6F).
   Если ADR подключен к GND (0 Ом), адрес = 0x60. */
#define MP3320A_I2C_ADDR_DEFAULT    0x60

/* Максимальный код тока: I = code * 0.2 мА  =>  255 * 0.2 = 51 мА */
#define MP3320A_CURRENT_CODE_MAX    255U

/* Полная шкала ШИМ: 11-битное значение 0x400 = 1024 = 100% */
#define MP3320A_PWM_FULL_SCALE      1024U

/* Спин-таймаут для ISR-safe записи (в итерациях цикла, не в мс) */
#define MP3320A_ISR_SPIN            200000UL

/* ================= Register Map ================= */
#define MP3320A_REG_ID              0x00
#define MP3320A_REG_MODE            0x01
#define MP3320A_REG_CP_MODE         0x02
#define MP3320A_REG_CH_SET          0x03
#define MP3320A_REG_STEP            0x04
#define MP3320A_REG_FPWM            0x05
#define MP3320A_REG_FBLK            0x06
#define MP3320A_REG_DUTYBLK         0x07
#define MP3320A_REG_BLK_TIMES       0x08
#define MP3320A_REG_PROT_PS         0x09
#define MP3320A_REG_ICH1            0x0A
#define MP3320A_REG_ICH2            0x0B
#define MP3320A_REG_ICH3            0x0C
#define MP3320A_REG_ICH4            0x0D
#define MP3320A_REG_PWM1_L          0x0E
#define MP3320A_REG_PWM1_H          0x0F
#define MP3320A_REG_PWM2_L          0x10
#define MP3320A_REG_PWM2_H          0x11
#define MP3320A_REG_PWM3_L          0x12
#define MP3320A_REG_PWM3_H          0x13
#define MP3320A_REG_PWM4_L          0x14
#define MP3320A_REG_PWM4_H          0x15
#define MP3320A_REG_FAULT           0x16

/* ================= Bit Definitions ================= */
/* REG01h: MODE */
#define MP3320A_MODE_DMBLK          (1 << 4)
#define MP3320A_MODE_EN             (1 << 3)
#define MP3320A_MODE_CH4MD          (1 << 2)
#define MP3320A_MODE_BLK_PWM        (1 << 1)
#define MP3320A_MODE_EN_CP          (1 << 0)

/* REG03h: CH_SET  [CLED4][CLED3][CLED2][CLED1][CH4EN][CH3EN][CH2EN][CH1EN] */
#define MP3320A_CH_SET_CH4EN        (1 << 3)
#define MP3320A_CH_SET_CH3EN        (1 << 2)
#define MP3320A_CH_SET_CH2EN        (1 << 1)
#define MP3320A_CH_SET_CH1EN        (1 << 0)
#define MP3320A_CH_SET_CLED1        (1 << 4)
#define MP3320A_CH_SET_CLED2        (1 << 5)
#define MP3320A_CH_SET_CLED3        (1 << 6)
#define MP3320A_CH_SET_CLED4        (1 << 7)
#define MP3320A_CH_SET_ALL_EN       (0x0F)
#define MP3320A_CH_SET_ALL_CLED     (0xF0)

/* Fault bits in REG16h */
#define MP3320A_FAULT_CH4S          (1 << 7)
#define MP3320A_FAULT_CH3S          (1 << 6)
#define MP3320A_FAULT_CH2S          (1 << 5)
#define MP3320A_FAULT_CH1S          (1 << 4)
#define MP3320A_FAULT_CH4O          (1 << 3)
#define MP3320A_FAULT_CH3O          (1 << 2)
#define MP3320A_FAULT_CH2O          (1 << 1)
#define MP3320A_FAULT_CH1O          (1 << 0)

/* ================= Structures ================= */
typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint16_t dev_addr; /* 8-bit I2C address (7-bit shifted left) */
    uint8_t  chip_id;  /* содержимое REG00h, прочитанное при инициализации */
} MP3320A_HandleTypeDef;

/* ================= API Functions ================= */
HAL_StatusTypeDef MP3320A_Init(MP3320A_HandleTypeDef *hmp, I2C_HandleTypeDef *hi2c, uint8_t i2c_addr_7bit);
HAL_StatusTypeDef MP3320A_ReadReg(MP3320A_HandleTypeDef *hmp, uint8_t reg, uint8_t *data);
HAL_StatusTypeDef MP3320A_WriteReg(MP3320A_HandleTypeDef *hmp, uint8_t reg, uint8_t data);
HAL_StatusTypeDef MP3320A_UpdateReg(MP3320A_HandleTypeDef *hmp, uint8_t reg, uint8_t mask, uint8_t value);

/* Запись одного регистра напрямую через регистры I2C, без HAL_GetTick().
   Единственный вариант, пригодный для вызова из обработчика прерывания:
   HAL_I2C_* внутри ISR под FreeRTOS может зависнуть навсегда, т.к. SysTick
   имеет более низкий приоритет и тик не наращивается. */
HAL_StatusTypeDef MP3320A_WriteReg_ISR(MP3320A_HandleTypeDef *hmp, uint8_t reg, uint8_t data);

HAL_StatusTypeDef MP3320A_Enable(MP3320A_HandleTypeDef *hmp, FunctionalState state);
HAL_StatusTypeDef MP3320A_EnableChargePump(MP3320A_HandleTypeDef *hmp, FunctionalState state);
HAL_StatusTypeDef MP3320A_EnableChannel(MP3320A_HandleTypeDef *hmp, uint8_t channel, FunctionalState state);

/* current_code: 0~255 (Ток = code * 0.2 мА, макс 51 мА) */
HAL_StatusTypeDef MP3320A_SetCurrent(MP3320A_HandleTypeDef *hmp, uint8_t channel, uint8_t current_code);
/* duty: 0~1024, 11-битное поле PWM[10:0] (0x400 = 100%) */
HAL_StatusTypeDef MP3320A_SetPWMDuty(MP3320A_HandleTypeDef *hmp, uint8_t channel, uint16_t duty);

HAL_StatusTypeDef MP3320A_ReadFaultStatus(MP3320A_HandleTypeDef *hmp, uint8_t *fault_status);

/* Настройка чисто аналогового режима (CLEDx = 1, без Charge Pump).
   Каналы остаются ВЫКЛЮЧЕННЫМИ - включает их вызывающий код. */
HAL_StatusTypeDef MP3320A_Config_Analog_PureCurrent(MP3320A_HandleTypeDef *hmp,
                                                    float current_mA_ch1,
                                                    float current_mA_ch2,
                                                    float current_mA_ch3,
                                                    float current_mA_ch4);

#ifdef __cplusplus
}
#endif

#endif /* __MP3320A_H */
