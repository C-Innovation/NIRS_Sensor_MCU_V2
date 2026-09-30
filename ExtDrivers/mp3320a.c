#include "mp3320a.h"

#define MP3320A_I2C_TIMEOUT     100 /* ms */

HAL_StatusTypeDef MP3320A_Init(MP3320A_HandleTypeDef *hmp, I2C_HandleTypeDef *hi2c, uint8_t i2c_addr_7bit) {
    if (hmp == NULL || hi2c == NULL) return HAL_ERROR;
    if (i2c_addr_7bit > 0x7F) return HAL_ERROR;

    hmp->hi2c = hi2c;
    /* HAL требует 8-битный адрес (7 бит сдвинуты влево на 1) */
    hmp->dev_addr = (uint16_t)(i2c_addr_7bit << 1);
    hmp->chip_id  = 0;

    /* Проверка связи путем чтения ID регистра.
       Значение сохраняем - при желании сверьте его с даташитом. */
    if (MP3320A_ReadReg(hmp, MP3320A_REG_ID, &hmp->chip_id) != HAL_OK) {
        return HAL_ERROR;
    }
    return HAL_OK;
}

HAL_StatusTypeDef MP3320A_ReadReg(MP3320A_HandleTypeDef *hmp, uint8_t reg, uint8_t *data) {
    return HAL_I2C_Mem_Read(hmp->hi2c, hmp->dev_addr, reg, I2C_MEMADD_SIZE_8BIT, data, 1, MP3320A_I2C_TIMEOUT);
}

HAL_StatusTypeDef MP3320A_WriteReg(MP3320A_HandleTypeDef *hmp, uint8_t reg, uint8_t data) {
    return HAL_I2C_Mem_Write(hmp->hi2c, hmp->dev_addr, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, MP3320A_I2C_TIMEOUT);
}

/* ---------------------------------------------------------------------------
 *  ISR-safe запись регистра (без HAL, без HAL_GetTick, без блокировок RTOS)
 *
 *  Причина существования: HAL_I2C_Mem_Write() отсчитывает таймаут по
 *  HAL_GetTick(). Под FreeRTOS SysTick имеет самый низкий приоритет, поэтому
 *  из обработчика с приоритетом 5 тик НИКОГДА не увеличится -> при зависании
 *  шины функция уходит в бесконечный цикл прямо в прерывании.
 * ------------------------------------------------------------------------- */
HAL_StatusTypeDef MP3320A_WriteReg_ISR(MP3320A_HandleTypeDef *hmp, uint8_t reg, uint8_t data) {
    if (hmp == NULL || hmp->hi2c == NULL) return HAL_ERROR;

    I2C_TypeDef *i2c = hmp->hi2c->Instance;
    const uint8_t tx[2] = { reg, data };
    uint32_t guard;

    /* Шина занята другим ведущим/незавершённой транзакцией */
    if (i2c->ISR & I2C_ISR_BUSY) {
        guard = MP3320A_ISR_SPIN;
        while ((i2c->ISR & I2C_ISR_BUSY) && --guard) { }
        if (guard == 0U) return HAL_BUSY;
    }

    i2c->ICR = I2C_ICR_NACKCF | I2C_ICR_STOPCF | I2C_ICR_BERRCF |
               I2C_ICR_ARLOCF | I2C_ICR_OVRCF;

    /* 7-битный адрес, запись, 2 байта, автоматический STOP */
    i2c->CR2 = ((uint32_t)hmp->dev_addr & I2C_CR2_SADD) |
               (2UL << I2C_CR2_NBYTES_Pos) |
               I2C_CR2_AUTOEND | I2C_CR2_START;

    for (int i = 0; i < 2; i++) {
        guard = MP3320A_ISR_SPIN;
        while (!(i2c->ISR & I2C_ISR_TXIS)) {
            if (i2c->ISR & I2C_ISR_NACKF) {
                i2c->ICR = I2C_ICR_NACKCF | I2C_ICR_STOPCF;
                i2c->CR2 = 0;
                return HAL_ERROR;
            }
            if (--guard == 0U) { i2c->CR2 = 0; return HAL_TIMEOUT; }
        }
        i2c->TXDR = tx[i];
    }

    guard = MP3320A_ISR_SPIN;
    while (!(i2c->ISR & I2C_ISR_STOPF)) {
        if (i2c->ISR & I2C_ISR_NACKF) {
            i2c->ICR = I2C_ICR_NACKCF | I2C_ICR_STOPCF;
            i2c->CR2 = 0;
            return HAL_ERROR;
        }
        if (--guard == 0U) { i2c->CR2 = 0; return HAL_TIMEOUT; }
    }

    i2c->ICR = I2C_ICR_STOPCF;
    i2c->CR2 = 0;
    return HAL_OK;
}

/* Чтение-модификация-запись для изменения отдельных битов */
HAL_StatusTypeDef MP3320A_UpdateReg(MP3320A_HandleTypeDef *hmp, uint8_t reg, uint8_t mask, uint8_t value) {
    uint8_t data;
    HAL_StatusTypeDef ret = MP3320A_ReadReg(hmp, reg, &data);
    if (ret != HAL_OK) return ret;
    data = (uint8_t)((data & (uint8_t)~mask) | (value & mask));
    return MP3320A_WriteReg(hmp, reg, data);
}

HAL_StatusTypeDef MP3320A_Enable(MP3320A_HandleTypeDef *hmp, FunctionalState state) {
    HAL_StatusTypeDef ret;
    if (state == ENABLE) {
        ret = MP3320A_UpdateReg(hmp, MP3320A_REG_MODE, MP3320A_MODE_EN, MP3320A_MODE_EN);
    } else {
        ret = MP3320A_UpdateReg(hmp, MP3320A_REG_MODE, MP3320A_MODE_EN, 0x00);
        /* ВАЖНО: Даташит (Rev 1.1) требует задержку >1.5мс после выключения EN.
           HAL_Delay() блокирующий - вызывать только из контекста задачи/main. */
        HAL_Delay(2);
    }
    return ret;
}

HAL_StatusTypeDef MP3320A_EnableChargePump(MP3320A_HandleTypeDef *hmp, FunctionalState state) {
    uint8_t val = (state == ENABLE) ? MP3320A_MODE_EN_CP : 0x00;
    return MP3320A_UpdateReg(hmp, MP3320A_REG_MODE, MP3320A_MODE_EN_CP, val);
}

HAL_StatusTypeDef MP3320A_EnableChannel(MP3320A_HandleTypeDef *hmp, uint8_t channel, FunctionalState state) {
    if (channel < 1 || channel > 4) return HAL_ERROR;
    /* CH1EN - бит 0, CH4EN - бит 3 в регистре 0x03 */
    uint8_t mask = (uint8_t)(1U << (channel - 1));
    uint8_t val = (state == ENABLE) ? mask : 0x00;
    return MP3320A_UpdateReg(hmp, MP3320A_REG_CH_SET, mask, val);
}

HAL_StatusTypeDef MP3320A_SetCurrent(MP3320A_HandleTypeDef *hmp, uint8_t channel, uint8_t current_code) {
    if (channel < 1 || channel > 4) return HAL_ERROR;
    uint8_t reg = (uint8_t)(MP3320A_REG_ICH1 + (channel - 1));
    return MP3320A_WriteReg(hmp, reg, current_code);
}

HAL_StatusTypeDef MP3320A_SetPWMDuty(MP3320A_HandleTypeDef *hmp, uint8_t channel, uint16_t duty) {
    if (channel < 1 || channel > 4) return HAL_ERROR;
    if (duty > MP3320A_PWM_FULL_SCALE) duty = MP3320A_PWM_FULL_SCALE;

    uint8_t reg_l = (uint8_t)(MP3320A_REG_PWM1_L + (channel - 1) * 2);
    uint8_t reg_h = (uint8_t)(reg_l + 1);

    uint8_t low_bits  = (uint8_t)(duty & 0x07);
    uint8_t high_bits = (uint8_t)((duty >> 3) & 0xFF);

    HAL_StatusTypeDef ret;
    /* Даташит: младшие биты PWM[2:0] защёлкиваются только ПОСЛЕ записи
       старших PWM[10:3]. Значит порядок: сначала LOW (в теневой регистр),
       затем HIGH (применяет всё значение целиком).
       ИСХОДНЫЙ КОД ПИСАЛ HIGH -> LOW, т.е. ровно наоборот, и новые младшие
       биты вступали в силу только при следующей записи HIGH. */
    ret = MP3320A_WriteReg(hmp, reg_l, low_bits);
    if (ret != HAL_OK) return ret;

    return MP3320A_WriteReg(hmp, reg_h, high_bits);
}

HAL_StatusTypeDef MP3320A_ReadFaultStatus(MP3320A_HandleTypeDef *hmp, uint8_t *fault_status) {
    return MP3320A_ReadReg(hmp, MP3320A_REG_FAULT, fault_status);
}

/**
  * @brief  Настройка MP3320A в режим чистого источника тока (без CP)
  *         и установка аналогового тока каждого канала.
  *         Каналы остаются ВЫКЛЮЧЕННЫМИ, чтобы не было вспышки при старте.
  * @param  current_mA_ch1...4: целевой ток (мА, диапазон 0.0 ~ 51.0)
  */
HAL_StatusTypeDef MP3320A_Config_Analog_PureCurrent(MP3320A_HandleTypeDef *hmp,
                                                    float current_mA_ch1,
                                                    float current_mA_ch2,
                                                    float current_mA_ch3,
                                                    float current_mA_ch4) {
    HAL_StatusTypeDef ret;
    const float in[4] = { current_mA_ch1, current_mA_ch2, current_mA_ch3, current_mA_ch4 };
    uint8_t code[4];

    /* I = code * 0.2 мА  =>  code = I / 0.2, с ограничением диапазона */
    for (int i = 0; i < 4; i++) {
        float v = in[i];
        if (v < 0.0f) v = 0.0f;
        uint32_t c = (uint32_t)(v / 0.2f + 0.5f);
        if (c > MP3320A_CURRENT_CODE_MAX) c = MP3320A_CURRENT_CODE_MAX;
        code[i] = (uint8_t)c;
    }

    /* 1. Убеждаемся, что чип выключен на время конфигурации.
          Даташит (Rev 1.1, REG01h): после перехода EN 1 -> 0 нужна пауза >1.5 мс
          до ЛЮБОЙ следующей операции чтения/записи по I2C. Раньше EN сбрасывался
          и сразу же шла следующая транзакция. При холодном старте EN уже 0 и
          перехода нет, но при перезапуске МК без снятия питания с MP3320A
          (EN=1 остался с прошлого запуска) транзакции попадали в это окно и
          могли быть проигнорированы. Поэтому сбрасываем EN только если он был
          установлен, и через MP3320A_Enable(), которая выдерживает паузу. */
    uint8_t mode;
    ret = MP3320A_ReadReg(hmp, MP3320A_REG_MODE, &mode);
    if (ret != HAL_OK) return ret;
    if (mode & MP3320A_MODE_EN) {
        ret = MP3320A_Enable(hmp, DISABLE);
        if (ret != HAL_OK) return ret;
    }

    /* 2. Отключаем Charge Pump (EN_CP = 0) */
    ret = MP3320A_UpdateReg(hmp, MP3320A_REG_MODE, MP3320A_MODE_EN_CP, 0x00);
    if (ret != HAL_OK) return ret;

    /* 3. CLEDx = 1 (чистый источник тока), CHxEN = 0 (все каналы пока выключены).
          Было 0xFF -> все 4 канала включались током по умолчанию. */
    ret = MP3320A_WriteReg(hmp, MP3320A_REG_CH_SET, MP3320A_CH_SET_ALL_CLED);
    if (ret != HAL_OK) return ret;

    /* 4. Токи каналов.
          Регистры PWM по умолчанию = 0x400 (100%) - трогать не нужно. */
    for (int i = 0; i < 4; i++) {
        ret = MP3320A_WriteReg(hmp, (uint8_t)(MP3320A_REG_ICH1 + i), code[i]);
        if (ret != HAL_OK) return ret;
    }

    /* 5. Включаем микросхему (EN = 1). Свет не загорится, пока не будут
          выставлены биты CHxEN. */
    return MP3320A_UpdateReg(hmp, MP3320A_REG_MODE, MP3320A_MODE_EN, MP3320A_MODE_EN);
}
