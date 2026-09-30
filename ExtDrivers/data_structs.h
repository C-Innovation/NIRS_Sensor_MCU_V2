/*
 * data_structs.h
 *
 *  Created on: 24 июл. 2026 г.
 *      Author: xwest
 */

#ifndef DATA_STRUCTS_H_
#define DATA_STRUCTS_H_


#include <stdint.h>
#include <string.h>
#include "protocol_base.h"

#define NIRS_HEADER (uint32_t)0x234E5253

/* Частота следования сценариев измерения (кадров 740/фон/850/фон), Гц.
 * От неё зависят: таймер сценария (analog_driver.h), цифровые фильтры
 * (NirsDSP) и постоянные времени ядра/нейросети (app_freertos.cpp).
 * Было 1000 Гц; 100 Гц = 1 мс сценарий + 9 мс покоя. */
#define NIRS_SAMPLE_RATE_HZ 100

/* Тактовая частота ядра (SYSCLK = HCLK = APB1 = APB2 = APB3), Гц. Должна совпадать
 * с настройкой PLL в SystemClock_Config() (main.cpp) - там стоит static_assert.
 * От неё зависят: таймеры сценария TIM3/TIM5 (analog_driver.h), TIM2 (микро-
 * секундная шкала времени, tim.c), тайминг I2C2 (i2c.c) и тактовая АЦП.
 * Было 160 МГц (PLLN=80), теперь 80 МГц (PLLN=40). */
#define NIRS_SYSCLK_HZ 80000000UL

#define NIRS_PACKET_STATUS					(uint8_t)0
#define NIRS_PACKET_DATA_RAW				(uint8_t)1
#define NIRS_PACKET_DATA_FILTERED		(uint8_t)2
#define NIRS_PACKET_DATA_AI					(uint8_t)3
#define NIRS_PACKET_SET_PARAM				(uint8_t)4
#define NIRS_PACKET_GET_PARAM				(uint8_t)5

typedef enum
{
	NIRS_ParamUsbOutEn = 0,
	NIRS_ParamAutoBright,
	NIRS_ParamBrightLevel,
	NIRS_ParamActiveAiModel,
	NIRS_ParamSaturationCalcEnable,
	NIRS_ParamActiveLPFilter,
}ParamCode_t;

typedef struct
{
  uint64_t Time;

  uint16_t Led740_1;
  uint16_t Led740_2;
  uint16_t Led740_3;
  uint16_t Led740_4;

  uint16_t Led740_Bgd_1;
  uint16_t Led740_Bgd_2;
  uint16_t Led740_Bgd_3;
  uint16_t Led740_Bgd_4;

  uint16_t Led850_1;
  uint16_t Led850_2;
  uint16_t Led850_3;
  uint16_t Led850_4;

  uint16_t Led850_Bgd_1;
  uint16_t Led850_Bgd_2;
  uint16_t Led850_Bgd_3;
  uint16_t Led850_Bgd_4;


}MesurementData_t;

typedef struct
{
  uint64_t Time;

  uint16_t Led740_1;
  uint16_t Led740_2;
  uint16_t Led740_3;
  uint16_t Led740_4;

  uint16_t Led850_1;
  uint16_t Led850_2;
  uint16_t Led850_3;
  uint16_t Led850_4;

}NirsFilteredData_t;

typedef struct
{
	uint8_t IsInput;
	uint8_t Len;
	uint8_t Data[64];
}UsbPacket_t;


typedef struct
{
	uint8_t ID;
	MesurementData_t Data;
}RawDataPacket_t;


class UsbDataSerialazer
{
public:
	UsbDataSerialazer();
	~UsbDataSerialazer();
	UsbPacket_t MesurementData2UsbPacket(MesurementData_t data);
	UsbPacket_t NirsFilteredData2UsbPacket(NirsFilteredData_t data);
	UsbPacket_t GetInputPacket(uint8_t * data, uint32_t len);

private:
	Serializer* _Serializer;
};


#endif /* DATA_STRUCTS_H_ */
