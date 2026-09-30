/*
 * data_structs.c
 *
 *  Created on: 24 июл. 2026 г.
 *      Author: xwest
 */

#include "data_structs.h"

UsbDataSerialazer::UsbDataSerialazer()
{
	_Serializer = new Serializer(NIRS_HEADER);
}
UsbDataSerialazer::~UsbDataSerialazer()
{

}

UsbPacket_t UsbDataSerialazer::MesurementData2UsbPacket(MesurementData_t data)
{

	UsbPacket_t outPack;
	outPack.Len = 41 + 8;
	uint8_t pDat[41] = {0};
	pDat[0] = NIRS_PACKET_DATA_RAW;
	memcpy(&pDat[1], &data, 40);
	_Serializer->serialize(&pDat[0], 41, &outPack.Data[0], outPack.Len);
	outPack.IsInput = 0;
	return outPack;
}

UsbPacket_t UsbDataSerialazer::NirsFilteredData2UsbPacket(NirsFilteredData_t data)
{

	UsbPacket_t outPack;
	outPack.Len = 25 + 8;
	uint8_t pDat[25] = {0};
	pDat[0] = NIRS_PACKET_DATA_FILTERED;
	memcpy(&pDat[1], &data, 24);
	_Serializer->serialize(&pDat[0], 25, &outPack.Data[0], outPack.Len);
	outPack.IsInput = 0;
	return outPack;
}


UsbPacket_t UsbDataSerialazer::GetInputPacket(uint8_t * data, uint32_t len)
{
	UsbPacket_t outPack;
	outPack.IsInput = 1;
	outPack.Len = (uint8_t)len;
	memcpy(&outPack.Data[0], data, len);

	return outPack;
}
