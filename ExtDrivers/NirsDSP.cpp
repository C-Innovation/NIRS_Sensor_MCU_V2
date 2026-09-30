/*
 * NirsDSP.cpp
 *
 *  Created on: 25 июл. 2026 г.
 *      Author: xwest
 */

#include <NirsDSP.h>

float32_t PrepareData(uint16_t mes, uint16_t bgd)
{
	return  ((float32_t)(mes - bgd) >= 0) ? (float32_t)(mes - bgd) : 0.0f;
}

NirsDSP::NirsDSP() {
	// TODO Auto-generated constructor stub

	DSP_Filters_5thOrder_Init();

}

NirsDSP::~NirsDSP() {
	// TODO Auto-generated destructor stub
}

NirsFilteredData_t NirsDSP::ProcessLP(MesurementData_t data)
{
	NirsFilteredData_t out_data;
	float32_t tOutVals[8];
//	float32_t tInVals[8]=
//	{
//			 PrepareData(data.Led740_1, data.Led740_Bgd_1),
//			 PrepareData(data.Led740_2, data.Led740_Bgd_2),
//			 PrepareData(data.Led740_3, data.Led740_Bgd_3),
//			 PrepareData(data.Led740_4, data.Led740_Bgd_4),
//
//			 PrepareData(data.Led850_1, data.Led850_Bgd_1),
//			 PrepareData(data.Led850_2, data.Led850_Bgd_2),
//			 PrepareData(data.Led850_3, data.Led850_Bgd_3),
//			 PrepareData(data.Led850_4, data.Led850_Bgd_4)
//	};

	float32_t tInVals[8]=
	{
			 PrepareData(data.Led740_1, 0),
			 PrepareData(data.Led740_2, 0),
			 PrepareData(data.Led740_3, 0),
			 PrepareData(data.Led740_4, 0),

			 PrepareData(data.Led850_1, 0),
			 PrepareData(data.Led850_2, 0),
			 PrepareData(data.Led850_3, 0),
			 PrepareData(data.Led850_4, 0)
	};
	for(int i = 0; i < 8; i++)
		DSP_Process_Signal_5thOrder(&filter_5th[i], &tInVals[i], &tOutVals[i], 1);

	out_data.Led740_1 = (uint16_t)tOutVals[0];
	out_data.Led740_2 = (uint16_t)tOutVals[1];
	out_data.Led740_3 = (uint16_t)tOutVals[2];
	out_data.Led740_4 = (uint16_t)tOutVals[3];

	out_data.Led850_1 = (uint16_t)tOutVals[4];
	out_data.Led850_2 = (uint16_t)tOutVals[5];
	out_data.Led850_3 = (uint16_t)tOutVals[6];
	out_data.Led850_4 = (uint16_t)tOutVals[7];
	out_data.Time = data.Time;
	return out_data;
}

// =====================================================================
// 4. Функция инициализации
// =====================================================================
void NirsDSP::DSP_Filters_5thOrder_Init(void) {
    // numStages = 3 для всех фильтров
	for(int i=0; i<8; i++){
//			main_coeffs[i] = coeffs_25Hz_5th;
			memset(&state_5th[i], 0, 12 * sizeof(float32_t));
			arm_biquad_cascade_df2T_init_f32(&filter_5th[i], 2, coeffs_15Hz_4th, state_5th[i]);

		}
}


// =====================================================================
// 5. Пример использования
// =====================================================================
void NirsDSP::DSP_Process_Signal_5thOrder(const arm_biquad_cascade_df2T_instance_f32 * S, float32_t *input, float32_t *output, uint32_t blockSize) {
	arm_biquad_cascade_df2T_f32(S, input, output, blockSize);

}
