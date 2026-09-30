/*
 * NirsDSP.cpp
 *
 *  Created on: 25 июл. 2026 г.
 *      Author: xwest
 */

#include <NirsDSP.h>

static_assert(NIRS_SAMPLE_RATE_HZ == 100,
              "Коэффициенты ФНЧ посчитаны для Fs = 100 Гц: пересчитайте kLpfTable "
              "(tools/gen_lpf_coeffs.py)");

// Формат CMSIS-DSP arm_biquad_cascade_df2T_f32: на секцию {b0, b1, b2, -a1, -a2}
// (знаки a инвертированы), усиление секции на постоянном токе равно 1.
// Сгенерировано: python3 tools/gen_lpf_coeffs.py 100 4 5 8 10 12 15 20
struct LpfEntry {
	uint32_t hz;
	float32_t coeffs[5 * NIRS_DSP_STAGES];
};

static const LpfEntry kLpfTable[] = {
	{ 5, {  // задержка на низких частотах ~82 мс
		0.0190368313f, 0.0380736627f, 0.0190368313f, 1.47967422f, -0.555821538f,
		0.0218838528f, 0.0437677056f, 0.0218838528f, 1.70096433f, -0.788499713f } },
	{ 8, {  // ~51 мс
		0.0427980162f, 0.0855960324f, 0.0427980162f, 1.21281207f, -0.384004176f,
		0.0522195138f, 0.104439028f, 0.0522195138f, 1.47979891f, -0.688676953f } },
	{ 10, { // ~40 мс
		0.0618851967f, 0.123770393f, 0.0618851967f, 1.0485996f, -0.296140343f,
		0.0779563412f, 0.155912682f, 0.0779563412f, 1.32091343f, -0.632738769f } },
	{ 12, { // ~33 мс
		0.0830142424f, 0.166028485f, 0.0830142424f, 0.893103659f, -0.225160584f,
		0.107384674f, 0.214769349f, 0.107384674f, 1.15529156f, -0.584830225f } },
	{ 15, { // ~26 мс
		0.117948569f, 0.235897139f, 0.117948569f, 0.672740936f, -0.144535199f,
		0.157382235f, 0.31476447f, 0.157382235f, 0.897657931f, -0.52718693f } },
	{ 20, { // ~18 мс
		0.183902994f, 0.367805988f, 0.183902994f, 0.328975677f, -0.0645876527f,
		0.253301501f, 0.506603003f, 0.253301501f, 0.453119516f, -0.466325581f } },
};

float32_t PrepareData(uint16_t mes, uint16_t bgd)
{
	return  ((float32_t)(mes - bgd) >= 0) ? (float32_t)(mes - bgd) : 0.0f;
}

/* float -> uint16 с насыщением: БИХ-фильтр может дать выброс ниже нуля или выше
   полной шкалы, а приведение вне диапазона типа - неопределённое поведение. */
static uint16_t ToU16(float32_t v)
{
	if (!(v > 0.0f))       return 0;      /* включая NaN */
	if (v >= 65535.0f)     return 65535;
	return (uint16_t)v;
}

NirsDSP::NirsDSP() {
	SetCutoff(NIRS_LPF_DEFAULT_HZ);
}

NirsDSP::~NirsDSP() {
}

bool NirsDSP::SetCutoff(uint32_t hz)
{
	for (const LpfEntry &e : kLpfTable) {
		if (e.hz != hz)
			continue;
		for (int i = 0; i < NIRS_DSP_CH; i++)
			/* init обнуляет state; коэффициенты лежат во flash и живут вечно */
			arm_biquad_cascade_df2T_init_f32(&filter[i], NIRS_DSP_STAGES, e.coeffs, state[i]);
		return true;
	}
	return false;
}

NirsFilteredData_t NirsDSP::ProcessLP(MesurementData_t data)
{
	NirsFilteredData_t out_data;
	float32_t tOutVals[NIRS_DSP_CH];
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

	float32_t tInVals[NIRS_DSP_CH]=
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
	for(int i = 0; i < NIRS_DSP_CH; i++)
		arm_biquad_cascade_df2T_f32(&filter[i], &tInVals[i], &tOutVals[i], 1);

	out_data.Led740_1 = ToU16(tOutVals[0]);
	out_data.Led740_2 = ToU16(tOutVals[1]);
	out_data.Led740_3 = ToU16(tOutVals[2]);
	out_data.Led740_4 = ToU16(tOutVals[3]);

	out_data.Led850_1 = ToU16(tOutVals[4]);
	out_data.Led850_2 = ToU16(tOutVals[5]);
	out_data.Led850_3 = ToU16(tOutVals[6]);
	out_data.Led850_4 = ToU16(tOutVals[7]);
	out_data.Time = data.Time;
	return out_data;
}
