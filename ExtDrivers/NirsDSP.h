/*
 * NirsDSP.h
 *
 *  Created on: 25 июл. 2026 г.
 *      Author: xwest
 *
 *  Цифровой ФНЧ по 8 каналам (2 длины волны x 4 фотодиода).
 *  Вызывается по одному разу на каждый сценарий измерения, то есть с
 *  частотой NIRS_SAMPLE_RATE_HZ (100 Гц). Коэффициенты в NirsDSP.cpp
 *  посчитаны под эту частоту: при её изменении пересчитайте таблицу
 *  (tools/gen_lpf_coeffs.py) - static_assert в NirsDSP.cpp напомнит об этом.
 */

#ifndef NIRSDSP_H_
#define NIRSDSP_H_

#include "data_structs.h"
#include "arm_math.h"

#define NIRS_DSP_CH      8
#define NIRS_DSP_STAGES  2      /* Баттерворт 4-го порядка = 2 биквадратные секции */

/* Частота среза ФНЧ по умолчанию, Гц. Допустимо: 5, 8, 10, 12, 15, 20.
 *
 * Почему 12 Гц, а не прежние 15 Гц (выбирались при Fs = 1 кГц):
 *  - при Fs = 100 Гц ФНЧ 15 Гц оставляет полосу 15..50 Гц почти нетронутой:
 *    на 20 Гц он ослабляет лишь на 12.6 дБ (12 Гц - на 21 дБ, 10 Гц - на 28 дБ);
 *  - на 20 Гц при этом лежит алиас 120 Гц (мерцание ламп при сети 60 Гц),
 *    а на 0 Гц - алиас 100 Гц (сеть 50 Гц): вторым фильтр уже не поможет;
 *  - прежняя цепочка (ФНЧ 15 Гц @1 кГц + усреднение 10 отсчётов перед сетью)
 *    давала суммарную задержку ~33 мс. Butterworth-4 12 Гц @100 Гц даёт те же
 *    ~33 мс, так что динамика для ядра и сети остаётся прежней.
 */
#ifndef NIRS_LPF_DEFAULT_HZ
#define NIRS_LPF_DEFAULT_HZ 12
#endif

class NirsDSP {
public:
	NirsDSP();
	virtual ~NirsDSP();
	NirsFilteredData_t ProcessLP(MesurementData_t data);

	/* Сменить частоту среза (только значения из таблицы). Состояние фильтров
	   сбрасывается. Вызывать из того же потока, что и ProcessLP (DspTask). */
	bool SetCutoff(uint32_t hz);

private:
	arm_biquad_cascade_df2T_instance_f32 filter[NIRS_DSP_CH];
	float32_t state[NIRS_DSP_CH][2 * NIRS_DSP_STAGES];   /* DF2T: 2 слова на секцию */
};

#endif /* NIRSDSP_H_ */
