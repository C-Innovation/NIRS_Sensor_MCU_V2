/* ============================================================================
 *  nirs_pipeline.h — единый API развёртывания на микроконтроллере.
 *
 *  Собирает всю цепочку в один вызов:
 *
 *      8 напряжений АЦП  ──►  nirs_pipe_update()  ──►  один сигнал 0..1
 *
 *  Внутри, в зависимости от режима:
 *
 *    NIRS_PIPE_ALGO   детерминированное ядро nirs_contraction. Работает
 *                     сразу, без обучения и без модели во flash.
 *    NIRS_PIPE_NN     предфильтр + нейросеть (TFLite Micro). Нужна
 *                     обученная модель.
 *    NIRS_PIPE_BOTH   считаются оба, наружу отдаётся выбранный, а
 *                     расхождение доступно для диагностики. Рекомендуется
 *                     на этапе внедрения: если расхождение растёт, сеть
 *                     попала в условия, которых не было в обучении.
 *
 *  Выравниватель уровней (nirs_leveler) в цепочку НЕ входит намеренно.
 *  Измерено: перед предфильтром он ухудшает результат в 17 раз, потому что
 *  его переменный коэффициент предфильтр принимает за полезный сигнал.
 *  Подробности — в LEVELER.md §7. Он нужен только там, где модель смотрит
 *  на сырые вольты без предфильтра.
 *
 *  C99, без malloc. Совместим с C++.
 * ==========================================================================*/
#ifndef NIRS_PIPELINE_H
#define NIRS_PIPELINE_H

#include <stdint.h>
#include "nirs_contraction.h"
#include "nn/nirs_nn_runtime.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    NIRS_PIPE_ALGO = 0,
    NIRS_PIPE_NN   = 1,
    NIRS_PIPE_BOTH = 2
} nirs_pipe_mode_t;

/* Флаги (nirs_pipe_flags) */
#define NIRS_PIPE_F_READY   0x0001u  /* выход достоверен                     */
#define NIRS_PIPE_F_ACTIVE  0x0002u  /* идёт сокращение                      */
#define NIRS_PIPE_F_NOSIG   0x0004u  /* нет пригодных каналов                */
#define NIRS_PIPE_F_SAT     0x0008u  /* канал в насыщении                    */
#define NIRS_PIPE_F_DIVERGE 0x0010u  /* сеть и алгоритм сильно разошлись     */
#define NIRS_PIPE_F_STABLE  0x0020u  /* устоялся и масштаб выхода            */
#define NIRS_PIPE_F_TRACKING 0x0040u /* датчик на мышце, форма верна         */

typedef struct {
    float fs;                 /* частота вызова, Гц                  (1000) */
    nirs_pipe_mode_t mode;    /*                          (NIRS_PIPE_ALGO)  */
    float out_vmax;           /* полная шкала выхода в вольтах        (3.3) */
    float diverge_thr;        /* порог флага DIVERGE, доля шкалы     (0.15) */
    float tau_diverge;        /* с, усреднение расхождения            (1.0) */
} nirs_pipe_cfg_t;

typedef struct {
    nirs_pipe_cfg_t cfg;
    nirs_t     algo;          /* детерминированное ядро */
    nirs_nn_t  nn;            /* предфильтр + сеть      */
    float      a_div;
    float      diverge;       /* сглаженное |сеть - алгоритм| */
    float      y_algo, y_nn, y;
    uint16_t   flags;
    uint32_t   n;
} nirs_pipe_t;

/* -------------------------------------------------------------------------
 *  API — три вызова, больше ничего не нужно
 * ---------------------------------------------------------------------- */

/* Значения по умолчанию. */
void  nirs_pipe_defaults(nirs_pipe_cfg_t *cfg, float fs, nirs_pipe_mode_t mode);

/* Инициализация. В режимах NN и BOTH ДО этого вызова должен быть поднят
 * интерпретатор TFLite Micro (см. nn/example_tflm.cc, nirs_nn_setup). */
void  nirs_pipe_init(nirs_pipe_t *s, const nirs_pipe_cfg_t *cfg);

/* ОСНОВНОЙ ВЫЗОВ: 8 напряжений [В] -> уровень мышечной активности 0..1.
 * Вызывать строго с частотой cfg.fs, ровно один раз на кадр. */
float nirs_pipe_update(nirs_pipe_t *s, const float *volts);

/* -------------------------------------------------------------------------
 *  Результаты и диагностика
 * ---------------------------------------------------------------------- */
float    nirs_pipe_volts (const nirs_pipe_t *s);   /* 0..out_vmax          */
uint16_t nirs_pipe_dac12 (const nirs_pipe_t *s);   /* 0..4095              */
uint16_t nirs_pipe_flags (const nirs_pipe_t *s);
float    nirs_pipe_algo  (const nirs_pipe_t *s);   /* выход алгоритма 0..1 */
float    nirs_pipe_nn    (const nirs_pipe_t *s);   /* выход сети 0..1      */
float    nirs_pipe_diverge(const nirs_pipe_t *s);  /* среднее расхождение  */

/* БЫСТРЫЙ признак: датчик на мышце, каналы набрали базу и знак, выход
 * правильно повторяет мышцу. Единицы секунд после старта; падает в ту же
 * выборку, когда датчик сняли. Это то, чем гейтить выход и индикацию. */
int      nirs_pipe_tracking(const nirs_pipe_t *s);

/* МЕДЛЕННЫЙ признак: вдобавок устоялся масштаб (автокалибровка максимума).
 * Нужен там, где важна абсолютная величина. Десятки секунд — раньше шкала
 * просто неизвестна.                                                    */
int      nirs_pipe_stable (const nirs_pipe_t *s);

/* Здоровье установки 0..1 — доля веса работающих каналов. */
float    nirs_pipe_health (const nirs_pipe_t *s);

/* Приблизительный ход адаптации 0..1 — для полоски прогресса. */
float    nirs_pipe_stable_progress(const nirs_pipe_t *s);

/* Калибровка шкалы алгоритма (в режиме NN не используется). */
void  nirs_pipe_calibrate(nirs_pipe_t *s);
void  nirs_pipe_set_mvc  (nirs_pipe_t *s, float mvc_od);
float nirs_pipe_get_mvc  (const nirs_pipe_t *s);

#ifdef __cplusplus
}
#endif
#endif /* NIRS_PIPELINE_H */
