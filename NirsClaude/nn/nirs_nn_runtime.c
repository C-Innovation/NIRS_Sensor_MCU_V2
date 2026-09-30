/* ============================================================================
 *  nirs_nn_runtime.c — предфильтр и потоковая обвязка сети.
 *
 *  ВАЖНО: арифметика предфильтра должна совпадать с nirs_prefilter.py,
 *  на котором обучалась сеть. Если правите одно — правьте и второе, иначе
 *  на микроконтроллере модель увидит другие числа, чем при обучении, и
 *  качество упадёт без единой ошибки в логах.
 *
 *  Точное совпадение не требуется и недостижимо: здесь float32, в Python
 *  float64. Измерено на записи 57 с: признаки расходятся не более чем на
 *  8.5e-4, а выход модели — на 1.6e-4, то есть в 200 раз меньше её
 *  собственной ошибки (0.036). Расхождение такого порядка нормально;
 *  расхождение в признаках уровня 0.01 и выше означает ошибку в переносе.
 * ==========================================================================*/
#include "nirs_nn_runtime.h"
#include <math.h>
#include <string.h>

/* Проверка «число конечно» ПО БИТАМ, а не через isfinite().
 *
 * isfinite()/isnan() нельзя: при сборке с -ffast-math (или
 * -ffinite-math-only, который она включает) компилятор ИМЕЕТ ПРАВО считать,
 * что нечисел не бывает, и сворачивает такие проверки в константу. Защита
 * тогда исчезает молча, ровно в релизной конфигурации. У float32 экспонента
 * из всех единиц — это inf или NaN, и такой тест не оптимизируется. */
static int nirs_finite(float x)
{
    union { float f; uint32_t u; } t;
    t.f = x;
    return (t.u & 0x7F800000u) != 0x7F800000u;
}

static float coef(float fs, float tau)
{
    if (tau <= 0.0f || fs <= 0.0f) return 1.0f;
    return 1.0f - expf(-1.0f / (fs * tau));
}

/* ------------------------------------------------------------------ */
void nirs_prefilter_init(nirs_prefilter_t *p, float fs)
{
    memset(p, 0, sizeof(*p));
    p->a_base_up = coef(fs, 0.75f);
    p->a_base_dn = coef(fs, 120.0f);
    p->a_span_up = coef(fs, 2.0f);
    p->a_span_dn = coef(fs, 60.0f);
    p->v_min     = 0.02f;
    p->v_max     = 4.30f;
    p->min_span  = 0.010f;
    p->freeze_k  = 4.0f;
    nirs_prefilter_reset(p);
}

void nirs_prefilter_reset(nirs_prefilter_t *p)
{
    int k;
    for (k = 0; k < NIRS_NN_CH; ++k) {
        p->base[k] = 0.0f;
        p->span[k] = p->min_span;
        p->ok[k]   = 0u;
        p->acc[k]  = 0.0f;
    }
    p->acc_n = 0;
}

void nirs_prefilter_step(nirs_prefilter_t *p, const float *v, float *f)
{
    int k;
    for (k = 0; k < NIRS_NN_CH; ++k) {
        float x = v[k], a, d, sp, r;

        /* сравнения с NaN дают 0 -> непригодные каналы отсеиваются здесь же */
        if (!(x > p->v_min && x < p->v_max)) {
            p->ok[k] = 0u;
            f[k] = 0.0f;
            continue;
        }
        a = -logf(x);

        if (!p->ok[k]) { p->base[k] = a; p->ok[k] = 1u; }

        if (a > p->base[k]) {
            p->base[k] += p->a_base_up * (a - p->base[k]);
        } else {
            sp = p->span[k] < p->min_span ? p->min_span : p->span[k];
            r  = (p->base[k] - a) / sp;
            p->base[k] += (p->a_base_dn / (1.0f + p->freeze_k * r * r))
                          * (a - p->base[k]);
        }

        d = p->base[k] - a;
        if (d < 0.0f) d = 0.0f;

        if (d > p->span[k]) p->span[k] += p->a_span_up * (d - p->span[k]);
        else                p->span[k] += p->a_span_dn * (d - p->span[k]);
        if (p->span[k] < p->min_span) p->span[k] = p->min_span;

        f[k] = d / p->span[k];

        /* Канал сам себя восстанавливает, если его состояние испортилось.
         * База и размах накапливаются рекуррентно: попади туда нечисло
         * (порча памяти, срыв контекста FPU), канал молчал бы до
         * перезагрузки. Дешевле переснять базу с нуля. */
        if (!nirs_finite(f[k]) || !nirs_finite(p->base[k])
                               || !nirs_finite(p->span[k])) {
            p->base[k] = a;
            p->span[k] = p->min_span;
            p->ok[k]   = 1u;
            f[k]       = 0.0f;
        }
        if (f[k] > NIRS_NN_FEAT_MAX) f[k] = NIRS_NN_FEAT_MAX;
    }
}

/* ------------------------------------------------------------------ */
void nirs_nn_init(nirs_nn_t *s, float fs)
{
    memset(s, 0, sizeof(*s));
    nirs_prefilter_init(&s->pre, fs);
    s->warmup_n = (uint32_t)(NIRS_NN_WARMUP_S * fs);
}

int nirs_nn_ready(const nirs_nn_t *s)
{
    return s->n >= s->warmup_n;
}

uint32_t nirs_nn_nan_events (const nirs_nn_t *s) { return s->nan_events;  }
uint32_t nirs_nn_bad_invokes(const nirs_nn_t *s) { return s->bad_invokes; }

float nirs_nn_update(nirs_nn_t *s, const float *volts)
{
    float f[NIRS_NN_CH];
    /* предфильтр работает на ПОЛНОЙ частоте: его постоянные времени
     * рассчитаны именно под неё */
    nirs_prefilter_step(&s->pre, volts, f);
    return nirs_nn_update_features(s, f);
}

float nirs_nn_update_features(nirs_nn_t *s, const float *f)
{
    int   k;

    s->n++;

    /* Накапливаем для прореживания (усреднение, как при обучении).
     * Накопитель тоже рекуррентный: одно нечисло в нём испортило бы все
     * последующие кадры, поэтому вход санируется ЗДЕСЬ, а не у вызывающего.
     * Непригодный канал даёт 0 — так же, как его отдаёт предфильтр. */
    for (k = 0; k < NIRS_NN_CH; ++k) {
        float x = f[k];
        if (!nirs_finite(x))          x = 0.0f;
        if (x < 0.0f)                 x = 0.0f;
        if (x > NIRS_NN_FEAT_MAX)     x = NIRS_NN_FEAT_MAX;
        s->pre.acc[k] += x;
    }
    s->pre.acc_n++;

    if (++s->sub >= NIRS_NN_DECIM) {
        float in[NIRS_NN_CH], y = 0.0f, st[NIRS_NN_STATE];
        float inv = (s->pre.acc_n > 0) ? (1.0f / (float)s->pre.acc_n) : 0.0f;
        int   bad = 0;

        for (k = 0; k < NIRS_NN_CH; ++k) {
            in[k] = s->pre.acc[k] * inv;
            s->pre.acc[k] = 0.0f;
        }
        s->pre.acc_n = 0;
        s->sub = 0;

        if (nirs_nn_invoke(in, s->state, &y, st) == 0) {
            /* ГЛАВНАЯ ЗАЩИТА. Скрытое состояние заводится само в себя:
             * стоит одному нечислу попасть в state[], и выход остаётся
             * нечислом навсегда — сеть из этого состояния не выходит.
             * Проверяем ДО копирования обратно. Обратите внимание, что
             * обычный ограничитель здесь бесполезен: все сравнения с NaN
             * ложны, и "if (y > 1) y = 1;" пропускает NaN насквозь. */
            for (k = 0; k < NIRS_NN_STATE; ++k)
                if (!nirs_finite(st[k])) { bad = 1; break; }
            if (!nirs_finite(y)) bad = 1;

            if (bad) {
                /* Сбрасываем контекст сети и держим прошлый выход. GRU
                 * набирает контекст заново за доли секунды, а залипания
                 * до перезагрузки больше не будет. */
                memset(s->state, 0, sizeof(s->state));
                s->nan_events++;
            } else {
                memcpy(s->state, st, sizeof(s->state));
                if (y < 0.0f) y = 0.0f;
                if (y > 1.0f) y = 1.0f;
                s->out = y;
            }
        } else {
            s->bad_invokes++;
            /* если инференс не удался — держим прошлое значение */
        }
    }
    return s->out;
}
