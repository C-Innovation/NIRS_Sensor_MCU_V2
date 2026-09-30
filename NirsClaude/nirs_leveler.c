/* ============================================================================
 *  nirs_leveler.c — реализация.
 *
 *  Порядок на каждой выборке:
 *    1. Пригодность канала (окно v_min..v_max, отсев NaN).
 *    2. Нижняя огибающая ref[k] — вниз быстро, вверх медленно. Не морозится.
 *    3. Признаки: dev = (v - ref)/ref  и  edge = (fast - slow)/ref.
 *       Активность = dev > thr_dev  ИЛИ  edge > thr_edge, плюс удержание.
 *    4. Относительный размах span[k] — чтобы не «оживить» мёртвый канал.
 *    5. В ПОКОЕ: rest[k] тянется к сигналу, gain -> target/rest со
 *       сглаживанием и ограничением скорости. В АКТИВНОСТИ всё заморожено.
 *    6. Выход = вход * gain  (или вход - rest + target в аддитивном режиме).
 * ==========================================================================*/

#include "nirs_leveler.h"
#include <math.h>
#include <string.h>

static float coef(float fs, float tau)
{
    if (tau <= 0.0f || fs <= 0.0f) return 1.0f;
    {
        float a = 1.0f - expf(-1.0f / (fs * tau));
        if (a > 1.0f) a = 1.0f;
        if (a < 0.0f) a = 0.0f;
        return a;
    }
}

/* ------------------------------------------------------------------ */
void nirs_lvl_defaults(nirs_lvl_cfg_t *c, float fs, float target_v)
{
    int k;
    memset(c, 0, sizeof(*c));
    c->fs = (fs > 0.0f) ? fs : 1000.0f;
    if (!(target_v > 0.0f)) target_v = 1.0f;
    for (k = 0; k < NIRS_LVL_CH; ++k) c->target[k] = target_v;

    c->v_min        = 0.02f;
    c->v_max        = 4.00f;   /* заметно ниже предела ОУ, см. заголовок */

    c->tau_ref_dn   = 0.5f;
    c->tau_ref_up   = 20.0f;

    c->tau_rest     = 1.0f;
    c->tau_gain     = 1.0f;
    c->converge_s   = 5.0f;

    c->gain_min     = 0.05f;
    c->gain_max     = 20.0f;
    c->gain_slew    = 0.10f;

    c->thr_dev      = 0.08f;
    c->tau_fast     = 0.008f;
    c->tau_slow     = 0.080f;
    c->thr_edge     = 0.12f;
    c->hold_s       = 0.5f;

    c->min_span_rel = 0.02f;
    c->span_grace_s = 10.0f;

    c->global_freeze       = 1;
    c->passthrough_invalid = 1;
    c->additive            = 0;
    c->warmup_s            = 2.0f;
}

/* ------------------------------------------------------------------ */
void nirs_lvl_init(nirs_lvl_t *s, const nirs_lvl_cfg_t *cfg)
{
    float fs;
    memset(s, 0, sizeof(*s));
    s->cfg = *cfg;
    fs = (cfg->fs > 0.0f) ? cfg->fs : 1000.0f;

    s->a_ref_dn  = coef(fs, cfg->tau_ref_dn);
    s->a_ref_up  = coef(fs, cfg->tau_ref_up);
    s->a_rest    = coef(fs, cfg->tau_rest);
    s->a_gain    = coef(fs, cfg->tau_gain);
    s->a_fast    = coef(fs, cfg->tau_fast);
    s->a_slow    = coef(fs, cfg->tau_slow);
    s->a_span_up = coef(fs, 2.0f);
    s->a_span_dn = coef(fs, 60.0f);

    s->slew_step = (cfg->gain_slew > 0.0f) ? (cfg->gain_slew / fs) : 1e9f;
    s->warmup_n  = (uint32_t)(cfg->warmup_s    * fs);
    s->hold_n    = (uint32_t)(cfg->hold_s      * fs);
    s->grace_n    = (uint32_t)(cfg->span_grace_s * fs);
    s->converge_n = (uint32_t)(cfg->converge_s   * fs);

    nirs_lvl_reset(s);
}

void nirs_lvl_reset(nirs_lvl_t *s)
{
    int k;
    for (k = 0; k < NIRS_LVL_CH; ++k) {
        s->ref[k]  = 0.0f;
        s->rest[k] = 0.0f;
        s->gain[k] = 1.0f;
        s->fast[k] = 0.0f;
        s->slow[k] = 0.0f;
        s->span[k] = 0.0f;
        s->ok[k]   = 0u;
        s->act[k]  = 0u;
        s->dead[k] = 0u;
        s->hold[k] = 0u;
    }
    s->n = 0;
    s->flags = 0;
}

/* ------------------------------------------------------------------ */
void nirs_lvl_process(nirs_lvl_t *s, const float *in, float *out)
{
    const nirs_lvl_cfg_t *c = &s->cfg;
    int k, any_active = 0;
    uint16_t fl = 0;
    float x[NIRS_LVL_CH];

    s->n++;

    /* --- 1..4 --- */
    for (k = 0; k < NIRS_LVL_CH; ++k) {
        float v = in[k];
        float r, dev, edge;
        x[k] = v;                                /* копия: in может == out */

        /* сравнение с NaN даёт 0 -> NaN отсеивается здесь же */
        if (!(v > c->v_min && v < c->v_max)) {
            s->ok[k]  = 0u;
            s->act[k] = 0u;
            fl |= NIRS_LVL_F_INVALID;
            continue;
        }

        if (!s->ok[k]) {                         /* первая пригодная выборка */
            s->ref[k]  = v;
            s->rest[k] = v;
            s->fast[k] = v;
            s->slow[k] = v;
            s->span[k] = 0.0f;
            s->gain[k] = c->target[k] / v;
            s->ok[k]   = 1u;
        }

        /* 2. нижняя огибающая — опора детектора, не морозится */
        if (v < s->ref[k]) s->ref[k] += s->a_ref_dn * (v - s->ref[k]);
        else               s->ref[k] += s->a_ref_up * (v - s->ref[k]);
        if (s->ref[k] < 1e-6f) s->ref[k] = 1e-6f;

        s->fast[k] += s->a_fast * (v - s->fast[k]);
        s->slow[k] += s->a_slow * (v - s->slow[k]);

        /* 3. признаки активности */
        r    = s->ref[k];
        dev  = (v - r) / r;
        edge = (s->fast[k] - s->slow[k]) / r;
        if (edge < 0.0f) edge = -edge;
        s->act[k] = (uint8_t)((dev > c->thr_dev) || (edge > c->thr_edge));
        if (s->act[k]) any_active = 1;

        /* 4. относительный размах: жив ли канал вообще */
        {
            float d = dev < 0.0f ? 0.0f : dev;
            if (d > s->span[k]) s->span[k] += s->a_span_up * (d - s->span[k]);
            else                s->span[k] += s->a_span_dn * (d - s->span[k]);
        }
        s->dead[k] = (uint8_t)((s->n > s->grace_n) && (s->span[k] < c->min_span_rel));
        if (s->dead[k]) fl |= NIRS_LVL_F_DEAD;
    }

    /* удержание — счётчик на канал; при global_freeze активность любого
     * канала перезаряжает счётчики всех */
    for (k = 0; k < NIRS_LVL_CH; ++k) {
        if (s->act[k] || (c->global_freeze && any_active)) s->hold[k] = s->hold_n;
        else if (s->hold[k]) s->hold[k]--;
        if (s->hold[k]) fl |= NIRS_LVL_F_ACTIVE;
    }

    /* --- 5. адаптация --- */
    {
        const int converging = (s->n <= s->converge_n);
        for (k = 0; k < NIRS_LVL_CH; ++k) {
            float g, d, lim;
            if (!s->ok[k] || s->dead[k]) continue;

            if (converging) {
                /* Начальная сходимость: опираемся на нижнюю огибающую
                 * (она уже нашла уровень покоя за доли секунды) и ставим
                 * коэффициент сразу, без сглаживания и предела скорости.
                 * Заморозка тут не нужна: ref по построению не поднимается
                 * за сокращением. */
                s->rest[k] = s->ref[k];
            } else {
                if (s->hold[k] != 0u) continue;          /* заморожено */
                s->rest[k] += s->a_rest * (x[k] - s->rest[k]);
            }
            if (s->rest[k] < 1e-6f) s->rest[k] = 1e-6f;

            g = c->target[k] / s->rest[k];
            if (g < c->gain_min) { g = c->gain_min; fl |= NIRS_LVL_F_CLAMPED; }
            if (g > c->gain_max) { g = c->gain_max; fl |= NIRS_LVL_F_CLAMPED; }

            if (converging) {
                s->gain[k] = g;
            } else {
                d   = s->a_gain * (g - s->gain[k]);
                lim = s->slew_step * s->gain[k];         /* предел относительный */
                if (d >  lim) d =  lim;
                if (d < -lim) d = -lim;
                s->gain[k] += d;
            }
        }
    }

    /* --- 6. выход --- */
    for (k = 0; k < NIRS_LVL_CH; ++k) {
        if (!s->ok[k]) {
            out[k] = c->passthrough_invalid ? x[k] : c->target[k];
        } else if (s->dead[k]) {
            out[k] = x[k];                       /* мёртвый канал не трогаем */
        } else if (c->additive) {
            out[k] = x[k] - s->rest[k] + c->target[k];
        } else {
            out[k] = x[k] * s->gain[k];
        }
    }

    if (s->n >= s->warmup_n) fl |= NIRS_LVL_F_READY;
    s->flags = fl;
}

/* ------------------------------------------------------------------ */
void nirs_lvl_set_target(nirs_lvl_t *s, int ch, float volts)
{
    if (ch < 0 || ch >= NIRS_LVL_CH) return;
    if (!(volts > 0.0f)) return;
    s->cfg.target[ch] = volts;
    /* коэффициент подтянется сам — со сглаживанием и ограничением скорости */
}

int      nirs_lvl_active(const nirs_lvl_t *s) { return (s->flags & NIRS_LVL_F_ACTIVE) != 0; }
uint16_t nirs_lvl_flags (const nirs_lvl_t *s) { return s->flags; }

int nirs_lvl_channel_ok(const nirs_lvl_t *s, int ch)
{
    if (ch < 0 || ch >= NIRS_LVL_CH) return 0;
    return s->ok[ch] && !s->dead[ch];
}

void nirs_lvl_get_gain(const nirs_lvl_t *s, float *g8)
{
    int k; for (k = 0; k < NIRS_LVL_CH; ++k) g8[k] = s->gain[k];
}

void nirs_lvl_get_rest(const nirs_lvl_t *s, float *r8)
{
    int k; for (k = 0; k < NIRS_LVL_CH; ++k) r8[k] = s->rest[k];
}
