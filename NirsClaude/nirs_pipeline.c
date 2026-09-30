/* ============================================================================
 *  nirs_pipeline.c — реализация единого API.
 * ==========================================================================*/
#include "nirs_pipeline.h"
#include <math.h>
#include <string.h>

static float coef(float fs, float tau)
{
    if (tau <= 0.0f || fs <= 0.0f) return 1.0f;
    return 1.0f - expf(-1.0f / (fs * tau));
}

void nirs_pipe_defaults(nirs_pipe_cfg_t *c, float fs, nirs_pipe_mode_t mode)
{
    memset(c, 0, sizeof(*c));
    c->fs          = (fs > 0.0f) ? fs : 1000.0f;
    c->mode        = mode;
    c->out_vmax    = 3.30f;
    c->diverge_thr = 0.15f;
    c->tau_diverge = 1.0f;
}

void nirs_pipe_init(nirs_pipe_t *s, const nirs_pipe_cfg_t *cfg)
{
    nirs_cfg_t ac;
    memset(s, 0, sizeof(*s));
    s->cfg   = *cfg;
    s->a_div = coef(cfg->fs, cfg->tau_diverge);

    nirs_defaults(&ac, cfg->fs);
    ac.out_vmax = cfg->out_vmax;
    nirs_init(&s->algo, &ac);

    if (cfg->mode != NIRS_PIPE_ALGO)
        nirs_nn_init(&s->nn, cfg->fs);
}

float nirs_pipe_update(nirs_pipe_t *s, const float *volts)
{
    const nirs_pipe_cfg_t *c = &s->cfg;
    uint16_t fl = 0, af;
    int ready;

    s->n++;

    /* Ядро считаем ВСЕГДА, даже в режиме NN: оно даёт флаги качества
     * сигнала (нет каналов, насыщение), которые сети взять неоткуда, и
     * стоит единицы микросекунд. */
    s->y_algo = nirs_update(&s->algo, volts);
    af = nirs_flags(&s->algo);

    if (c->mode != NIRS_PIPE_ALGO) {
        /* Признаки для сети берём у ядра: оно уже посчитано на этом шаге, а
         * его od/span учитывают полярность канала и слежение базы за
         * дрейфом. Отдельный предфильтр этого не умеет и остаётся только
         * для совместимости со старыми моделями (nirs_nn_update). */
        float feat[NIRS_NN_CH];
        nirs_nn_features(&s->algo, feat);
        s->y_nn = nirs_nn_update_features(&s->nn, feat);
    }

    switch (c->mode) {
        case NIRS_PIPE_NN:
        case NIRS_PIPE_BOTH: s->y = s->y_nn;   break;
        default:             s->y = s->y_algo; break;
    }

    /* расхождение — только когда оба выхода достоверны */
    if (c->mode == NIRS_PIPE_BOTH && (af & NIRS_F_READY) && nirs_nn_ready(&s->nn)) {
        float d = s->y_nn - s->y_algo;
        if (d < 0.0f) d = -d;
        s->diverge += s->a_div * (d - s->diverge);
        if (s->diverge > c->diverge_thr) fl |= NIRS_PIPE_F_DIVERGE;
    }

    ready = (c->mode == NIRS_PIPE_ALGO) ? ((af & NIRS_F_READY) != 0)
                                        : (nirs_nn_ready(&s->nn) &&
                                           (af & NIRS_F_READY) != 0);
    if (ready)                 fl |= NIRS_PIPE_F_READY;
    if (af & NIRS_F_ACTIVE)    fl |= NIRS_PIPE_F_ACTIVE;
    if (af & NIRS_F_NO_CHAN)   fl |= NIRS_PIPE_F_NOSIG;
    if (af & NIRS_F_SAT)       fl |= NIRS_PIPE_F_SAT;

    /* Готовность: признаки берутся у ядра, а в режимах с сетью к ним
     * добавляется сходимость скрытого состояния GRU (3 с). После снятия
     * датчика оба флага падают в ту же выборку. */
    {
        int nn_ok = (c->mode == NIRS_PIPE_ALGO) || nirs_nn_ready(&s->nn);
        if ((af & NIRS_F_TRACKING) && nn_ok) fl |= NIRS_PIPE_F_TRACKING;
        if ((af & NIRS_F_STABLE)   && nn_ok) fl |= NIRS_PIPE_F_STABLE;
    }

    s->flags = fl;

    if (s->y < 0.0f) s->y = 0.0f;
    if (s->y > 1.0f) s->y = 1.0f;
    return s->y;
}

float    nirs_pipe_volts  (const nirs_pipe_t *s) { return s->y * s->cfg.out_vmax; }
uint16_t nirs_pipe_flags  (const nirs_pipe_t *s) { return s->flags; }
float    nirs_pipe_algo   (const nirs_pipe_t *s) { return s->y_algo; }
float    nirs_pipe_nn     (const nirs_pipe_t *s) { return s->y_nn; }
float    nirs_pipe_diverge(const nirs_pipe_t *s) { return s->diverge; }

int      nirs_pipe_tracking(const nirs_pipe_t *s)
{
    return (s->flags & NIRS_PIPE_F_TRACKING) != 0u;
}

int      nirs_pipe_stable (const nirs_pipe_t *s)
{
    return (s->flags & NIRS_PIPE_F_STABLE) != 0u;
}

float    nirs_pipe_health (const nirs_pipe_t *s)
{
    return nirs_health(&s->algo);
}

float    nirs_pipe_stable_progress(const nirs_pipe_t *s)
{
    return nirs_stable_progress(&s->algo);
}

uint16_t nirs_pipe_dac12(const nirs_pipe_t *s)
{
    float f = s->y * 4095.0f + 0.5f;
    if (f < 0.0f)    f = 0.0f;
    if (f > 4095.0f) f = 4095.0f;
    return (uint16_t)f;
}

void  nirs_pipe_calibrate(nirs_pipe_t *s)            { nirs_calibrate_mvc(&s->algo); }
void  nirs_pipe_set_mvc  (nirs_pipe_t *s, float mvc) { nirs_set_mvc(&s->algo, mvc); }

float nirs_pipe_get_mvc(const nirs_pipe_t *s)
{
    nirs_extra_t e;
    nirs_get_extra(&s->algo, &e);
    return e.mvc;
}
