/* ============================================================================
 *  nirs_contraction.c  —  реализация
 *
 *  Идея алгоритма (по шагам, все операции причинные и O(1)):
 *
 *   1) V -> ослабление.  a = -ln(V).  Логарифм превращает мультипликативные
 *      эффекты (мощность светодиода, коэффициент передачи ОУ, оптический
 *      контакт с кожей) в аддитивные, поэтому дальше их убирает вычитание
 *      базовой линии.
 *
 *   2) Базовая линия = уровень ослабления в ПОКОЕ.
 *      Сокращение мышцы увеличивает принимаемый свет, т.е. УМЕНЬШАЕТ a.
 *      Поэтому база следует вверх быстро (tau_base_up ~ 0.75 с) и вниз
 *      очень медленно (tau_base_dn ~ 120 с), причём скорость спуска
 *      дополнительно давится множителем 1/(1+k*r^2), где r — текущая
 *      глубина сокращения. Это даёт устойчивость и к дрейфу, и к длинным
 *      удержаниям, и — в отличие от «заморозки по флагу» — не может
 *      залипнуть навсегда, потому что регулировка непрерывная.
 *
 *      d = base - a  >= 0.   d == 0 в покое, растёт при сокращении.
 *
 *   3) Оценка размаха канала span[k] (пиковый детектор с медленным спадом)
 *      и оценка шума noise[k] (средний модуль приращения a за выборку).
 *
 *   4) Вес канала w = r^2/(1+r^2), r = span/(snr_k*noise) — мягкий
 *      винеровский вес по ОСШ. Насыщенный или мёртвый канал имеет
 *      span ~ 0 -> w ~ 0 и выпадает из слияния автоматически.
 *      Именно так на приложенной записи сами собой отключаются каналы
 *      5.5 мм и 10 мм (у них span в 250-1000 раз меньше, чем у дальних).
 *
 *   5) Слияние: u = (Σ w*d/span) / (Σ w) * span_ref, где span_ref —
 *      взвешенное среднее размахов. Деление на span выравнивает каналы
 *      разной чувствительности, обратное умножение возвращает физические
 *      единицы OD, чтобы выход оставался пропорционален усилию.
 *
 *   6) lead-компенсатор u += (lead_tau/tau_lp)*(u - LP(u)) — фазовое
 *      опережение, компенсирует собственную инерцию отклика ткани.
 *
 *   7) Асимметричное сглаживание: атака 10 мс, спад 60 мс.
 *
 *   8) Нормировка на MVC (пиковый детектор с очень медленным спадом либо
 *      ручная калибровка) -> 0..1, мёртвая зона, ограничение, ЦАП.
 * ==========================================================================*/

#include "nirs_contraction.h"
#include <math.h>
#include <string.h>

/* ------------------------------------------------------------------ */
/*  Быстрый логарифм. Определите NIRS_FAST_LOG, чтобы не тянуть libm.  */
/*  Относительная погрешность < 1e-6, ~25 тактов на Cortex-M4F.        */
/* ------------------------------------------------------------------ */
#ifdef NIRS_FAST_LOG
static inline float nirs_logf(float x)
{
    union { float f; uint32_t i; } u;
    int   e;
    float m, t, t2;
    u.f = x;
    e   = (int)((u.i >> 23) & 0xFFu) - 127;
    u.i = (u.i & 0x007FFFFFu) | 0x3F800000u;   /* мантисса в [1,2) */
    m   = u.f;
    if (m > 1.41421356f) { m *= 0.5f; e += 1; }  /* сузить до [0.707,1.414) */
    t   = (m - 1.0f) / (m + 1.0f);
    t2  = t * t;
    /* ln(m) = 2*atanh(t), ряд до t^9 */
    return 2.0f * t * (1.0f + t2 * (0.33333333f + t2 * (0.2f
                        + t2 * (0.14285714f + t2 * 0.11111111f))))
           + (float)e * 0.69314718f;
}
#else
#define nirs_logf(x)  logf(x)
#endif

/* Корень нужен ровно в одном месте — нормировке корреляции при оценке
 * полярности. Обёртка оставлена отдельной на случай платформы без libm. */
#define nirs_sqrtf(x) sqrtf(x)

/* коэффициент однополюсного фильтра для постоянной времени tau [с] */
static float coef(float fs, float tau)
{
    if (tau <= 0.0f)   return 1.0f;
    if (fs  <= 0.0f)   return 1.0f;
    {
        float a = 1.0f - expf(-1.0f / (fs * tau));
        if (a > 1.0f) a = 1.0f;
        if (a < 0.0f) a = 0.0f;
        return a;
    }
}

/* ------------------------------------------------------------------ */
void nirs_defaults(nirs_cfg_t *c, float fs)
{
    static const float d[NIRS_MAX_CH] = { 5.5f, 10.0f, 26.5f, 32.0f,
                                          5.5f, 10.0f, 26.5f, 32.0f };
    static const float w[NIRS_MAX_CH] = { 740.f, 740.f, 740.f, 740.f,
                                          850.f, 850.f, 850.f, 850.f };
    int k;
    memset(c, 0, sizeof(*c));
    c->fs  = (fs > 0.0f) ? fs : 1000.0f;
    c->nch = NIRS_MAX_CH;
    for (k = 0; k < NIRS_MAX_CH; ++k) { c->dist_mm[k] = d[k]; c->wl_nm[k] = w[k]; }

    c->v_min       = 0.02f;
    c->v_max       = 4.30f;   /* OPA2380 @5В упирается около 4.4 В         */
    c->v_hyst      = 0.05f;   /* гистерезис возврата канала в диапазон      */

    c->tau_base_up = 0.75f;
    c->tau_base_dn = 120.0f;
    c->freeze_k    = 4.0f;
    c->base_rest_thr = 0.15f;

    c->tau_span_up = 2.0f;
    c->tau_span_dn = 60.0f;
    c->min_span    = 0.010f;

    c->tau_noise   = 1.0f;
    c->snr_k       = 6.0f;
    c->w_min       = 0.05f;

    c->lead_tau    = 0.030f;
    c->tau_lead_lp = 0.030f;

    c->tau_attack  = 0.010f;
    c->tau_release = 0.060f;

    c->tau_mvc_lp  = 0.15f;
    c->tau_mvc_up  = 0.30f;
    c->tau_mvc_dn  = 300.0f;
    c->mvc_min     = 0.05f;
    c->mvc_span_k  = 0.75f;
    c->dn_max      = 2.0f;
    c->soft_knee   = 0.85f;

    c->deadzone    = 0.030f;
    c->gate_hi     = 0.30f;
    c->gate_lo     = 0.15f;
    c->out_vmax    = 3.30f;
    c->warmup_s    = 1.5f;
    c->track_s        = 1.5f;
    c->health_hi      = 0.50f;
    c->health_lo      = 0.35f;
    c->tau_w_ref      = 60.0f;
    c->mvc_settle_s   = 10.0f;
    c->mvc_settle_thr = 0.05f;

    c->auto_polarity = 1;
    c->tau_pol       = 5.0f;
    c->pol_thr       = 0.35f;
    c->pol_hold_s    = 10.0f;
}

/* ------------------------------------------------------------------ */
void nirs_init(nirs_t *s, const nirs_cfg_t *cfg)
{
    int k;
    float fs;

    memset(s, 0, sizeof(*s));
    s->cfg = *cfg;
    fs     = (cfg->fs > 0.0f) ? cfg->fs : 1000.0f;
    s->nch = cfg->nch;
    if (s->nch < 1)           s->nch = 1;
    if (s->nch > NIRS_MAX_CH) s->nch = NIRS_MAX_CH;

    s->a_base_up = coef(fs, cfg->tau_base_up);
    s->a_base_dn = coef(fs, cfg->tau_base_dn);
    s->a_span_up = coef(fs, cfg->tau_span_up);
    s->a_span_dn = coef(fs, cfg->tau_span_dn);
    s->a_noise   = coef(fs, cfg->tau_noise);
    s->a_att     = coef(fs, cfg->tau_attack);
    s->a_rel     = coef(fs, cfg->tau_release);
    s->a_mvc_lp  = coef(fs, cfg->tau_mvc_lp);
    s->a_mvc_up  = coef(fs, cfg->tau_mvc_up);
    s->a_mvc_dn  = coef(fs, cfg->tau_mvc_dn);
    s->a_lead    = coef(fs, cfg->tau_lead_lp);
    s->a_pol     = coef(fs, cfg->tau_pol);
    s->pol_hold_n= (uint32_t)(cfg->pol_hold_s * fs);
    s->lead_gain = (cfg->tau_lead_lp > 0.0f) ? (cfg->lead_tau / cfg->tau_lead_lp) : 0.0f;
    s->warmup_n  = (uint32_t)(cfg->warmup_s * fs);
    s->track_n      = (uint32_t)(cfg->track_s * fs);
    s->mvc_settle_n = (uint32_t)(cfg->mvc_settle_s * fs);
    s->a_w_ref      = coef(fs, cfg->tau_w_ref);

    /* деление каналов на группы: по расстоянию и по длине волны.
     * Порог по расстоянию — середина между минимальным и максимальным. */
    {
        float dmin = cfg->dist_mm[0], dmax = cfg->dist_mm[0], split;
        for (k = 1; k < s->nch; ++k) {
            if (cfg->dist_mm[k] < dmin) dmin = cfg->dist_mm[k];
            if (cfg->dist_mm[k] > dmax) dmax = cfg->dist_mm[k];
        }
        split = 0.5f * (dmin + dmax);
        for (k = 0; k < s->nch; ++k) {
            s->is_far[k] = (uint8_t)(cfg->dist_mm[k] >= split);
            s->is_850[k] = (uint8_t)(cfg->wl_nm  [k] >  800.0f);
        }
    }
    nirs_reset(s);
}

/* ------------------------------------------------------------------ */
void nirs_reset(nirs_t *s)
{
    int k;
    for (k = 0; k < NIRS_MAX_CH; ++k) {
        s->base[k]  = 0.0f;
        s->span[k]  = s->cfg.min_span;
        s->noise[k] = 1e-5f;
        s->prev[k]  = 0.0f;
        s->od[k]    = 0.0f;
        s->w[k]     = 0.0f;
        s->ch_ok[k] = 0u;
        s->rearm[k] = 0u;
        s->ch_ready[k] = 0u;
        s->ch_run[k]   = 0u;
        s->pol[k]      = 1.0f;
        s->pol_cov[k]  = 0.0f;
        s->pol_mdn[k]  = 0.0f;
        s->pol_vdn[k]  = 0.0f;
        s->pol_lock[k] = 0u;
    }
    s->pol_mref = 0.0f;
    s->pol_vref = 0.0f;
    s->far_act  = 0.0f;
    s->far_g    = 0.0f;
    s->n        = 0;
    s->w_ref      = 0.0f;
    s->health     = 0.0f;
    s->tracking   = 0u;
    s->mvc_settled= 0u;
    s->mvc_mark   = 0.0f;
    s->mvc_mark_n = 0u;
    s->stable_progress = 0.0f;
    s->u_lp     = 0.0f;
    s->level    = 0.0f;
    s->mvc      = s->cfg.mvc_min;
    s->mvc_lp   = 0.0f;
    s->spatial  = 0.0f;
    s->spectral = 0.0f;
    s->u_od     = 0.0f;
    s->wsum     = 0.0f;
    s->flags    = 0;
    s->out01    = 0.0f;
    s->out_v    = 0.0f;
}

/* ------------------------------------------------------------------ */
float nirs_update(nirs_t *s, const float *v)
{
    const nirs_cfg_t *c = &s->cfg;
    const int   nch   = s->nch;
    const int   first = (s->n == 0);
    int   k;
    float wsum = 0.0f, acc = 0.0f, span_ref = 0.0f;
    float u, a_smooth, o;
    float far_s = 0.0f, far_w = 0.0f, near_s = 0.0f, near_w = 0.0f;
    float w850_s = 0.0f, w850_w = 0.0f, w740_s = 0.0f, w740_w = 0.0f;
    float farn_s = 0.0f;                 /* нормированный вклад дальних     */
    float w_ready = 0.0f;                /* вес каналов, готовых к работе   */
    float dn[NIRS_MAX_CH];               /* d/span по каналам, 0 если не в слиянии */
    uint16_t fl = 0;

    s->n++;

    /* ---------------- обработка по каналам ---------------- */
    for (k = 0; k < nch; ++k) {
        float x = v[k];
        float a, d, r, w, sp, nz;
        int   ok;

        /* Проверка пригодности. Сравнения с NaN дают 0, поэтому это же
         * условие отсеивает NaN, Inf и отрицательные значения.        */
        /* Гистерезис: выпавший канал возвращается не на самой границе, а
         * на v_hyst глубже. Канал, стоящий вплотную к пределу шкалы ОУ,
         * иначе входит и выходит из диапазона на каждой пульсовой волне, а
         * каждый возврат сбрасывает его базу в текущий уровень: канал
         * получает полный вес с нулевым od и разбавляет общий выход.
         * На записях S02-S04 в таком состоянии до трёх каналов из восьми. */
        if (s->ch_ok[k] != 0u)
            ok = (x > c->v_min) && (x < c->v_max);
        else
            ok = (x > c->v_min + c->v_hyst) && (x < c->v_max - c->v_hyst);
        if (!ok) {
            /* Состояние непригодного канала НЕ обновляем: иначе один
             * NaN или выброс навсегда отравил бы адаптивные переменные
             * (база, размах, шум) и канал не восстановился бы даже
             * после возврата нормального сигнала.                     */
            fl |= NIRS_F_SAT;
            s->od[k]    = 0.0f;
            /* Канал выпал. Это событие ЛОКАЛЬНОЕ: оно отнимает у общего
             * «здоровья» установки ровно долю этого канала и глобальных
             * флагов само по себе не роняет. Датчик, снятый с мышцы,
             * роняет все каналы сразу — вот тогда здоровье падает в ноль. */
            s->w [k]        = 0.0f;
            s->rearm[k]     = 0u;
            s->ch_ok[k]     = 0u;
            s->ch_ready[k]  = 0u;
            s->ch_run[k]    = 0u;
            continue;
        }

        a = -nirs_logf(x) * s->pol[k];           /* ослабление с учётом знака */

        /* Первая выборка вообще либо первая после возврата канала.
         * База снимается «как есть», но если в этот момент остальные
         * каналы показывают сокращение, то снятый уровень покоем не
         * является. Тогда канал переводится в состояние 1 («ждёт
         * покоя») и в слиянии не участвует, пока сокращение не
         * закончится — иначе он занизил бы общий выход.              */
        if (first || s->ch_ok[k] == 0u) {
            s->base[k]  = a;
            s->prev[k]  = a;
            s->span[k]  = c->min_span;
            /* Размах тоже сбрасывается. Иначе канал, вернувшийся в диапазон,
             * получает ПОЛНЫЙ вес (span уцелел) при нулевой активности (база
             * только что снята) и разбавляет общий выход. Канал, висящий на
             * пределе шкалы ОУ, делает это с частотой пульса. С обнулением
             * размаха он молчит, пока не покажет настоящую модуляцию. */
            s->rearm[k] = 1u;   /* база с нуля; возмущение засчитаем, когда
                                 * канал снова начнёт что-то весить */
            s->ch_ok[k] = (s->out01 <= c->gate_lo) ? 2u : 1u;
        } else if (s->ch_ok[k] == 1u && s->out01 <= c->gate_lo) {
            s->base[k]  = a;                      /* вот теперь это покой */
            s->ch_ok[k] = 2u;
        }

        /* --- базовая линия: быстро вверх, медленно вниз ---
         * Исключение: пока дальние каналы согласно показывают покой,
         * вниз тоже идём быстро. Тогда база догоняет дрейф уровня покоя,
         * какого бы знака он ни был, и остаток между сокращениями
         * возвращается к нулю у каналов любой полярности.            */
        if (a > s->base[k]) {
            s->base[k] += s->a_base_up * (a - s->base[k]);
        } else {
            float rate;
            sp = s->span[k]; if (sp < c->min_span) sp = c->min_span;
            r  = (s->base[k] - a) / sp;          /* относительная глубина */
            rate = s->a_base_dn / (1.0f + c->freeze_k * r * r);
            /* Плавный переход, а не порог: жёсткое переключение на границе
             * дребезжит и делает результат чувствительным к последнему
             * биту. s->far_g падает от 1 (дальние в покое) до ~0.001 при
             * сокращении, так что защита базы во время удержания цела. */
            rate += s->far_g * (s->a_base_up - rate);
            s->base[k] += rate * (a - s->base[k]);
        }

        d = s->base[k] - a;
        if (d < 0.0f) d = 0.0f;
        s->od[k] = d;

        /* --- размах канала: пиковый детектор с медленным спадом --- */
        if (d > s->span[k]) s->span[k] += s->a_span_up * (d - s->span[k]);
        else                s->span[k] += s->a_span_dn * (d - s->span[k]);
        if (s->span[k] < c->min_span) s->span[k] = c->min_span;

        /* --- шум: средний модуль приращения --- */
        {
            float diff = a - s->prev[k];
            if (diff < 0.0f) diff = -diff;
            s->noise[k] += s->a_noise * (diff - s->noise[k]);
            s->prev[k]   = a;
        }
        nz = s->noise[k]; if (nz < 1e-7f) nz = 1e-7f;

        /* --- вес по ОСШ (канал в состоянии «ждёт покоя» не участвует) --- */
        if (s->ch_ok[k] == 2u && s->span[k] > c->min_span * 1.001f) {
            r = s->span[k] / (c->snr_k * nz);
            w = (r * r) / (1.0f + r * r);
        } else {
            w = 0.0f;
            fl |= NIRS_F_LOWSIG;
        }
        if (w < c->w_min) w = 0.0f;
        if (s->rearm[k] != 0u && w > 0.0f && s->span[k] > 2.0f * c->min_span)
            s->rearm[k] = 0u;            /* канал доучился и снова в деле */
        s->w[k] = w;

        /* --- готовность канала ---
         * Канал считается готовым, когда он непрерывно пригоден уже track_s,
         * прошёл фазу «ждёт покоя», не переучивает базу, набрал осмысленный
         * размах и не менял знак в пределах блокировки. Это и есть
         * «датчик на этом канале стоит и канал работает». */
        s->ch_run[k]++;
        s->ch_ready[k] = (uint8_t)(
            s->ch_ok[k] == 2u && s->rearm[k] == 0u &&
            s->ch_run[k] >= s->track_n &&
            s->span[k] > 2.0f * c->min_span &&
            s->n >= s->pol_lock[k] && w > 0.0f);
        if (s->ch_ready[k]) w_ready += w;

        dn[k] = 0.0f;
        if (w > 0.0f) {
            dn[k]     = d / s->span[k];
            if (dn[k] > c->dn_max) dn[k] = c->dn_max;
            acc      += w * dn[k];               /* выровненный вклад */
            span_ref += w * s->span[k];
            wsum     += w;
            if (s->is_far[k]) { far_s  += w * d; far_w  += w; farn_s += w * dn[k]; }
            else              { near_s += w * d; near_w += w; }
            if (s->is_850[k]) { w850_s += w * d; w850_w += w; }
            else              { w740_s += w * d; w740_w += w; }
        }
    }

    s->wsum = wsum;

    /* ---------------- слияние ---------------- */
    if (wsum > 1e-6f) {
        span_ref /= wsum;
        u = (acc / wsum) * span_ref;             /* обратно в единицы OD */
    } else {
        u = 0.0f;
        fl |= NIRS_F_NO_CHAN;
    }
    s->u_od = u;

    /* ---------------- оценка полярности ближних каналов ----------------
     * Опорный сигнал — нормированный вклад дальних каналов, их знак принят
     * за +1. Для каждого ближнего канала ведётся ковариация его вклада с
     * опорным. Устойчиво отрицательная ковариация означает, что канал
     * меряет ту же активность в обратную сторону: знак переключается, а
     * база и размах канала переучиваются с нуля (ch_ok = 1 — «ждёт покоя»,
     * до этого момента канал в слиянии не участвует).                  */
    /* согласие дальних каналов: используется на следующем шаге как признак
     * покоя для базовой линии и здесь — как опора для оценки полярности */
    s->far_act = (far_w > 0.0f) ? (farn_s / far_w) : 0.0f;
    if (far_w > 0.0f && c->base_rest_thr > 0.0f) {
        float q = s->far_act / c->base_rest_thr;
        q *= q; q *= q; q *= q;               /* (far_act/thr)^8 */
        s->far_g = 1.0f / (1.0f + q);
    } else {
        s->far_g = 0.0f;                      /* нет дальних каналов — как раньше */
    }

    if (c->auto_polarity && far_w > 0.0f) {
        float ref = s->far_act;
        float er, vr;
        s->pol_mref += s->a_pol * (ref - s->pol_mref);
        er = ref - s->pol_mref;
        s->pol_vref += s->a_pol * (er * er - s->pol_vref);
        vr = s->pol_vref;
        for (k = 0; k < nch; ++k) {
            float ec, corr;
            if (s->is_far[k] || s->w[k] <= 0.0f) continue;
            s->pol_mdn[k] += s->a_pol * (dn[k] - s->pol_mdn[k]);
            ec = dn[k] - s->pol_mdn[k];
            s->pol_vdn[k] += s->a_pol * (ec * ec - s->pol_vdn[k]);
            s->pol_cov[k] += s->a_pol * (ec * er - s->pol_cov[k]);
            /* Знак меняется только при уверенной ОТРИЦАТЕЛЬНОЙ корреляции.
             * Нормировка на дисперсии обязательна: абсолютная ковариация
             * зависит от размаха канала, и слабый шумный канал с почти
             * случайным знаком иначе переключался бы туда-сюда.        */
            corr = s->pol_cov[k] / nirs_sqrtf(s->pol_vdn[k] * vr + 1e-20f);
            if (vr > 1e-6f && corr < -c->pol_thr && s->n >= s->pol_lock[k]) {
                s->pol[k]      = -s->pol[k];
                s->rearm[k]    = 1u;                /* знак сменился, база с нуля */
                s->ch_ready[k] = 0u;                /* и канал временно не в счёт */
                s->pol_lock[k] = s->n + s->pol_hold_n;
                s->pol_cov[k]  = 0.0f;
                s->pol_mdn[k]  = 0.0f;
                s->pol_vdn[k]  = 0.0f;
                s->base[k]     = 0.0f;   /* переучить базу и размах */
                s->span[k]     = c->min_span;
                s->od[k]       = 0.0f;
                s->w[k]        = 0.0f;
                s->ch_ok[k]    = 0u;
                fl |= NIRS_F_LOWSIG;
            }
        }
    }

    /* дополнительные компоненты (диагностика / признаки для НС) */
    s->spatial  = ((far_w  > 0.0f) ? far_s  / far_w  : 0.0f)
                - ((near_w > 0.0f) ? near_s / near_w : 0.0f);
    s->spectral = ((w850_w > 0.0f) ? w850_s / w850_w : 0.0f)
                - ((w740_w > 0.0f) ? w740_s / w740_w : 0.0f);

    /* ---------------- lead-компенсатор ---------------- */
    if (s->lead_gain > 0.0f) {
        s->u_lp += s->a_lead * (u - s->u_lp);
        u += s->lead_gain * (u - s->u_lp);
    }

    /* ---------------- асимметричное сглаживание ---------------- */
    a_smooth = (u > s->level) ? s->a_att : s->a_rel;
    s->level += a_smooth * (u - s->level);

    /* ---------------- автокалибровка MVC ----------------
     * Шкалу задаёт устойчивый уровень, а не выброс на фронте.
     *
     * ШКАЛА НЕ ОБНОВЛЯЕТСЯ, ПОКА ДАТЧИК НЕ НА МЫШЦЕ (нет TRACKING).
     * Снятие и переклейка дают на слитом сигнале выброс в 15-20 раз выше
     * настоящего сокращения: на записи S05 u_od доходил до 5.7 при обычном
     * сокращении 0.3, и пиковый детектор поднимал шкалу с 0.28 до 3.4.
     * Дальше она спадала с постоянной 300 с, и ВСЯ ОСТАВШАЯСЯ ЗАПИСЬ (180 с)
     * шла с амплитудой выхода 0.02-0.1 вместо 0.8. Ядро в этот момент
     * прекрасно знает, что датчика на мышце нет, — надо просто этим
     * воспользоваться. Используется состояние с предыдущей выборки: флаг
     * этого шага считается ниже, а задержка в одну выборку роли не играет.
     *
     * Сглаженный уровень mvc_lp тоже замораживается: иначе выброс просто
     * доедет до шкалы на выборку позже.                                */
    if (s->tracking) {
        s->mvc_lp += s->a_mvc_lp * (s->level - s->mvc_lp);
        if (s->mvc_lp > s->mvc) s->mvc += s->a_mvc_up * (s->mvc_lp - s->mvc);
        else                    s->mvc += s->a_mvc_dn * (s->mvc_lp - s->mvc);
    }
    /* Нижняя граница шкалы по размахам каналов.
     *
     * span_ref — взвешенный средний размах канала, и по построению слитый
     * сигнал на полном сокращении приходит примерно к нему же. Значит
     * span_ref уже несёт оценку шкалы, причём готовую через пару секунд, а
     * не через десятки: размах канала сходится за tau_span_up = 2 с.
     * Пиковый детектор без этой подсказки стартует с mvc_min = 0.05 и
     * ползёт вверх, пока не случится сильное сокращение, — на записях
     * S01-S07 уровень выходил на правильный за 23-61 с.
     * Коэффициент mvc_span_k < 1: шкалу лучше чуть занизить (выход упрётся
     * в потолок и это видно по флагу CLIPPED), чем завысить (сокращения
     * покажутся слабыми, и понять это по выходу нельзя).            */
    if (c->mvc_span_k > 0.0f && span_ref > 0.0f) {
        float lo = c->mvc_span_k * span_ref;
        if (s->mvc < lo) s->mvc = lo;
    }
    if (s->mvc < c->mvc_min) s->mvc = c->mvc_min;

    /* ---------------- выход ---------------- */
    o = s->level / s->mvc;
    if (c->deadzone > 0.0f && c->deadzone < 1.0f)
        o = (o - c->deadzone) / (1.0f - c->deadzone);
    if (o < 0.0f) o = 0.0f;
    {   /* мягкое ограничение: 0..1 -> 0..knee линейно, выше — сжатие
         * функцией u/(1+u) (монотонна, ограничена, без libm).          */
        const float kn = c->soft_knee;
        if (kn >= 1.0f) {
            if (o > 1.0f) { o = 1.0f; fl |= NIRS_F_CLIPPED; }
        } else if (o > 1.0f) {
            float u = o - 1.0f;
            o = kn + (1.0f - kn) * (u / (1.0f + u));
            fl |= NIRS_F_CLIPPED;          /* превышен калиброванный максимум */
        } else {
            o *= kn;
        }
    }
    s->out01 = o;
    s->out_v = o * c->out_vmax;

    /* дискретный детектор с гистерезисом */
    if (s->flags & NIRS_F_ACTIVE) { if (o < c->gate_lo) s->flags &= (uint16_t)~NIRS_F_ACTIVE; }
    else                          { if (o > c->gate_hi) s->flags |=  NIRS_F_ACTIVE;          }

    fl |= (uint16_t)(s->flags & NIRS_F_ACTIVE);
    if (s->n >= s->warmup_n) fl |= NIRS_F_READY;

    /* ---------------- TRACKING: датчик на мышце, форма верна ------------
     * Здоровье установки = вес готовых каналов / «сколько их тут обычно».
     * Опорная сумма w_ref поднимается быстро (за постоянную базы) и спадает
     * очень медленно (tau_w_ref), то есть помнит норму для этой установки.
     *
     * Отсюда нужное поведение: выпад ОДНОГО канала отнимает его долю
     * (у пяти рабочих каналов это 0.2) и порога не пробивает, а снятие
     * датчика роняет все каналы разом — health падает в ноль в ту же
     * выборку. Гистерезис health_hi/health_lo убирает дребезг на границе. */
    if (wsum > s->w_ref) s->w_ref += s->a_base_up * (wsum - s->w_ref);
    else                 s->w_ref += s->a_w_ref   * (wsum - s->w_ref);

    if (s->w_ref > 1e-6f) {
        s->health = w_ready / s->w_ref;
        if (s->health > 1.0f) s->health = 1.0f;
    } else {
        s->health = 0.0f;
    }

    if (s->tracking) { if (s->health < c->health_lo) s->tracking = 0u; }
    else             { if (s->health > c->health_hi) s->tracking = 1u; }

    if (s->tracking && (fl & NIRS_F_READY) != 0u && (fl & NIRS_F_NO_CHAN) == 0u)
        fl |= NIRS_F_TRACKING;

    /* ---------------- STABLE: устоялся ещё и масштаб --------------------
     * Уровень выхода задаёт автокалибровка максимума. Пока она ползёт,
     * одно и то же усилие даёт разные числа. Проверяем прямо это: за окно
     * mvc_settle_s шкала сдвинулась меньше чем на mvc_settle_thr, и она
     * поднята настоящим сокращением, а не сидит на нижнем пределе.
     * Измерено на S01-S04: уровень сходится за 23-61 с — столько эта
     * проверка и держит флаг опущенным, не больше и не меньше. */
    if (s->n - s->mvc_mark_n >= s->mvc_settle_n) {
        float rel = (s->mvc > 1e-6f) ? ((s->mvc - s->mvc_mark) / s->mvc) : 1.0f;
        if (rel < 0.0f) rel = -rel;
        s->mvc_settled = (uint8_t)(rel < c->mvc_settle_thr &&
                                   s->mvc > c->mvc_min * 1.2f);
        s->mvc_mark   = s->mvc;
        s->mvc_mark_n = s->n;
    }
    if (!s->tracking) s->mvc_settled = 0u;
    if ((fl & NIRS_F_TRACKING) != 0u && s->mvc_settled) fl |= NIRS_F_STABLE;

    /* приблизительный ход адаптации — для полоски прогресса */
    if (fl & NIRS_F_STABLE)        s->stable_progress = 1.0f;
    else if (fl & NIRS_F_TRACKING) {
        float q = (s->mvc_settle_n > 0u)
                ? (float)(s->n - s->mvc_mark_n) / (float)s->mvc_settle_n : 1.0f;
        if (q > 1.0f) q = 1.0f;
        s->stable_progress = 0.5f + 0.5f * q;
    } else {
        s->stable_progress = 0.5f * s->health;
    }

    s->flags = fl;

    return o;
}

/* ------------------------------------------------------------------ */
float    nirs_out_volts(const nirs_t *s) { return s->out_v; }
uint16_t nirs_flags    (const nirs_t *s) { return s->flags; }

int   nirs_tracking       (const nirs_t *s)
{
    return (s->flags & NIRS_F_TRACKING) != 0u;
}
int   nirs_stable         (const nirs_t *s)
{
    return (s->flags & NIRS_F_STABLE) != 0u;
}
float nirs_health         (const nirs_t *s) { return s->health; }
float nirs_stable_progress(const nirs_t *s) { return s->stable_progress; }

uint16_t nirs_out_dac12(const nirs_t *s)
{
    float f = s->out01 * 4095.0f + 0.5f;
    if (f < 0.0f)    f = 0.0f;
    if (f > 4095.0f) f = 4095.0f;
    return (uint16_t)f;
}

/* ------------------------------------------------------------------ */
void nirs_nn_features(const nirs_t *s, float *out)
{
    int k;
    for (k = 0; k < s->nch; ++k) {
        float sp = s->span[k];
        if (sp < s->cfg.min_span) sp = s->cfg.min_span;
        out[k] = (s->w[k] > 0.0f) ? (s->od[k] / sp) : 0.0f;
        /* Ограничение сверху. span — пиковый детектор с постоянной 2 с, и
         * на резком выбросе od/span успевает уйти далеко за единицу (на
         * реальных записях до 8, а в первые секунды до 26). Сети такой
         * хвост не нужен, а на квантованной модели он съедает всю шкалу.
         * Выше потолка значение и так недостоверно: там оно очень
         * чувствительно к мелким различиям в истории span, и две
         * реализации одной арифметики (C и C#) там расходятся. Признаки
         * для обучения берите тем же кодом, что стоит в прошивке. */
        if (out[k] > 4.0f) out[k] = 4.0f;
    }
}

void nirs_get_extra(const nirs_t *s, nirs_extra_t *e)
{
    int k;
    for (k = 0; k < NIRS_MAX_CH; ++k) {
        e->od[k]    = s->od[k];
        e->w[k]     = s->w[k];
        e->pol[k]   = s->pol[k];
        e->span[k]  = s->span[k];
        e->noise[k] = s->noise[k];
    }
    e->u_od     = s->u_od;
    e->level    = s->level;
    e->mvc      = s->mvc;
    e->spatial  = s->spatial;
    e->spectral = s->spectral;
    e->n_valid  = s->wsum;
    e->stable_progress = s->stable_progress;
}

void nirs_calibrate_mvc(nirs_t *s)
{
    /* берём сглаженный уровень: калибровка не должна попасть на выброс */
    float v = (s->mvc_lp > s->level) ? s->mvc_lp : s->level;
    s->mvc = (v > s->cfg.mvc_min) ? v : s->cfg.mvc_min;
}

void nirs_set_mvc(nirs_t *s, float mvc_od)
{
    s->mvc    = (mvc_od > s->cfg.mvc_min) ? mvc_od : s->cfg.mvc_min;
    s->mvc_lp = s->mvc;
}
