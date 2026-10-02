/**
 * tests/test_pid.c — перевірка PID-регулятора на моделі RC-кола (на ПК, без плати).
 *
 * Збірка і запуск:
 *   gcc -std=c11 -Wall -Wextra -I../Core/Inc test_pid.c ../Core/Src/alg_pid.c -lm -o tp && ./tp
 *
 * Модель — саме ваш макет, з навантаженням першого ступеня другим:
 *   ШІМ·3,3 В ─ R1 10 кОм ─┬─ R2 100 кОм ─┬─► АЦП
 *                          C1 51 мкФ       C2 1 мкФ
 * АЦП: 12 біт, шум ±2 молодші розряди. Регулятор: такт 10 мс, як на платі.
 * Коефіцієнти — ті самі, що в app_config.h (CFG_PID_KP/KI/KD).
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "alg_pid.h"

#define KP 2.0f
#define KI 3.0f
#define KD 0.2f
#define TAU_D 0.02f
#define TS 0.01f

static int failures;
#define CHECK(cond, ...) do { if (!(cond)) { failures++; printf("  FAIL: "); \
    printf(__VA_ARGS__); printf("\n"); } } while (0)

/* ------------------------------ модель RC-кола ------------------------------ */
typedef struct { float v1, v2, c1, r_load, last_u; } Plant_t;   /* r_load: навантаження, 0 = немає;
                                                                   last_u: ШІМ попереднього такту, <0 = ще не було */

static void plant_step(Plant_t *p, float duty, float dt_total)
{
    const float R1 = 10e3f, R2 = 100e3f, C2 = 1e-6f, VCC = 3.3f, h = 1e-5f;
    for (int i = 0; i < (int)(dt_total / h + 0.5f); i++) {
        float i1 = (duty * VCC - p->v1) / R1;
        float i2 = (p->v1 - p->v2) / R2;
        float il = (p->r_load > 0) ? p->v2 / p->r_load : 0.0f;
        p->v1 += h * (i1 - i2) / p->c1;
        p->v2 += h * (i2 - il) / C2;
    }
}

static float adc_read(const Plant_t *p)
{
    float noise = (float)((rand() % 5) - 2) * (3.3f / 4096.0f);
    return roundf(p->v2 / 3.3f * 4095.0f) * 3.3f / 4095.0f + noise;
}

/* Прогін: повертає характеристики перехідного процесу для кроку уставки */
typedef struct { float overshoot_pct, settle_s, final_v, max_du; } StepResult_t;

static StepResult_t run_step(PID_Controller_t *pid, Plant_t *p, float sp, float seconds)
{
    StepResult_t r = { 0, -1, 0, 0 };
    float start = p->v2, peak = start;
    int in_band = 0;
    for (int k = 0; k < (int)(seconds / TS); k++) {
        float u = PID_Update(pid, sp, adc_read(p));
        if (p->last_u >= 0 && fabsf(u - p->last_u) > r.max_du) r.max_du = fabsf(u - p->last_u);
        p->last_u = u;
        plant_step(p, u, TS);
        float t = (k + 1) * TS;
        if ((sp > start && p->v2 > peak) || (sp < start && p->v2 < peak)) peak = p->v2;
        if (fabsf(p->v2 - sp) > 0.02f * fabsf(sp - start) + 0.005f) { in_band = 0; r.settle_s = -1; }
        else if (!in_band) { in_band = 1; r.settle_s = t; }
    }
    float span = fabsf(sp - start);
    r.overshoot_pct = span > 0 ? (fabsf(peak - start) - span) / span * 100.0f : 0;
    if (r.overshoot_pct < 0) r.overshoot_pct = 0;
    r.final_v = p->v2;
    return r;
}

static void fresh(PID_Controller_t *pid, Plant_t *p, float c1)
{
    PID_Init(pid, KP, KI, KD, TS, TAU_D, 0.0f, 1.0f);
    *p = (Plant_t){ 0, 0, c1, 0, -1 };
}

/* ------------------------------------ тести ------------------------------------ */
static void test_step_response(void)
{
    puts("крок 0 → 1,5 В (C1 = 51 мкФ)");
    PID_Controller_t pid; Plant_t p; fresh(&pid, &p, 51e-6f);
    StepResult_t r = run_step(&pid, &p, 1.5f, 5.0f);
    printf("  перерегулювання %.1f %%, встановлення %.2f с, кінцева %.3f В\n",
           r.overshoot_pct, r.settle_s, r.final_v);
    CHECK(r.overshoot_pct < 5.0f, "перерегулювання %.1f %% > 5 %%", r.overshoot_pct);
    CHECK(r.settle_s > 0 && r.settle_s < 1.5f, "встановлення %.2f с", r.settle_s);
    CHECK(fabsf(r.final_v - 1.5f) < 0.01f, "статична похибка %.3f В", r.final_v - 1.5f);
}

static void test_encoder_steps(void)
{
    puts("кроки енкодера: 1,5 → 1,6 → 1,7 → 1,6 В (по 100 мВ)");
    PID_Controller_t pid; Plant_t p; fresh(&pid, &p, 51e-6f);
    run_step(&pid, &p, 1.5f, 5.0f);
    const float sps[] = { 1.6f, 1.7f, 1.6f };
    for (int i = 0; i < 3; i++) {
        StepResult_t r = run_step(&pid, &p, sps[i], 3.0f);
        CHECK(fabsf(r.final_v - sps[i]) < 0.01f, "не вийшов на %.1f В (%.3f)", sps[i], r.final_v);
        CHECK(r.settle_s > 0 && r.settle_s < 1.5f, "крок до %.1f В: %.2f с", sps[i], r.settle_s);
    }
}

static void test_no_derivative_kick(void)
{
    puts("крок уставки 1,0 → 1,2 В не дає 'удару' D-складової");
    PID_Controller_t pid; Plant_t p; fresh(&pid, &p, 51e-6f);
    run_step(&pid, &p, 1.0f, 5.0f);
    StepResult_t r = run_step(&pid, &p, 1.2f, 2.0f);
    /* Очікуємо стрибок ШІМ лише від P: Kp·Δsp = 2·0,2 = 0,4.
       Якби D рахувалась по похибці, додалось би ще 2·Kd/(2τ+T)·Δsp = 8·0,2 = 1,6
       → ШІМ миттєво в 100 % (стрибок ≈ 0,7). */
    printf("  найбільший стрибок ШІМ за такт: %.2f\n", r.max_du);
    CHECK(r.max_du <= KP * 0.2f + 0.1f, "стрибок ШІМ %.2f (очікував ≤ %.2f)", r.max_du, KP * 0.2f + 0.1f);
}

static void test_anti_windup(void)
{
    puts("anti-windup: 5 с недосяжної уставки, потім повернення — без великого викиду");
    PID_Controller_t pid; Plant_t p; fresh(&pid, &p, 51e-6f);
    run_step(&pid, &p, 1.0f, 5.0f);
    p.r_load = 100e3f;                   /* збурення: максимум на виході ≈ 1,57 В */
    run_step(&pid, &p, 2.5f, 5.0f);      /* ШІМ у насиченні 100 % */
    CHECK(pid.integ <= 1.0f + 1e-6f, "інтегратор вийшов за межі: %.2f", pid.integ);
    p.r_load = 0;                        /* прибрали навантаження */
    StepResult_t r = run_step(&pid, &p, 1.0f, 4.0f);
    printf("  після насичення: встановлення %.2f с, кінцева %.3f В\n", r.settle_s, r.final_v);
    CHECK(fabsf(r.final_v - 1.0f) < 0.01f, "не повернувся на 1,0 В");
    CHECK(r.settle_s > 0 && r.settle_s < 2.5f, "довго виходив із насичення: %.2f с", r.settle_s);
}

static void test_disturbance_rejection(void)
{
    puts("збурення (навантаження 470 кОм на виході) — I-складова повертає на уставку");
    PID_Controller_t pid; Plant_t p; fresh(&pid, &p, 51e-6f);
    run_step(&pid, &p, 1.5f, 5.0f);
    p.r_load = 470e3f;
    StepResult_t r = run_step(&pid, &p, 1.5f, 5.0f);
    CHECK(fabsf(r.final_v - 1.5f) < 0.01f, "після збурення похибка %.3f В", r.final_v - 1.5f);
}

static void test_capacitor_tolerance(void)
{
    puts("розкид ємності ±20 % і заміна на 10 мкФ — ті самі коефіцієнти працюють");
    const float cs[] = { 41e-6f, 61e-6f, 10e-6f };
    for (int i = 0; i < 3; i++) {
        PID_Controller_t pid; Plant_t p; fresh(&pid, &p, cs[i]);
        StepResult_t r = run_step(&pid, &p, 1.5f, 5.0f);
        printf("  C1 = %2.0f мкФ: перерегулювання %.1f %%, встановлення %.2f с\n",
               cs[i] * 1e6f, r.overshoot_pct, r.settle_s);
        CHECK(r.overshoot_pct < 5.0f && r.settle_s > 0 && r.settle_s < 1.5f, "C1 = %.0f мкФ", cs[i] * 1e6f);
    }
}

static void test_p_only_has_offset(void)
{
    puts("для звіту: лише P (Ki = Kd = 0) — статична похибка залишається");
    PID_Controller_t pid; Plant_t p;
    PID_Init(&pid, KP, 0, 0, TS, TAU_D, 0.0f, 1.0f);
    p = (Plant_t){ 0, 0, 51e-6f, 0, -1 };
    StepResult_t r = run_step(&pid, &p, 1.5f, 8.0f);
    printf("  Kp = %.1f: вихід %.3f В замість 1,500 В\n", KP, r.final_v);
    CHECK(r.final_v < 1.45f, "очікував помітну статичну похибку");
}

int main(void)
{
    srand(1);
    test_step_response();
    test_encoder_steps();
    test_no_derivative_kick();
    test_anti_windup();
    test_disturbance_rejection();
    test_capacitor_tolerance();
    test_p_only_has_offset();

    if (failures) { printf("\nПРОВАЛЕНО перевірок: %d\n", failures); return 1; }
    puts("\nУсі тести PID пройдено.");
    return 0;
}
