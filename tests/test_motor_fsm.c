/**
 * tests/test_motor_fsm.c — модульні тести автомата станів на ПК (без плати).
 *
 * Збірка і запуск (Linux/macOS/MinGW):
 *   gcc -std=c11 -Wall -Wextra -I../Core/Inc test_motor_fsm.c ../Core/Src/app_motor_fsm.c -o t && ./t
 *
 * Фейковий мотор (патерн Test Double / Fake) реалізує той самий інтерфейс Motor_t,
 * що й реальний драйвер на реле, і перевіряє інваріанти безпеки:
 *  - напрямок ніколи не змінюється "на ходу" — лише через зупинку;
 *  - між зупинкою і стартом у ІНШИЙ бік минає щонайменше DEADTIME.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "app_motor_fsm.h"

#define DEADTIME 150u
#define TIMEOUT  600u

/* ------------------------------ фейковий мотор ------------------------------ */
typedef struct {
    bool       running;
    MotorDir_t dir;
    MotorDir_t last_dir;
    bool       ever_ran;
    uint32_t   t_stop;
    uint32_t  *clock;
    int        runs, stops;
} FakeMotor_t;

static int failures;
#define CHECK(cond, msg) do { if (!(cond)) { failures++; \
    printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, msg); } } while (0)

static void fake_run(void *self, MotorDir_t dir)
{
    FakeMotor_t *m = self;
    CHECK(!(m->running && m->dir != dir), "реверс на ходу без зупинки");
    if (!m->running && m->ever_ran && dir != m->last_dir)
        CHECK(*m->clock - m->t_stop >= DEADTIME, "реверс раніше, ніж минула пауза");
    m->running = true; m->dir = dir; m->last_dir = dir; m->ever_ran = true; m->runs++;
}
static void fake_stop(void *self)
{
    FakeMotor_t *m = self;
    if (m->running) m->t_stop = *m->clock;
    m->running = false; m->stops++;
}
static const MotorOps_t FAKE_OPS = { fake_run, fake_stop };

/* ------------------------------ тестове оточення ------------------------------ */
typedef struct {
    uint32_t     t;
    FakeMotor_t  motor;
    MotorFsm_t   fsm;
    MotorState_t notified[64];
    int          n_notified;
} Env_t;

static void on_state(void *ctx, MotorState_t s)
{
    Env_t *e = ctx;
    if (e->n_notified < 64) e->notified[e->n_notified++] = s;
}

static void env_init(Env_t *e, uint32_t timeout)
{
    memset(e, 0, sizeof *e);
    e->t = 1000;
    e->motor.clock = &e->t;
    MotorFsmConfig_t cfg = { .deadtime_ms = DEADTIME, .idle_timeout_ms = timeout };
    Motor_t m = { &FAKE_OPS, &e->motor };
    MotorFsm_Init(&e->fsm, &cfg, m, on_state, e, e->t);
}

/* просуває час із тактами по 10 мс, як це робить задача мотора */
static void advance(Env_t *e, uint32_t ms)
{
    for (uint32_t i = 0; i < ms; i += 10) {
        e->t += 10;
        MotorFsm_Dispatch(&e->fsm, MEV_TICK, e->t);
    }
}
static void ev(Env_t *e, MotorEvent_t x) { MotorFsm_Dispatch(&e->fsm, x, e->t); }
static MotorState_t st(Env_t *e) { return MotorFsm_State(&e->fsm); }

/* ------------------------------------ тести ------------------------------------ */
static void test_start_cw_immediately(void)
{
    puts("старт за годинниковою — одразу, без паузи");
    Env_t e; env_init(&e, TIMEOUT);
    ev(&e, MEV_STEP_CW);
    CHECK(st(&e) == MS_RUN_CW, "очікував RUN_CW");
    CHECK(e.motor.running && e.motor.dir == MOTOR_DIR_CW, "мотор мав крутитись CW");
    CHECK(e.n_notified == 1 && e.notified[0] == MS_RUN_CW, "підписник мав отримати RUN_CW");
}

static void test_keeps_running_while_turning(void)
{
    puts("крутиться, поки крутять енкодер; зупиняється після тайм-ауту");
    Env_t e; env_init(&e, TIMEOUT);
    ev(&e, MEV_STEP_CCW);
    for (int i = 0; i < 10; i++) { advance(&e, 200); ev(&e, MEV_STEP_CCW); }  /* 2 с кручення */
    CHECK(st(&e) == MS_RUN_CCW && e.motor.running, "мав крутитись увесь час");
    advance(&e, TIMEOUT - 20);
    CHECK(st(&e) == MS_RUN_CCW, "ще рано зупинятись");
    advance(&e, 30);
    CHECK(st(&e) == MS_IDLE && !e.motor.running, "мав зупинитись після тайм-ауту");
}

static void test_reverse_goes_through_deadtime(void)
{
    puts("реверс: стоп → пауза 150 мс → інший бік");
    Env_t e; env_init(&e, TIMEOUT);
    ev(&e, MEV_STEP_CW);
    advance(&e, 100);
    ev(&e, MEV_STEP_CCW);
    CHECK(st(&e) == MS_SWITCHING && !e.motor.running, "мав зупинитись і чекати");
    advance(&e, DEADTIME - 20);
    CHECK(st(&e) == MS_SWITCHING, "пауза ще не минула");
    advance(&e, 30);
    CHECK(st(&e) == MS_RUN_CCW && e.motor.dir == MOTOR_DIR_CCW, "після паузи — CCW");
}

static void test_change_mind_during_pause(void)
{
    puts("передумали під час паузи — їде в останній обраний бік");
    Env_t e; env_init(&e, TIMEOUT);
    ev(&e, MEV_STEP_CW);
    ev(&e, MEV_STEP_CCW);
    advance(&e, 50);
    ev(&e, MEV_STEP_CW);
    advance(&e, DEADTIME + 10);
    CHECK(st(&e) == MS_RUN_CW, "мав поїхати CW");
}

static void test_restart_after_stop_respects_deadtime(void)
{
    puts("зупинився від тайм-ауту і одразу крутять у інший бік — теж через паузу");
    Env_t e; env_init(&e, TIMEOUT);
    ev(&e, MEV_STEP_CW);
    advance(&e, TIMEOUT + 10);
    CHECK(st(&e) == MS_IDLE, "мав зупинитись");
    ev(&e, MEV_STEP_CCW);                       /* одразу після зупинки */
    CHECK(st(&e) == MS_SWITCHING, "мав чекати паузу — мотор ще на вибігу");
    advance(&e, DEADTIME);
    CHECK(st(&e) == MS_RUN_CCW, "після паузи — CCW");
}

static void test_stop_button(void)
{
    puts("кнопка стоп — з будь-якого стану");
    Env_t e; env_init(&e, 0);                   /* тайм-аут вимкнено: крутиться до стопу */
    ev(&e, MEV_STEP_CW);
    advance(&e, 5000);
    CHECK(st(&e) == MS_RUN_CW, "без тайм-ауту мав крутитись далі");
    ev(&e, MEV_STOP);
    CHECK(st(&e) == MS_IDLE && !e.motor.running, "STOP з RUN");
    ev(&e, MEV_STEP_CCW);
    ev(&e, MEV_STOP);
    CHECK(st(&e) == MS_IDLE && !e.motor.running, "STOP із SWITCHING");
    advance(&e, 1000);
    CHECK(!e.motor.running, "після STOP не мав сам поїхати");
}

static void test_random_stress(void)
{
    puts("стрес: 100 000 випадкових подій — інваріанти безпеки не порушено");
    Env_t e; env_init(&e, TIMEOUT);
    srand(12345);
    for (int i = 0; i < 100000; i++) {
        int r = rand() % 10;
        if      (r < 4) ev(&e, MEV_STEP_CW);
        else if (r < 8) ev(&e, MEV_STEP_CCW);
        else if (r < 9) ev(&e, MEV_STOP);
        advance(&e, (uint32_t)(rand() % 5) * 10);
        bool run_state = (st(&e) == MS_RUN_CW || st(&e) == MS_RUN_CCW);
        if (run_state != e.motor.running) {
            CHECK(0, "стан автомата не збігається з мотором");
            break;
        }
    }
    printf("  (стартів: %d, зупинок: %d)\n", e.motor.runs, e.motor.stops);
}

static void test_tick_wraparound(void)
{
    puts("переповнення лічильника часу (через ~49 діб роботи)");
    Env_t e; env_init(&e, TIMEOUT);
    e.t = 0xFFFFFF00u;
    ev(&e, MEV_STEP_CW);
    advance(&e, 100);                           /* час перейшов через 0 */
    ev(&e, MEV_STEP_CCW);
    advance(&e, DEADTIME + 10);
    CHECK(st(&e) == MS_RUN_CCW, "після переповнення все мало працювати");
}

int main(void)
{
    test_start_cw_immediately();
    test_keeps_running_while_turning();
    test_reverse_goes_through_deadtime();
    test_change_mind_during_pause();
    test_restart_after_stop_respects_deadtime();
    test_stop_button();
    test_random_stress();
    test_tick_wraparound();

    if (failures) { printf("\nПРОВАЛЕНО перевірок: %d\n", failures); return 1; }
    puts("\nУсі тести пройдено.");
    return 0;
}
