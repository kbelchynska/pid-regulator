/* ============================================================================
 * app_motor_fsm.c — реалізація автомата станів
 * ============================================================================
 * ПАТЕРН STATE У C: кожному стану — своя функція-обробник, таблиця
 *   STATE_HANDLERS[] вибирає потрібну за поточним станом. Додати новий стан =
 *   рядок в enum + одна функція + рядок у таблиці, без гігантського switch.
 *
 * ЧОМУ (uint32_t)(now - f->t_stopped), а не now >= t_stopped + 150:
 *   лічильник часу в мс переповнюється через ~49 діб (2^32 мс). Різниця
 *   беззнакових чисел дає правильний результат НАВІТЬ через переповнення:
 *   (5 − 4294967290) як uint32 = 11. Порівняння з сумою в цей момент
 *   зламалося б. Це перевіряє окремий тест.
 * ========================================================================== */
#include "app_motor_fsm.h"
#include <stddef.h>

typedef void (*StateHandler_t)(MotorFsm_t *f, MotorEvent_t ev, uint32_t now);
// ↑ тип "обробника стану": функція, яка вирішує, що робити з подією в цьому стані

/* ----------------------------- допоміжні дії ----------------------------- */
static void enter(MotorFsm_t *f, MotorState_t s)      // ПЕРЕЙТИ в новий стан
{
    if (f->state == s) return;                        // уже в ньому — нічого не робити
    f->state = s;                                     // змінити стан
    if (f->on_state) f->on_state(f->cb_ctx, s);       // повідомити (задача мотора передасть на шину →
}                                                     // індикатор увімкне потрібний світлодіод)

static void motor_stop(MotorFsm_t *f, uint32_t now)   // ЗУПИНИТИ мотор
{
    Motor_Stop(&f->motor);                            // через ІНТЕРФЕЙС — автомат не знає про реле
    f->t_stopped = now;                               // запам'ятати момент зупинки (для паузи)
}

/* t_last_step тут НЕ оновлюємо: тайм-аут рахується від останнього клацання
   енкодера, а не від моменту, коли після паузи нарешті ввімкнули мотор. */
static void motor_start(MotorFsm_t *f, MotorDir_t dir)    // ЗАПУСТИТИ мотор
{
    Motor_Run(&f->motor, dir);                            // увімкнути в потрібному напрямку
    enter(f, dir == MOTOR_DIR_CW ? MS_RUN_CW : MS_RUN_CCW);   // і перейти у відповідний стан
}

static bool is_step(MotorEvent_t ev, MotorDir_t *dir)     // це клацання? Якщо так — в який бік
{
    if (ev == MEV_STEP_CW)  { *dir = MOTOR_DIR_CW;  return true; }  // *dir = … записує результат
    if (ev == MEV_STEP_CCW) { *dir = MOTOR_DIR_CCW; return true; }  // у змінну того, хто викликав
    return false;
}

static void request_direction(MotorFsm_t *f, MotorDir_t dir, uint32_t now)  // "хочу крутити в dir"
{
    f->t_last_step = now;                                        // оновити час останнього клацання
    if ((uint32_t)(now - f->t_stopped) >= f->cfg.deadtime_ms) {  // минуло ≥ 150 мс від зупинки?
        motor_start(f, dir);                                     // так → стартуємо одразу
    } else {
        f->target = dir;                                         // ні (мотор ще на вибігу) →
        enter(f, MS_SWITCHING);                                  // запам'ятати напрямок і чекати
    }
}

/* ------------------------------- стани ------------------------------- */
static void st_idle(MotorFsm_t *f, MotorEvent_t ev, uint32_t now)    // СТАН: стоїть
{
    MotorDir_t dir;
    if (is_step(ev, &dir)) request_direction(f, dir, now);           // клацання → поїхали
    /* STOP і TICK у стані спокою нічого не роблять */
}

static void st_running(MotorFsm_t *f, MotorEvent_t ev, uint32_t now) // СТАН: крутиться
{
    MotorDir_t cur = (f->state == MS_RUN_CW) ? MOTOR_DIR_CW : MOTOR_DIR_CCW;  // поточний напрямок
    MotorDir_t dir;

    if (is_step(ev, &dir)) {                   // прийшло клацання:
        if (dir == cur) {                      //   у той самий бік →
            f->t_last_step = now;              //   крутимо далі, оновити час
        } else {                               //   у протилежний бік → РЕВЕРС:
            motor_stop(f, now);                //   1) зупинити
            f->t_last_step = now;
            f->target = dir;                   //   2) запам'ятати новий напрямок
            enter(f, MS_SWITCHING);            //   3) чекати паузу
        }
        return;
    }

    if (ev == MEV_STOP) {                      // кнопка →
        motor_stop(f, now);                    // негайна зупинка
        enter(f, MS_IDLE);
        return;
    }

    if (ev == MEV_TICK && f->cfg.idle_timeout_ms != 0 &&                 // такт, тайм-аут увімкнено
        (uint32_t)(now - f->t_last_step) >= f->cfg.idle_timeout_ms) {    // і 600 мс не крутили →
        motor_stop(f, now);                                              // зупинка
        enter(f, MS_IDLE);
    }
}

static void st_switching(MotorFsm_t *f, MotorEvent_t ev, uint32_t now) // СТАН: пауза перед реверсом
{
    MotorDir_t dir;

    if (is_step(ev, &dir)) {                   // передумали під час паузи →
        f->target = dir;                       // беремо ОСТАННІЙ обраний напрямок
        f->t_last_step = now;
        return;
    }

    if (ev == MEV_STOP) {                      // кнопка → мотор уже стоїть,
        enter(f, MS_IDLE);                     // просто в IDLE
        return;
    }

    if (ev == MEV_TICK && (uint32_t)(now - f->t_stopped) >= f->cfg.deadtime_ms) {
        motor_start(f, f->target);             // пауза минула → стартуємо в обраний бік
    }
}

/* --------------------------- ТАБЛИЦЯ ПЕРЕХОДІВ --------------------------- */
static const StateHandler_t STATE_HANDLERS[MS_COUNT] = {   // масив вказівників на функції,
    [MS_IDLE]      = st_idle,                              // індекс = стан.
    [MS_RUN_CW]    = st_running,                           // [індекс] = … — ініціалізація
    [MS_RUN_CCW]   = st_running,                           // за індексом (C99).
    [MS_SWITCHING] = st_switching,                         // Один обробник на два RUN-стани
};

/* ------------------------------ публічне API ------------------------------ */
void MotorFsm_Init(MotorFsm_t *f, const MotorFsmConfig_t *cfg, Motor_t motor,
                   MotorStateCb_t on_state, void *cb_ctx, uint32_t now_ms)
{
    f->cfg      = *cfg;                   // скопіювати налаштування
    f->motor    = motor;                  // абстрактний мотор
    f->on_state = on_state;               // кого повідомляти
    f->cb_ctx   = cb_ctx;
    f->state    = MS_IDLE;                // старт — стоїть
    f->target   = MOTOR_DIR_CW;
    f->t_last_step = now_ms;
    f->t_stopped   = now_ms - cfg->deadtime_ms;   // ТРЮК: "зупинили 150 мс тому" → перше
                                                  // клацання після старту вмикає мотор одразу
    Motor_Stop(&f->motor);                        // гарантовано вимкнений
}

void MotorFsm_Dispatch(MotorFsm_t *f, MotorEvent_t ev, uint32_t now_ms)
{
    if (f->state < MS_COUNT && STATE_HANDLERS[f->state] != NULL)  // захист від некоректного стану
        STATE_HANDLERS[f->state](f, ev, now_ms);                  // ВИКЛИКАТИ обробник поточного
}                                                                 // стану. Увесь автомат — один рядок!

MotorState_t MotorFsm_State(const MotorFsm_t *f)
{
    return f->state;
}

const char *MotorFsm_StateName(MotorState_t s)
{
    static const char *const NAMES[MS_COUNT] = { "IDLE", "RUN_CW", "RUN_CCW", "SWITCHING" };
    return (s < MS_COUNT) ? NAMES[s] : "?";       // назва стану текстом (для відлагодження)
}
