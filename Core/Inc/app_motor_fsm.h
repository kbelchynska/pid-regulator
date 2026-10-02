/* ============================================================================
 * app_motor_fsm.h — СКІНЧЕННИЙ АВТОМАТ СТАНІВ МОТОРА (патерн State)
 * ============================================================================
 * НАВІЩО: уся логіка "коли вмикати мотор, коли реверс, коли стоп" — в одному
 *   місці і без плутанини if-ів. Головна гарантія: напрямок НІКОЛИ не
 *   змінюється на ходу — лише через зупинку і паузу 150 мс.
 *
 * ОСОБЛИВІСТЬ: модуль "чистий". Час приходить параметром now, мотор — через
 *   інтерфейс Motor_t. Ні HAL, ні FreeRTOS → тестується на ПК
 *   (tests/test_motor_fsm.c, 100 000 випадкових подій).
 *
 * ДІАГРАМА СТАНІВ:
 *               STEP_CW                         STEP_CCW
 *   ┌─────────► RUN_CW ─────────┐   ┌──────── RUN_CCW ◄─────────┐
 *   │             │ STOP/таймаут │   │ STOP/таймаут │            │
 *   │             ▼   STEP_CCW   ▼   ▼   STEP_CW    ▼            │
 * IDLE ◄───────────────────── SWITCHING ───────────────────────┘
 *   ▲    STOP              (мотор стоїть, пауза 150 мс,
 *                           потім вмикає останній обраний напрямок)
 * ========================================================================== */
#ifndef APP_MOTOR_FSM_H
#define APP_MOTOR_FSM_H

#include "drv_motor.h"        // лише ІНТЕРФЕЙС мотора, не реле
#include <stdbool.h>
#include <stdint.h>

typedef enum {                // СТАНИ
    MS_IDLE = 0,              // мотор стоїть
    MS_RUN_CW,                // крутиться за годинниковою
    MS_RUN_CCW,               // крутиться проти
    MS_SWITCHING,             // зупинено, чекаємо паузу перед реверсом
    MS_COUNT                  // кількість станів (4)
} MotorState_t;

typedef enum {                // ПОДІЇ
    MEV_STEP_CW = 0,          // енкодер клацнув за годинниковою
    MEV_STEP_CCW,             // енкодер клацнув проти
    MEV_STOP,                 // кнопка — аварійна зупинка
    MEV_TICK                  // такт кожні 10 мс (щоб рахувати час)
} MotorEvent_t;

typedef void (*MotorStateCb_t)(void *ctx, MotorState_t new_state);  // "повідом, коли стан змінився"

typedef struct {
    uint32_t deadtime_ms;      // пауза між напрямками (150)
    uint32_t idle_timeout_ms;  // стоп, якщо енкодер не крутять (600); 0 = вимкнено
} MotorFsmConfig_t;

typedef struct {               // ДАНІ АВТОМАТА
    MotorState_t     state;         // поточний стан
    MotorDir_t       target;        // куди поїдемо після паузи
    uint32_t         t_last_step;   // коли було останнє клацання, мс
    uint32_t         t_stopped;     // коли мотор востаннє зупинили, мс
    MotorFsmConfig_t cfg;           // налаштування
    Motor_t          motor;         // абстрактний мотор
    MotorStateCb_t   on_state;      // кого повідомити про зміну стану (Observer)
    void            *cb_ctx;
} MotorFsm_t;

void         MotorFsm_Init(MotorFsm_t *f, const MotorFsmConfig_t *cfg, Motor_t motor,
                           MotorStateCb_t on_state, void *cb_ctx, uint32_t now_ms);
void         MotorFsm_Dispatch(MotorFsm_t *f, MotorEvent_t ev, uint32_t now_ms); // обробити подію
MotorState_t MotorFsm_State(const MotorFsm_t *f);                                // поточний стан
const char  *MotorFsm_StateName(MotorState_t s);                                 // назва текстом

#endif /* APP_MOTOR_FSM_H */
