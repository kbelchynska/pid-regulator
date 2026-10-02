/* ============================================================================
 * app_motor_ctrl.c — задача-власник мотора
 * ============================================================================
 * ЦИКЛ ЗАДАЧІ: чекати подію не довше 10 мс → обробити її → завжди додати
 *   такт MEV_TICK (щоб таймери паузи і тайм-ауту йшли навіть тоді, коли
 *   енкодер крутять безперервно).
 *
 * СТАТИЧНА ПАМ'ЯТЬ: xTaskCreateStatic / xQueueCreateStatic беруть пам'ять із
 *   НАШИХ масивів, які виділив компілятор. Розмір видно в Build Analyzer,
 *   купа FreeRTOS не потрібна, фрагментації немає.
 * ========================================================================== */
#include "app_motor_ctrl.h"
#include "app_config.h"
#include "svc_event_bus.h"

#include "FreeRTOS.h"          // ядро FreeRTOS
#include "task.h"              // задачі
#include "queue.h"             // черги

#define CTRL_STACK_WORDS  256u  // стек задачі: 256 слів × 4 байти = 1 КБ
#define CTRL_QUEUE_LEN    16u   // черга на 16 подій

/* --------- СТАТИЧНА ПАМ'ЯТЬ: виділяє компілятор, а не купа FreeRTOS --------- */
static StaticTask_t  tcb;                     // TCB — "паспорт" задачі (стан, пріоритет, вказівник стеку)
static StackType_t   stack[CTRL_STACK_WORDS]; // стек задачі: локальні змінні, адреси повернення
static StaticQueue_t qcb;                     // службова структура черги
static uint8_t       qbuf[CTRL_QUEUE_LEN * sizeof(MotorEvent_t)];   // буфер для 16 подій
static QueueHandle_t queue;                   // "номерок" черги для xQueueSend/Receive

static MotorFsm_t         fsm;                // сам автомат станів (приватний — лише ця задача)
static volatile MotorState_t published_state = MS_IDLE;   // останній стан (для GetState)
static volatile uint32_t  dropped;            // лічильник загублених подій
/* volatile: "змінну можуть читати/змінювати поза цим кодом (інша задача, дебагер) —
             не тримай її копію в регістрі, щоразу читай з пам'яті"                 */

static inline uint32_t now_ms(void)           // поточний час у мс
{
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);   // тіки ОС × мс на тік (1)
}

/* --------- ПІДПИСКИ НА ШИНУ (виконуються в задачі INPUT, а не тут!) --------- */
static void on_encoder(void *ctx, BusTopic_t t, int32_t steps)
{
    (void)ctx; (void)t;                       // (void)x; — "параметр навмисно не використовується"
    MotorEvent_t ev = (steps > 0) ? MEV_STEP_CW : MEV_STEP_CCW;     // знак → напрямок
    for (int32_t n = (steps > 0) ? steps : -steps; n > 0; n--) {    // по одній події
        MotorCtrl_Post(ev);                                         // на кожне клацання
    }
}   // ↑ лише кладемо в ЧЕРГУ — з мотором нічого не робимо, бо це чужа задача

static void on_button(void *ctx, BusTopic_t t, int32_t v)
{
    (void)ctx; (void)t; (void)v;
    MotorCtrl_Post(MEV_STOP);                 // кнопка → подія "стоп" у чергу
}

/* --------- автомат повідомив про новий стан → транслюємо на шину (Observer) --------- */
static void on_fsm_state(void *ctx, MotorState_t s)
{
    (void)ctx;
    published_state = s;                                // запам'ятати
    EventBus_Publish(TOPIC_MOTOR_STATE, (int32_t)s);    // розіслати всім (індикатор увімкне LED)
}

/* --------------------------- САМА ЗАДАЧА --------------------------- */
static void MotorCtrlTask(void *arg)
{
    (void)arg;
    MotorEvent_t ev;
    for (;;) {                                          // задача FreeRTOS — нескінченний цикл,
                                                        // вона ніколи не повертається
        if (xQueueReceive(queue, &ev, pdMS_TO_TICKS(CFG_CTRL_TICK_MS)) == pdTRUE) {
            // ↑ ЧЕКАТИ подію НЕ ДОВШЕ 10 мс. Поки чекає — задача ЗАБЛОКОВАНА
            //   і не витрачає процесор. pdMS_TO_TICKS — мс → тіки ОС
            MotorFsm_Dispatch(&fsm, ev, now_ms());      // подія прийшла → передати автомату
        }
        MotorFsm_Dispatch(&fsm, MEV_TICK, now_ms());    // ЗАВЖДИ такт — щоб рахувались паузи й тайм-аут
    }
}

/* ------------- ІНІЦІАЛІЗАЦІЯ (викликається з App_Init до старту ОС) ------------- */
void MotorCtrl_Init(Motor_t motor)
{
    const MotorFsmConfig_t cfg = {
        .deadtime_ms     = CFG_DEADTIME_MS,             // 150 мс
        .idle_timeout_ms = CFG_IDLE_TIMEOUT_MS,         // 600 мс
    };
    MotorFsm_Init(&fsm, &cfg, motor, on_fsm_state, NULL, 0);   // створити автомат

    queue = xQueueCreateStatic(CTRL_QUEUE_LEN, sizeof(MotorEvent_t), qbuf, &qcb);
    // ↑ створити чергу в НАШІЙ пам'яті: 16 елементів, розмір елемента, буфер, службова структура

    bool ok = EventBus_Subscribe(TOPIC_ENCODER_STEP, on_encoder, NULL) &&   // підписатись на енкодер
              EventBus_Subscribe(TOPIC_BUTTON,       on_button,  NULL);     // і на кнопку

    TaskHandle_t h = xTaskCreateStatic(MotorCtrlTask,        // функція задачі
                                       "motor",              // ім'я (видно в дебагері)
                                       CTRL_STACK_WORDS,     // розмір стеку, слів
                                       NULL,                 // параметр (не потрібен)
                                       CFG_PRIO_MOTOR_CTRL,  // пріоритет 3
                                       stack, &tcb);         // НАШ стек і НАШ TCB
    configASSERT(queue != NULL && ok && h != NULL);   // щось не створилось → зупинитись тут
    (void)ok;
}

bool MotorCtrl_Post(MotorEvent_t ev)
{
    if (xQueueSend(queue, &ev, 0) == pdTRUE) return true;   // тайм-аут 0: якщо черга повна —
    dropped++;                                              // НЕ ЧЕКАТИ (задача вводу не блокується),
    return false;                                           // а порахувати загублену подію
}

MotorState_t MotorCtrl_GetState(void)
{
    return published_state;
}

uint32_t MotorCtrl_DroppedEvents(void)
{
    return dropped;
}
