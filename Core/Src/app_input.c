/* ============================================================================
 * app_input.c — періодичне опитування енкодера і кнопки (кожні 10 мс)
 * ============================================================================
 * ВАЖЛИВО: коли тут викликається EventBus_Publish, шина В ЦІЙ ЖЕ ЗАДАЧІ
 *   викликає функції-підписники (on_encoder мотора і on_encoder регулятора).
 *   Кожна з них лише кладе подію у СВОЮ чергу і одразу повертається. Самі
 *   мотор і PID обробляють подію пізніше — у своїх задачах.
 * ========================================================================== */
#include "app_input.h"
#include "app_config.h"
#include "svc_event_bus.h"         // лише шина — НІ мотора, НІ регулятора тут немає

#include "FreeRTOS.h"
#include "task.h"

#define INPUT_STACK_WORDS      192u  // стек 768 байт: задача проста
#define MAX_STEPS_PER_PERIOD   4     // не більше 4 клацань за 10 мс (не "засипаємо" черги)

static StaticTask_t tcb;             // статична пам'ять задачі
static StackType_t  stack[INPUT_STACK_WORDS];

static Encoder_t *enc;               // драйвери, які дали ззовні
static Button_t  *btn;

static void InputTask(void *arg)
{
    (void)arg;
    TickType_t last = xTaskGetTickCount();          // точка відліку періоду

    for (;;) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(CFG_INPUT_PERIOD_MS));   // прокидатись рівно кожні 10 мс.
                                                                      // Між пробудженнями — 0 % процесора

        /* --- 1. кнопка: аварійний стоп мотора + уставка 0 В --- */
        if (Button_PollPressed(btn)) {              // натиснули (вже з антибрязкотом)?
            EventBus_Publish(TOPIC_BUTTON, 1);      // → повідомити всім (мотор: стоп; PID: уставка 0)
        }

        /* --- 2. енкодер: кількість клацань зі знаком за останні 10 мс --- */
        int32_t det = Encoder_ReadDetents(enc);     // зазвичай 0 або ±1
        if (det >  MAX_STEPS_PER_PERIOD) det =  MAX_STEPS_PER_PERIOD;   // обмежити зверху
        if (det < -MAX_STEPS_PER_PERIOD) det = -MAX_STEPS_PER_PERIOD;   // і знизу
        if (det != 0) {                             // лише якщо ЩОСЬ змінилось →
            EventBus_Publish(TOPIC_ENCODER_STEP, det);   // повідомити всім (мотор і PID)
        }
    }
}

void Input_Init(Encoder_t *encoder, Button_t *button)
{
    enc = encoder;                                  // запам'ятати драйвери
    btn = button;
    TaskHandle_t h = xTaskCreateStatic(InputTask, "input", INPUT_STACK_WORDS, NULL,
                                       CFG_PRIO_INPUT, stack, &tcb);    // пріоритет 2
    configASSERT(h != NULL);
}
