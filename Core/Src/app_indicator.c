/* ============================================================================
 * app_indicator.c — підписник на стан мотора і стан регулятора
 * ============================================================================
 * Колбеки викликаються в контексті задач-видавців (див. svc_event_bus.h),
 * тому лише перемикають піни або міняють період таймера — без блокувань.
 *
 * ЧОМУ ПРОГРАМНИЙ ТАЙМЕР, А НЕ ЗАДАЧА: щоб "кожні N мс зробити щось коротке",
 * окрема задача з власним стеком (~0,5 КБ) — марнотратство. Усі програмні
 * таймери обслуговує одна службова задача FreeRTOS (Tmr Svc).
 * ========================================================================== */
#include "app_indicator.h"
#include "app_motor_fsm.h"             // лише щоб знати назви станів (MS_RUN_CW …)
#include "app_config.h"
#include "svc_event_bus.h"

#include "FreeRTOS.h"
#include "timers.h"                    // програмні таймери FreeRTOS

#define BLINK_FAST_MS  100u            // швидке блимання: PID ще в перехідному процесі

static Led_t *led_red, *led_blue, *led_heart;   // світлодіоди, які дали ззовні

static StaticTimer_t heart_tcb;        // статична пам'ять програмного таймера
static TimerHandle_t heart_timer;      // "номерок" таймера

/* --- змінився стан мотора (викликається з задачі motor) --- */
static void on_motor_state(void *ctx, BusTopic_t topic, int32_t value)
{
    (void)ctx; (void)topic;
    MotorState_t s = (MotorState_t)value;       // число → стан
    Led_Set(led_red,  s == MS_RUN_CW);          // червоний горить ТОДІ Й ЛИШЕ ТОДІ, коли RUN_CW
    Led_Set(led_blue, s == MS_RUN_CCW);         // синій — коли RUN_CCW. В IDLE/SWITCHING обидва гаснуть
}                                               // (вираз s == … вже є true/false)

/* --- PID вийшов / не вийшов на уставку (викликається з задачі pid) --- */
static void on_reg_settled(void *ctx, BusTopic_t topic, int32_t settled)
{
    (void)ctx; (void)topic;
    xTimerChangePeriod(heart_timer,             // змінити період таймера:
                       pdMS_TO_TICKS(settled ? CFG_HEARTBEAT_MS    // вийшов → 500 мс (повільно)
                                             : BLINK_FAST_MS),     // ні → 100 мс (швидко)
                       0);                      // 0 = не чекати, якщо черга команд таймерів зайнята
}

/* --- спрацював програмний таймер (викликає FreeRTOS) --- */
static void on_heartbeat(TimerHandle_t t)
{
    (void)t;
    Led_Toggle(led_heart);                      // перемкнути LED на платі
}

void Indicator_Init(Led_t *red, Led_t *blue, Led_t *heartbeat)
{
    led_red   = red;                            // запам'ятати світлодіоди
    led_blue  = blue;
    led_heart = heartbeat;

    Led_Set(led_red,  false);                   // на старті вимкнені
    Led_Set(led_blue, false);

    heart_timer = xTimerCreateStatic("heart",                      // ім'я (для дебагера)
                                     pdMS_TO_TICKS(BLINK_FAST_MS), // період 100 мс
                                     pdTRUE,                       // автоперезапуск (періодичний)
                                     NULL,                         // ID таймера (не потрібен)
                                     on_heartbeat,                 // що викликати
                                     &heart_tcb);                  // наша статична пам'ять

    bool ok = EventBus_Subscribe(TOPIC_MOTOR_STATE, on_motor_state, NULL) &&   // підписатись на мотор
              EventBus_Subscribe(TOPIC_REG_SETTLED, on_reg_settled, NULL);     // і на PID
    configASSERT(ok && heart_timer != NULL);
    (void)ok;
    xTimerStart(heart_timer, 0);     // запустити (реально почне працювати разом з планувальником)
}
