/* ============================================================================
 * drv_motor_relay.c — мотор на двох реле
 * ============================================================================
 * ЗАХИСТ "В ГЛИБИНУ" (defense in depth):
 *   1) автомат станів (app_motor_fsm) витримує паузу 150 мс між напрямками;
 *   2) цей драйвер додатково гарантує "break-before-make": спершу вимикає
 *      протилежне реле, і лише потім вмикає потрібне.
 *   Навіть якщо обидва реле ввімкнуться — у цій схемі це гальмування, а не КЗ.
 * ========================================================================== */
#include "drv_motor_relay.h"

static void relay_motor_stop(void *self)             // реалізація "стоп"
{
    MotorRelay_t *m = (MotorRelay_t *)self;          // void* → наш тип (ми знаємо, що це MotorRelay_t)
    Relay_Set(m->relay_cw,  false);                  // реле 1 вимкнути: дріт 1 мотора → GND
    Relay_Set(m->relay_ccw, false);                  // реле 2 вимкнути: дріт 2 мотора → GND
}                                                    // обидва дроти на GND → мотор гальмує

static void relay_motor_run(void *self, MotorDir_t dir)   // реалізація "крутити"
{
    MotorRelay_t *m = (MotorRelay_t *)self;
    Relay_t *on  = (dir == MOTOR_DIR_CW) ? m->relay_cw  : m->relay_ccw;  // яке реле ВВІМКНУТИ
    Relay_t *off = (dir == MOTOR_DIR_CW) ? m->relay_ccw : m->relay_cw;   // яке ВИМКНУТИ

    Relay_Set(off, false);      // СПОЧАТКУ вимкнути протилежне  (break …
    Relay_Set(on,  true);       // ПОТІМ увімкнути потрібне      … before make)
}                               // → ніколи не буде обидва ввімкнені одночасно

static const MotorOps_t RELAY_MOTOR_OPS = {   // таблиця функцій ДЛЯ МОТОРА НА РЕЛЕ.
    .run  = relay_motor_run,                  // const → лежить у флеш-пам'яті, не займає ОЗП.
    .stop = relay_motor_stop,                 // .run = … — іменована ініціалізація полів (C99)
};

void MotorRelay_Init(MotorRelay_t *m, Relay_t *relay_cw, Relay_t *relay_ccw)
{
    m->relay_cw  = relay_cw;      // запам'ятати, які реле використовувати
    m->relay_ccw = relay_ccw;
    relay_motor_stop(m);          // на старті — стоп
}

Motor_t MotorRelay_AsMotor(MotorRelay_t *m)
{
    Motor_t motor = { .ops = &RELAY_MOTOR_OPS, .self = m };  // таблиця функцій + дані реалізації
    return motor;                                            // → абстрактний мотор для логіки
}
