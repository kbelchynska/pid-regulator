/* ============================================================================
 * app.c — "КОРІНЬ КОМПОЗИЦІЇ" (Composition Root) + Dependency Injection
 * ============================================================================
 * НАВІЩО: це ЄДИНЕ місце, де
 *   - створюються всі об'єкти системи (статично, без malloc);
 *   - вирішується, яка реалізація мотора використовується (зараз — на реле);
 *   - модулі з'єднуються між собою.
 * Решта модулів отримують усе потрібне параметрами і не знають одне про одного.
 *
 * ЗАМІНА РЕЛЕ НА ДРАЙВЕР MX1616 — змінюється ЛИШЕ цей файл:
 *     static MotorHbridge_t motor_hb;
 *     MotorHbridge_Init(&motor_hb, &htim1, ...);
 *     MotorCtrl_Init(MotorHbridge_AsMotor(&motor_hb));   // замість MotorRelay_AsMotor
 *
 * ПОТОКИ ПОДІЙ:
 *                              ┌──► app_motor_ctrl ──► app_motor_fsm ──► Motor_t ◄── drv_motor_relay ──► drv_relay
 *   app_input ──► ШИНА ПОДІЙ ──┤        │ TOPIC_MOTOR_STATE
 *   (енкодер,     (Observer)   │        ▼
 *    кнопка)                   │   app_indicator ──► drv_led (червоний, синій, PC13)
 *                              │        ▲ TOPIC_REG_SETTLED
 *                              └──► app_regulator ──► alg_pid
 *                                       │    ▲
 *                                   drv_pwm  drv_adc      (RC-коло між ними)
 *
 *   усі драйвери GPIO ──► bsp_gpio ──► HAL
 * ========================================================================== */
#include "app.h"
#include "app_config.h"

#include "bsp_gpio.h"            // ↓ усі драйвери
#include "drv_led.h"
#include "drv_relay.h"
#include "drv_motor_relay.h"
#include "drv_encoder.h"
#include "drv_button.h"
#include "drv_pwm.h"
#include "drv_adc.h"

#include "app_motor_ctrl.h"      // ↓ усі модулі застосунку
#include "app_input.h"
#include "app_indicator.h"
#include "app_regulator.h"

extern TIM_HandleTypeDef htim4;  // extern: "ці змінні оголошені в main.c (їх створив CubeMX)"
extern TIM_HandleTypeDef htim3;  // TIM3: ШІМ CH1 (PA6) → RC-коло
extern ADC_HandleTypeDef hadc1;  // ADC1: IN1 (PA1) ← вихід RC-кола

/* ---------------- ВСІ ОБ'ЄКТИ СИСТЕМИ — статично, в одному місці ---------------- */
static Relay_t      relay_cw, relay_ccw;          // два реле
static MotorRelay_t motor_relay;                  // мотор із двох реле
static Led_t        led_red, led_blue, led_heart; // три світлодіоди
static Encoder_t    encoder;                      // енкодер
static Button_t     button;                       // кнопка
static Pwm_t        pwm;                          // вихід регулятора
static AdcIn_t      adc;                          // вхід регулятора

void App_Init(void)
{
    /* 1) РЕЛЕ — ПЕРШИМИ: якомога раніше гарантуємо, що мотор вимкнений */
    const bool active_low = (CFG_RELAY_ACTIVE_LOW != 0);          // тип модулів реле з конфігурації
    Relay_Init(&relay_cw,  (DigitalOut_t){ CFG_RELAY_CW_PORT,  CFG_RELAY_CW_PIN,
                                           active_low,             // полярність: 0 = увімкнене
                                           active_low });          // open-drain для Low-trigger
    Relay_Init(&relay_ccw, (DigitalOut_t){ CFG_RELAY_CCW_PORT, CFG_RELAY_CCW_PIN,
                                           active_low, active_low });
    // ↑ (DigitalOut_t){…} — compound literal: опис піна створюється прямо у виклику
    MotorRelay_Init(&motor_relay, &relay_cw, &relay_ccw);         // з двох реле — один мотор

    /* 2) СВІТЛОДІОДИ */
    Led_Init(&led_red,   (DigitalOut_t){ CFG_LED_RED_PORT,   CFG_LED_RED_PIN,   false, false });
    Led_Init(&led_blue,  (DigitalOut_t){ CFG_LED_BLUE_PORT,  CFG_LED_BLUE_PIN,  false, false });
    Led_Init(&led_heart, (DigitalOut_t){ CFG_LED_HEART_PORT, CFG_LED_HEART_PIN, true,  false });
    //                                                         ↑ PC13 світиться від НУЛЯ → active_low=true.
    //                                      Решта коду про цю особливість плати не знає

    /* 3) ВХОДИ */
    Encoder_Init(&encoder, &htim4, CFG_ENC_COUNTS_PER_DETENT, CFG_ENC_INVERT != 0);
    //                     ↑ драйвер отримує ТАЙМЕР ззовні — сам не знає, що це саме TIM4
    Button_Init(&button, (DigitalIn_t){ CFG_BUTTON_PORT, CFG_BUTTON_PIN,
                                        true,             // натиснута = 0 (кнопка на GND)
                                        GPIO_PULLUP },    // внутрішня підтяжка до 3,3 В
                CFG_BUTTON_DEBOUNCE_N);                   // 3 відліки для антибрязкоту

    /* 4) ВИХІД І ВХІД РЕГУЛЯТОРА */
    Pwm_Init(&pwm, &htim3, TIM_CHANNEL_1);                                 // ШІМ на PA6
    AdcIn_Init(&adc, &hadc1, CFG_REG_ADC_OVERSAMPLE, CFG_REG_VREF_MV);     // АЦП на PA1, 8 відліків

    /* 5) З'ЄДНАТИ МОДУЛІ. Порядок важливий: спершу всі ПІДПИСНИКИ, останнім — ВИДАВЕЦЬ */
    Indicator_Init(&led_red, &led_blue, &led_heart);     // підписується на стан мотора і PID
    MotorCtrl_Init(MotorRelay_AsMotor(&motor_relay));    // ← ТУТ вибирається реалізація мотора:
                                                         //   логіка отримує АБСТРАКТНИЙ мотор
    Regulator_Init(&pwm, &adc);                          // PID отримує свій вихід і вхід
    Input_Init(&encoder, &button);                       // ввід — останнім (усі вже підписані)
}
