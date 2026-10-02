/* ============================================================================
 * app_regulator.c — задача PID-регулятора
 * ============================================================================
 * ПРАВИЛО РЕАЛЬНОГО ЧАСУ: ця задача НІКОЛИ ні на що не чекає, крім свого
 *   таймера. Черги читає з тайм-аутом 0, ШІМ і АЦП належать лише їй. Тому
 *   період 10 мс витримується точно, а PID отримує рівний Ts.
 *
 * vTaskDelay(10) проти vTaskDelayUntil(&last, 10):
 *   vTaskDelay:       |робота 1мс|--сон 10мс--|робота|--сон 10мс--|  період 11 мс, "пливе"
 *   vTaskDelayUntil:  |робота|----сон 9мс----|робота|----сон 9мс--|  період рівно 10 мс
 *
 * НАЙВИЩИЙ ПРІОРИТЕТ (4) — правило Rate Monotonic: задача з найжорсткішим
 *   періодом отримує найвищий пріоритет; енкодер чи мотор не зсунуть такт.
 *
 * УСТАВКА ЗМІНЮЄТЬСЯ ЧЕРЕЗ ЧЕРГУ: on_encoder виконується в задачі input.
 *   Якби вона сама міняла sp_mv — дві задачі писали б одну змінну (гонка
 *   даних). А так уставкою володіє ЛИШЕ задача pid.
 * ========================================================================== */
#include "app_regulator.h"
#include "app_config.h"
#include "alg_pid.h"
#include "svc_event_bus.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include <stdlib.h>                // abs()

/* --------------------------- команди в черзі регулятора --------------------------- */
typedef enum { RCMD_SP_DELTA, RCMD_SP_SET } RegCmdType_t;   // "змінити на Δ" / "встановити значення"
typedef struct { RegCmdType_t type; int32_t value_mv; } RegCmd_t;

#define REG_STACK_WORDS  256u      // стек 1 КБ
#define REG_QUEUE_LEN    16u       // черга на 16 команд

static StaticTask_t  tcb;          // статична пам'ять задачі і черги (як у мотора)
static StackType_t   stack[REG_STACK_WORDS];
static StaticQueue_t qcb;
static uint8_t       qbuf[REG_QUEUE_LEN * sizeof(RegCmd_t)];
static QueueHandle_t queue;

static Pwm_t   *pwm;               // вихід (дали ззовні)
static AdcIn_t *adc;               // вхід (дали ззовні)

volatile RegStatus_t g_reg;                                                   // телеметрія
volatile PidTuning_t g_pid_tuning = { CFG_PID_KP, CFG_PID_KI, CFG_PID_KD };   // стартові 2 / 3 / 0,2

/* ------------------- підписки на шину (виконуються в задачі input) ------------------- */
static void on_encoder(void *ctx, BusTopic_t t, int32_t steps)
{
    (void)ctx; (void)t;
    RegCmd_t c = { RCMD_SP_DELTA, steps * CFG_SP_STEP_MV };   // +1 клацання → +100 мВ
    xQueueSend(queue, &c, 0);          // у чергу; уставку змінить сама задача pid
}

static void on_button(void *ctx, BusTopic_t t, int32_t v)
{
    (void)ctx; (void)t; (void)v;
    RegCmd_t c = { RCMD_SP_SET, CFG_SP_MIN_MV };              // кнопка → уставка 0 В
    xQueueSend(queue, &c, 0);
}

/* ------------------------------- допоміжне ------------------------------- */
static int32_t clamp_sp(int32_t mv)    // обмежити уставку 0 … 3000 мВ
{
    if (mv < CFG_SP_MIN_MV) return CFG_SP_MIN_MV;
    if (mv > CFG_SP_MAX_MV) return CFG_SP_MAX_MV;
    return mv;
}

static void dwt_init(void)             // увімкнути лічильник тактів процесора (для вимірювання часу)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;   // увімкнути блок трасування ядра Cortex-M4
    DWT->CYCCNT = 0;                                  // обнулити лічильник тактів
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;             // запустити: +1 кожен такт (84 млн/с)
}

/* ================================ САМА ЗАДАЧА ================================ */
static void RegulatorTask(void *arg)
{
    (void)arg;
    PID_Controller_t pid;                        // об'єкт PID живе на стеку ЦІЄЇ задачі —
                                                 // ніхто інший до нього не має доступу
    PidTuning_t applied = { CFG_PID_KP, CFG_PID_KI, CFG_PID_KD };    // які коефіцієнти застосовані
    PID_Init(&pid, applied.kp, applied.ki, applied.kd,
             CFG_REG_PERIOD_MS / 1000.0f,        // Ts = 10/1000 = 0,01 с
             CFG_PID_DFILT_TAU_S, 0.0f, 1.0f);   // τ = 0,02 с, вихід 0…1

    int32_t    sp_mv    = CFG_SP_INITIAL_MV;     // уставка — 1500 мВ
    bool       settled  = false;                 // вийшли на уставку?
    TickType_t in_band_since = 0;                // коли увійшли в смугу ±30 мВ
    bool       in_band  = false;                 // зараз у смузі?

    const uint32_t   cycles_per_us = HAL_RCC_GetHCLKFreq() / 1000000u;   // 84 такти = 1 мкс
    const TickType_t period = pdMS_TO_TICKS(CFG_REG_PERIOD_MS);          // 10 мс у тіках
    TickType_t last = xTaskGetTickCount();                               // точка відліку періоду

    for (;;) {
        /* --- 0. рівний період --- */
        if ((TickType_t)(xTaskGetTickCount() - last) >= 2u * period) g_reg.overruns++;
        // ↑ минуло ≥ 2 періоди з минулого пробудження → ми НЕ ВСТИГЛИ (overrun)
        vTaskDelayUntil(&last, period);
        // ↑ спати ДО моменту last + 10 мс (а не "10 мс від зараз") → період рівно 10 мс
        uint32_t c0 = DWT->CYCCNT;               // засікти такт процесора на початку роботи

        /* --- 1. нові уставки з черги (тайм-аут 0 — НЕ чекати) --- */
        RegCmd_t cmd;
        while (xQueueReceive(queue, &cmd, 0) == pdTRUE) {     // забрати ВСІ накопичені команди
            sp_mv = clamp_sp(cmd.type == RCMD_SP_DELTA ? sp_mv + cmd.value_mv   // +Δ від енкодера
                                                       : cmd.value_mv);         // або 0 від кнопки
        }

        /* --- 2. коефіцієнти, змінені в дебагері / CubeMonitor --- */
        PidTuning_t want = { g_pid_tuning.kp, g_pid_tuning.ki, g_pid_tuning.kd };
        if (want.kp != applied.kp || want.ki != applied.ki || want.kd != applied.kd) {
            applied = want;                                    // змінились → застосувати
            PID_SetGains(&pid, applied.kp, applied.ki, applied.kd);
        }

        /* --- 3. ЗАМКНУТИЙ КОНТУР: три рядки — серце проєкту --- */
        int32_t y_mv = (int32_t)AdcIn_ReadMv(adc);                       // ВИМІРЯТИ   (очі)
        float   u    = PID_Update(&pid, sp_mv / 1000.0f, y_mv / 1000.0f); // ПОРАХУВАТИ (мозок)
        Pwm_SetDuty(pwm, u);                                             // ВПЛИНУТИ   (руки)
        // мВ → В: ділимо на 1000.0f (з f — щоб ділення було у float, а не в цілих)

        /* --- 4. "вийшов на уставку": похибка в смузі N мс поспіль --- */
        int32_t err = sp_mv - y_mv;
        if (abs(err) <= CFG_SETTLED_BAND_MV) {                   // похибка ≤ 30 мВ?
            if (!in_band) { in_band = true; in_band_since = xTaskGetTickCount(); }  // запам'ятати коли
        } else {
            in_band = false;                                     // вийшли зі смуги — скинути
        }
        bool now_settled = in_band &&                            // у смузі …
            (TickType_t)(xTaskGetTickCount() - in_band_since) >= pdMS_TO_TICKS(CFG_SETTLED_TIME_MS);
                                                                 // … вже ≥ 300 мс поспіль
        if (now_settled != settled) {                            // стан ЗМІНИВСЯ →
            settled = now_settled;
            EventBus_Publish(TOPIC_REG_SETTLED, settled ? 1 : 0);   // повідомити (LED на платі
        }                                                           // змінить частоту блимання)

        /* --- 5. телеметрія для дебагера --- */
        g_reg.setpoint_mv = sp_mv;
        g_reg.measured_mv = y_mv;
        g_reg.error_mv    = err;
        g_reg.duty_pct    = u * 100.0f;                          // 0…1 → 0…100 %
        g_reg.p = pid.p_term; g_reg.i = pid.i_term; g_reg.d = pid.d_term;
        g_reg.settled     = settled;

        uint32_t us = (DWT->CYCCNT - c0) / cycles_per_us;        // тактів минуло → мікросекунди
        g_reg.exec_us = us;                                      // час цього такту
        if (us > g_reg.exec_us_max) g_reg.exec_us_max = us;      // найгірший час
    }
}

/* ================================ ІНІЦІАЛІЗАЦІЯ ================================ */
void Regulator_Init(Pwm_t *pwm_out, AdcIn_t *adc_in)
{
    pwm = pwm_out;                   // запам'ятати вихід
    adc = adc_in;                    // і вхід
    dwt_init();                      // увімкнути лічильник тактів

    queue = xQueueCreateStatic(REG_QUEUE_LEN, sizeof(RegCmd_t), qbuf, &qcb);    // черга команд
    bool ok = EventBus_Subscribe(TOPIC_ENCODER_STEP, on_encoder, NULL) &&       // підписки
              EventBus_Subscribe(TOPIC_BUTTON,       on_button,  NULL);

    TaskHandle_t h = xTaskCreateStatic(RegulatorTask, "pid", REG_STACK_WORDS, NULL,
                                       CFG_PRIO_REGULATOR, stack, &tcb);        // пріоритет 4 — НАЙВИЩИЙ
    configASSERT(queue != NULL && ok && h != NULL);
    (void)ok;
}
