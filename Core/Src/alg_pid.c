/* ============================================================================
 * alg_pid.c — програмний PID-регулятор (чиста математика, без HAL і RTOS)
 * ============================================================================
 * ПРИКЛАД ОДНОГО ТАКТУ (порахувати разом зі студентами):
 *   уставка 1,6 В, вимір 1,5 В, попередній вимір 1,49 В, integ = 0,45, diff = 0
 *     err   = 1,6 − 1,5                   = 0,10
 *     P     = 2,0 × 0,10                  = 0,20   (тисне вперед)
 *     D     = 0,6 × 0 − 8 × (1,50 − 1,49) = −0,08  (гальмує: напруга вже росте)
 *     di    = 3,0 × 0,10 × 0,01           = 0,003
 *     integ = 0,45 + 0,003                = 0,453  (тримає основу)
 *     u     = 0,20 + 0,453 − 0,08         = 0,573  → ШІМ 57,3 %
 * ========================================================================== */
#include "alg_pid.h"

static inline float clampf(float x, float lo, float hi)   // обмежити x межами [lo, hi]
{
    return (x < lo) ? lo : (x > hi) ? hi : x;             // менше lo → lo; більше hi → hi; інакше x
}

void PID_Init(PID_Controller_t *pid, float kp, float ki, float kd,
              float ts, float tau, float out_min, float out_max)
{
    pid->ts      = ts;             // 0,01 с
    pid->tau     = tau;            // 0,02 с
    pid->out_min = out_min;        // 0
    pid->out_max = out_max;        // 1
    PID_SetGains(pid, kp, ki, kd); // записати коефіцієнти
    PID_Reset(pid);                // обнулити пам'ять
}

void PID_SetGains(PID_Controller_t *pid, float kp, float ki, float kd)
{
    /* integ вже містить Ki, тому зміна Ki "на льоту" не дає стрибка виходу.
       Якби зберігали "чисту" суму похибок і множили на Ki щоразу — при зміні
       Ki з 3 на 6 I-складова миттєво подвоїлась би, і ШІМ стрибнув би.        */
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

void PID_Reset(PID_Controller_t *pid)
{
    pid->integ = 0.0f;             // стерти накопичений інтеграл
    pid->diff  = 0.0f;             // стерти фільтр D
    pid->prev_meas = 0.0f;
    pid->first = true;             // наступний виклик — "перший"
    pid->p_term = pid->i_term = pid->d_term = 0.0f;
}

float PID_Update(PID_Controller_t *pid, float setpoint, float measurement)
{
    /* --- 0. Перший виклик --- */
    if (pid->first) {                    // попереднього вимірювання ще немає →
        pid->prev_meas = measurement;    // вважаємо, що попереднє = поточне,
        pid->first = false;              // інакше похідна на 1-му кроці дала б величезний стрибок
    }

    /* --- 1. Похибка --- */
    const float err = setpoint - measurement;       // наскільки ми не там, де треба

    /* --- 2. P-складова --- */
    const float p = pid->kp * err;                  // пропорційно похибці

    /* --- 3. D-складова: похідна ВИМІРЮВАННЯ через фільтр нижніх частот ---
       Коефіцієнти дискретного фільтра — перетворення Тастіна для ланки Kd·s/(τs+1):
       d[k] = (2τ−T)/(2τ+T)·d[k−1] − 2Kd/(2τ+T)·(y[k]−y[k−1])                    */
    const float a = (2.0f * pid->tau - pid->ts) / (2.0f * pid->tau + pid->ts);  // = 0,6
    const float b = (2.0f * pid->kd)           / (2.0f * pid->tau + pid->ts);   // = 8,0
    pid->diff = a * pid->diff - b * (measurement - pid->prev_meas);
    // ↑ 60 % попереднього значення (згладжування шуму) МІНУС зміна вимірювання за такт.
    //   Мінус: e = sp − y, отже de/dt = −dy/dt (коли уставка стала).
    //   Чому від вимірювання: при стрибку уставки похибка стрибає, а її похідна
    //   стала б величезною (ШІМ миттєво в 100 %). Вимірювання ж фізично не стрибає.
    pid->prev_meas = measurement;                   // запам'ятати для наступного кроку

    /* --- 4. I-складова: приріст інтеграла за такт --- */
    const float di = pid->ki * err * pid->ts;       // Ki × похибка × Ts = площа "прямокутника"
    const float u_unsat = p + pid->integ + di + pid->diff;  // яким був би вихід БЕЗ обмеження

    /* --- 5. Anti-windup: не накопичувати інтеграл, коли це шкодить ---
       Windup: уставка недосяжна, ШІМ уже 100 %, а інтеграл росте й росте.
       Коли уставку знизять — "накачаний" інтеграл довго тримає ШІМ на максимумі
       і вихід сильно перелітає (у наших тестах: 14 % без захисту, 0,3 % з ним). */
    const bool sat_hi = (u_unsat > pid->out_max) && (err > 0.0f); // вже >100 % і похибка тягне вище
    const bool sat_lo = (u_unsat < pid->out_min) && (err < 0.0f); // вже <0 %   і похибка тягне нижче
    if (!sat_hi && !sat_lo) {
        pid->integ += di;                           // інтегруємо ЛИШЕ коли це має сенс
    }
    pid->integ = clampf(pid->integ, pid->out_min, pid->out_max);  // запобіжник: інтеграл у 0…1

    /* --- 6. Вихід --- */
    const float u = clampf(p + pid->integ + pid->diff, pid->out_min, pid->out_max);  // P+I+D у 0…1

    pid->p_term = p;                 // зберегти складові для діагностики
    pid->i_term = pid->integ;
    pid->d_term = pid->diff;
    return u;                        // заповнення ШІМ 0…1
}
