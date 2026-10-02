/* ============================================================================
 * drv_pwm.c — ШІМ на таймері
 * ========================================================================== */
#include "drv_pwm.h"

void Pwm_Init(Pwm_t *p, TIM_HandleTypeDef *htim, uint32_t channel)
{
    p->htim    = htim;                         // запам'ятати таймер
    p->channel = channel;                      // і канал
    __HAL_TIM_SET_COMPARE(htim, channel, 0);   // CCR = 0 → 0 % ДО старту
    HAL_TIM_PWM_Start(htim, channel);          // запустити генерацію ШІМ. Далі — апаратно
}

void Pwm_SetDuty(Pwm_t *p, float duty01)
{
    if (duty01 < 0.0f) duty01 = 0.0f;          // обмеження знизу (захист від некоректного значення)
    if (duty01 > 1.0f) duty01 = 1.0f;          // обмеження зверху
    uint32_t period = __HAL_TIM_GET_AUTORELOAD(p->htim) + 1u;   // період з таймера: ARR+1 = 4200.
                                                                // Драйвер не залежить від CubeMX
    __HAL_TIM_SET_COMPARE(p->htim, p->channel, (uint32_t)(duty01 * (float)period));
    // ↑ CCR = заповнення × період. Напр. 0,45 × 4200 = 1890.
    //   Один запис у регістр — і таймер з наступного періоду видає нову ширину імпульсу
}
