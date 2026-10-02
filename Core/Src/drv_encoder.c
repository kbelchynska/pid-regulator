/* ============================================================================
 * drv_encoder.c — енкодер на TIM4
 * ============================================================================
 * НАЙЦІКАВІШИЙ РЯДОК: (int16_t)(now - e->last_cnt)
 *   Лічильник 16-бітний і "перекидається" 65535 ↔ 0.
 *   Приклад: було 2, крутнули назад на 4 імпульси, стало 65534.
 *     65534 − 2 = 65532;  як знакове 16-бітне число 65532 = −4  ✔
 *   Приведення до int16_t автоматично дає правильний знак і величину навіть
 *   через "перекидання" (якщо між читаннями < 32768 імпульсів — при 10 мс завжди).
 * ========================================================================== */
#include "drv_encoder.h"

void Encoder_Init(Encoder_t *e, TIM_HandleTypeDef *htim,
                  int32_t counts_per_detent, bool invert)
{
    e->htim = htim;                                           // запам'ятати таймер
    e->counts_per_detent = (counts_per_detent > 0) ? counts_per_detent : 1;
                                                              // захист від ділення на 0
    e->invert    = invert;
    e->remainder = 0;

    HAL_TIM_Encoder_Start(htim, TIM_CHANNEL_ALL);             // запустити таймер у режимі енкодера.
                                                              // Далі він рахує САМ, без процесора
    e->last_cnt = (uint16_t)__HAL_TIM_GET_COUNTER(htim);      // стартове значення лічильника
}

int32_t Encoder_ReadDetents(Encoder_t *e)
{
    uint16_t now  = (uint16_t)__HAL_TIM_GET_COUNTER(e->htim); // прочитати регістр CNT таймера
    int16_t  diff = (int16_t)(now - e->last_cnt);             // ЗМІНА з минулого разу (див. шапку)
    e->last_cnt = now;                                        // запам'ятати для наступного разу

    e->remainder += e->invert ? -diff : diff;                 // накопичити імпульси (з інверсією)

    int32_t detents = e->remainder / e->counts_per_detent;    // скільки ПОВНИХ клацань (ділення
                                                              // цілих відкидає дробову частину)
    e->remainder   -= detents * e->counts_per_detent;         // залишок — на наступний раз.
                                                              // Напр. 6 імпульсів → 1 клацання, 2 лишаються
    return detents;                                           // клацання зі знаком
}
