/* ============================================================================
 * bsp_gpio.c — реалізація цифрових входів/виходів поверх HAL
 * ============================================================================
 * ЯК ПРАЦЮЄ:
 *   Логічний стан ("увімкнено/вимкнено") переводиться у фізичний рівень піна
 *   (0/1) через "виключне АБО" (XOR) з полярністю піна.
 *
 * ЧОМУ БЕЗПЕЧНО З РІЗНИХ ЗАДАЧ:
 *   HAL_GPIO_WritePin пише в регістр BSRR ОДНІЄЮ командою (атомарно). Навіть
 *   якщо дві задачі одночасно змінюють різні піни одного порту — вони не
 *   заважають одна одній.
 * ========================================================================== */
#include "bsp_gpio.h"

static void enable_port_clock(GPIO_TypeDef *port)   // static = приватна функція цього файлу
{
    if      (port == GPIOA) __HAL_RCC_GPIOA_CLK_ENABLE();   // увімкнути тактування потрібного порту.
    else if (port == GPIOB) __HAL_RCC_GPIOB_CLK_ENABLE();   // Модуль не покладається на те, що це
    else if (port == GPIOC) __HAL_RCC_GPIOC_CLK_ENABLE();   // вже зробив CubeMX — він самодостатній
    else if (port == GPIOH) __HAL_RCC_GPIOH_CLK_ENABLE();
}

static GPIO_PinState level_for(const DigitalOut_t *d, bool active)
{
    return (active != d->active_low) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    // ↑ КЛЮЧОВИЙ РЯДОК: "виключне АБО" (XOR) логічного стану і полярності.
    //   active=true,  active_low=false → 1 на піні (світлодіод світиться)
    //   active=true,  active_low=true  → 0 на піні (реле Low-trigger увімкнене)
    //   active=false, active_low=true  → 1 на піні (реле вимкнене)
    //   active=false, active_low=false → 0 на піні (світлодіод згаслий)
}

void DigitalOut_Init(const DigitalOut_t *d)
{
    enable_port_clock(d->port);                              // увімкнути тактування порту

    HAL_GPIO_WritePin(d->port, d->pin, level_for(d, false)); // СПОЧАТКУ записати у вихідний регістр
                                                             // НЕАКТИВНИЙ рівень (пін ще не вихід)
    GPIO_InitTypeDef g = {0};                                // структура налаштувань, обнулена
    g.Pin   = d->pin;                                        // який пін
    g.Mode  = d->open_drain ? GPIO_MODE_OUTPUT_OD            // open-drain …
                            : GPIO_MODE_OUTPUT_PP;           // … або push-pull, залежно від опису
    g.Pull  = GPIO_NOPULL;                                   // без внутрішніх підтяжок
    g.Speed = GPIO_SPEED_FREQ_LOW;                           // повільні фронти (менше перешкод)
    HAL_GPIO_Init(d->port, &g);                              // ЛИШЕ ТЕПЕР пін стає виходом —
}                                                            // і на ньому вже безпечний рівень

void DigitalOut_Set(const DigitalOut_t *d, bool active)
{
    HAL_GPIO_WritePin(d->port, d->pin, level_for(d, active)); // "логічне" → "фізичне" → на пін
}

void DigitalIn_Init(const DigitalIn_t *d)
{
    enable_port_clock(d->port);
    GPIO_InitTypeDef g = {0};
    g.Pin  = d->pin;
    g.Mode = GPIO_MODE_INPUT;          // режим входу
    g.Pull = d->pull;                  // підтяжка з опису (для кнопки — PULLUP)
    HAL_GPIO_Init(d->port, &g);
}

bool DigitalIn_IsActive(const DigitalIn_t *d)
{
    bool high = (HAL_GPIO_ReadPin(d->port, d->pin) == GPIO_PIN_SET);  // прочитати пін: 1 чи 0
    return high != d->active_low;      // той самий XOR: для кнопки на GND (active_low=true)
}                                      // 0 на піні → повертає true ("натиснута")
