/* ============================================================================
 * bsp_gpio.h — ШАР BSP (Board Support Package): цифрові входи і виходи
 * ============================================================================
 * НАВІЩО ЦЕЙ ФАЙЛ:
 *   Це ЄДИНИЙ модуль, який знає, як пін працює фізично: який рівень означає
 *   "увімкнено", push-pull чи open-drain. Решта коду каже лише "увімкни реле"
 *   чи "чи натиснута кнопка".
 *
 *   Приклад: реле Low-trigger вмикається НУЛЕМ, а світлодіод — ОДИНИЦЕЮ.
 *   Драйвери реле і світлодіода однакові — різниця лише в ОПИСІ піна.
 *
 * ПАТЕРН: Adapter — простий інтерфейс поверх складного HAL.
 * ========================================================================== */
#ifndef BSP_GPIO_H
#define BSP_GPIO_H

#include "main.h"          // типи HAL: GPIO_TypeDef, GPIO_PIN_x
#include <stdbool.h>       // тип bool (true/false)

typedef struct {                // ОПИС ЦИФРОВОГО ВИХОДУ
    GPIO_TypeDef *port;         // порт: GPIOA, GPIOB, GPIOC …
    uint16_t      pin;          // номер піна: GPIO_PIN_14 …
    bool          active_low;   // true: "увімкнено" = логічний 0 на піні
    bool          open_drain;   // true: вихід open-drain (лише тягне до 0 або відпускає)
} DigitalOut_t;

typedef struct {                // ОПИС ЦИФРОВОГО ВХОДУ
    GPIO_TypeDef *port;
    uint16_t      pin;
    bool          active_low;   // true: "активний" (натиснута кнопка) = 0 на піні
    uint32_t      pull;         // підтяжка: GPIO_NOPULL / GPIO_PULLUP / GPIO_PULLDOWN
} DigitalIn_t;

/* const DigitalOut_t *d — вказівник на опис, який функція НЕ змінює (const).
   Передаємо вказівник, а не копію структури — так швидше.                  */
void DigitalOut_Init(const DigitalOut_t *d);              // налаштувати вихід (одразу вимкнений)
void DigitalOut_Set (const DigitalOut_t *d, bool active); // увімкнути / вимкнути

void DigitalIn_Init (const DigitalIn_t *d);               // налаштувати вхід
bool DigitalIn_IsActive(const DigitalIn_t *d);            // чи активний (напр., кнопка натиснута)

#endif /* BSP_GPIO_H */
