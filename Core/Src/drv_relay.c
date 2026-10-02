/* ============================================================================
 * drv_relay.c — драйвер реле
 * ========================================================================== */
#include "drv_relay.h"

void Relay_Init(Relay_t *r, DigitalOut_t out)
{
    r->out = out;                 // зберегти опис піна
    r->on  = false;               // вимкнене
    DigitalOut_Init(&r->out);     // BSP поставить НЕАКТИВНИЙ рівень ДО режиму виходу → не клацне
}

void Relay_Set(Relay_t *r, bool on)
{
    r->on = on;                   // запам'ятати
    DigitalOut_Set(&r->out, on);  // клацнути (або відпустити)
}

bool Relay_IsOn(const Relay_t *r)
{
    return r->on;                 // повернути збережений стан
}
