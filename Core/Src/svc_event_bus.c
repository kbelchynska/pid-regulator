/* ============================================================================
 * svc_event_bus.c — статична таблиця підписників, без динамічної пам'яті
 * ========================================================================== */
#include "svc_event_bus.h"
#include <stddef.h>                 // NULL

typedef struct {                    // ОДИН ПІДПИСНИК:
    BusTopic_t    topic;            // на яку тему
    BusCallback_t cb;               // кого викликати
    void         *ctx;              // що йому передати
} Subscriber_t;

static Subscriber_t subs[EVENT_BUS_MAX_SUBSCRIBERS];  // СТАТИЧНИЙ масив на 8 місць — без malloc,
                                                      // розмір відомий при компіляції
static uint8_t      n_subs;         // скільки вже підписалось (static-змінні в C = 0 автоматично)

bool EventBus_Subscribe(BusTopic_t topic, BusCallback_t cb, void *ctx)
{
    if (cb == NULL || topic >= TOPIC_COUNT || n_subs >= EVENT_BUS_MAX_SUBSCRIBERS)
        return false;               // перевірки: є функція? тема існує? є вільне місце?
    subs[n_subs++] = (Subscriber_t){ .topic = topic, .cb = cb, .ctx = ctx };
    // ↑ (Subscriber_t){…} — "compound literal" (C99): створити структуру на льоту.
    //   subs[n_subs++] — записати в поточну позицію, ПОТІМ збільшити лічильник
    return true;
}

void EventBus_Publish(BusTopic_t topic, int32_t value)
{
    for (uint8_t i = 0; i < n_subs; i++) {        // пройти всіх підписників
        if (subs[i].topic == topic)               // підписаний на цю тему?
            subs[i].cb(subs[i].ctx, topic, value);// → викликати його функцію
    }
}
