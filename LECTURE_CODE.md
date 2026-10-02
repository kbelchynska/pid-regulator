# Розбір коду проєкту: конспект для лекції

**Проєкт:** програмний PID-регулятор на STM32F401 (Black Pill) + FreeRTOS
**Як читати:** код розібрано **знизу вгору**, від заліза до логіки, в тому ж порядку, в якому залежать модулі. Кожен наступний файл використовує лише попередні.

Номери рядків відповідають файлам з архіву `PID_Encoder_Motor.zip`. Коментарі-шапки у файлах пропущено, щоб не дублювати.

---

## Зміст

0. [План лекцій і головна ідея](#0-план-лекцій-і-головна-ідея)
1. [main.c: налаштування периферії (CubeMX)](#1-mainc-налаштування-периферії-cubemx)
2. [app_config.h: єдине місце налаштувань](#2-app_configh-єдине-місце-налаштувань)
3. [bsp_gpio: цифрові входи/виходи](#3-bsp_gpio-цифрові-входивиходи)
4. [drv_led і drv_relay: найпростіші драйвери](#4-drv_led-і-drv_relay-найпростіші-драйвери)
5. [drv_motor.h + drv_motor_relay: інтерфейс мотора](#5-drv_motorh--drv_motor_relay-інтерфейс-мотора)
6. [drv_encoder: енкодер на апаратному таймері](#6-drv_encoder-енкодер-на-апаратному-таймері)
7. [drv_button: антибрязкіт](#7-drv_button-антибрязкіт)
8. [drv_pwm: вихід регулятора](#8-drv_pwm-вихід-регулятора)
9. [drv_adc: вхід регулятора](#9-drv_adc-вхід-регулятора)
10. [alg_pid: сам PID-регулятор](#10-alg_pid-сам-pid-регулятор)
11. [svc_event_bus: шина подій](#11-svc_event_bus-шина-подій)
12. [app_motor_fsm: автомат станів мотора](#12-app_motor_fsm-автомат-станів-мотора)
13. [app_motor_ctrl: задача мотора](#13-app_motor_ctrl-задача-мотора)
14. [app_regulator: задача PID](#14-app_regulator-задача-pid)
15. [app_input: задача вводу](#15-app_input-задача-вводу)
16. [app_indicator: світлодіоди і програмний таймер](#16-app_indicator-світлодіоди-і-програмний-таймер)
17. [app.c: збирання системи](#17-appc-збирання-системи)
18. [Тести на ПК](#18-тести-на-пк)
19. [Контрольні питання для студентів](#19-контрольні-питання-для-студентів)

---

## 0. План лекцій і головна ідея

### Що робить система (показати на макеті на початку)

```
Енкодер ──► задача input ──► ШИНА ПОДІЙ ──┬──► задача motor ──► реле ──► мотор
                                           │         └──► LED червоний / синій
                                           └──► задача pid ──► ШІМ ──► RC-коло ──► АЦП ──┐
                                                    ▲                                      │
                                                    └──────────── зворотний зв'язок ───────┘
```

### Шари коду

| Шар | Файли | Правило |
|---|---|---|
| Конфігурація | `app_config.h` | усі числа й піни в одному місці |
| BSP (Board Support Package) | `bsp_gpio` | єдиний, хто знає, як пін працює фізично |
| Драйвери | `drv_*` | один пристрій = один модуль |
| Алгоритми | `alg_pid` | чиста математика, без заліза |
| Сервіси | `svc_event_bus` | інфраструктура для зв'язку модулів |
| Застосунок | `app_*` | логіка і задачі FreeRTOS |

**Головне правило:** кожен шар знає лише про шари **під** собою. PID не знає, що є ШІМ; автомат мотора не знає, що є реле.

### Можливий розподіл за часом

| Лекція | Розділи | Тема |
|---|---|---|
| 1 | 0–9 | периферія і драйвери: як мікроконтролер «бачить» і «керує» залізом |
| 2 | 10 | PID: математика і дискретна реалізація |
| 3 | 11–18 | FreeRTOS, архітектура, патерни, тестування |

---

## 1. main.c: налаштування периферії (CubeMX)

Цей файл генерує CubeMX. Студентам треба пояснити **одне правило**: свій код пишемо **лише між** `/* USER CODE BEGIN ... */` і `/* USER CODE END ... */`, бо все поза маркерами CubeMX перезапише при наступній генерації.

### 1.1 Послідовність у `main()`

```c
HAL_Init();              // скидання периферії, системний тік HAL
SystemClock_Config();    // тактування 84 МГц
MX_GPIO_Init();          // піни LED, реле, кнопки
MX_TIM4_Init();          // енкодер
MX_ADC1_Init();          // АЦП
MX_TIM3_Init();          // ШІМ
osKernelInitialize();    // підготовка FreeRTOS
App_Init();              // ← НАШ КОД (між маркерами RTOS_THREADS)
osKernelStart();         // старт планувальника — назад не повертається
```

| Рядок | Пояснення |
|---|---|
| `HAL_Init()` | скидає всю периферію в початковий стан, налаштовує пріоритети переривань і запускає таймер для `HAL_Delay`/`HAL_GetTick` (у нас TIM11) |
| `SystemClock_Config()` | перемикає процесор з початкових 16 МГц на 84 МГц |
| `MX_..._Init()` | кожна функція налаштовує один периферійний блок так, як виставили в CubeMX |
| `osKernelInitialize()` | готує внутрішні структури FreeRTOS, але задачі ще не працюють |
| `App_Init()` | наш код: створює драйвери, черги, задачі |
| `osKernelStart()` | запускає планувальник. Відтепер процесор виконує лише задачі; `while(1)` нижче ніколи не виконується |

### 1.2 Дескриптори периферії

```c
ADC_HandleTypeDef hadc1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
```

**Handle** — це структура, що описує один периферійний блок: адресу його регістрів (`Instance`), налаштування (`Init`) і поточний стан. Кожна функція HAL приймає вказівник на handle: `HAL_ADC_Start(&hadc1)`. Наш код знаходить ці змінні через `extern` у `app.c`.

### 1.3 `SystemClock_Config()`: тактування

```c
RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
RCC_OscInitStruct.HSEState = RCC_HSE_ON;
RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
RCC_OscInitStruct.PLL.PLLM = 25;
RCC_OscInitStruct.PLL.PLLN = 336;
RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
```

| Рядок | Пояснення |
|---|---|
| `OscillatorType = HSE` | джерело — зовнішній кварц 25 МГц на платі (точніший за внутрішній HSI) |
| `PLLSource = HSE` | кварц іде на вхід PLL — схеми множення частоти |
| `PLLM = 25` | ділимо 25 МГц на 25, отримуємо 1 МГц (PLL любить вхід 1–2 МГц) |
| `PLLN = 336` | множимо: 1 × 336 = 336 МГц (внутрішня частота PLL) |
| `PLLP = DIV4` | ділимо: 336 ÷ 4 = **84 МГц**, це частота ядра |

```c
RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);
HAL_RCC_EnableCSS();
```

| Рядок | Пояснення |
|---|---|
| `SYSCLKSource = PLLCLK` | процесор тактується від PLL. Якщо тут лишити HSI, буде 16 МГц |
| `AHB DIV1` | шина ядра й пам'яті: 84 МГц |
| `APB1 DIV2` | шина повільної периферії: 42 МГц (її максимум). Таймери на APB1 (TIM3, TIM4) автоматично отримують ×2 = **84 МГц** |
| `APB2 DIV1` | шина швидкої периферії (АЦП, TIM11): 84 МГц |
| `FLASH_LATENCY_2` | флеш-пам'ять повільніша за ядро, тому при кожному читанні процесор чекає 2 такти |
| `EnableCSS()` | Clock Security System: якщо кварц відмовить, мікроконтролер сам перейде на HSI і не зависне |

### 1.4 `MX_TIM3_Init()`: ШІМ (вихід регулятора)

```c
htim3.Init.Prescaler = 0;
htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
htim3.Init.Period = 4199;
htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
...
sConfigOC.OCMode = TIM_OCMODE_PWM1;
sConfigOC.Pulse = 0;
sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);
HAL_TIM_MspPostInit(&htim3);
```

| Рядок | Пояснення |
|---|---|
| `Prescaler = 0` | дільник (0+1) = 1: лічильник рахує на 84 МГц |
| `CounterMode = UP` | лічильник рахує вгору 0, 1, 2 … |
| `Period = 4199` | …до 4199, потім скидається в 0. Період = 4200 тактів, частота = 84 000 000 ÷ 4200 = **20 кГц** |
| `AutoReloadPreload = ENABLE` | нове значення заповнення застосовується з **наступного** періоду, а не посеред поточного. Імпульси не «рвуться» |
| `OCMode = PWM1` | режим ШІМ 1: вихід = 1, поки лічильник **менший** за регістр порівняння CCR, інакше 0 |
| `Pulse = 0` | початковий CCR = 0, тобто заповнення 0 %: на старті на RC-коло нічого не подається |
| `OCPolarity = HIGH` | «активний» рівень — високий (3,3 В) |
| `HAL_TIM_MspPostInit` | підключає фізичний пін **PA6** до каналу таймера (альтернативна функція AF2) |

**Для лекції:** намалюйте «пилку» лічильника і горизонтальну лінію CCR. Вихід високий, поки пилка нижче лінії.

### 1.5 `MX_TIM4_Init()`: енкодер

```c
htim4.Init.Period = 65535;
sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
sConfig.IC1Filter = 10;
sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
sConfig.IC2Filter = 10;
HAL_TIM_Encoder_Init(&htim4, &sConfig);
```

| Рядок | Пояснення |
|---|---|
| `Period = 65535` | лічильник 16-бітний, рахує 0…65535 і «перекидається» |
| `EncoderMode = TI12` | таймер рахує фронти **обох** сигналів енкодера (A = PB6, B = PB7). На одне клацання KY-040 — **4** зміни лічильника |
| `ICxPolarity = RISING` | базова полярність. Напрямок лічби (+ чи −) таймер визначає сам: котрий із сигналів A/B змінився першим |
| `ICxFilter = 10` | апаратний фільтр: імпульс зараховується, лише якщо він стабільний кілька тактів. Відсікає брязкіт контактів |

**Для лекції:** намалюйте дві прямокутні хвилі A і B, зсунуті на чверть періоду (квадратурний сигнал). Якщо A випереджає B, лічильник росте; якщо B випереджає A, спадає.

### 1.6 `MX_ADC1_Init()`: АЦП (вхід регулятора)

```c
hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
hadc1.Init.Resolution = ADC_RESOLUTION_12B;
hadc1.Init.ScanConvMode = DISABLE;
hadc1.Init.ContinuousConvMode = DISABLE;
hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
hadc1.Init.NbrOfConversion = 1;
hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
sConfig.Channel = ADC_CHANNEL_1;
sConfig.Rank = 1;
sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
```

| Рядок | Пояснення |
|---|---|
| `PCLK2_DIV4` | 84 ÷ 4 = 21 МГц тактування АЦП (максимум для F401 — 36 МГц) |
| `Resolution = 12B` | 2¹² = 4096 рівнів, крок 3,3 В ÷ 4096 ≈ **0,8 мВ** |
| `ScanConvMode = DISABLE` | вимірюємо один канал, а не список каналів |
| `ContinuousConvMode = DISABLE` | одне вимірювання на одну команду, без безперервної роботи |
| `SOFTWARE_START` | запуск функцією `HAL_ADC_Start()` з програми |
| `DataAlign = RIGHT` | 12 біт результату в молодших розрядах, число 0…4095 |
| `EOC_SINGLE_CONV` | прапорець «готово» після кожного перетворення |
| `Channel = 1` | канал 1 = пін **PA1** (PA0 не беремо: на Black Pill там кнопка) |
| `SamplingTime = 84 cycles` | внутрішній конденсатор АЦП заряджається 84 такти ≈ 4 мкс. Сигнал іде через 100 кОм, тож потрібен довгий час вибірки. З 3 тактами показання були б занижені |

### 1.7 `MX_GPIO_Init()`: цифрові піни

```c
HAL_GPIO_WritePin(LED_HEART_GPIO_Port, LED_HEART_Pin, GPIO_PIN_SET);
HAL_GPIO_WritePin(GPIOB, LED_RED_Pin|LED_BLUE_Pin, GPIO_PIN_RESET);
HAL_GPIO_WritePin(GPIOB, RELAY_CW_Pin|RELAY_CCW_Pin, GPIO_PIN_SET);
...
GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;   // (для реле краще OUTPUT_OD)
...
GPIO_InitStruct.Pin = ENC_SW_Pin;
GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
GPIO_InitStruct.Pull = GPIO_PULLUP;
```

| Рядок | Пояснення |
|---|---|
| `__HAL_RCC_GPIOx_CLK_ENABLE()` | вмикає тактування порту. Без цього порт «мертвий», запис у регістри ігнорується |
| `WritePin(... SET)` для PC13 | вбудований LED світиться від **0**, тому SET = вимкнений |
| `WritePin(... RESET)` для LED | зовнішні LED вимкнені на старті |
| `WritePin(... SET)` для реле | модулі реле Low-trigger: 1 = вимкнене |
| **спершу WritePin, потім Init** | пін стає виходом **уже** в безпечному стані, і реле не клацне при старті |
| `OUTPUT_PP` / `OUTPUT_OD` | push-pull активно тягне і до 0, і до 3,3 В; open-drain тягне лише до 0, а «1» — це відпущений пін (його підтягує модуль реле до 5 В) |
| `INPUT + PULLUP` | кнопка енкодера замикає пін на GND. Внутрішній резистор тримає 1, коли кнопку не натиснуто |

### 1.8 Службові функції

```c
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM11) { HAL_IncTick(); }
}
```

TIM11 кожну 1 мс збільшує лічильник часу HAL. Окремий таймер потрібен, бо **SysTick** (системний таймер ядра) забрала FreeRTOS для перемикання задач.

```c
void Error_Handler(void) { __disable_irq(); while (1) {} }
```

Сюди потрапляємо, якщо ініціалізація периферії не вдалася. Переривання вимикаються, і плата «замерзає». У дебагері одразу видно, де зупинилися.

---

## 2. app_config.h: єдине місце налаштувань

```c
7:  #ifndef APP_CONFIG_H
8:  #define APP_CONFIG_H
10: #include "main.h"
```

| Рядок | Пояснення |
|---|---|
| 7–8 | **include guard**: якщо файл підключили двічі, вміст не дублюється |
| 10 | `main.h` дає доступ до назв портів і пінів (`GPIOB`, `GPIO_PIN_12`) |

```c
13: #define CFG_LED_RED_PORT      GPIOB
14: #define CFG_LED_RED_PIN       GPIO_PIN_12
...
37: #define CFG_RELAY_ACTIVE_LOW  1
40: #define CFG_ENC_COUNTS_PER_DETENT  4
45: #define CFG_DEADTIME_MS        150u
47: #define CFG_IDLE_TIMEOUT_MS    600u
59: #define CFG_REG_PERIOD_MS        10u
66: #define CFG_PID_KP             2.0f
67: #define CFG_PID_KI             3.0f
68: #define CFG_PID_KD             0.2f
69: #define CFG_PID_DFILT_TAU_S    0.02f
71: #define CFG_SP_STEP_MV          100
73: #define CFG_SP_MAX_MV          3000
82: #define CFG_PRIO_REGULATOR    (tskIDLE_PRIORITY + 4)
83: #define CFG_PRIO_MOTOR_CTRL   (tskIDLE_PRIORITY + 3)
84: #define CFG_PRIO_INPUT        (tskIDLE_PRIORITY + 2)
```

| Група | Пояснення |
|---|---|
| Піни (13–27) | якщо на макеті світлодіод на іншому піні, міняємо **тут**, а не в драйвері |
| `CFG_RELAY_ACTIVE_LOW` | тип модуля реле. Від нього залежить, 0 чи 1 вмикає реле |
| `CFG_ENC_COUNTS_PER_DETENT` | 4 імпульси таймера = 1 клацання |
| `CFG_DEADTIME_MS` | пауза між напрямками мотора |
| `CFG_IDLE_TIMEOUT_MS` | мотор зупиняється через 0,6 с після останнього клацання |
| `CFG_REG_PERIOD_MS = 10` | PID рахує 100 разів на секунду (Ts = 0,01 с) |
| `CFG_PID_KP/KI/KD` | коефіцієнти, підібрані на моделі RC-кола |
| `CFG_PID_DFILT_TAU_S` | стала часу фільтра D-складової (20 мс) |
| `CFG_SP_*` | крок уставки 100 мВ, межі 0…3000 мВ |
| Пріоритети | pid (4) > motor (3) > input (2) |

**Суфікси `u` і `f`:** `150u` — беззнакове ціле, `2.0f` — `float` (а не `double`). На Cortex-M4 апаратний FPU працює лише з `float`, а `double` рахувався б програмно й повільно.

**Навіщо окремий файл:** немає «магічних чисел» у коді. Усі параметри, які можна захотіти змінити, лежать в одному місці.

---

## 3. bsp_gpio: цифрові входи/виходи

### 3.1 Заголовок: опис піна

```c
typedef struct {
    GPIO_TypeDef *port;
    uint16_t      pin;
    bool          active_low;
    bool          open_drain;
} DigitalOut_t;
```

| Поле | Пояснення |
|---|---|
| `port`, `pin` | який саме пін (наприклад, GPIOB, пін 14) |
| `active_low` | якщо `true`, «увімкнено» означає логічний **0** на піні |
| `open_drain` | тип виходу: тягне лише до 0 або в обидва боки |

**Навіщо:** драйвер реле хоче сказати «увімкни реле», а не думати, чи це 0, чи 1. Полярність — фізична властивість підключення, і вона описується в даних, а не в логіці.

### 3.2 bsp_gpio.c

```c
10: static void enable_port_clock(GPIO_TypeDef *port)
11: {
12:     if      (port == GPIOA) __HAL_RCC_GPIOA_CLK_ENABLE();
13:     else if (port == GPIOB) __HAL_RCC_GPIOB_CLK_ENABLE();
14:     else if (port == GPIOC) __HAL_RCC_GPIOC_CLK_ENABLE();
15:     else if (port == GPIOH) __HAL_RCC_GPIOH_CLK_ENABLE();
16: }
```

| Рядок | Пояснення |
|---|---|
| 10 | `static` означає, що функцію видно лише в цьому файлі (приватна) |
| 12–15 | вмикаємо тактування потрібного порту. Модуль не покладається на те, що CubeMX це вже зробив |

```c
18: static GPIO_PinState level_for(const DigitalOut_t *d, bool active)
19: {
21:     return (active != d->active_low) ? GPIO_PIN_SET : GPIO_PIN_RESET;
22: }
```

| Рядок | Пояснення |
|---|---|
| 21 | **ключовий рядок**: логічне «виключне АБО» (XOR). Таблиця нижче |

| active | active_low | active != active_low | На піні |
|---|---|---|---|
| true | false | true | 1 (SET) |
| false | false | false | 0 |
| true | true | false | **0** (для Low-trigger реле це «увімкнено») |
| false | true | true | 1 |

```c
24: void DigitalOut_Init(const DigitalOut_t *d)
25: {
26:     enable_port_clock(d->port);
31:     HAL_GPIO_WritePin(d->port, d->pin, level_for(d, false));
33:     GPIO_InitTypeDef g = {0};
34:     g.Pin   = d->pin;
35:     g.Mode  = d->open_drain ? GPIO_MODE_OUTPUT_OD : GPIO_MODE_OUTPUT_PP;
36:     g.Pull  = GPIO_NOPULL;
37:     g.Speed = GPIO_SPEED_FREQ_LOW;
38:     HAL_GPIO_Init(d->port, &g);
39: }
```

| Рядок | Пояснення |
|---|---|
| 31 | **спершу** записуємо в регістр виходу **неактивний** рівень |
| 33 | `= {0}` обнуляє всю структуру, щоб не лишилося «сміття» в полях, які ми не заповнюємо |
| 35 | тернарний оператор: open-drain або push-pull залежно від опису |
| 36 | без внутрішніх підтяжок (для реле підтяжка є на модулі) |
| 37 | низька швидкість фронтів: менше перешкод, для LED і реле швидкість не потрібна |
| 38 | **лише тепер** пін стає виходом, і на ньому вже правильний рівень. Реле не клацне при старті |

```c
41: void DigitalOut_Set(const DigitalOut_t *d, bool active)
43:     HAL_GPIO_WritePin(d->port, d->pin, level_for(d, active));
```

Уся функція — один рядок: перевести «логічне» в «фізичне» і записати. `HAL_GPIO_WritePin` пише в регістр **BSRR** атомарно, одним записом, тож безпечно навіть із різних задач.

```c
56: bool DigitalIn_IsActive(const DigitalIn_t *d)
58:     bool high = (HAL_GPIO_ReadPin(d->port, d->pin) == GPIO_PIN_SET);
59:     return high != d->active_low;
```

Те саме XOR, але для входу: кнопка на GND із підтяжкою (`active_low = true`) при натисканні дає 0, а функція повертає `true`.

**Патерн:** **Adapter** — простий інтерфейс «увімкни/вимкни» поверх складного HAL.

---

## 4. drv_led і drv_relay: найпростіші драйвери

### drv_led.c

```c
 6: void Led_Init(Led_t *led, DigitalOut_t out)
 8:     led->out = out;
 9:     led->on  = false;
10:     DigitalOut_Init(&led->out);
13: void Led_Set(Led_t *led, bool on)
15:     led->on = on;
16:     DigitalOut_Set(&led->out, on);
19: void Led_Toggle(Led_t *led)
21:     Led_Set(led, !led->on);
```

| Рядок | Пояснення |
|---|---|
| 8 | об'єкт LED **копіює** опис піна в себе, тож кожен LED — самодостатній об'єкт |
| 9 | пам'ятаємо стан |
| 15 | запам'ятовуємо новий стан **до** запису на пін |
| 21 | Toggle — це Set протилежного стану. Для цього й потрібне поле `on`: читати вихідний пін назад ненадійно |

**Для лекції:** це «об'єкт» на C. Структура — це дані (поля), функції з першим параметром-вказівником — це методи. Як `this` у C++.

### drv_relay.c

Точно та сама структура, що й у LED: `Relay_Init`, `Relay_Set`, `Relay_IsOn`. Коментар у рядку 10 важливий: `DigitalOut_Init` гарантує, що реле **вимкнене** відразу після ініціалізації.

**Питання студентам:** чому не використати для реле просто `Led_t`? Відповідь: це різні пристрої з різним змістом. Завтра реле може отримати, скажімо, лічильник спрацювань або захист від частих перемикань, а LED — ні. Окремий тип робить код зрозумілим.

---

## 5. drv_motor.h + drv_motor_relay: інтерфейс мотора

### 5.1 drv_motor.h: інтерфейс («віртуальні методи» на C)

```c
13: typedef enum {
14:     MOTOR_DIR_CW = 0,
15:     MOTOR_DIR_CCW
16: } MotorDir_t;
18: typedef struct {
19:     void (*run) (void *self, MotorDir_t dir);
20:     void (*stop)(void *self);
21: } MotorOps_t;
23: typedef struct {
24:     const MotorOps_t *ops;
25:     void             *self;
26: } Motor_t;
28: static inline void Motor_Run (const Motor_t *m, MotorDir_t dir) { m->ops->run(m->self, dir); }
29: static inline void Motor_Stop(const Motor_t *m)                 { m->ops->stop(m->self); }
```

| Рядок | Пояснення |
|---|---|
| 13–16 | `enum`: іменовані константи замість «магічних» 0 і 1 |
| 19 | `void (*run)(...)` — **вказівник на функцію**. Змінна, в якій зберігається адреса функції |
| 18–21 | таблиця функцій = **vtable** (як віртуальні методи в C++) |
| 24 | `ops` вказує на таблицю функцій конкретної реалізації |
| 25 | `self` — вказівник на дані конкретної реалізації. `void *` = «вказівник на що завгодно» |
| 28–29 | зручні обгортки: `Motor_Run(&m, CW)` викликає ту функцію, адресу якої записано в таблиці |

**Для лекції (ключова ідея):** автомат станів мотора викликає `Motor_Run()` і **не знає**, що всередині два реле. Можна написати `drv_motor_hbridge.c` для драйвера MX1616 з тими ж двома функціями, і автомат не зміниться.

**Патерни і принципи:** **Strategy / Interface**, **Open/Closed** (система відкрита для розширення і закрита для змін), **Dependency Inversion** (логіка залежить від абстракції, а не від реле).

### 5.2 drv_motor_relay.c: конкретна реалізація

```c
12: static void relay_motor_stop(void *self)
14:     MotorRelay_t *m = (MotorRelay_t *)self;
15:     Relay_Set(m->relay_cw,  false);
16:     Relay_Set(m->relay_ccw, false);
```

| Рядок | Пояснення |
|---|---|
| 14 | перетворюємо «анонімний» `void *` назад на наш тип. Ми знаємо, що це `MotorRelay_t` |
| 15–16 | обидва реле вимкнені: обидва дроти мотора на GND, мотор гальмує |

```c
19: static void relay_motor_run(void *self, MotorDir_t dir)
21:     MotorRelay_t *m = (MotorRelay_t *)self;
22:     Relay_t *on  = (dir == MOTOR_DIR_CW) ? m->relay_cw  : m->relay_ccw;
23:     Relay_t *off = (dir == MOTOR_DIR_CW) ? m->relay_ccw : m->relay_cw;
25:     Relay_Set(off, false);
26:     Relay_Set(on,  true);
```

| Рядок | Пояснення |
|---|---|
| 22–23 | вибираємо, яке реле ввімкнути, а яке вимкнути |
| 25 | **спершу вимикаємо** протилежне (break) |
| 26 | **потім вмикаємо** потрібне (make). Принцип **break-before-make**: ніколи обидва одночасно |

```c
29: static const MotorOps_t RELAY_MOTOR_OPS = {
30:     .run  = relay_motor_run,
31:     .stop = relay_motor_stop,
32: };
41: Motor_t MotorRelay_AsMotor(MotorRelay_t *m)
43:     Motor_t motor = { .ops = &RELAY_MOTOR_OPS, .self = m };
44:     return motor;
```

| Рядок | Пояснення |
|---|---|
| 29–32 | заповнюємо vtable адресами наших функцій. `const` означає, що таблиця лежить у флеш-пам'яті й не займає ОЗП |
| 30–31 | `.run = ...` — іменована ініціалізація полів (стандарт C99) |
| 43 | «упаковуємо» реле-мотор в абстрактний `Motor_t`: таблиця функцій + вказівник на дані |

---

## 6. drv_encoder: енкодер на апаратному таймері

```c
10: void Encoder_Init(Encoder_t *e, TIM_HandleTypeDef *htim,
11:                   int32_t counts_per_detent, bool invert)
13:     e->htim = htim;
14:     e->counts_per_detent = (counts_per_detent > 0) ? counts_per_detent : 1;
15:     e->invert    = invert;
16:     e->remainder = 0;
18:     HAL_TIM_Encoder_Start(htim, TIM_CHANNEL_ALL);
19:     e->last_cnt = (uint16_t)__HAL_TIM_GET_COUNTER(htim);
```

| Рядок | Пояснення |
|---|---|
| 14 | захист від ділення на 0: якщо передали 0, беремо 1 |
| 18 | запускаємо таймер у режимі енкодера на обох каналах. Відтепер він рахує **сам**, без процесора |
| 19 | запам'ятовуємо стартове значення лічильника |

```c
22: int32_t Encoder_ReadDetents(Encoder_t *e)
24:     uint16_t now  = (uint16_t)__HAL_TIM_GET_COUNTER(e->htim);
25:     int16_t  diff = (int16_t)(now - e->last_cnt);
26:     e->last_cnt = now;
28:     e->remainder += e->invert ? -diff : diff;
30:     int32_t detents = e->remainder / e->counts_per_detent;
31:     e->remainder   -= detents * e->counts_per_detent;
32:     return detents;
```

| Рядок | Пояснення |
|---|---|
| 24 | читаємо поточне значення лічильника (регістр CNT таймера) |
| 25 | **найцікавіший рядок.** Різниця двох `uint16_t`, приведена до `int16_t`, дає правильний знак навіть через «перекидання» лічильника. Приклад: було 2, стало 65534 (крутнули назад на 4). `65534 − 2 = 65532`, а як `int16_t` це **−4**. Правильно! |
| 26 | запам'ятовуємо для наступного разу |
| 28 | накопичуємо імпульси; при `invert` міняємо знак |
| 30 | скільки **повних** клацань набралося (ділення цілих у C відкидає дробову частину в бік нуля) |
| 31 | залишок зберігаємо: якщо прийшло 6 імпульсів, повертаємо 1 клацання, а 2 імпульси лишаються на наступний раз |
| 32 | повертаємо кількість клацань зі знаком: +2 = два за годинниковою, −1 = одне проти |

**Для лекції:** чому апаратний таймер, а не переривання на піні? Таймер рахує **кожен** імпульс, навіть якщо процесор зайнятий. При перериваннях на кожен фронт швидке кручення завантажувало б процесор і могло губити кроки.

---

## 7. drv_button: антибрязкіт

**Проблема:** механічна кнопка при натисканні «брязкає»: за 1–10 мс контакти замикаються і розмикаються десятки разів. Без фільтра одне натискання зарахувалося б кілька разів.

```c
 6: void Button_Init(Button_t *b, DigitalIn_t in, uint8_t threshold)
 9:     b->threshold = threshold ? threshold : 1;
10:     b->count     = 0;
11:     DigitalIn_Init(&b->in);
12:     b->stable    = DigitalIn_IsActive(&b->in);
```

| Рядок | Пояснення |
|---|---|
| 9 | скільки однакових відліків поспіль потрібно (у нас 3 × 10 мс = 30 мс) |
| 12 | стартовий «підтверджений» стан беремо з піна, щоб кнопка, затиснута при старті, не дала хибного натискання |

```c
15: bool Button_PollPressed(Button_t *b)
17:     bool raw = DigitalIn_IsActive(&b->in);
19:     if (raw == b->stable) {
20:         b->count = 0;
21:         return false;
22:     }
23:     if (++b->count < b->threshold) {
24:         return false;
25:     }
26:     b->count  = 0;
27:     b->stable = raw;
28:     return raw;
```

| Рядок | Пояснення |
|---|---|
| 17 | «сирий» стан піна зараз |
| 19–21 | нічого не змінилося, скидаємо лічильник. Одиничний «стрибок» від брязкоту так і відфільтровується |
| 23 | стан відрізняється, але ще не підтверджений. `++b->count` спершу збільшує, потім порівнює |
| 26–27 | зміна трималася 3 відліки поспіль, приймаємо новий стабільний стан |
| 28 | повертаємо `true` **лише** якщо новий стан «натиснуто». Відпускання кнопки не рахується |

**Для лекції:** таблиця на дошці з послідовністю відліків `1 0 1 0 0 0 0` і тим, як змінюється `count`.

---

## 8. drv_pwm: вихід регулятора

```c
 6: void Pwm_Init(Pwm_t *p, TIM_HandleTypeDef *htim, uint32_t channel)
 8:     p->htim    = htim;
 9:     p->channel = channel;
10:     __HAL_TIM_SET_COMPARE(htim, channel, 0);
11:     HAL_TIM_PWM_Start(htim, channel);
```

| Рядок | Пояснення |
|---|---|
| 10 | CCR = 0, тобто 0 % **до** старту таймера |
| 11 | запускаємо генерацію ШІМ. Далі таймер працює сам |

```c
14: void Pwm_SetDuty(Pwm_t *p, float duty01)
16:     if (duty01 < 0.0f) duty01 = 0.0f;
17:     if (duty01 > 1.0f) duty01 = 1.0f;
18:     uint32_t period = __HAL_TIM_GET_AUTORELOAD(p->htim) + 1u;
19:     __HAL_TIM_SET_COMPARE(p->htim, p->channel, (uint32_t)(duty01 * (float)period));
```

| Рядок | Пояснення |
|---|---|
| 16–17 | обмеження 0…1 (насичення). Захист від неправильного значення |
| 18 | читаємо період з таймера (ARR + 1 = 4200). Драйвер не залежить від того, що виставили в CubeMX |
| 19 | CCR = заповнення × період. Наприклад, 0,45 × 4200 = 1890. Один запис у регістр, і все |

**Для лекції:** драйвер переводить абстрактне «45 %» у конкретне число регістра. PID працює з частками 0…1 і не знає про таймери.

---

## 9. drv_adc: вхід регулятора

```c
17: uint16_t AdcIn_ReadMv(AdcIn_t *a)
19:     uint32_t acc = 0, n = 0;
20:     for (uint8_t i = 0; i < a->oversample; i++) {
21:         HAL_ADC_Start(a->hadc);
22:         if (HAL_ADC_PollForConversion(a->hadc, 1) == HAL_OK) {
23:             acc += HAL_ADC_GetValue(a->hadc);
24:             n++;
25:         }
26:     }
27:     if (n == 0) return 0;
28:     return (uint16_t)(((acc / n) * a->vref_mv) / 4095u);
```

| Рядок | Пояснення |
|---|---|
| 19 | `acc` — сума, `n` — кількість вдалих вимірювань. `uint32_t`, бо 8 × 4095 не вміщається в 16 біт |
| 20 | 8 вимірювань поспіль (**oversampling**) |
| 21 | запустити перетворення |
| 22 | чекати результат не довше 1 мс. Перевіряємо `HAL_OK`, бо помилку не можна додавати в суму |
| 23 | додати результат 0…4095 |
| 27 | захист від ділення на 0, якщо всі 8 вимірювань не вдалися |
| 28 | середнє → мілівольти: `код × 3300 / 4095`. Спершу множимо, потім ділимо, щоб не втратити точність у цілих числах |

**Для лекції:** усереднення N вимірювань зменшує випадковий шум у √N разів (√8 ≈ 2,8). Ціна — 8 × 4,6 мкс ≈ 40 мкс кожні 10 мс, тобто менше 0,5 % процесорного часу.

---

## 10. alg_pid: сам PID-регулятор

**Найважливіший розділ.** Модуль не залежить від заліза й RTOS, тому його перевіряють тестами на ПК.

### 10.1 Теорія (на дошці)

Неперервний PID:

```
u(t) = Kp·e(t) + Ki·∫e(t)dt + Kd·de(t)/dt,        e = уставка − вимір
```

| Складова | «Дивиться» на | Що робить |
|---|---|---|
| **P** | похибку зараз | що більша похибка, то сильніше тиснемо |
| **I** | накопичену похибку в минулому | прибирає **статичну похибку**: навіть мала похибка з часом накопичується |
| **D** | швидкість зміни | «гальмує», коли вихід швидко наближається: менше перелітання |

Мікроконтролер рахує **дискретно**, раз на Ts = 0,01 с: інтеграл стає сумою, похідна — різницею.

### 10.2 Стан регулятора (alg_pid.h)

```c
float kp, ki, kd;          // коефіцієнти
float ts;                  // період дискретизації, с
float tau;                 // стала часу фільтра D
float out_min, out_max;    // межі виходу (0…1 = 0…100 % ШІМ)
float integ;               // накопичена I-складова (вже помножена на Ki)
float diff;                // відфільтрована D-складова
float prev_meas;           // попереднє вимірювання (для похідної)
bool  first;               // перший виклик?
float p_term, i_term, d_term;  // для діагностики
```

**Для лекції:** регулятор має **пам'ять** (`integ`, `diff`, `prev_meas`), тому він «об'єкт», а не просто функція.

### 10.3 Ініціалізація

```c
 9: static inline float clampf(float x, float lo, float hi)
11:     return (x < lo) ? lo : (x > hi) ? hi : x;
14: void PID_Init(...)
17:     pid->ts      = ts;
...
21:     PID_SetGains(pid, kp, ki, kd);
22:     PID_Reset(pid);
33: void PID_Reset(PID_Controller_t *pid)
35:     pid->integ = 0.0f;
36:     pid->diff  = 0.0f;
37:     pid->prev_meas = 0.0f;
38:     pid->first = true;
```

| Рядок | Пояснення |
|---|---|
| 9–11 | `clampf` обмежує число межами [lo, hi]. `static inline` — компілятор вставляє тіло на місце виклику, без накладних витрат |
| 21–22 | ініціалізація складається з двох кроків, які можна викликати й окремо |
| 35–38 | скидання «пам'яті» регулятора |

### 10.4 `PID_Update()`: рядок за рядком

```c
44:     if (pid->first) {
45:         pid->prev_meas = measurement;
46:         pid->first = false;
47:     }
```

На першому виклику немає «попереднього» вимірювання. Якби `prev_meas` було 0, похідна на першому кроці дала б величезний стрибок. Тому вважаємо, що попереднє = поточне.

```c
49:     const float err = setpoint - measurement;
52:     const float p = pid->kp * err;
```

| Рядок | Пояснення |
|---|---|
| 49 | **похибка**: наскільки ми не там, де треба. Уставка 1,6 В, вимір 1,5 В, отже err = +0,1 |
| 52 | **P-складова**: 2,0 × 0,1 = 0,2, тобто +20 % ШІМ |

```c
56:     const float a = (2.0f * pid->tau - pid->ts) / (2.0f * pid->tau + pid->ts);
57:     const float b = (2.0f * pid->kd)           / (2.0f * pid->tau + pid->ts);
58:     pid->diff = a * pid->diff - b * (measurement - pid->prev_meas);
59:     pid->prev_meas = measurement;
```

| Рядок | Пояснення |
|---|---|
| 56–57 | коефіцієнти дискретного фільтра нижніх частот. Формули — результат **перетворення Тастіна** (білінійного) для ланки `Kd·s / (τs + 1)`. Для τ = 0,02 і Ts = 0,01: a = 0,6, b = 8 |
| 58 | **D-складова з фільтром**: 60 % попереднього значення (згладжування) мінус нова зміна вимірювання |
| 58, знак «−» | похідна береться **від вимірювання**, а не від похибки. Оскільки e = sp − y, то de/dt = −dy/dt (коли уставка стала). Звідси мінус |
| 59 | запам'ятовуємо вимір для наступного кроку |

**Для лекції: чому D від вимірювання, а не від похибки?** Коли енкодером стрибком змінюють уставку, похибка теж змінюється стрибком, і її «похідна» стає нескінченною. ШІМ миттєво злетів би в 100 %: це так званий **derivative kick**. Вимірювання ж фізично не може стрибнути, бо конденсатор заряджається плавно. У тесті це перевіряється.

**Для лекції: навіщо фільтр?** Похідна підсилює шум. АЦП тремтить на ±2 мВ, і без фільтра D-складова «смикала» б ШІМ щотакту.

```c
62:     const float di = pid->ki * err * pid->ts;
63:     const float u_unsat = p + pid->integ + di + pid->diff;
```

| Рядок | Пояснення |
|---|---|
| 62 | **приріст інтеграла** за один такт: Ki × похибка × Ts (площа прямокутника під графіком похибки) |
| 63 | яким був би вихід **без обмеження**. Потрібно, щоб зрозуміти, чи ми в насиченні |

```c
67:     const bool sat_hi = (u_unsat > pid->out_max) && (err > 0.0f);
68:     const bool sat_lo = (u_unsat < pid->out_min) && (err < 0.0f);
69:     if (!sat_hi && !sat_lo) {
70:         pid->integ += di;
71:     }
72:     pid->integ = clampf(pid->integ, pid->out_min, pid->out_max);
```

| Рядок | Пояснення |
|---|---|
| 67 | «вихід уже вище 100 % **і** похибка штовхає ще вище» |
| 68 | те саме для нижньої межі (0 %) |
| 69–71 | **anti-windup** (умовне інтегрування): інтегруємо **лише** тоді, коли це має сенс |
| 72 | додатковий запобіжник: інтеграл не може вийти за межі 0…1 |

**Для лекції: що таке windup?** Уставку поставили недосяжною: ШІМ уже 100 %, а напруга ще не там. Без anti-windup інтеграл росте й росте. Коли уставку знизили, «накачаний» інтеграл ще довго тримає ШІМ на максимумі, і вихід сильно перелітає. У тестах без anti-windup: **14 % перелітання замість 0,3 %**.

```c
74:     const float u = clampf(p + pid->integ + pid->diff, pid->out_min, pid->out_max);
76:     pid->p_term = p;
77:     pid->i_term = pid->integ;
78:     pid->d_term = pid->diff;
79:     return u;
```

| Рядок | Пояснення |
|---|---|
| 74 | **остаточний вихід** = P + I + D, обмежений 0…1 |
| 76–78 | зберігаємо складові окремо, щоб бачити їх у дебагері (`g_reg.p/i/d`) |
| 79 | повертаємо заповнення ШІМ |

### 10.5 Чому `integ` зберігає вже помножене на Ki

```c
27:     /* integ вже містить Ki, тому зміна Ki "на льоту" не дає стрибка виходу */
```

Якби зберігали «чисту» суму похибок `Σe` і множили на Ki на кожному кроці, то при зміні Ki з 3 на 6 I-складова **миттєво подвоїлася** б, і ШІМ стрибнув би. А так нове Ki впливає лише на **майбутні** прирости, і перехід плавний.

### 10.6 Числовий приклад одного такту (на дошці)

Уставка 1,6 В, вимір 1,5 В, попередній вимір 1,49 В, integ = 0,45, diff = 0:

| Крок | Обчислення | Результат |
|---|---|---|
| err | 1,6 − 1,5 | 0,10 |
| P | 2,0 × 0,10 | 0,20 |
| D | 0,6 × 0 − 8 × (1,5 − 1,49) | −0,08 |
| di | 3,0 × 0,10 × 0,01 | 0,003 |
| integ | 0,45 + 0,003 | 0,453 |
| u | 0,20 + 0,453 − 0,08 | **0,573 → ШІМ 57,3 %** |

---

## 11. svc_event_bus: шина подій

### 11.1 Заголовок: теми

```c
typedef enum {
    TOPIC_ENCODER_STEP = 0,  /* value = +N / −N клацань */
    TOPIC_BUTTON,
    TOPIC_MOTOR_STATE,
    TOPIC_REG_SETTLED,
    TOPIC_COUNT
} BusTopic_t;

typedef void (*BusCallback_t)(void *ctx, BusTopic_t topic, int32_t value);
```

| Елемент | Пояснення |
|---|---|
| Теми | «канали», на які можна підписатися |
| `TOPIC_COUNT` | прийом: останній елемент enum = кількість тем. Зручно для перевірок і розмірів масивів |
| `BusCallback_t` | тип «функції-слухача», яку викличе шина |

### 11.2 Реалізація

```c
 7: typedef struct {
 8:     BusTopic_t    topic;
 9:     BusCallback_t cb;
10:     void         *ctx;
11: } Subscriber_t;
13: static Subscriber_t subs[EVENT_BUS_MAX_SUBSCRIBERS];
14: static uint8_t      n_subs;
```

| Рядок | Пояснення |
|---|---|
| 7–11 | один підписник: на що підписаний, кого викликати, і довільний контекст |
| 13 | **статичний** масив на 8 підписників. Без `malloc`: розмір відомий при компіляції |
| 14 | скільки вже підписалося. Статичні змінні в C автоматично дорівнюють 0 |

```c
16: bool EventBus_Subscribe(BusTopic_t topic, BusCallback_t cb, void *ctx)
18:     if (cb == NULL || topic >= TOPIC_COUNT || n_subs >= EVENT_BUS_MAX_SUBSCRIBERS)
19:         return false;
20:     subs[n_subs++] = (Subscriber_t){ .topic = topic, .cb = cb, .ctx = ctx };
21:     return true;
```

| Рядок | Пояснення |
|---|---|
| 18 | три перевірки: є функція, тема існує, є місце |
| 20 | `(Subscriber_t){...}` — **compound literal** (C99): створити структуру на льоту. `n_subs++` записує в поточну позицію і збільшує лічильник |

```c
24: void EventBus_Publish(BusTopic_t topic, int32_t value)
26:     for (uint8_t i = 0; i < n_subs; i++) {
27:         if (subs[i].topic == topic)
28:             subs[i].cb(subs[i].ctx, topic, value);
```

| Рядок | Пояснення |
|---|---|
| 26–28 | пройти всіх підписників і викликати тих, хто підписаний на цю тему |

**Для лекції: патерн Observer.** Задача вводу публікує «клацання +1» і **не знає**, що його отримають і мотор, і PID. Щоб додати дисплей, досить одного рядка `Subscribe`, а код видавця не змінюється.

**Обмеження (важливо для RTOS):**
- підписуємося лише **до старту** планувальника, тому масив під час роботи не змінюється і блокування не потрібні;
- колбек виконується **в задачі видавця**, тому має бути коротким і не блокуватися.

---

## 12. app_motor_fsm: автомат станів мотора

### 12.1 Діаграма станів (на дошці)

```
              STEP_CW                          STEP_CCW
   ┌─────────► RUN_CW ─────────┐    ┌──────── RUN_CCW ◄─────────┐
   │             │ STOP/таймаут │    │ STOP/таймаут │            │
   │             ▼   STEP_CCW   ▼    ▼   STEP_CW    ▼            │
 IDLE ◄──────────────────── SWITCHING ─────────────────────────┘
   ▲    STOP                (пауза 150 мс, потім вмикає обраний напрямок)
```

| Стан | Значення |
|---|---|
| IDLE | мотор стоїть |
| RUN_CW / RUN_CCW | крутиться за / проти годинникової |
| SWITCHING | мотор зупинено, чекаємо паузу перед реверсом |

| Подія | Звідки |
|---|---|
| STEP_CW / STEP_CCW | клацання енкодера |
| STOP | кнопка |
| TICK | кожні 10 мс, щоб рахувати час |

### 12.2 Допоміжні функції

```c
13: typedef void (*StateHandler_t)(MotorFsm_t *f, MotorEvent_t ev, uint32_t now);
16: static void enter(MotorFsm_t *f, MotorState_t s)
18:     if (f->state == s) return;
19:     f->state = s;
20:     if (f->on_state) f->on_state(f->cb_ctx, s);
```

| Рядок | Пояснення |
|---|---|
| 13 | тип «обробника стану»: функція, що отримує подію і час |
| 18 | вже в цьому стані, нічого не робимо (і не сповіщаємо даремно) |
| 19 | перехід у новий стан |
| 20 | сповіщаємо того, хто підписався (задача мотора передасть на шину, а індикатор увімкне LED) |

```c
23: static void motor_stop(MotorFsm_t *f, uint32_t now)
25:     Motor_Stop(&f->motor);
26:     f->t_stopped = now;
31: static void motor_start(MotorFsm_t *f, MotorDir_t dir)
33:     Motor_Run(&f->motor, dir);
34:     enter(f, dir == MOTOR_DIR_CW ? MS_RUN_CW : MS_RUN_CCW);
```

| Рядок | Пояснення |
|---|---|
| 25 | виклик через **інтерфейс** — автомат не знає про реле |
| 26 | запам'ятовуємо, **коли** зупинили, щоб відрахувати паузу |
| 33–34 | увімкнули мотор і перейшли у відповідний стан |

```c
45: static void request_direction(MotorFsm_t *f, MotorDir_t dir, uint32_t now)
47:     f->t_last_step = now;
48:     if ((uint32_t)(now - f->t_stopped) >= f->cfg.deadtime_ms) {
49:         motor_start(f, dir);
50:     } else {
51:         f->target = dir;
52:         enter(f, MS_SWITCHING);
53:     }
```

| Рядок | Пояснення |
|---|---|
| 48 | чи минуло 150 мс від останньої зупинки? Мотор міг зупинитися щойно і ще крутиться за інерцією |
| 49 | так: стартуємо одразу |
| 51–52 | ні: запам'ятовуємо бажаний напрямок і чекаємо в SWITCHING |

**Для лекції: чому `(uint32_t)(now - f->t_stopped)`?** Лічильник часу в мс переповнюється через ~49 днів (2³² мс). Віднімання беззнакових чисел дає правильну різницю **навіть через переповнення**. Порівнювати `now >= t_stopped + 150` було б помилкою. Тест `test_tick_wraparound` це перевіряє.

### 12.3 Обробники станів

```c
57: static void st_idle(MotorFsm_t *f, MotorEvent_t ev, uint32_t now)
59:     MotorDir_t dir;
60:     if (is_step(ev, &dir)) request_direction(f, dir, now);
```

У спокої реагуємо лише на клацання. STOP і TICK нічого не роблять.

```c
64: static void st_running(MotorFsm_t *f, MotorEvent_t ev, uint32_t now)
66:     MotorDir_t cur = (f->state == MS_RUN_CW) ? MOTOR_DIR_CW : MOTOR_DIR_CCW;
69:     if (is_step(ev, &dir)) {
70:         if (dir == cur) {
71:             f->t_last_step = now;
72:         } else {
73:             motor_stop(f, now);
74:             f->t_last_step = now;
75:             f->target = dir;
76:             enter(f, MS_SWITCHING);
77:         }
78:         return;
79:     }
81:     if (ev == MEV_STOP) { motor_stop(f, now); enter(f, MS_IDLE); return; }
87:     if (ev == MEV_TICK && f->cfg.idle_timeout_ms != 0 &&
88:         (uint32_t)(now - f->t_last_step) >= f->cfg.idle_timeout_ms) {
89:         motor_stop(f, now);
90:         enter(f, MS_IDLE);
```

| Рядок | Пояснення |
|---|---|
| 66 | один обробник на два стани: визначаємо поточний напрямок |
| 70–71 | клацання в той самий бік: продовжуємо, оновлюємо час |
| 73–76 | **реверс**: спершу зупинка, потім пауза. Ніколи не перемикаємо напрямок на ходу |
| 81 | кнопка: негайна зупинка |
| 87–90 | тайм-аут: енкодер не крутили 600 мс, отже зупинка |

```c
94: static void st_switching(...)
98:     if (is_step(ev, &dir)) { f->target = dir; f->t_last_step = now; return; }
104:    if (ev == MEV_STOP) { enter(f, MS_IDLE); return; }
109:    if (ev == MEV_TICK && (uint32_t)(now - f->t_stopped) >= f->cfg.deadtime_ms) {
110:        motor_start(f, f->target);
```

| Рядок | Пояснення |
|---|---|
| 98 | передумали під час паузи: беремо **останній** обраний напрямок |
| 104 | стоп: мотор уже вимкнений, просто переходимо в IDLE |
| 109–110 | пауза минула, стартуємо в обраному напрямку |

### 12.4 Таблиця переходів і диспетчер

```c
114: static const StateHandler_t STATE_HANDLERS[MS_COUNT] = {
115:     [MS_IDLE]      = st_idle,
116:     [MS_RUN_CW]    = st_running,
117:     [MS_RUN_CCW]   = st_running,
118:     [MS_SWITCHING] = st_switching,
119: };
136: void MotorFsm_Dispatch(MotorFsm_t *f, MotorEvent_t ev, uint32_t now_ms)
138:     if (f->state < MS_COUNT && STATE_HANDLERS[f->state] != NULL)
139:         STATE_HANDLERS[f->state](f, ev, now_ms);
```

| Рядок | Пояснення |
|---|---|
| 114–119 | **масив вказівників на функції**, індексований станом. `[MS_IDLE] = ...` — ініціалізація за індексом (C99) |
| 138 | захист від некоректного стану |
| 139 | **виклик обробника поточного стану.** Весь автомат — один рядок замість великого `switch` |

**Патерн State:** додати новий стан = один рядок у enum, одна функція, один рядок у таблиці.

```c
132:     f->t_stopped   = now_ms - cfg->deadtime_ms;   /* перший старт — без паузи */
```

Трюк: вдаємо, що мотор зупинили 150 мс тому, щоб перше клацання після старту одразу вмикало мотор.

**Чому модуль «чистий»:** час передається параметром `now`, а мотор приходить через інтерфейс. Немає ні HAL, ні FreeRTOS, тому 100 000 випадкових подій можна прогнати на ПК за секунду.

---

## 13. app_motor_ctrl: задача мотора

### 13.1 Статична пам'ять

```c
17: #define CTRL_STACK_WORDS  256u
18: #define CTRL_QUEUE_LEN    16u
20: static StaticTask_t  tcb;
21: static StackType_t   stack[CTRL_STACK_WORDS];
22: static StaticQueue_t qcb;
23: static uint8_t       qbuf[CTRL_QUEUE_LEN * sizeof(MotorEvent_t)];
24: static QueueHandle_t queue;
```

| Рядок | Пояснення |
|---|---|
| 20 | **TCB (Task Control Block)**: службова структура задачі (стан, пріоритет, вказівник стеку) |
| 21 | **стек задачі**: 256 слів × 4 = 1 КБ. Тут живуть локальні змінні і адреси повернення |
| 22 | службова структура черги |
| 23 | буфер черги: 16 подій |
| 24 | handle — «номерок», через який звертаємося до черги |

**Для лекції:** уся пам'ять виділяється **компілятором**. У Build Analyzer видно точний розмір. Купа FreeRTOS не використовується, фрагментація неможлива.

```c
26: static MotorFsm_t         fsm;
27: static volatile MotorState_t published_state = MS_IDLE;
28: static volatile uint32_t  dropped;
```

`volatile` каже компілятору: «змінну можуть читати або змінювати поза цим кодом (інша задача, дебагер), тож не кешуй її в регістрі».

### 13.2 Підписки на шину

```c
36: static void on_encoder(void *ctx, BusTopic_t t, int32_t steps)
38:     (void)ctx; (void)t;
39:     MotorEvent_t ev = (steps > 0) ? MEV_STEP_CW : MEV_STEP_CCW;
40:     for (int32_t n = (steps > 0) ? steps : -steps; n > 0; n--) {
41:         MotorCtrl_Post(ev);
42:     }
```

| Рядок | Пояснення |
|---|---|
| 38 | `(void)x;` — «параметр не використовується, це навмисно» (прибирає попередження компілятора) |
| 39 | знак кількості клацань визначає напрямок |
| 40–41 | одна подія в чергу на кожне клацання |

**Важливо:** цей колбек виконується **в задачі input**, а не motor. Тому він нічого не робить з мотором, а лише кладе подію в чергу.

### 13.3 Сама задача

```c
59: static void MotorCtrlTask(void *arg)
62:     MotorEvent_t ev;
63:     for (;;) {
64:         if (xQueueReceive(queue, &ev, pdMS_TO_TICKS(CFG_CTRL_TICK_MS)) == pdTRUE) {
65:             MotorFsm_Dispatch(&fsm, ev, now_ms());
66:         }
67:         MotorFsm_Dispatch(&fsm, MEV_TICK, now_ms());
68:     }
```

| Рядок | Пояснення |
|---|---|
| 63 | задача FreeRTOS — **нескінченний цикл**, вона ніколи не повертається |
| 64 | **чекати подію не довше 10 мс.** Поки чекає, задача **заблокована** і не витрачає процесор. `pdMS_TO_TICKS` переводить мс у тіки ОС |
| 65 | подія прийшла, передаємо автомату |
| 67 | такт завжди, щоб таймери паузи й тайм-ауту йшли |

**Патерн Active Object:** власна задача + власна черга + приватний стан. Реле керує **лише** ця задача, тому м'ютекси не потрібні.

### 13.4 Ініціалізація і відправка

```c
79:     queue = xQueueCreateStatic(CTRL_QUEUE_LEN, sizeof(MotorEvent_t), qbuf, &qcb);
80:     bool ok = EventBus_Subscribe(TOPIC_ENCODER_STEP, on_encoder, NULL) &&
81:               EventBus_Subscribe(TOPIC_BUTTON,       on_button,  NULL);
82:     TaskHandle_t h = xTaskCreateStatic(MotorCtrlTask, "motor", CTRL_STACK_WORDS, NULL,
83:                                        CFG_PRIO_MOTOR_CTRL, stack, &tcb);
84:     configASSERT(queue != NULL && ok && h != NULL);
88: bool MotorCtrl_Post(MotorEvent_t ev)
91:     if (xQueueSend(queue, &ev, 0) == pdTRUE) return true;
92:     dropped++;
```

| Рядок | Пояснення |
|---|---|
| 79 | створити чергу в **нашій** пам'яті (static) |
| 82–83 | створити задачу: функція, ім'я, розмір стеку, параметр, **пріоритет 3**, наш стек, наш TCB |
| 84 | `configASSERT`: якщо щось не створилося, зупинитися тут (видно в дебагері) |
| 91 | **тайм-аут 0**: якщо черга повна, не чекати. Задача вводу ніколи не блокується |
| 92 | рахуємо загублені події для діагностики |

---

## 14. app_regulator: задача PID

### 14.1 Дані

```c
21: typedef enum { RCMD_SP_DELTA, RCMD_SP_SET } RegCmdType_t;
22: typedef struct { RegCmdType_t type; int32_t value_mv; } RegCmd_t;
36: volatile RegStatus_t g_reg;
37: volatile PidTuning_t g_pid_tuning = { CFG_PID_KP, CFG_PID_KI, CFG_PID_KD };
```

| Рядок | Пояснення |
|---|---|
| 21–22 | команда в черзі: «змінити уставку на Δ» (енкодер) або «встановити значення» (кнопка → 0) |
| 36 | **телеметрія**: глобальна структура, яку видно в дебагері (Live Expressions) |
| 37 | коефіцієнти, які можна змінити **в дебагері** на льоту |

### 14.2 Підписки

```c
41: static void on_encoder(void *ctx, BusTopic_t t, int32_t steps)
44:     RegCmd_t c = { RCMD_SP_DELTA, steps * CFG_SP_STEP_MV };
45:     xQueueSend(queue, &c, 0);
```

Одне клацання = +100 мВ. Команда йде в **чергу регулятора**, а не міняє уставку напряму, бо уставкою володіє лише задача pid.

### 14.3 Лічильник тактів процесора (DWT)

```c
63: static void dwt_init(void)
65:     CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
66:     DWT->CYCCNT = 0;
67:     DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
```

| Рядок | Пояснення |
|---|---|
| 65 | увімкнути блок трасування ядра Cortex-M4 |
| 66 | обнулити лічильник тактів |
| 67 | запустити: `CYCCNT` +1 на кожен такт процесора (84 млн разів за секунду) |

Потрібно, щоб точно виміряти, скільки мікросекунд займає один такт PID.

### 14.4 Задача: підготовка

```c
71: static void RegulatorTask(void *arg)
74:     PID_Controller_t pid;
75:     PidTuning_t applied = { CFG_PID_KP, CFG_PID_KI, CFG_PID_KD };
76:     PID_Init(&pid, applied.kp, applied.ki, applied.kd,
77:              CFG_REG_PERIOD_MS / 1000.0f, CFG_PID_DFILT_TAU_S, 0.0f, 1.0f);
79:     int32_t    sp_mv    = CFG_SP_INITIAL_MV;
84:     const uint32_t   cycles_per_us = HAL_RCC_GetHCLKFreq() / 1000000u;
85:     const TickType_t period = pdMS_TO_TICKS(CFG_REG_PERIOD_MS);
86:     TickType_t last = xTaskGetTickCount();
```

| Рядок | Пояснення |
|---|---|
| 74 | об'єкт PID живе **на стеку задачі**: ніхто інший до нього не має доступу |
| 76–77 | Ts = 10 / 1000 = 0,01 с; вихід 0…1 |
| 79 | уставка після старту — 1500 мВ |
| 84 | 84 000 000 / 1 000 000 = 84 такти на мікросекунду |
| 86 | «точка відліку» для рівного періоду |

### 14.5 Задача: цикл (серце проєкту)

```c
90:  if ((TickType_t)(xTaskGetTickCount() - last) >= 2u * period) g_reg.overruns++;
91:  vTaskDelayUntil(&last, period);
92:  uint32_t c0 = DWT->CYCCNT;
```

| Рядок | Пояснення |
|---|---|
| 90 | якщо з останнього пробудження минуло ≥ 2 періоди, ми не встигли. Рахуємо **overrun** |
| 91 | **`vTaskDelayUntil`** спить **до** моменту `last + 10 мс`, а не 10 мс від «зараз». Період рівний, без накопичення похибки |
| 92 | засікаємо такт процесора на початку роботи |

**Для лекції: `vTaskDelay(10)` проти `vTaskDelayUntil(&last, 10)`.** Перший чекає 10 мс **після** закінчення роботи, тож реальний період = 10 мс + час роботи і «пливе». Другий будить рівно кожні 10 мс. PID розрахований на фіксований Ts, тому потрібен другий.

```c
95:  RegCmd_t cmd;
96:  while (xQueueReceive(queue, &cmd, 0) == pdTRUE) {
97:      sp_mv = clamp_sp(cmd.type == RCMD_SP_DELTA ? sp_mv + cmd.value_mv : cmd.value_mv);
98:  }
```

| Рядок | Пояснення |
|---|---|
| 96 | забрати **всі** команди, що накопичилися. Тайм-аут **0**: задача PID ніколи ні на що не чекає, крім свого таймера |
| 97 | змінити уставку та обмежити 0…3000 мВ |

```c
101: PidTuning_t want = { g_pid_tuning.kp, g_pid_tuning.ki, g_pid_tuning.kd };
102: if (want.kp != applied.kp || want.ki != applied.ki || want.kd != applied.kd) {
103:     applied = want;
104:     PID_SetGains(&pid, applied.kp, applied.ki, applied.kd);
105: }
```

Якщо в дебагері змінили коефіцієнти, застосувати їх. Так налаштовують PID без перепрошивки.

```c
108: int32_t y_mv = (int32_t)AdcIn_ReadMv(adc);
109: float   u    = PID_Update(&pid, sp_mv / 1000.0f, y_mv / 1000.0f);
110: Pwm_SetDuty(pwm, u);
```

**Три рядки, в яких увесь замкнутий контур:**

| Рядок | Пояснення |
|---|---|
| 108 | **виміряти** (очі регулятора) |
| 109 | **порахувати** (мозок). Мілівольти → вольти: `/ 1000.0f` (`f` — щоб ділення було у `float`) |
| 110 | **вплинути** (руки) |

```c
113: int32_t err = sp_mv - y_mv;
114: if (abs(err) <= CFG_SETTLED_BAND_MV) {
115:     if (!in_band) { in_band = true; in_band_since = xTaskGetTickCount(); }
116: } else {
117:     in_band = false;
118: }
119: bool now_settled = in_band &&
120:     (TickType_t)(xTaskGetTickCount() - in_band_since) >= pdMS_TO_TICKS(CFG_SETTLED_TIME_MS);
121: if (now_settled != settled) {
122:     settled = now_settled;
123:     EventBus_Publish(TOPIC_REG_SETTLED, settled ? 1 : 0);
124: }
```

| Рядок | Пояснення |
|---|---|
| 114–115 | похибка ≤ 30 мВ: запам'ятати, **коли** увійшли в смугу |
| 117 | вийшли зі смуги, скидаємо |
| 119–120 | «вийшли на уставку» = у смузі **300 мс поспіль** (а не випадково на мить) |
| 121–123 | публікуємо **лише зміну** стану, а не щотакту. Індикатор змінить частоту блимання |

```c
127: g_reg.setpoint_mv = sp_mv;
...
134: uint32_t us = (DWT->CYCCNT - c0) / cycles_per_us;
135: g_reg.exec_us = us;
136: if (us > g_reg.exec_us_max) g_reg.exec_us_max = us;
```

| Рядок | Пояснення |
|---|---|
| 127–132 | телеметрія для дебагера |
| 134 | скільки тактів минуло → мікросекунди |
| 136 | найгірший час виконання. Має бути набагато менше 10 000 мкс |

### 14.6 Ініціалізація

```c
150: TaskHandle_t h = xTaskCreateStatic(RegulatorTask, "pid", REG_STACK_WORDS, NULL,
151:                                    CFG_PRIO_REGULATOR, stack, &tcb);
```

**Пріоритет 4, найвищий.** Правило **Rate Monotonic**: найвищий пріоритет у задачі з найжорсткішими вимогами до часу. Такт PID ніколи не зсунеться через енкодер чи мотор.

---

## 15. app_input: задача вводу

```c
18: #define INPUT_STACK_WORDS      192u
19: #define MAX_STEPS_PER_PERIOD   4
27: static void InputTask(void *arg)
30:     TickType_t last = xTaskGetTickCount();
32:     for (;;) {
33:         vTaskDelayUntil(&last, pdMS_TO_TICKS(CFG_INPUT_PERIOD_MS));
36:         if (Button_PollPressed(btn)) {
37:             EventBus_Publish(TOPIC_BUTTON, 1);
38:         }
41:         int32_t det = Encoder_ReadDetents(enc);
42:         if (det >  MAX_STEPS_PER_PERIOD) det =  MAX_STEPS_PER_PERIOD;
43:         if (det < -MAX_STEPS_PER_PERIOD) det = -MAX_STEPS_PER_PERIOD;
44:         if (det != 0) {
45:             EventBus_Publish(TOPIC_ENCODER_STEP, det);
46:         }
```

| Рядок | Пояснення |
|---|---|
| 18 | стек менший (768 байт): задача проста |
| 33 | опитування рівно кожні 10 мс |
| 36–37 | кнопку натиснули, публікуємо подію |
| 41 | скільки клацань за 10 мс (зазвичай 0 або ±1) |
| 42–43 | обмеження: якщо крутнули дуже швидко, не засипаємо черги десятками подій |
| 44–45 | публікуємо, **лише якщо щось змінилось** |

**Для лекції:** задача вводу — **продюсер**. Вона не знає, хто отримає події. Пріоритет **найнижчий (2)**: навіть якщо вона запізниться, апаратний таймер не загубить жодного імпульсу енкодера.

---

## 16. app_indicator: світлодіоди і програмний таймер

```c
17: #define BLINK_FAST_MS  100u
21: static StaticTimer_t heart_tcb;
22: static TimerHandle_t heart_timer;
24: static void on_motor_state(void *ctx, BusTopic_t topic, int32_t value)
27:     MotorState_t s = (MotorState_t)value;
28:     Led_Set(led_red,  s == MS_RUN_CW);
29:     Led_Set(led_blue, s == MS_RUN_CCW);
```

| Рядок | Пояснення |
|---|---|
| 28 | червоний світиться **тоді й лише тоді**, коли стан RUN_CW. Вираз `s == MS_RUN_CW` уже є `bool` |
| 29 | синій — для RUN_CCW. В інших станах обидва гаснуть |

```c
32: static void on_reg_settled(void *ctx, BusTopic_t topic, int32_t settled)
35:     xTimerChangePeriod(heart_timer,
36:                        pdMS_TO_TICKS(settled ? CFG_HEARTBEAT_MS : BLINK_FAST_MS), 0);
39: static void on_heartbeat(TimerHandle_t t)
42:     Led_Toggle(led_heart);
```

| Рядок | Пояснення |
|---|---|
| 35–36 | PID вийшов на уставку: блимати повільно (500 мс); інакше швидко (100 мс) |
| 39–42 | функцію викликає FreeRTOS щоразу, коли спрацьовує таймер |

```c
54:     heart_timer = xTimerCreateStatic("heart", pdMS_TO_TICKS(BLINK_FAST_MS), pdTRUE,
55:                                      NULL, on_heartbeat, &heart_tcb);
61:     xTimerStart(heart_timer, 0);
```

| Параметр | Пояснення |
|---|---|
| `"heart"` | ім'я для дебагера |
| `pdMS_TO_TICKS(100)` | період |
| `pdTRUE` | **автоперезапуск**: таймер періодичний, а не одноразовий |
| `on_heartbeat` | функція, яку викликати |
| `&heart_tcb` | наша статична пам'ять |

**Для лекції: програмний таймер замість задачі.** Для «кожні N мс зробити щось коротке» окрема задача з власним стеком (~0,5 КБ) — марнотратство. Програмні таймери обслуговує одна службова задача FreeRTOS (Tmr Svc).

---

## 17. app.c: збирання системи

```c
42: extern TIM_HandleTypeDef htim4;
43: extern TIM_HandleTypeDef htim3;
44: extern ADC_HandleTypeDef hadc1;
```

`extern` означає, що змінна оголошена в іншому файлі (`main.c`), і тут ми лише кажемо, що вона існує.

```c
47: static Relay_t      relay_cw, relay_ccw;
48: static MotorRelay_t motor_relay;
49: static Led_t        led_red, led_blue, led_heart;
50: static Encoder_t    encoder;
51: static Button_t     button;
52: static Pwm_t        pwm;
53: static AdcIn_t      adc;
```

**Усі об'єкти системи в одному місці**, статично. Це «інвентар» проєкту.

```c
58:     const bool active_low = (CFG_RELAY_ACTIVE_LOW != 0);
59:     Relay_Init(&relay_cw,  (DigitalOut_t){ CFG_RELAY_CW_PORT,  CFG_RELAY_CW_PIN,
60:                                            active_low, active_low });
63:     MotorRelay_Init(&motor_relay, &relay_cw, &relay_ccw);
```

| Рядок | Пояснення |
|---|---|
| 58–60 | реле створюємо **першими**: якомога раніше гарантуємо, що мотор вимкнений. Для Low-trigger і полярність інверсна, і вихід open-drain |
| 63 | «мотор» складається з двох реле |

```c
66:     Led_Init(&led_red,   (DigitalOut_t){ ..., false, false });
68:     Led_Init(&led_heart, (DigitalOut_t){ ..., true,  false });
```

Вбудований LED на PC13 світиться від нуля, тому `active_low = true`. Решта коду про це не знає.

```c
71:     Encoder_Init(&encoder, &htim4, CFG_ENC_COUNTS_PER_DETENT, CFG_ENC_INVERT != 0);
72:     Button_Init(&button, (DigitalIn_t){ CFG_BUTTON_PORT, CFG_BUTTON_PIN, true, GPIO_PULLUP }, ...);
76:     Pwm_Init(&pwm, &htim3, TIM_CHANNEL_1);
77:     AdcIn_Init(&adc, &hadc1, CFG_REG_ADC_OVERSAMPLE, CFG_REG_VREF_MV);
```

Драйвери отримують **handle** периферії ззовні. Драйвер енкодера не знає, що це саме TIM4.

```c
80:     Indicator_Init(&led_red, &led_blue, &led_heart);
81:     MotorCtrl_Init(MotorRelay_AsMotor(&motor_relay));
82:     Regulator_Init(&pwm, &adc);
83:     Input_Init(&encoder, &button);
```

| Рядок | Пояснення |
|---|---|
| 80 | індикатор підписується на стан мотора і PID |
| 81 | контролер мотора отримує **абстрактний** мотор. Тут і відбувається вибір реалізації |
| 82 | регулятор отримує свій вихід і вхід |
| 83 | ввід створюється **останнім**: до того часу всі підписники вже зареєстровані |

**Патерни:** **Composition Root** (усе «склеювання» в одному місці) і **Dependency Injection** (залежності передаються параметрами, модулі їх не створюють самі).

**Для лекції:** щоб перейти з реле на драйвер MX1616, міняється **лише рядок 81** і рядки створення драйвера. Решта коду не змінюється.

---

## 18. Тести на ПК

### test_pid.c: модель RC-кола

```c
static void plant_step(Plant_t *p, float duty, float dt_total)
{
    const float R1 = 10e3f, R2 = 100e3f, C2 = 1e-6f, VCC = 3.3f, h = 1e-5f;
    for (int i = 0; i < (int)(dt_total / h + 0.5f); i++) {
        float i1 = (duty * VCC - p->v1) / R1;
        float i2 = (p->v1 - p->v2) / R2;
        float il = (p->r_load > 0) ? p->v2 / p->r_load : 0.0f;
        p->v1 += h * (i1 - i2) / p->c1;
        p->v2 += h * (i2 - il) / C2;
    }
}
```

| Рядок | Пояснення |
|---|---|
| `h = 1e-5` | крок моделювання 10 мкс, у 1000 разів дрібніший за такт PID |
| `i1` | закон Ома: струм через R1 = різниця напруг / опір |
| `i2` | струм через R2 (другий ступінь **навантажує** перший) |
| `il` | струм навантаження (збурення) |
| `v1 += ...` | конденсатор: dV/dt = I / C. Метод Ейлера: нове = старе + крок × похідна |

**Для лекції:** це чисельне розв'язання системи двох диференціальних рівнянь, тобто цифровий «двійник» макета.

### Що перевіряють тести

| Тест | Що доводить |
|---|---|
| крок 0 → 1,5 В | перерегулювання < 5 %, вихід на уставку < 1,5 с, статична похибка < 10 мВ |
| кроки енкодера | уставка 1,5 → 1,6 → 1,7 → 1,6 відпрацьовується |
| відсутність «удару» D | стрибок ШІМ лише від P |
| anti-windup | після насичення немає великого перелітання |
| збурення | I-складова повертає на уставку |
| розкид ємності | ті самі коефіцієнти працюють для 41, 61 і 10 мкФ |
| лише P | залишається статична похибка (1,30 В замість 1,50) |

**Для лекції: як перевірити, що тест чогось вартий.** Навмисно зламати код (прибрати anti-windup) і переконатися, що тест **провалюється**. Для нашого проєкту це перевірено: без anti-windup — 7 провалених перевірок.

---

## 19. Контрольні питання для студентів

**Периферія**
1. Як отримати 84 МГц із кварцу 25 МГц? Які значення M, N, P?
2. Чому таймери на APB1 тактуються 84 МГц, хоча шина APB1 — 42 МГц?
3. Як розрахувати частоту ШІМ через Prescaler і Period? Яке значення CCR дає заповнення 30 %?
4. Навіщо АЦП 84 такти вибірки, а не 3?
5. Чому в `MX_GPIO_Init` спершу `WritePin`, а потім `GPIO_Init`?

**Драйвери та архітектура**
6. Як рядок `(int16_t)(now - last)` правильно рахує зміну енкодера через переповнення лічильника?
7. Що таке `void *self` і таблиця функцій у `drv_motor.h`? Як це пов'язано з віртуальними методами C++?
8. Що потрібно змінити в коді, щоб замінити реле на драйвер MX1616?
9. Як працює антибрязкіт у `drv_button.c`? Що буде з послідовністю `1 0 1 1 1`?

**PID**
10. Що робить кожна зі складових P, I, D? Що буде без I?
11. Чому D рахується від вимірювання, а не від похибки?
12. Що таке windup і як його запобігає код у рядках 67–72?
13. Навіщо фільтр на D-складовій?
14. Чому `integ` зберігає вже помножене на Ki значення?
15. Порахуйте вручну вихід PID для заданих значень (приклад з розділу 10.6).

**FreeRTOS**
16. Чим `vTaskDelayUntil` відрізняється від `vTaskDelay`? Чому для PID потрібен саме перший?
17. Чому в проєкті немає жодного м'ютекса, і як це запобігає deadlock?
18. Чому задача PID має найвищий пріоритет, а задача вводу — найнижчий?
19. Чому `xQueueSend` викликається з тайм-аутом 0?
20. Переваги статичного створення задач (`xTaskCreateStatic`) перед динамічним?
21. Навіщо програмний таймер замість окремої задачі для блимання LED?
22. У контексті якої задачі виконується колбек шини подій? Чому він має бути коротким?

**Тестування**
23. Чому модулі `alg_pid` і `app_motor_fsm` можна тестувати на ПК, а `drv_adc` — ні?
24. Як перевірити, що тест справді ловить помилки?
