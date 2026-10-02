/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* ============================================================================
 * ЛЕКЦІЙНА ВЕРСІЯ main.c (main_lecture.c)
 * ============================================================================
 * Це main.c, згенерований STM32CubeMX для проєкту PID_Encoder_Motor, з
 * УСІМА службовими коментарями CubeMX + лекційними поясненнями до кожного
 * рядка. Відповідає .ioc ПІСЛЯ виправлень:
 *   - задачі defaultTask / pid / motor / input у CubeMX видалені
 *     (наші задачі створює App_Init());
 *   - ADC Sampling Time = 84 Cycles;
 *   - PB14/PB15 (реле) — Output Open Drain.
 *
 * НАВІЩО ЦЕЙ ФАЙЛ:
 *   main.c — точка входу програми. Після ввімкнення мікроконтролер починає
 *   виконання саме з main(). Файл робить три речі:
 *     1) налаштовує залізо: тактову частоту, піни, таймери, АЦП;
 *     2) запускає наш код (App_Init());
 *     3) запускає планувальник FreeRTOS — після цього main() фактично
 *        закінчується, і далі працюють лише задачі.
 *
 * ПРАВИЛО CubeMX (пояснити студентам першим):
 *   свій код пишемо ТІЛЬКИ між  USER CODE BEGIN ...  і  USER CODE END ... .
 *   Усе поза цими маркерами CubeMX перезапише при натисканні Generate Code.
 *   Коментарі-роздільники "Private includes ---", "Private variables ---"
 *   тощо — теж службові коментарі CubeMX: вони показують, куди що писати.
 * ========================================================================== */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"          // назви пінів (LED_RED_Pin …), прототипи HAL, Error_Handler
#include "cmsis_os.h"      // обгортка CMSIS-RTOS v2 над FreeRTOS (osKernelInitialize/Start)

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app.h"           // НАШ модуль: оголошення функції App_Init()
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* (тут можна оголошувати власні типи — нам не потрібно) */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* (тут — власні #define; наші налаштування лежать в app_config.h) */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* "Паспорти" (handle) периферії. Handle — структура, що описує один
   периферійний блок: адреса регістрів + налаштування + поточний стан.
   Кожна функція HAL отримує вказівник на неї: HAL_ADC_Start(&hadc1).
   Наш код знаходить ці змінні через extern у app.c.                       */
ADC_HandleTypeDef hadc1;   // АЦП №1 — вимірювання напруги RC-кола (PA1)

TIM_HandleTypeDef htim3;   // таймер 3 — ШІМ (PA6)
TIM_HandleTypeDef htim4;   // таймер 4 — енкодер (PB6/PB7)

/* (Після видалення задач у CubeMX тут БІЛЬШЕ НЕМАЄ блоків
   "Definitions for defaultTask / pid / motor / input" — задачі створює
   наш код статично, у модулях app_regulator.c, app_motor_ctrl.c, app_input.c) */

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);      // прототипи функцій, описаних нижче в цьому файлі
static void MX_GPIO_Init(void);     // static — функція видна лише в main.c
static void MX_TIM4_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();              // скинути всю периферію, налаштувати пріоритети переривань,
                           // запустити 1-мс тік HAL (у нас від TIM11)

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();    // перейти з 16 МГц (після скидання) на 84 МГц

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();          // піни: світлодіоди, реле, кнопка
  MX_TIM4_Init();          // таймер 4 у режимі енкодера
  MX_ADC1_Init();          // АЦП для вимірювання напруги RC-кола
  MX_TIM3_Init();          // таймер 3 у режимі ШІМ
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();    // підготувати внутрішні структури FreeRTOS (задачі ще НЕ працюють)

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* (м'ютексів у проєкті немає: кожен ресурс має одного власника) */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* (програмний таймер блимання створює app_indicator.c) */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* (черги створюють app_motor_ctrl.c і app_regulator.c) */
  /* USER CODE END RTOS_QUEUES */

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  App_Init();              // НАШ КОД: створює драйвери, черги, задачі pid / motor / input
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();         // запустити планувальник. З цього моменту процесор виконує
                           // лише задачі FreeRTOS і сюди більше НЕ повертається

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)                // сюди програма ніколи не доходить (страховка)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
/* ЛЕКЦІЯ: після ввімкнення МК працює від внутрішнього генератора 16 МГц.
   Функція вмикає зовнішній кварц і множить частоту до 84 МГц (максимум F401).

     Кварц 25 МГц → ÷25 → 1 МГц → ×336 → 336 МГц → ÷4 → 84 МГц (ядро)
                                                    ├─ APB1 ÷2 = 42 МГц → таймери ×2 = 84 МГц
                                                    └─ APB2 ÷1 = 84 МГц → АЦП, TIM11          */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};   // налаштування генераторів; {0} — обнулити
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};   // налаштування шин

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();                                    // увімкнути блок живлення (PWR)
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2); // режим регулятора напруги ядра,
                                                                 // достатній для 84 МГц

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;     // джерело — зовнішній кварц (HSE)
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;                       // увімкнути кварц 25 МГц на платі
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;                   // увімкнути PLL — помножувач частоти
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;           // на вхід PLL подати кварц
  RCC_OscInitStruct.PLL.PLLM = 25;             // 25 МГц ÷ 25 = 1 МГц   (вхід PLL має бути 1–2 МГц)
  RCC_OscInitStruct.PLL.PLLN = 336;            // 1 МГц × 336 = 336 МГц (внутрішня частота PLL)
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;  // 336 ÷ 4 = 84 МГц      ← частота ядра
  RCC_OscInitStruct.PLL.PLLQ = 4;              // 336 ÷ 4 = 84 МГц для USB/SDIO (не використовується)
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)   // застосувати; якщо кварц не запустився —
  {
    Error_Handler();                                     // зупинитися тут
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;  // які частоти налаштовуємо
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;  // ядро тактувати від PLL (84 МГц)
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;         // шина AHB (ядро, пам'ять): 84 МГц
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;          // шина APB1: 84÷2 = 42 МГц (її максимум).
                                                             // Таймери на APB1 (TIM3, TIM4) при цьому
                                                             // автоматично отримують ×2 = 84 МГц
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;          // шина APB2 (АЦП, TIM11): 84 МГц

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {                                    // FLASH_LATENCY_2: флеш повільніша за ядро,
    Error_Handler();                   // тому процесор чекає 2 такти при кожному читанні
  }

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();                 // Clock Security System: якщо кварц відмовить,
}                                      // МК сам перейде на внутрішній HSI і не зависне

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
/* ЛЕКЦІЯ: АЦП — "ОЧІ" РЕГУЛЯТОРА (пін PA1). Перетворює напругу на виході
   RC-кола (0…3,3 В) у число 0…4095, яке PID порівнює з уставкою. За командою
   програми внутрішній конденсатор АЦП заряджається від піна (вибірка),
   потім АЦП за 12 кроків визначає число.                                   */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};                   // структура налаштувань каналу

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;                                  // handle описує АЦП №1
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;   // 84 МГц ÷ 4 = 21 МГц (максимум — 36 МГц)
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;             // 12 біт: 4096 рівнів, крок ≈ 0,8 мВ
  hadc1.Init.ScanConvMode = DISABLE;                      // один канал, а не список каналів
  hadc1.Init.ContinuousConvMode = DISABLE;                // одне вимірювання на одну команду
  hadc1.Init.DiscontinuousConvMode = DISABLE;             // (стосується сканування — вимкнено)
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE; // не від зовнішнього сигналу
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;       // запуск — програмно, HAL_ADC_Start()
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;             // результат у молодших бітах: 0…4095
  hadc1.Init.NbrOfConversion = 1;                         // за один запуск — одне перетворення
  hadc1.Init.DMAContinuousRequests = DISABLE;             // DMA не використовуємо
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;          // прапорець "готово" після кожного перетворення
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;                 // канал 1 = пін PA1 (PA0 зайнятий кнопкою KEY)
  sConfig.Rank = 1;                                // перший (і єдиний) у черзі вимірювань
  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;  // час вибірки 84 такти ≈ 4 мкс. Сигнал іде через
                                                   // 100 кОм — внутрішньому конденсатору АЦП потрібен
                                                   // час зарядитися. (3 такти — замало!)
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
/* ЛЕКЦІЯ: ШІМ — "РУКИ" РЕГУЛЯТОРА (таймер 3, канал 1, пін PA6).
   Лічильник рахує 0,1,2 … 4199 і скидається в 0. Поки лічильник МЕНШИЙ за
   регістр порівняння CCR — на піні 3,3 В, інакше 0 В.

     лічильник 4199 ┤   /|   /|   /|
         CCR=1260 ──┼──/─┼──/─┼──/─┼──   ← 1260/4200 = 30 %
                  0 ┤ /  | /  | /  |
     PA6:  3,3 В    ▇▁▁▁▁▇▁▁▁▁▇▁▁▁▁
                    ←50 мкс→                                                 */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};  // звідки таймер бере такти
  TIM_MasterConfigTypeDef sMasterConfig = {0};      // зв'язок з іншими таймерами
  TIM_OC_InitTypeDef sConfigOC = {0};               // налаштування каналу виходу

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;                            // цей handle описує саме таймер 3
  htim3.Init.Prescaler = 0;                         // дільник (0+1)=1 → рахує на 84 МГц
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;      // рахувати вгору: 0,1,2…
  htim3.Init.Period = 4199;                         // …до 4199, потім знову 0.
                                                    // Період 4200 тактів → 84 МГц / 4200 = 20 кГц
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;// для ШІМ не впливає (стосується фільтрів входів)
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE; // нові значення — з НАСТУПНОГО
                                                    // періоду, імпульс не "рветься"
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)          // застосувати базові налаштування лічильника
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;   // такти — від внутрішньої шини
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)           // перевести таймер у режим ШІМ
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;          // не видавати сигналів іншим таймерам
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE; // працює самостійно
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;           // режим ШІМ 1: вихід = 1, поки лічильник < CCR
  sConfigOC.Pulse = 0;                          // CCR = 0 → 0 %: на старті нічого не подаємо
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;   // "активний" рівень — високий (3,3 В)
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;    // прискорений режим не потрібен
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)  // канал 1
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);                  // під'єднати фізичний пін PA6 до каналу 1
                                                // (альтернативна функція AF2)
}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
/* ЛЕКЦІЯ: ЕНКОДЕР (таймер 4, піни PB6 = A, PB7 = B). Таймер сам рахує
   імпульси і сам визначає напрямок: сигнали A і B зсунуті на чверть періоду.
   Якщо A змінюється раніше за B — лічильник +1, інакше −1. Процесор нічого
   не робить, лише іноді читає лічильник.                                   */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};        // налаштування режиму енкодера
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;                        // handle описує таймер 4
  htim4.Init.Prescaler = 0;                     // рахувати кожен імпульс
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;  // (в режимі енкодера напрям визначає сам таймер)
  htim4.Init.Period = 65535;                    // 16-бітний лічильник: 0…65535, далі "перекидання"
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;   // базова частота цифрових фільтрів
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;   // рахувати фронти ОБОХ каналів (A і B):
                                                // 4 зміни лічильника на одне клацання KY-040
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;  // канал A (PB6): без інверсії
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI; // канал 1 бере сигнал зі свого піна
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;        // рахувати кожен фронт
  sConfig.IC1Filter = 10;                       // апаратний фільтр: імпульс має бути стабільним
                                                // кілька тактів — відсікає брязкіт контактів
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;  // канал B (PB7): те саме
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)   // застосувати режим енкодера
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
/* ЛЕКЦІЯ: ЦИФРОВІ ПІНИ. ВАЖЛИВО: спершу WritePin (рівень), ПОТІМ GPIO_Init
   (режим виходу) — пін стає виходом уже в безпечному стані, і реле не
   клацне при ввімкненні плати.                                             */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};       // структура налаштувань піна
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();    // увімкнути тактування порту C (без цього порт "мертвий")
  __HAL_RCC_GPIOH_CLK_ENABLE();    // порт H (піни кварцу PH0/PH1)
  __HAL_RCC_GPIOA_CLK_ENABLE();    // порт A (PA1 — АЦП, PA6 — ШІМ)
  __HAL_RCC_GPIOB_CLK_ENABLE();    // порт B (енкодер, LED, реле, кнопка)

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_HEART_GPIO_Port, LED_HEART_Pin, GPIO_PIN_SET);
                                   // PC13: вбудований LED світиться від 0 → SET = вимкнений

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_RED_Pin|LED_BLUE_Pin, GPIO_PIN_RESET);
                                   // PB12, PB13: зовнішні LED вимкнені

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RELAY_CW_Pin|RELAY_CCW_Pin, GPIO_PIN_SET);
                                   // PB14, PB15: реле Low-trigger → 1 = вимкнене

  /*Configure GPIO pin : LED_HEART_Pin */
  GPIO_InitStruct.Pin = LED_HEART_Pin;           // PC13
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;    // push-pull: активно тягне і до 0, і до 3,3 В
  GPIO_InitStruct.Pull = GPIO_NOPULL;            // без внутрішніх підтяжок
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;   // повільні фронти — менше перешкод
  HAL_GPIO_Init(LED_HEART_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_RED_Pin LED_BLUE_Pin */
  GPIO_InitStruct.Pin = LED_RED_Pin|LED_BLUE_Pin;   // PB12, PB13 — світлодіоди
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : RELAY_CW_Pin RELAY_CCW_Pin */
  GPIO_InitStruct.Pin = RELAY_CW_Pin|RELAY_CCW_Pin; // PB14, PB15 — реле
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;       // open-drain: вміє лише "притягнути до 0"
                                                    // або "відпустити"; "1" дає підтяжка модуля реле
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : ENC_SW_Pin */
  GPIO_InitStruct.Pin = ENC_SW_Pin;                 // PB8 — кнопка енкодера
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;           // вхід
  GPIO_InitStruct.Pull = GPIO_PULLUP;               // резистор до 3,3 В: не натиснута = 1,
  HAL_GPIO_Init(ENC_SW_GPIO_Port, &GPIO_InitStruct);// натиснута замикає на GND = 0

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM11 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
/* ЛЕКЦІЯ: TIM11 кожну 1 мс збільшує лічильник часу HAL. Окремий таймер
   потрібен, бо SysTick забрала FreeRTOS для перемикання задач (тому в
   CubeMX Timebase Source = TIM11).                                          */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM11)     // це саме TIM11 (часова база HAL)?
  {
    HAL_IncTick();                 // +1 мс до лічильника HAL_GetTick()
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
/* ЛЕКЦІЯ: сюди потрапляємо, якщо ініціалізація периферії не вдалась.
   У дебагері одразу видно, де сталася помилка.                              */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();    // вимкнути всі переривання
  while (1)           // "замерзнути" тут назавжди
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
/* ЛЕКЦІЯ: компілюється лише якщо визначено USE_FULL_ASSERT. Тоді HAL
   перевіряє параметри своїх функцій і при помилці викликає цю функцію з
   ім'ям файлу і номером рядка.                                              */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
