/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "gpio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// Macro to combine all TIM flags
#define TIM_IT_ALL  (TIM_IT_UPDATE | TIM_IT_CC1 | TIM_IT_CC1 | TIM_IT_CC1 | TIM_IT_CC1 | TIM_IT_COM | TIM_IT_TRIGGER | TIM_IT_BREAK)

// Enable to start in even field,odd field otherwise
#define START_IN_EVEN_FIELD

#define _INT(n) ( (int) n )
// System timings
#define MAIN_CLK          (99500UL)                               // Timer clock: 99.5MHz
#define PWM_T(n)          ((((n)*MAIN_CLK)+500000UL)/1000000UL)           // Period value in ns
#define TIM_T(n)          (PWM_T(n)-1)                            // Duty value in ns

//NTSC standard definition
#define FPS               (2997)                                  // 29.97fps
#define FRAME             (480)                                   // 480, 720, 1080... for 525, 765, 1125 lines

// NTSC timings (Computed based on the defined frame/fps and system speed)
#define LINES             (FRAME+45)                              // 525, 765, 1125
#define FIELD             (LINES/2)
#define LINE_T            ((100000000000UL+(FPS*LINES/2))/(FPS*LINES))    // Line: 63.55us = 15734.25Hz
#define HLINE_T           (LINE_T+1/2)
#define TIM_LINE          (TIM_T(LINE_T))
#define TIM_HLINE         (((TIM_LINE+1)/2)-1)                            // Half line: 31.8us
#define HSY               (PWM_T(4700UL))                                 // HSYNC width: 4.7us
#define EQ_LOW            (PWM_T(HLINE_T-4700))                           // EQU low:  HLINE-4.7 = 27.1us    PERIOD = TIM_HLINE
#define PEQ_LOW           (PWM_T(2400UL))                                 // PRE-POST low: 2.4us              PERIOD = TIM_HLINE

#define VSYNC_LINES       (3)                                             // Vsync low pulse width
#define VSYNC_DELAY_LINES (3)                                             // Delay from Pre-Equ start
#define VSYNC_DELAY_HSYNC_ODD (TIM_T(300UL))                              // Delay from hsync fall to vsync toggle odd field (300ns)
#define VSYNC_DELAY_HSYNC_EVEN (TIM_HLINE)                                // Delay from hsync fall to vsync toggle even field (300ns + half line)
/*
#define XSTR(x) STR(x)
#define STR(x) #x
#pragma message "LINES: " XSTR(LINES)
#pragma message "FIELD: " XSTR(FIELD)
#pragma message "LINE_T: " XSTR(LINE_T)
#pragma message "TIM_LINE: " XSTR(TIM_LINE)
#pragma message "TIM_HLINE: " XSTR(TIM_HLINE)
#pragma message "EQ_LOW: " XSTR(EQ_LOW)
#pragma message "PEQ_LOW: " XSTR(PEQ_LOW)
#pragma message "VSYNC_LINES: " XSTR(VSYNC_LINES)
#pragma message "VSYNC_DELAY_LINES: " XSTR(VSYNC_DELAY_LINES)
#pragma message "VSYNC_DELAY_HSYNC: " XSTR(VSYNC_DELAY_HSYNC)
#pragma message "2xHL: " XSTR((TIM_HLINE+1)*2)
*/

/*
   NTSC 525i frame composite sync

   Hfreq      15734.25Hz, T=63.556us

                  Pulses  Width                       Pulse     Start line    Lines   Note
   pre-equ        6       31.78us, 2.5us  low,        0-5       1             3
   equ            6       31.78us, 27.1us low,        6-11      4             3
   post-equ       6       31.78us, 2.5us  low,        12-17     7             3
   10-262         253     63.56us, 4.7us  low         18-270    10            253
   262.5          1       31.78us, 4.7us  low         271       262           0.5     (odd field end, 1/2 line)
   pre-equ        6       31.78us, 2.5us  low,        272-277   262.5         3
   equ            6       31.78us, 27.1us low,        278-283   265.5         3
   post-equ       5       31.78us, 2.5us  low,        284-288   268.5         2.5
   post-equ-end   1       63.56us, 2.5us  low,        289       271           1       (even field start, 1/2 line)
   264-525        253     63.56us, 4.7us  low         290-525   272           253
   end (Roll over)                                              525
*/

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
DMA_HandleTypeDef hdma_tim1_up;
DMA_HandleTypeDef hdma_tim1_ch1_ch2_ch3;
DMA_HandleTypeDef hdma_tim3_ch4_up;
DMA_HandleTypeDef hdma_tim4_up;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

const uint32_t vsync_toggle[] = { VSYNC_Pin, ((uint32_t)VSYNC_Pin<<16) };           // Vsync GPIO BSRR values: low, high.

#ifdef START_IN_EVEN_FIELD

const uint16_t csync[][3] = { // DMA burst write for TIM3

                                                              // EVEN FIELD START
  [0 ... 5]               = { TIM_HLINE,  0,    PEQ_LOW },    // 6x pre-equalizing pulses     T=31.78us   low: 2.44us (3 lines)
  [6 ... 11]              = { TIM_HLINE,  0,    EQ_LOW  },    // 6x equalizing pulses         T=31.78us   low: 27.1us (3 lines)
  [12 ... 16]             = { TIM_HLINE,  0,    PEQ_LOW },    // 5x post-equalizing pulses    T=31.78us   low: 2.44us (2.5 lines)
  [17]                    = { TIM_LINE,   0,    PEQ_LOW },    // 1x post-equ + 0.5 line       T=63.56us   low: 2.44us (0.5 lines)
  [18 ... FIELD+7]        = { TIM_LINE,   0,    HSY     },    // even field                   T=63.56us   low: 4.7us  (253 lines for 525i)

                                                              // ODD FIELD START
  [FIELD+8 ... FIELD+13]  = { TIM_HLINE,  0,    PEQ_LOW },    // 6x pre-equalizing pulses     T=31.78us   low: 2.44us (3 lines)
  [FIELD+14 ... FIELD+19] = { TIM_HLINE,  0,    EQ_LOW  },    // 6x equalizing pulses         T=31.78us   low: 27.1us (3 lines)
  [FIELD+20 ... FIELD+25] = { TIM_HLINE,  0,    PEQ_LOW },    // 6x post-equalizing pulses    T=31.78us   low: 2.44us (3 lines)
  [FIELD+26 ... LINES+16] = { TIM_LINE,   0,    HSY     },    // even field                   T=63.56us   low: 4.7us  (253 lines for 525i)
  [LINES+17]              = { TIM_HLINE,  0,    HSY     },    // 0.5 line                     T=31.78us   low: 4.7us  (0.5 lines)

};
const uint16_t hsync_vsync_div[] = { FIELD-VSYNC_LINES-1, VSYNC_LINES-1, FIELD-VSYNC_LINES, VSYNC_LINES-1 };   // Hsync divider for vsync generation.
const uint16_t hsync_vsync_delay[]   = { VSYNC_DELAY_HSYNC_EVEN, VSYNC_DELAY_HSYNC_ODD, VSYNC_DELAY_HSYNC_ODD, VSYNC_DELAY_HSYNC_EVEN };

#else

const uint16_t csync[][3] = { // DMA burst write for TIM3     // ODD FIELD START
  [0 ... 5]               = { TIM_HLINE,  0,    PEQ_LOW },    // 6x pre-equalizing pulses     T=31.78us   low: 2.44us (3 lines)
  [6 ... 11]              = { TIM_HLINE,  0,    EQ_LOW  },    // 6x equalizing pulses         T=31.78us   low: 27.1us (3 lines)
  [12 ... 17]             = { TIM_HLINE,  0,    PEQ_LOW },    // 6x post-equalizing pulses    T=31.78us   low: 2.44us (3 lines)
  [18 ... FIELD+9]        = { TIM_LINE,   0,    HSY     },    // odd field                    T=63.56us   low: 4.7us  (253 lines for 525i)
  [FIELD+10]              = { TIM_HLINE,  0,    HSY     },    // 0.5 line                     T=31.78us   low: 4.7us  (0.5 lines)
                                                              // EVEN FIELD START
  [FIELD+11 ... FIELD+16] = { TIM_HLINE,  0,    PEQ_LOW },    // 6x pre-equalizing pulses     T=31.78us   low: 2.44us (3 lines)
  [FIELD+17 ... FIELD+22] = { TIM_HLINE,  0,    EQ_LOW  },    // 6x equalizing pulses         T=31.78us   low: 27.1us (3 lines)
  [FIELD+23 ... FIELD+27] = { TIM_HLINE,  0,    PEQ_LOW },    // 5x post-equalizing pulses    T=31.78us   low: 2.44us (2.5 lines)
  [FIELD+28]              = { TIM_LINE,   0,    PEQ_LOW },    // 1x post-equ + 0.5 line       T=63.56us   low: 2.44us (0.5 lines)
  [FIELD+29 ... LINES+17] = { TIM_LINE,   0,    HSY     },    // even field                   T=63.56us   low: 4.7us  (253 lines for 525i)
};
const uint16_t hsync_vsync_div[]   = { FIELD-VSYNC_LINES, VSYNC_LINES-1, FIELD-VSYNC_LINES-1, VSYNC_LINES-1 };   // Hsync divider for vsync generation.
const uint16_t hsync_vsync_delay[]   = { VSYNC_DELAY_HSYNC_ODD, VSYNC_DELAY_HSYNC_EVEN, VSYNC_DELAY_HSYNC_EVEN, VSYNC_DELAY_HSYNC_ODD };
#endif

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t test_mode;
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
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM1_Init();
  MX_TIM5_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

#ifdef DEBUG
  __HAL_DBGMCU_FREEZE_TIM1();
  __HAL_DBGMCU_FREEZE_TIM2();
  __HAL_DBGMCU_FREEZE_TIM3();
  __HAL_DBGMCU_FREEZE_TIM4();
  __HAL_DBGMCU_FREEZE_TIM5();
#endif

  __HAL_TIM_CLEAR_IT(&htim1, TIM_IT_ALL);
  __HAL_TIM_CLEAR_IT(&htim3, TIM_IT_ALL);
  __HAL_TIM_CLEAR_IT(&htim4, TIM_IT_ALL);
  __HAL_TIM_CLEAR_IT(&htim5, TIM_IT_ALL);

  // The ICS logic won't work properly with 3.3V signals.
  // XXX: HSYNC / VSYNC PINS CONFIGURED IN OPEN DRAIN MODE, USE EXTERNAL 4k7-10K PULLUPS TO 5V !!!!
  HAL_setPinHigh(HSYNC);                                                                                                                // This is the behavior on the ICS at boot
  HAL_setPinLow(VSYNC);
  HAL_setPinMode(HSYNC, MODE_OUTPUT);                                                                                                   // Set HSYNC pin to GPIO output
  HAL_setPinHigh(LED);                                                                                                                  // LED off

  /*        TIM1
      Makes HSYNC to VSYNC delay
      Uses system clock.
      Slave trigger mode from TIM4 (Hsync to VSYNC divider).
      Uses TIM_UP DMA to write to the VSYNC GPIO.
      The next delay is self-updated by TIM1_CC1, which writes the next overflow value to TIM1-ARR
   */
  __HAL_TIM_ENABLE_DMA(&htim1, TIM_DMA_UPDATE);
  __HAL_TIM_ENABLE_DMA(&htim1, TIM_DMA_CC1);
  HAL_DMA_Start(&hdma_tim1_up,  (uint32_t)vsync_toggle, (uint32_t)&VSYNC_GPIO_Port->BSRR, sizeof(vsync_toggle)/sizeof(vsync_toggle[0]));                                           // use UPDATE DMA to write to GPIO
  HAL_DMA_Start(&hdma_tim1_ch1_ch2_ch3, (uint32_t)hsync_vsync_delay, (uint32_t)&TIM1->ARR, sizeof(hsync_vsync_delay)/sizeof(hsync_vsync_delay[0]));                                           // use UPDATE DMA to write to GPIO
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);                                                                                             // Start TIM1 PWM


  // TIM3  -  CSYNC - Slave reset mode from TIM2 to ensure sync at all times.
  TIM3->ARR = TIM_HLINE;                                                                                                                // Start in pre-equalizing phase
  TIM3->CCR1 = PEQ_LOW;
  __HAL_TIM_ENABLE_DMA(&htim3, TIM_DMA_UPDATE);
  HAL_DMA_Start(&hdma_tim3_ch4_up, (uint32_t)csync, (uint32_t)&TIM3->DMAR, sizeof(csync)/sizeof(csync[0][0]));                          // Start TIM3 Update DMA: Updates TIM4 ARR value on each update for VSYNC delay
  TIM3->DCR = TIM_DMABASE_ARR | TIM_DMABURSTLENGTH_3TRANSFERS;                                                                          // Enable burst transfers


  /*        TIM4
      Takes HSYNC clock from TIM2, divides by field lines, updates next overflow value (Odd / Even fields).
      TRGO for TIM1, which makes a delay, then triggers a DMA write to VSYNC GPIO
  */
  __HAL_TIM_ENABLE_DMA(&htim4, TIM_DMA_UPDATE);
  HAL_DMA_Start(&hdma_tim4_up, (uint32_t)hsync_vsync_div, (uint32_t)&TIM4->ARR, sizeof(hsync_vsync_div)/sizeof(hsync_vsync_div[0]));    // Enable TIM4 Update DMA: Updates TIM4 ARR value on each update
  TIM4->ARR = VSYNC_LINES+VSYNC_DELAY_LINES-1;                   // We start with vsync=low, so preload vsync width.
  HAL_TIM_Base_Start(&htim4);


  while(HAL_GetTick()==0);                                                                                                              // Make sure at least 1ms have passed, otherwise the following loop won't work
  uint32_t start=0;
  while(1){
    if(start==0){
      if(HAL_readPin(VSENSE) || !HAL_readPin(TEST))                                                                                     // Wait for target power or test mode
        start = HAL_GetTick();
    }
    else{
      if(!HAL_readPin(VSENSE) && HAL_readPin(TEST))                                                                                     // Target power removed or test button not pressed for long enough
        start = 0;                                                                                                                      // Restart loop
      else if(HAL_GetTick()-start > 50 ){                                                                                               // Target power or test button held for >50ms, start
        test_mode =  !HAL_readPin(TEST);                                                                                                // store test flag
        break;
      }
    }
  }
  HAL_setPinLow(LED);                                                                                                                   // LED on
  HAL_setPinMode(HSYNC, MODE_AF);                                                                                                       // Restore HSYNC pin to PWM output

  // TIM2  -  HSYNC for head unit - Master timer for TIM3, TIM4 & TIM5
  TIM2->ARR = TIM_LINE;
  TIM2->CCR1 = HSY;
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);                                                                                             // Start TIM2 PWM
  __HAL_TIM_MOE_ENABLE(&htim2);                                                                                                         // Enable TIM2 PWM output (HSYNC signal), Master timer

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);                                                                                             // Start TIM3 PWM
  __HAL_TIM_MOE_ENABLE(&htim3);                                                                                                         // Enable TIM3 PWM output (CSYNC signal)

  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);                                                                                             // Start TIM5 PWM
  __HAL_TIM_MOE_ENABLE(&htim5);                                                                                                         // Enable TIM5 PWM output (Pixel clock)
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if( !HAL_readPin(VSENSE) && !test_mode ){                                                                                           // Target power removed, not in test mode
      NVIC_SystemReset();                                                                                                               // Reboot system
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 199;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 3161;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OnePulse_Init(&htim1, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;
  sSlaveConfig.InputTrigger = TIM_TS_ITR3;
  if (HAL_TIM_SlaveConfigSynchro(&htim1, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 5;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 6323;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = 497;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_TIM_DISABLE_OCxPRELOAD(&htim2, TIM_CHANNEL_1);
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 6323;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_ITR1;
  if (HAL_TIM_SlaveConfigSynchro(&htim3, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = 458;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_TIM_DISABLE_OCxPRELOAD(&htim3, TIM_CHANNEL_1);
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 3;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_EXTERNAL1;
  sSlaveConfig.InputTrigger = TIM_TS_ITR1;
  if (HAL_TIM_SlaveConfigSynchro(&htim4, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 29;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  if (HAL_TIM_SlaveConfigSynchro(&htim5, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = 15;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(VSYNC_GPIO_Port, VSYNC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TEST_Pin */
  GPIO_InitStruct.Pin = TEST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(TEST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : VSYNC_Pin */
  GPIO_InitStruct.Pin = VSYNC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(VSYNC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : VSENSE_Pin */
  GPIO_InitStruct.Pin = VSENSE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(VSENSE_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
