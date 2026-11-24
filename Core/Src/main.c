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
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include<string.h>
#include "FreeRTOS.h"
#include "task.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum { EV_DOOR = 1, EV_TRIP, EV_SHOCK, EV_TILT, EV_BUTTON } sensor_event_t;

typedef enum { AE_DOOR_OPEN = 1, AE_DOOR_CLOSED, AE_TRIP_BROKEN, AE_TRIP_OK,
               AE_SHOCK, AE_TILTED, AE_UPRIGHT, AE_BUTTON, AE_ENTRY_TIMEOUT } alarm_event_t;

typedef enum { ST_DISARMED = 0, ST_ARMED, ST_ENTRY_DELAY, ST_ALARM } alarm_state_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ENTRY_DELAY_MS  10000
#define BUZZER_ENABLE 1
#define WDG_SENSOR   (1U << 0)
#define WDG_ALARM    (1U << 1)
#define WDG_OUTPUT   (1U << 2)
#define WDG_ALL      (WDG_SENSOR | WDG_ALARM | WDG_OUTPUT)
#define WDG_TEST_HANG 0     // set to 1 to test: a knock (shock) freezes sensorTask
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

COMP_HandleTypeDef hcomp2;

IWDG_HandleTypeDef hiwdg;

LPTIM_HandleTypeDef hlptim1;

UART_HandleTypeDef huart2;

/* Definitions for sensorTask */
osThreadId_t sensorTaskHandle;
const osThreadAttr_t sensorTask_attributes = {
  .name = "sensorTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for alarmTask */
osThreadId_t alarmTaskHandle;
const osThreadAttr_t alarmTask_attributes = {
  .name = "alarmTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for outputTask */
osThreadId_t outputTaskHandle;
const osThreadAttr_t outputTask_attributes = {
  .name = "outputTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for wdgTask */
osThreadId_t wdgTaskHandle;
const osThreadAttr_t wdgTask_attributes = {
  .name = "wdgTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for sensorQueue */
osMessageQueueId_t sensorQueueHandle;
const osMessageQueueAttr_t sensorQueue_attributes = {
  .name = "sensorQueue"
};
/* Definitions for alarmQueue */
osMessageQueueId_t alarmQueueHandle;
const osMessageQueueAttr_t alarmQueue_attributes = {
  .name = "alarmQueue"
};
/* Definitions for entryTimer */
osTimerId_t entryTimerHandle;
const osTimerAttr_t entryTimer_attributes = {
  .name = "entryTimer"
};
/* Definitions for uartMutex */
osMutexId_t uartMutexHandle;
const osMutexAttr_t uartMutex_attributes = {
  .name = "uartMutex"
};
/* USER CODE BEGIN PV */
volatile alarm_state_t g_state = ST_DISARMED;
osEventFlagsId_t wdgEventsHandle;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_COMP2_Init(void);
static void MX_IWDG_Init(void);
static void MX_LPTIM1_Init(void);
void StartSensorTask(void *argument);
void StartAlarmTask(void *argument);
void StartOutputTask(void *argument);
void StartWdgTask(void *argument);
void EntryTimerCallback(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static const char *ev_text[] = { "", "Door OPEN", "Door CLOSED", "Tripwire BROKEN", "Tripwire OK",
                                 "Tamper: SHOCK", "Tamper: TILTED", "Tilt: upright",
                                 "Button", "Entry delay expired" };
static const char *state_text[] = { "DISARMED", "ARMED", "ENTRY DELAY (10 s)", "*** ALARM ***" };

static void log_str(const char *s)
{
  osMutexAcquire(uartMutexHandle, osWaitForever);
  HAL_UART_Transmit(&huart2, (uint8_t*)s, strlen(s), 100);
  osMutexRelease(uartMutexHandle);
}

static void post_alarm(uint8_t ev)
{
  osMessageQueuePut(alarmQueueHandle, &ev, 0, 0);
}

static void post_sensor(uint8_t ev)
{
  osMessageQueuePut(sensorQueueHandle, &ev, 0, 0);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == DOOR_SW_Pin)       post_sensor(EV_DOOR);
  else if (GPIO_Pin == SHOCK_SW_Pin) post_sensor(EV_SHOCK);
  else if (GPIO_Pin == TILT_SW_Pin)  post_sensor(EV_TILT);
  else if (GPIO_Pin == USER_BUTTON_PIN) post_sensor(EV_BUTTON);
}

void HAL_COMP_TriggerCallback(COMP_HandleTypeDef *hcomp)
{
  if (hcomp->Instance == COMP2) post_sensor(EV_TRIP);
}

void BSP_PB_Callback(Button_TypeDef Button)
{
  if (Button == BUTTON_USER) post_sensor(EV_BUTTON);
}
static void set_rgb(int r, int g, int b)
{
  HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, r ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, g ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, b ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void set_buzzer(int on)
{
  HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin,
                    (on && BUZZER_ENABLE) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
static void set_relay(int on)
{
  HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
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
  MX_USART2_UART_Init();
  MX_COMP2_Init();
  MX_IWDG_Init();
  MX_LPTIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_COMP_Start(&hcomp2);
  HAL_UART_Transmit(&huart2, (uint8_t*)"Security node ready\r\n", 21, 100);
  __HAL_DBGMCU_FREEZE_IWDG();   // don't reset while paused in the debugger

   if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST))
     HAL_UART_Transmit(&huart2, (uint8_t*)"Reset cause: WATCHDOG\r\n", 23, 100);
   else
     HAL_UART_Transmit(&huart2, (uint8_t*)"Reset cause: normal\r\n", 21, 100);
   __HAL_RCC_CLEAR_RESET_FLAGS();
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of uartMutex */
  uartMutexHandle = osMutexNew(&uartMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of entryTimer */
  entryTimerHandle = osTimerNew(EntryTimerCallback, osTimerOnce, NULL, &entryTimer_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of sensorQueue */
  sensorQueueHandle = osMessageQueueNew (16, sizeof(uint8_t), &sensorQueue_attributes);

  /* creation of alarmQueue */
  alarmQueueHandle = osMessageQueueNew (16, sizeof(uint8_t), &alarmQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of sensorTask */
  sensorTaskHandle = osThreadNew(StartSensorTask, NULL, &sensorTask_attributes);

  /* creation of alarmTask */
  alarmTaskHandle = osThreadNew(StartAlarmTask, NULL, &alarmTask_attributes);

  /* creation of outputTask */
  outputTaskHandle = osThreadNew(StartOutputTask, NULL, &outputTask_attributes);

  /* creation of wdgTask */
  wdgTaskHandle = osThreadNew(StartWdgTask, NULL, &wdgTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */wdgEventsHandle = osEventFlagsNew(NULL);
  /* USER CODE END RTOS_EVENTS */

  /* Initialize leds */
  BSP_LED_Init(LED_GREEN);

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
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
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_LSE
                              |RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief COMP2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_COMP2_Init(void)
{

  /* USER CODE BEGIN COMP2_Init 0 */

  /* USER CODE END COMP2_Init 0 */

  /* USER CODE BEGIN COMP2_Init 1 */

  /* USER CODE END COMP2_Init 1 */
  hcomp2.Instance = COMP2;
  hcomp2.Init.InvertingInput = COMP_INPUT_MINUS_3_4VREFINT;
  hcomp2.Init.NonInvertingInput = COMP_INPUT_PLUS_IO1;
  hcomp2.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
  hcomp2.Init.Hysteresis = COMP_HYSTERESIS_HIGH;
  hcomp2.Init.BlankingSrce = COMP_BLANKINGSRC_NONE;
  hcomp2.Init.Mode = COMP_POWERMODE_ULTRALOWPOWER;
  hcomp2.Init.WindowMode = COMP_WINDOWMODE_DISABLE;
  hcomp2.Init.TriggerMode = COMP_TRIGGERMODE_IT_RISING_FALLING;
  if (HAL_COMP_Init(&hcomp2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN COMP2_Init 2 */

  /* USER CODE END COMP2_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
  hiwdg.Init.Window = 4095;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief LPTIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPTIM1_Init(void)
{

  /* USER CODE BEGIN LPTIM1_Init 0 */

  /* USER CODE END LPTIM1_Init 0 */

  /* USER CODE BEGIN LPTIM1_Init 1 */

  /* USER CODE END LPTIM1_Init 1 */
  hlptim1.Instance = LPTIM1;
  hlptim1.Init.Clock.Source = LPTIM_CLOCKSOURCE_APBCLOCK_LPOSC;
  hlptim1.Init.Clock.Prescaler = LPTIM_PRESCALER_DIV1;
  hlptim1.Init.Trigger.Source = LPTIM_TRIGSOURCE_SOFTWARE;
  hlptim1.Init.OutputPolarity = LPTIM_OUTPUTPOLARITY_HIGH;
  hlptim1.Init.UpdateMode = LPTIM_UPDATE_IMMEDIATE;
  hlptim1.Init.CounterSource = LPTIM_COUNTERSOURCE_INTERNAL;
  hlptim1.Init.Input1Source = LPTIM_INPUT1SOURCE_GPIO;
  hlptim1.Init.Input2Source = LPTIM_INPUT2SOURCE_GPIO;
  if (HAL_LPTIM_Init(&hlptim1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPTIM1_Init 2 */

  /* USER CODE END LPTIM1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  HAL_GPIO_WritePin(GPIOA, BUZZER_Pin|LED_B_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RELAY_Pin|LED_G_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : BUZZER_Pin LED_B_Pin */
  GPIO_InitStruct.Pin = BUZZER_Pin|LED_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : RELAY_Pin LED_G_Pin */
  GPIO_InitStruct.Pin = RELAY_Pin|LED_G_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_R_Pin */
  GPIO_InitStruct.Pin = LED_R_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_R_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SHOCK_SW_Pin */
  GPIO_InitStruct.Pin = SHOCK_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SHOCK_SW_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : TILT_SW_Pin DOOR_SW_Pin */
  GPIO_InitStruct.Pin = TILT_SW_Pin|DOOR_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* ===== STOP2 tickless idle using LPTIM1 (LSE 32.768 kHz) ===== */
#define LPTIM_HZ          32768U
#define MIN_STOP2_TICKS   5U      /* not worth sleeping for < 5 ms        */
#define MAX_STOP2_TICKS   1900U   /* 16-bit LPTIM overflows at ~2 s       */

static uint32_t lptim_read_cnt(void)
{
  /* LPTIM runs on its own clock: read until two reads agree */
  uint32_t a, b;
  do { a = LPTIM1->CNT; b = LPTIM1->CNT; } while (a != b);
  return a;
}

void vPortSuppressTicksAndSleep(TickType_t xExpectedIdleTime)
{
  if (xExpectedIdleTime < MIN_STOP2_TICKS) return;
  if (xExpectedIdleTime > MAX_STOP2_TICKS) xExpectedIdleTime = MAX_STOP2_TICKS;

  /* 1. Stop the RTOS tick */
  SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;

  __disable_irq();
  if (eTaskConfirmSleepModeStatus() == eAbortSleep) {   /* something became ready */
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
    __enable_irq();
    return;
  }

  /* 2. Arm LPTIM1 to wake us when the next task is due */
  uint32_t counts = (xExpectedIdleTime * LPTIM_HZ) / 1000U;
  HAL_LPTIM_TimeOut_Start_IT(&hlptim1, 0xFFFF, counts);

  /* 3. Deep sleep. Any EXTI (sensors/button), COMP2 or LPTIM1 wakes us */
  HAL_SuspendTick();
  HAL_PWREx_EnterSTOP2Mode(PWR_STOPENTRY_WFI);
  HAL_ResumeTick();            /* we wake up on MSI 4 MHz: no clock restore needed */

  /* 4. Tell FreeRTOS how long we really slept */
  uint32_t slept = (lptim_read_cnt() * 1000U) / LPTIM_HZ;
  HAL_LPTIM_TimeOut_Stop_IT(&hlptim1);
  if (slept > xExpectedIdleTime) slept = xExpectedIdleTime;
  vTaskStepTick(slept);

  /* 5. Restart the RTOS tick; the wake-up interrupt runs right after */
  SysTick->VAL = 0;
  SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
  __enable_irq();
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartSensorTask */
/**
  * @brief  Function implementing the sensorTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartSensorTask */
void StartSensorTask(void *argument)
{
  /* USER CODE BEGIN 5 */
	  int last_door = -1, last_trip = -1, last_tilt = -1;
	  uint32_t last_shock = 0, last_btn = 0;
	  uint8_t ev;

	  for (;;)
	  {
	    osEventFlagsSet(wdgEventsHandle, WDG_SENSOR);
	    if (osMessageQueueGet(sensorQueueHandle, &ev, NULL, 1000) != osOK) continue;

	    switch (ev)
	    {
	      case EV_DOOR: {
	        osDelay(20);
	        int open = (HAL_GPIO_ReadPin(DOOR_SW_GPIO_Port, DOOR_SW_Pin) == GPIO_PIN_SET);
	        if (open != last_door) { last_door = open; post_alarm(open ? AE_DOOR_OPEN : AE_DOOR_CLOSED); }
	        break;
	      }
	      case EV_TRIP: {
	        osDelay(20);
	        int beam_on = (HAL_COMP_GetOutputLevel(&hcomp2) == COMP_OUTPUT_LEVEL_HIGH);
	        if (beam_on != last_trip) { last_trip = beam_on; post_alarm(beam_on ? AE_TRIP_OK : AE_TRIP_BROKEN); }
	        break;
	      }
	      case EV_SHOCK: {
	#if WDG_TEST_HANG
	        log_str("!!! Simulating sensorTask hang\r\n");
	        for (;;) { }
	#endif
	        if (osKernelGetTickCount() - last_shock > 1000) { last_shock = osKernelGetTickCount(); post_alarm(AE_SHOCK); }
	        break;
	      }
	      case EV_TILT: {
	        osDelay(50);
	        int tilted = (HAL_GPIO_ReadPin(TILT_SW_GPIO_Port, TILT_SW_Pin) == GPIO_PIN_SET);
	        if (tilted != last_tilt) { last_tilt = tilted; post_alarm(tilted ? AE_TILTED : AE_UPRIGHT); }
	        break;
	      }
	      case EV_BUTTON: {
	          if (osKernelGetTickCount() - last_btn > 300) { last_btn = osKernelGetTickCount(); post_alarm(AE_BUTTON); }
	      }
	    }
	  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartAlarmTask */
/**
* @brief Function implementing the alarmTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartAlarmTask */
void StartAlarmTask(void *argument)
{
  /* USER CODE BEGIN StartAlarmTask */
	  alarm_state_t state = ST_DISARMED;
	  uint8_t ev;
	  char line[48];

	  log_str(">>> State: DISARMED\r\n");

	  for (;;)
	 	  {
	 	    osEventFlagsSet(wdgEventsHandle, WDG_ALARM);
	 	    if (osMessageQueueGet(alarmQueueHandle, &ev, NULL, 1000) != osOK) continue;
	 	    if (ev < AE_DOOR_OPEN || ev > AE_ENTRY_TIMEOUT) continue;
	    snprintf(line, sizeof line, "[%s] %s\r\n", state_text[state], ev_text[ev]);
	    log_str(line);

	    alarm_state_t next = state;
	    int tamper = (ev == AE_SHOCK || ev == AE_TILTED);

	    switch (state)
	    {
	      case ST_DISARMED:
	        if (ev == AE_BUTTON) next = ST_ARMED;
	        break;

	      case ST_ARMED:
	        if (ev == AE_BUTTON) next = ST_DISARMED;
	        else if (ev == AE_DOOR_OPEN) { osTimerStart(entryTimerHandle, ENTRY_DELAY_MS); next = ST_ENTRY_DELAY; }
	        else if (ev == AE_TRIP_BROKEN || tamper) next = ST_ALARM;
	        break;

	      case ST_ENTRY_DELAY:
	        if (ev == AE_BUTTON) { osTimerStop(entryTimerHandle); next = ST_DISARMED; }
	        else if (ev == AE_ENTRY_TIMEOUT) next = ST_ALARM;
	        else if (tamper) { osTimerStop(entryTimerHandle); next = ST_ALARM; }
	        break;

	      case ST_ALARM:
	        if (ev == AE_BUTTON) next = ST_DISARMED;
	        break;
	    }

	    if (next != state) {
	      state = next;
	      g_state = state;
	      osThreadFlagsSet(outputTaskHandle, 0x01);
	      snprintf(line, sizeof line, ">>> State: %s\r\n", state_text[state]);
	      log_str(line);
	    }
	  }
  /* USER CODE END StartAlarmTask */
}

/* USER CODE BEGIN Header_StartOutputTask */
/**
* @brief Function implementing the outputTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartOutputTask */
void StartOutputTask(void *argument)
{
  /* USER CODE BEGIN StartOutputTask */
	int blink = 0, beat = 0;
	  for (;;)
	  {
	    osEventFlagsSet(wdgEventsHandle, WDG_OUTPUT);
	    uint32_t wait = 1000;

	    set_relay(g_state != ST_DISARMED);   // locked unless disarmed

	    switch (g_state)
	    {
	      case ST_DISARMED:    beat ^= 1; set_rgb(0, beat, 0); set_buzzer(0); wait = beat ? 30 : 970; break;
	      case ST_ARMED:       beat ^= 1; set_rgb(0, 0, beat); set_buzzer(0); wait = beat ? 30 : 970; break;
	      case ST_ENTRY_DELAY: blink ^= 1; set_rgb(0, 0, blink); set_buzzer(blink); wait = 250; break;
	      case ST_ALARM:       blink ^= 1; set_rgb(blink, 0, 0); set_buzzer(blink); wait = 100; break;
	    }

	    osThreadFlagsWait(0x01, osFlagsWaitAny, wait);
	  }
  /* USER CODE END StartOutputTask */
}

/* USER CODE BEGIN Header_StartWdgTask */
/**
* @brief Function implementing the wdgTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartWdgTask */
void StartWdgTask(void *argument)
{
  /* USER CODE BEGIN StartWdgTask */
  /* Infinite loop */
	for (;;)
	  {
	    uint32_t f = osEventFlagsWait(wdgEventsHandle, WDG_ALL, osFlagsWaitAll, 2000);

	    if (!(f & osFlagsError)) {
	      HAL_IWDG_Refresh(&hiwdg);          // everyone checked in: kick the dog
	    } else {
	      char m[48];
	      uint32_t missing = WDG_ALL & ~osEventFlagsGet(wdgEventsHandle);
	      snprintf(m, sizeof m, "WDG: missed check-in, mask 0x%02lX\r\n", missing);
	      log_str(m);                        // no kick: IWDG will reset us
	    }
	  }
  /* USER CODE END StartWdgTask */
}

/* EntryTimerCallback function */
void EntryTimerCallback(void *argument)
{
  /* USER CODE BEGIN EntryTimerCallback */
	post_alarm(AE_ENTRY_TIMEOUT);
  /* USER CODE END EntryTimerCallback */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
