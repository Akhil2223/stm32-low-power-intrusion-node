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
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

COMP_HandleTypeDef hcomp2;

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
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_COMP2_Init(void);
void StartSensorTask(void *argument);
void StartAlarmTask(void *argument);
void StartOutputTask(void *argument);
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
  /* USER CODE BEGIN 2 */
  HAL_COMP_Start(&hcomp2);
  HAL_UART_Transmit(&huart2, (uint8_t*)"Security node ready\r\n", 21, 100);
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

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
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
  HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : BUZZER_Pin LED_B_Pin */
  GPIO_InitStruct.Pin = BUZZER_Pin|LED_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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

  /*Configure GPIO pin : LED_G_Pin */
  GPIO_InitStruct.Pin = LED_G_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_G_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
	    if (osMessageQueueGet(sensorQueueHandle, &ev, NULL, osWaitForever) != osOK) continue;

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
	        if (HAL_GetTick() - last_shock > 1000) { last_shock = HAL_GetTick(); post_alarm(AE_SHOCK); }
	        break;
	      }
	      case EV_TILT: {
	        osDelay(50);
	        int tilted = (HAL_GPIO_ReadPin(TILT_SW_GPIO_Port, TILT_SW_Pin) == GPIO_PIN_SET);
	        if (tilted != last_tilt) { last_tilt = tilted; post_alarm(tilted ? AE_TILTED : AE_UPRIGHT); }
	        break;
	      }
	      case EV_BUTTON: {
	        if (HAL_GetTick() - last_btn > 300) { last_btn = HAL_GetTick(); post_alarm(AE_BUTTON); }
	        break;
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
	    if (osMessageQueueGet(alarmQueueHandle, &ev, NULL, osWaitForever) != osOK) continue;
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
  int blink = 0;
  for (;;)
  {
    uint32_t wait = osWaitForever;

    switch (g_state)
        {
          case ST_DISARMED:    set_rgb(0, 1, 0); set_buzzer(0); break;
          case ST_ARMED:       set_rgb(0, 0, 1); set_buzzer(0); break;
          case ST_ENTRY_DELAY: blink ^= 1; set_rgb(0, 0, blink); set_buzzer(blink); wait = 250; break;
          case ST_ALARM:       blink ^= 1; set_rgb(blink, 0, 0); set_buzzer(blink); wait = 100; break;
        }
    osThreadFlagsWait(0x01, osFlagsWaitAny, wait);
  }
  /* USER CODE END StartOutputTask */
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
