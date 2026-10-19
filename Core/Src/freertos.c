/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app_tasks.h"
#include "c30d_safety.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for controlTask */
osThreadId_t controlTaskHandle;
uint32_t controlStack[ 512 ];
osStaticThreadDef_t controlTcb;
const osThreadAttr_t controlTask_attributes = {
  .name = "controlTask",
  .cb_mem = &controlTcb,
  .cb_size = sizeof(controlTcb),
  .stack_mem = &controlStack[0],
  .stack_size = sizeof(controlStack),
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for commTask */
osThreadId_t commTaskHandle;
uint32_t commStack[ 512 ];
osStaticThreadDef_t commTcb;
const osThreadAttr_t commTask_attributes = {
  .name = "commTask",
  .cb_mem = &commTcb,
  .cb_size = sizeof(commTcb),
  .stack_mem = &commStack[0],
  .stack_size = sizeof(commStack),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for sensorTask */
osThreadId_t sensorTaskHandle;
uint32_t sensorStack[ 384 ];
osStaticThreadDef_t sensorTcb;
const osThreadAttr_t sensorTask_attributes = {
  .name = "sensorTask",
  .cb_mem = &sensorTcb,
  .cb_size = sizeof(sensorTcb),
  .stack_mem = &sensorStack[0],
  .stack_size = sizeof(sensorStack),
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for monitorTask */
osThreadId_t monitorTaskHandle;
uint32_t monitorStack[ 256 ];
osStaticThreadDef_t monitorTcb;
const osThreadAttr_t monitorTask_attributes = {
  .name = "monitorTask",
  .cb_mem = &monitorTcb,
  .cb_size = sizeof(monitorTcb),
  .stack_mem = &monitorStack[0],
  .stack_size = sizeof(monitorStack),
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartControlTask(void *argument);
void StartCommTask(void *argument);
void StartSensorTask(void *argument);
void StartMonitorTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);
void vApplicationMallocFailedHook(void);

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
    (void)xTask;
    (void)pcTaskName;
    C30D_TemplatePanic();
}
/* USER CODE END 4 */

/* USER CODE BEGIN 5 */
void vApplicationMallocFailedHook(void)
{
    C30D_TemplatePanic();
}
/* USER CODE END 5 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  if (osKernelGetState() != osKernelReady) {
    C30D_TemplatePanic();
  }
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of controlTask */
  controlTaskHandle = osThreadNew(StartControlTask, NULL, &controlTask_attributes);

  /* creation of commTask */
  commTaskHandle = osThreadNew(StartCommTask, NULL, &commTask_attributes);

  /* creation of sensorTask */
  sensorTaskHandle = osThreadNew(StartSensorTask, NULL, &sensorTask_attributes);

  /* creation of monitorTask */
  monitorTaskHandle = osThreadNew(StartMonitorTask, NULL, &monitorTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  if ((controlTaskHandle == NULL) || (commTaskHandle == NULL) ||
      (sensorTaskHandle == NULL) || (monitorTaskHandle == NULL)) {
    C30D_TemplatePanic();
  }
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartControlTask */
/**
  * @brief  Function implementing the controlTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartControlTask */
void StartControlTask(void *argument)
{
  /* USER CODE BEGIN StartControlTask */
  Navbot_ControlTask(argument);
  /* USER CODE END StartControlTask */
}

/* USER CODE BEGIN Header_StartCommTask */
/**
* @brief Function implementing the commTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCommTask */
void StartCommTask(void *argument)
{
  /* USER CODE BEGIN StartCommTask */
  Navbot_CommTask(argument);
  /* USER CODE END StartCommTask */
}

/* USER CODE BEGIN Header_StartSensorTask */
/**
* @brief Function implementing the sensorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSensorTask */
void StartSensorTask(void *argument)
{
  /* USER CODE BEGIN StartSensorTask */
  Navbot_SensorTask(argument);
  /* USER CODE END StartSensorTask */
}

/* USER CODE BEGIN Header_StartMonitorTask */
/**
* @brief Function implementing the monitorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMonitorTask */
void StartMonitorTask(void *argument)
{
  /* USER CODE BEGIN StartMonitorTask */
  Navbot_MonitorTask(argument);
  /* USER CODE END StartMonitorTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

