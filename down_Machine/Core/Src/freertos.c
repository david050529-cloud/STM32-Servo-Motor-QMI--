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
#include <stdio.h>
#include "comm_protocol.h"
#include "motor_porting.h"
#include "QMI8658.h"
#include <string.h>
#include "queue.h"
#include "tim.h"
#include "usart.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// 外部引用电机和IMU数据
extern volatile float imu_roll, imu_pitch, imu_yaw;
extern EncoderMotorObjectTypeDef motor1, motor2;

// 串口接收缓冲区
// static uint8_t rx_buffer[sizeof(CommandPacket) + 4]; // 帧头2字节 + 数据 + 校验1字节
// static uint8_t rx_index = 0;
// static uint8_t rx_state = 0;  // 0=等待帧头1, 1=等待帧头2, 2=接收数据, 3=等待校验

// 发送缓冲区
static uint8_t tx_buffer[sizeof(TelemetryPacket) + 4];

// 队列句柄，用于接收串口数据
QueueHandle_t xUartRxQueue;
// 接收字节缓冲（用于中断）
uint8_t uart_rx_byte;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for UART_Task1 */
osThreadId_t UART_Task1Handle;
const osThreadAttr_t UART_Task1_attributes = {
  .name = "UART_Task1",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for UART_Task2 */
osThreadId_t UART_Task2Handle;
const osThreadAttr_t UART_Task2_attributes = {
  .name = "UART_Task2",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for IMU_Task */
osThreadId_t IMU_TaskHandle;
const osThreadAttr_t IMU_Task_attributes = {
  .name = "IMU_Task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void Send_Task(void *argument);
void Receive_Task(void *argument);
void IMU_Task(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  if (QMI8658_Init()) {
    printf("QMI8658 init success\r\n");
  } else {
    printf("QMI8658 init failed\r\n");
    Error_Handler();
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
  // 创建队列，存储 uint8_t，队列长度64
  xUartRxQueue = xQueueCreate(64, sizeof(uint8_t));
  if (xUartRxQueue == NULL) {
    // 创建失败处理
    Error_Handler();
  }
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of UART_Task1 */
  UART_Task1Handle = osThreadNew(Send_Task, NULL, &UART_Task1_attributes);

  /* creation of UART_Task2 */
  UART_Task2Handle = osThreadNew(Receive_Task, NULL, &UART_Task2_attributes);

  /* creation of IMU_Task */
  IMU_TaskHandle = osThreadNew(IMU_Task, NULL, &IMU_Task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_Send_Task */
/**
* @brief Function implementing the UART_Task1 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Send_Task */
void Send_Task(void *argument)
{
  /* USER CODE BEGIN Send_Task */
  TelemetryPacket tele;
  uint8_t checksum;
  const TickType_t period = pdMS_TO_TICKS(50); // 50ms发送一次
  /* Infinite loop */
  for(;;)
  {
    // 获取当前状态（注意加临界区保护，因为数据在中断中更新）
    taskENTER_CRITICAL();
    tele.roll = imu_roll;
    tele.pitch = imu_pitch;
    tele.yaw = imu_yaw;
    tele.motor1_actual_rps = motor1.rps;
    tele.motor2_actual_rps = motor2.rps;
    taskEXIT_CRITICAL();

    // 添加帧头0xAA 0x55
    tx_buffer[0] = 0xAA;
    tx_buffer[1] = 0x55;
    memcpy(tx_buffer + 2, &tele, sizeof(TelemetryPacket));

    // 计算校验和（异或）
    checksum = 0;
    for (int i = 0; i < sizeof(TelemetryPacket); i++)
      checksum ^= tx_buffer[2 + i];
    tx_buffer[2 + sizeof(TelemetryPacket)] = checksum;
    // 发送数据（阻塞方式，注意避免任务阻塞过久）
    HAL_UART_Transmit(&huart3, tx_buffer, sizeof(TelemetryPacket) + 3, period);
    vTaskDelay(period);
  }
  /* USER CODE END Send_Task */
}

/* USER CODE BEGIN Header_Receive_Task */
/**
* @brief Function implementing the UART_Task2 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Receive_Task */
void Receive_Task(void *argument)
{
  CommandPacket cmd;
  uint8_t c;
  uint8_t checksum;
  static uint8_t rx_buffer[sizeof(CommandPacket)]; // 数据缓冲区
  static uint8_t rx_index = 0;
  static uint8_t rx_state = 0;

  // 启动第一次中断接收
  HAL_UART_Receive_IT(&huart3, &uart_rx_byte, 1);

  for(;;)
  {
    // 阻塞等待队列中有字节
    if (xQueueReceive(xUartRxQueue, &c, portMAX_DELAY) != pdTRUE) continue;

    // 状态机解析（与之前相同）
    switch(rx_state)
    {
      case 0: // 等待帧头0xAA
        if (c == 0xAA) rx_state = 1;
        break;
      case 1:
        if (c == 0x55) rx_state = 2;
        else {
          rx_state = 0;
          rx_index = 0;
        }
        break;
      case 2: // 接收数据
        if (rx_index < sizeof(CommandPacket))
        {
          rx_buffer[rx_index++] = c;
        }
        if (rx_index >= sizeof(CommandPacket))
        {
          rx_state = 3;
        }
        break;
      case 3: // 接收校验和
      {
        checksum = 0;
        for (int i = 0; i < sizeof(CommandPacket); i++)
          checksum ^= rx_buffer[i];
        if (checksum == c)
        {
          memcpy(&cmd, rx_buffer, sizeof(CommandPacket));
          encoder_motor_set_speed(&motor1, cmd.motor1_target_rps);
          encoder_motor_set_speed(&motor2, -cmd.motor2_target_rps);
          servo_set_angle(cmd.servo_angle);
        }
        // 重置状态机
        rx_state = 0;
        rx_index = 0;
      }
        break;
      default:
        rx_state = 0;
        rx_index = 0;
        break;
    }
  }
  /* USER CODE END Receive_Task */
}

/* USER CODE BEGIN Header_IMU_Task */
/**
* @brief Function implementing the IMU_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_IMU_Task */
void IMU_Task(void *argument)
{
  const TickType_t period = pdMS_TO_TICKS(10); // 10ms周期
  for (;;)
  {
    float dt = 0.01f;  // 固定周期10ms
    float roll, pitch, yaw;
    // 读取IMU数据（非阻塞I2C，但HAL_I2C_Mem_Read内部使用无限等待，不影响其他任务）
    QMI8658_GetEuler(dt, &roll, &pitch, &yaw);
    // 更新全局变量，加临界区保护
    taskENTER_CRITICAL();
    imu_roll = roll;
    imu_pitch = pitch;
    imu_yaw = yaw;
    taskEXIT_CRITICAL();
    vTaskDelay(period);
  }
}


/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART3)
  {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    // 将接收到的字节放入队列
    xQueueSendFromISR(xUartRxQueue, &uart_rx_byte, &xHigherPriorityTaskWoken);
    // 重新启动下一次接收
    HAL_UART_Receive_IT(&huart3, &uart_rx_byte, 1);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
}

/* USER CODE END Application */

