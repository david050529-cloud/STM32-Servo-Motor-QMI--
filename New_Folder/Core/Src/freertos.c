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
#include "comm_protocol.h"
#include "motor_porting.h"
#include "QMI8658.h"
#include "usart.h"
#include <string.h>
#include "pid.h"
#include "queue.h"
#include "tim.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct {
  float motor1_target_rps;
  float motor2_target_rps;
} MotorCmd_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
static MotorCmd_t MotorCmd;
// 遥测数据发送周期（毫秒）
#define TELEMETRY_SEND_PERIOD_MS    20
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
// 队列句柄
static QueueHandle_t xMotorCmdQueue = NULL;
static QueueHandle_t xServoCmdQueue = NULL;

// 共享姿态数据（由IMU任务更新，遥测任务读取）
static volatile float g_roll = 0.0f;
static volatile float g_pitch = 0.0f;
static volatile float g_yaw = 0.0f;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for myTask01 */
osThreadId_t myTask01Handle;
const osThreadAttr_t myTask01_attributes = {
  .name = "myTask01",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for myTask02 */
osThreadId_t myTask02Handle;
const osThreadAttr_t myTask02_attributes = {
  .name = "myTask02",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for myTask03 */
osThreadId_t myTask03Handle;
const osThreadAttr_t myTask03_attributes = {
  .name = "myTask03",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myTask04 */
osThreadId_t myTask04Handle;
const osThreadAttr_t myTask04_attributes = {
  .name = "myTask04",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for myTask05 */
osThreadId_t myTask05Handle;
const osThreadAttr_t myTask05_attributes = {
  .name = "myTask05",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static void SystemHardwareInit(void);  // 硬件初始化（电机、IMU、PWM等）
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void MotorCtrlTask(void *argument);
void CmdParseTask(void *argument);
void IMUTask(void *argument);
void ServoTask(void *argument);
void DataSendTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  SystemHardwareInit();

  // 创建队列
  xMotorCmdQueue = xQueueCreate(5, sizeof(MotorCmd_t));
  xServoCmdQueue = xQueueCreate(5, sizeof(uint16_t));
  if (xMotorCmdQueue == NULL || xServoCmdQueue == NULL) {
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
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of myTask01 */
  myTask01Handle = osThreadNew(MotorCtrlTask, NULL, &myTask01_attributes);

  /* creation of myTask02 */
  myTask02Handle = osThreadNew(CmdParseTask, NULL, &myTask02_attributes);

  /* creation of myTask03 */
  myTask03Handle = osThreadNew(IMUTask, NULL, &myTask03_attributes);

  /* creation of myTask04 */
  myTask04Handle = osThreadNew(ServoTask, NULL, &myTask04_attributes);

  /* creation of myTask05 */
  myTask05Handle = osThreadNew(DataSendTask, NULL, &myTask05_attributes);

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

/* USER CODE BEGIN Header_MotorCtrlTask */
/**
* @brief Function implementing the myTask01 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_MotorCtrlTask */
void MotorCtrlTask(void *argument)
{
  /* USER CODE BEGIN MotorCtrlTask */
  /* Infinite loop */
  for(;;)
  {
    if (xQueueReceive(xMotorCmdQueue, &MotorCmd, portMAX_DELAY) == pdTRUE)
    {
      // 设置 PID 目标值
      PID_SetTarget(&pid_motor1, MotorCmd.motor1_target_rps);
      PID_SetTarget(&pid_motor2, MotorCmd.motor2_target_rps);

      // 获取实际转速（已在 TIM6 中断中更新 motor.rps）
      float rps1 = motor1.rps;
      float rps2 = motor2.rps;

      // 计算 PID 输出（增量式，内部自动累加）
      float pulse1 = PID_Update_Position(&pid_motor1, rps1);
      float pulse2 = PID_Update_Position(&pid_motor2, rps2);

      // 电机 2 方向取反（根据原有逻辑）
      motor_set_pulse(&motor1, (int)pulse1);
      motor_set_pulse(&motor2, -(int)pulse2);
    }
    osDelay(1);  // 控制周期可适当调整，例如 10ms
  }
  /* USER CODE END MotorCtrlTask */
}

/* USER CODE BEGIN Header_CmdParseTask */
/**
* @brief Function implementing the myTask02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CmdParseTask */
void CmdParseTask(void *argument)
{
  /* USER CODE BEGIN CmdParseTask */
  CommandPacket rxCmd;
  MotorCmd_t motorCmd;
  uint16_t servoAngle;
  /* Infinite loop */
  for(;;)
  {
    // 阻塞接收一帧指令（超时100ms）
    if (HAL_UART_Receive(&huart3, (uint8_t*)&rxCmd, sizeof(CommandPacket), 100) == HAL_OK)
    {
      // 解析电机指令
      motorCmd.motor1_target_rps = rxCmd.motor1_target_rps;
      motorCmd.motor2_target_rps = rxCmd.motor2_target_rps;
      xQueueSend(xMotorCmdQueue, &motorCmd, 0);

      // 解析舵机角度（0~180）
      servoAngle = rxCmd.servo_angle;
      if (servoAngle > 180) servoAngle = 180;
      xQueueSend(xServoCmdQueue, &servoAngle, 0);
    }
    osDelay(1);
  }
  /* USER CODE END CmdParseTask */
}

/* USER CODE BEGIN Header_IMUTask */
/**
* @brief Function implementing the myTask03 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_IMUTask */
void IMUTask(void *argument)
{
  /* USER CODE BEGIN IMUTask */
  float roll, pitch, yaw;
  /* Infinite loop */
  for(;;)
  {
    // 获取欧拉角（内部使用上次调用时间差，建议固定周期10ms）
    QMI8658_GetEulerSimple(&roll, &pitch, &yaw);
    // 更新全局变量（volatile保证其他任务可见）
    g_roll = roll;
    g_pitch = pitch;
    g_yaw = yaw;
    osDelay(10);  // 10ms周期，与传感器ODR匹配
  }
  /* USER CODE END IMUTask */
}

/* USER CODE BEGIN Header_ServoTask */
/**
* @brief Function implementing the myTask04 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ServoTask */
void ServoTask(void *argument)
{
  /* USER CODE BEGIN ServoTask */
  uint16_t angle = 90;  // 默认90度
  /* Infinite loop */
  for(;;)
  {
    // 尝试接收舵机指令（非阻塞，若无新指令则保持上次角度）
    if (xQueueReceive(xServoCmdQueue, &angle, 0) == pdTRUE)
    {
      // 将角度(0~180)映射到PWM比较值
      // 假设舵机脉宽范围：500us~2500us，周期20ms(50Hz)
      // TIM3时钟频率需根据实际配置计算，此处示例映射到0~2000范围
      // 用户应根据实际PWM周期调整映射系数
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, angle);
    }
    osDelay(5);  // 每5ms检查一次
  }
  /* USER CODE END ServoTask */
}

/* USER CODE BEGIN Header_DataSendTask */
/**
* @brief Function implementing the myTask05 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_DataSendTask */
void DataSendTask(void *argument)
{
  /* USER CODE BEGIN DataSendTask */
  TelemetryPacket txPacket;
  TickType_t lastWakeTime = xTaskGetTickCount();
  /* Infinite loop */
  for(;;)
  {
    // 等待固定周期
    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(TELEMETRY_SEND_PERIOD_MS));

    // 填充遥测数据
    txPacket.roll = g_roll;
    txPacket.pitch = g_pitch;
    txPacket.yaw = g_yaw;
    txPacket.motor1_actual_rps = motor1.rps;
    txPacket.motor2_actual_rps = motor2.rps;

    // 通过串口发送（阻塞方式，由于数据量小且优先级低，影响可控）
    HAL_UART_Transmit(&huart3, (uint8_t*)&txPacket, sizeof(TelemetryPacket), 100);
    osDelay(1);
  }
  /* USER CODE END DataSendTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
static void SystemHardwareInit(void) {
  // 初始化电机（包含PWM启动、编码器启动、TIM6中断启动）
  motor_init();

  // 初始化QMI8658姿态传感器
  if (!QMI8658_Init())
  {
    Error_Handler();
  }

  // 启动舵机PWM
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  // 设置初始角度90度
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 90 * 2000 / 180);
}
/* USER CODE END Application */

