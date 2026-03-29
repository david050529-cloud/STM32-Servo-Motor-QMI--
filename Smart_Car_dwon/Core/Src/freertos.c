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
#include "qmi8658.h"
#include "motor_control.h"
#include "servo_control.h"
#include "encoder.h"
#include "pid.h"
#include "queue.h"
#include "string.h"
#include "usart.h"
/* USER CODE END Includes */

/* USER CODE BEGIN PV */
// 电机控制命令结构体
typedef struct {
  uint8_t motor_id;
  int16_t speed;
} MotorCmd_t;

// 舵机控制命令结构体
typedef struct {
  uint8_t servo_id;
  uint8_t angle;
} ServoCmd_t;

// 队列句柄
QueueHandle_t motor_cmd_queue;
QueueHandle_t servo_cmd_queue;

// 协议解析状态机变量
static uint8_t parse_state = 0;
static uint8_t packet[64];
static uint8_t packet_idx = 0;
static uint8_t data_len = 0;
static uint8_t cmd_type = 0;

// PID控制器实例
PID_HandleTypeDef pid_motor1;
PID_HandleTypeDef pid_motor2;

// 电机目标速度
int16_t motor1_target = 0;
int16_t motor2_target = 0;
/* USER CODE END PV */

/* Private typedef -----------------------------------------------------------*/
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
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Servo_Task */
osThreadId_t Servo_TaskHandle;
const osThreadAttr_t Servo_Task_attributes = {
  .name = "Servo_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Comm_Task */
osThreadId_t Comm_TaskHandle;
const osThreadAttr_t Comm_Task_attributes = {
  .name = "Comm_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for Motor_PID_Task */
osThreadId_t Motor_PID_TaskHandle;
const osThreadAttr_t Motor_PID_Task_attributes = {
  .name = "Motor_PID_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
/* USER CODE BEGIN 0 */
static uint8_t calc_checksum(uint8_t *data, int len)
{
    uint8_t sum = 0;
    for (int i = 0; i < len; i++) {
        sum += data[i];
    }
    return sum;
}

void ProcessUARTData(uint8_t *data, uint16_t len)
{
    for (int i = 0; i < len; i++) {
        uint8_t byte = data[i];
        switch (parse_state) {
            case 0: // 等待帧头1
                if (byte == 0xAA) {
                    parse_state = 1;
                    packet_idx = 0;
                    packet[packet_idx++] = byte;
                }
                break;
            case 1: // 等待帧头2
                if (byte == 0x55) {
                    parse_state = 2;
                    packet[packet_idx++] = byte;
                } else {
                    parse_state = 0;
                }
                break;
            case 2: // 读取数据长度
                data_len = byte;
                packet[packet_idx++] = byte;
                parse_state = 3;
                break;
            case 3: // 读取命令类型
                cmd_type = byte;
                packet[packet_idx++] = byte;
                parse_state = 4;
                break;
            case 4: // 读取数据域
                packet[packet_idx++] = byte;
                if (packet_idx - 4 >= data_len) {
                    parse_state = 5;
                }
                break;
            case 5: // 读取校验和
                packet[packet_idx++] = byte;
                parse_state = 6;
                break;
            case 6: // 读取帧尾
                if (byte == 0x0D) {
                    packet[packet_idx++] = byte;
                    // 校验
                    uint8_t calc_crc = calc_checksum(packet, packet_idx - 2);
                    if (calc_crc == packet[packet_idx - 2]) {
                        // 根据命令类型处理
                        switch (cmd_type) {
                            case 0x01: { // 电机控制
                                if (data_len >= 4) {
                                    MotorCmd_t cmd;
                                    cmd.motor_id = packet[4];
                                    cmd.speed = (packet[5] << 8) | packet[6];
                                    xQueueSend(motor_cmd_queue, &cmd, 0);
                                }
                                break;
                            }
                            case 0x02: { // 舵机控制
                                if (data_len >= 3) {
                                    ServoCmd_t cmd;
                                    cmd.servo_id = packet[4];
                                    cmd.angle = packet[5];
                                    xQueueSend(servo_cmd_queue, &cmd, 0);
                                }
                                break;
                            }
                            default:
                                break;
                        }
                    }
                }
                parse_state = 0;
                break;
            default:
                parse_state = 0;
                break;
        }
    }
}
/* USER CODE END 0 */
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartTask02(void *argument);
void StartTask03(void *argument);
void StartTask04(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

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
  motor_cmd_queue = xQueueCreate(5, sizeof(MotorCmd_t));
  servo_cmd_queue = xQueueCreate(5, sizeof(ServoCmd_t));
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of Servo_Task */
  Servo_TaskHandle = osThreadNew(StartTask02, NULL, &Servo_Task_attributes);

  /* creation of Comm_Task */
  Comm_TaskHandle = osThreadNew(StartTask03, NULL, &Comm_Task_attributes);

  /* creation of Motor_PID_Task */
  Motor_PID_TaskHandle = osThreadNew(StartTask04, NULL, &Motor_PID_Task_attributes);

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
    // 初始化各个外设
    Motor_Init();
    Servo_Init();
    Encoder_Init();
    QMI8658_Init();

    // 初始化PID参数（根据实际调整）
    PID_Init(&pid_motor1, 1.0f, 0.1f, 0.05f, 10000, -10000);
    PID_Init(&pid_motor2, 1.0f, 0.1f, 0.05f, 10000, -10000);
  /* Infinite loop */
  for(;;)
  {
    osDelay(1000);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the Servo_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
    ServoCmd_t cmd;
  /* Infinite loop */
  for(;;)
  {
      if (xQueueReceive(servo_cmd_queue, &cmd, 10) == pdTRUE) {
          Servo_SetAngle(cmd.servo_id, cmd.angle);
      }
      osDelay(1);
  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the Comm_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void *argument)
{
  /* USER CODE BEGIN StartTask03 */
    uint8_t tx_buffer[64];
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(50);  // 50ms发送一次

  /* Infinite loop */
    for(;;) {
        vTaskDelayUntil(&last_wake_time, period);

        // 读取IMU数据
        int16_t ax, ay, az, gx, gy, gz, temp;
        QMI8658_GetData(&ax, &ay, &az, &gx, &gy, &gz);
        temp = QMI8658_GetTemperature();

        // 构建数据包
        uint8_t data_field[14];
        data_field[0] = (ax >> 8) & 0xFF;
        data_field[1] = ax & 0xFF;
        data_field[2] = (ay >> 8) & 0xFF;
        data_field[3] = ay & 0xFF;
        data_field[4] = (az >> 8) & 0xFF;
        data_field[5] = az & 0xFF;
        data_field[6] = (gx >> 8) & 0xFF;
        data_field[7] = gx & 0xFF;
        data_field[8] = (gy >> 8) & 0xFF;
        data_field[9] = gy & 0xFF;
        data_field[10] = (gz >> 8) & 0xFF;
        data_field[11] = gz & 0xFF;
        data_field[12] = (temp >> 8) & 0xFF;
        data_field[13] = temp & 0xFF;

        // 构建完整帧
        tx_buffer[0] = 0xAA;
        tx_buffer[1] = 0x55;
        tx_buffer[2] = 14;
        tx_buffer[3] = 0x03;
        memcpy(&tx_buffer[4], data_field, 14);
        uint8_t checksum = calc_checksum(tx_buffer, 18);
        tx_buffer[18] = checksum;
        tx_buffer[19] = 0x0D;

        // 发送数据
        HAL_UART_Transmit(&huart3, tx_buffer, 20, 100);
        osDelay(1);
    }
  /* USER CODE END StartTask03 */
}

/* USER CODE BEGIN Header_StartTask04 */
/**
* @brief Function implementing the Motor_PID_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask04 */
void StartTask04(void *argument)
{
  /* USER CODE BEGIN StartTask04 */
    MotorCmd_t cmd;
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(10);  // 10ms控制周期
  /* Infinite loop */
    for(;;) {
        // 接收电机命令
        if (xQueueReceive(motor_cmd_queue, &cmd, 0) == pdTRUE) {
            if (cmd.motor_id == 1) {
                motor1_target = cmd.speed;
            } else if (cmd.motor_id == 2) {
                motor2_target = cmd.speed;
            } else if (cmd.motor_id == 0) {
                motor1_target = cmd.speed;
                motor2_target = cmd.speed;
            }
        }

        // 读取实际速度
        int16_t speed1 = GetMotorSpeed(1);
        int16_t speed2 = GetMotorSpeed(2);

        // 计算PID输出
        float output1 = PID_Update(&pid_motor1, motor1_target, speed1);
        float output2 = PID_Update(&pid_motor2, motor2_target, speed2);

        // 设置电机PWM
        Motor_SetSpeed(1, (int16_t)output1);
        Motor_SetSpeed(2, (int16_t)output2);

        vTaskDelayUntil(&last_wake_time, period);
        osDelay(1);
    }

  /* USER CODE END StartTask04 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

