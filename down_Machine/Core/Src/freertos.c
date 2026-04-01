/* USER CODE BEGIN Header */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include "comm_protocol.h"
#include "usart.h"
#include "encoder_motor.h"
#include "motor_porting.h"
#include "QMI8658.h"
#include "queue.h"
#include "semphr.h"
#include "tim.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// static TelemetryPacket cmd;
// static CommandPacket txPacket;

float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
float act_rps_1 = 0.0f, act_rps_2 = 0.0f;

float target_rps_1 = 0.0f, target_rps_2 = 0.0f;
uint16_t servo_angle = 0;

/* 任务间通信对象 */
static QueueHandle_t telemetryQueue = NULL;      // 存放遥测数据的队列
static SemaphoreHandle_t uartTxCompleteSem = NULL; // DMA发送完成信号量
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
/* Definitions for myTask01 */
osThreadId_t myTask01Handle;
const osThreadAttr_t myTask01_attributes = {
  .name = "myTask01",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for myTask02 */
osThreadId_t myTask02Handle;
const osThreadAttr_t myTask02_attributes = {
  .name = "myTask02",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal2,
};
/* Definitions for myTask03 */
osThreadId_t myTask03Handle;
const osThreadAttr_t myTask03_attributes = {
  .name = "myTask03",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for myTask04 */
osThreadId_t myTask04Handle;
const osThreadAttr_t myTask04_attributes = {
  .name = "myTask04",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal4,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void Send_and_Receive(void *argument);
void Read_IMU(void *argument);
void Control(void *argument);
void UART_Tx_Task(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  //初始化姿态传感器
  if (QMI8658_Init()) {
  } else {
    Error_Handler();
  }

  //初始化舵机
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  uint16_t angle_init = 90;
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, angle_init);

  motor_init();                      // 初始化电机

  /* 创建队列和信号量 */
  telemetryQueue = xQueueCreate(5, sizeof(TelemetryPacket)); // 最多缓存5个遥测包
  uartTxCompleteSem = xSemaphoreCreateBinary();
  if (telemetryQueue == NULL || uartTxCompleteSem == NULL) {
    Error_Handler();
  }
  
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of myTask01 */
  myTask01Handle = osThreadNew(Send_and_Receive, NULL, &myTask01_attributes);

  /* creation of myTask02 */
  myTask02Handle = osThreadNew(Read_IMU, NULL, &myTask02_attributes);

  /* creation of myTask03 */
  myTask03Handle = osThreadNew(Control, NULL, &myTask03_attributes);

  /* creation of myTask04 */
  myTask04Handle = osThreadNew(UART_Tx_Task, NULL, &myTask04_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
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

/* USER CODE BEGIN Header_Send_and_Receive */
/* USER CODE END Header_Send_and_Receive */
void Send_and_Receive(void *argument)
{
  /* USER CODE BEGIN Send_and_Receive */
  uint8_t rx_buffer[sizeof(CommandPacket)];
  CommandPacket cmd;
  TelemetryPacket txPacket;
  /* Infinite loop */
  for(;;)
  {
    // 接收命令（阻塞10ms）
    if (HAL_UART_Receive(&huart3, rx_buffer, sizeof(CommandPacket), 10) == HAL_OK) {
      memcpy(&cmd, rx_buffer, sizeof(CommandPacket));
      target_rps_1 = cmd.motor1_target_rps;
      target_rps_2 = cmd.motor2_target_rps;
      servo_angle = cmd.servo_angle;
    }

    // 构造遥测数据
    txPacket.roll = roll;
    txPacket.pitch = pitch;
    txPacket.yaw = yaw;
    txPacket.motor1_actual_rps = motor1.rps;
    txPacket.motor2_actual_rps = motor2.rps;

    // 将遥测数据放入队列（非阻塞，如果队列满则丢弃旧数据）
    xQueueSend(telemetryQueue, &txPacket, 0);

    osDelay(1);  // 适当延时
  }
  /* USER CODE END Send_and_Receive */
}

/* USER CODE BEGIN Header_Read_IMU */
/* USER CODE END Header_Read_IMU */
void Read_IMU(void *argument)
{
  /* USER CODE BEGIN Read_IMU */
  /* Infinite loop */
  for(;;)
  {
    QMI8658_GetEulerSimple(&roll, &pitch, &yaw);
    osDelay(10);
  }
  /* USER CODE END Read_IMU */
}

/* USER CODE BEGIN Header_Control */
/**
* @brief Function implementing the myTask03 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Control */
void Control(void *argument)
{
  /* USER CODE BEGIN Control */
  /* Infinite loop */
  for(;;)
  {
    encoder_motor_set_speed(&motor1, target_rps_1);
    encoder_motor_set_speed(&motor2, -target_rps_2);

    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, servo_angle);

    osDelay(1);
  }
  /* USER CODE END Control */
}

/* USER CODE BEGIN Header_UART_Tx_Task */
/**
* @brief Function implementing the myTask04 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_UART_Tx_Task */
void UART_Tx_Task(void *argument)
{
  /* USER CODE BEGIN UART_Tx_Task */
  TelemetryPacket txPacket;
  uint8_t tx_buffer[sizeof(TelemetryPacket)];
  BaseType_t xResult;
  /* Infinite loop */
  for(;;)
  {
    // 等待队列中的遥测数据（阻塞等待）
    xResult = xQueueReceive(telemetryQueue, &txPacket, portMAX_DELAY);
    if (xResult == pdPASS)
    {
      // 将数据复制到静态缓冲区（确保DMA期间不被修改）
      memcpy(tx_buffer, &txPacket, sizeof(TelemetryPacket));

      // 启动DMA发送
      if (HAL_UART_Transmit_DMA(&huart3, tx_buffer, sizeof(TelemetryPacket)) != HAL_OK)
      {
        // 发送失败处理，可重试或丢弃
        continue;
      }

      // 等待发送完成信号量（超时时间可配置）
      if (xSemaphoreTake(uartTxCompleteSem, pdMS_TO_TICKS(100)) != pdTRUE)
      {
        // 超时，可能DMA卡死，可尝试中止发送
        HAL_UART_AbortTransmit(&huart3);
      }
    }
    osDelay(1);
  }
  /* USER CODE END UART_Tx_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART3)
  {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(uartTxCompleteSem, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
}
/* USER CODE END Application */

