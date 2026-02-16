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
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include "mpu6050.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// 机械臂坐标范围限制 (单位: mm)
#define X_MAX 179
#define X_MIN -159  // 机械臂不能缩到底座里面
#define Y_MAX 169
#define Y_MIN -169
#define Z_MAX 225
#define Z_MIN 0

// 这是一个死区，手如果不怎么动（倾斜小于10度），坐标就不动，防止抖动
#define DEAD_ZONE 10.0f 
// 移动速度系数
#define SPEED_FACTOR 1.0f 

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

volatile uint8_t mpu_interrupt_flag = 0;
uint32_t last_send_time = 0;

// 当前的目标坐标 (全局变量)
float Target_X = 10.0f; // 初始位置 X
float Target_Y = 0.0f;   // 初始位置 Y
float Target_Z = 225.0f; // 初始位置 Z

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int fputc(int ch, FILE *f)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_5) // 你的 PB5 引脚
    {
        mpu_interrupt_flag = 1; // 只要置位就行，不要在这里做读取操作！
    }
}

void Calculate_Coordinate_From_Attitude(float pitch, float roll)
{
    // --- X轴控制 (基于 Pitch 俯仰) ---
    // 假设：手向前倾 (Pitch > 10) -> X增加
    //       手向后倾 (Pitch < -10) -> X减小
    if (pitch > DEAD_ZONE) {
        Target_X += (pitch - DEAD_ZONE) * 0.05f * SPEED_FACTOR;
    } 
    else if (pitch < -DEAD_ZONE) {
        Target_X += (pitch + DEAD_ZONE) * 0.05f * SPEED_FACTOR;
    }

    // --- Y轴控制 (基于 Roll 横滚) ---
    // 假设：手向右翻 (Roll > 10) -> Y增加
    //       手向左翻 (Roll < -10) -> Y减小
    if (roll > DEAD_ZONE) {
        Target_Y += (roll - DEAD_ZONE) * 0.05f * SPEED_FACTOR;
    } 
    else if (roll < -DEAD_ZONE) {
        Target_Y += (roll + DEAD_ZONE) * 0.05f * SPEED_FACTOR;
    }

    // --- 范围限制 (防止算出非法坐标) ---
    if (Target_X > X_MAX) Target_X = X_MAX;
    if (Target_X < X_MIN) Target_X = X_MIN;
    
    if (Target_Y > Y_MAX) Target_Y = Y_MAX;
    if (Target_Y < Y_MIN) Target_Y = Y_MIN;
    
    // Target_Z 暂时固定，或者你可以用 Yaw 来控制 Z，或者加按键
}

//// 发送坐标包给机械臂
//void Send_Coordinate_Packet(float x, float y, float z)
//{
//    uint8_t data_buffer[11];
//    
//    // 坐标转整数 (mm)
//    int16_t x_int = (int16_t)x;
//    int16_t y_int = (int16_t)y;
//    int16_t z_int = (int16_t)z;
//    
//    data_buffer[0] = 0xA5; // 帧头
//    data_buffer[1] = 0x5A; 
//    data_buffer[2] = 0x02; // 类型0x02代表坐标包 (与角度包区分)
//    
//    // X
//    data_buffer[3] = x_int >> 8;
//    data_buffer[4] = x_int & 0xFF;
//    // Y
//    data_buffer[5] = y_int >> 8;
//    data_buffer[6] = y_int & 0xFF;
//    // Z
//    data_buffer[7] = z_int >> 8;
//    data_buffer[8] = z_int & 0xFF;
//    
//    // 校验和
//    uint8_t check_sum = 0;
//    for(int i=0; i<9; i++) check_sum += data_buffer[i];
//    data_buffer[9] = check_sum;
//    
//    data_buffer[10] = 0xAA; // 帧尾
//    
//    HAL_UART_Transmit(&huart1, data_buffer, 11, 0xFFFF);
//}

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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  
    // 1. 初始化 MPU6050 (基础寄存器)
  printf("MPU6050 Init...\r\n"); // 打印提示
  MPU6050_initialize();
  
  // 2. 初始化 DMP (加载固件，比较慢，约 2秒)
  printf("DMP Init (Wait ~2s)...\r\n"); 
  DMP_Init(); 
  HAL_Delay(2000);
  printf("DMP Ready!\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
     if (MPU_Check_And_Read()) 
    {
        // 1. 根据当前姿态，计算下一步的坐标
        Calculate_Coordinate_From_Attitude(Pitch, Roll);
     }   
        // 2. 打印调试 (看看坐标变没变)
//        printf("X:%.1f, Y:%.1f, P:%.1f, R:%.1f\r\n", Target_X, Target_Y, Pitch, Roll);
        
        // 3. 发送给机械臂 (机械臂收到这个XYZ后，直接跑逆运动学去抓取)
//        Send_Coordinate_Packet(Target_X, Target_Y, Target_Z);
			
     if (HAL_GetTick() - last_send_time > 600) 
    {
        // 更新时间戳
        last_send_time = HAL_GetTick();
        
        // 发送当前最新的坐标
        // 这里的 Target_X 已经是经过几十次微小累加后的最新结果了
		printf("x%.1f y%.1f z%.1f;", Target_X, Target_Y, Target_Z);
        
        // 可选：翻转一个 LED 提示发送状态
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); 
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
