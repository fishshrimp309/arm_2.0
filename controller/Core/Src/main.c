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
#define DEAD_ZONE 15.0f 
// 移动速度系数
#define SPEED_FACTOR 0.005f 

#define Z_MODE_THRESHOLD 50.0f
#define SIDE_MODE_THRESHOLD 50.0f

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
float Last_Sent_X;
float Last_Sent_Y;
float Last_Sent_Z;

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

float my_abs(float v) { return v > 0 ? v : -v; }

//void Calculate_Coordinate_From_Attitude(float pitch, float roll)
//{
//    float move_step = 0;

//    // --- 判断模式 ---
//    // 如果 Roll (横滚) 角度很大 (>50度)，说明手侧立起来了 -> 进入 Z轴模式
//    if (my_abs(roll) > Z_MODE_THRESHOLD) 
//    {
//        // === Z 轴控制模式 ===
//        // 这时候，利用 Pitch (前后倾斜) 来控制 Z 轴升降
//        if (my_abs(pitch) > DEAD_ZONE) 
//        {
//            move_step = (my_abs(pitch) - DEAD_ZONE) * SPEED_FACTOR;
//            if (pitch > 0) Target_Z -= move_step; // 低头 -> 下降
//            else           Target_Z += move_step; // 抬头 -> 上升
//        }
//    }
//    else 
//    {
//        // === XY 轴控制模式 (普通平放) ===
//        
//        // X轴 (Pitch控制)
//        if (my_abs(pitch) > DEAD_ZONE) 
//        {
//            // 现在的速度是动态的：倾斜角度越大，跑得越快
//            move_step = (my_abs(pitch) - DEAD_ZONE) * SPEED_FACTOR;
//            
//            if (pitch > 0) Target_X += move_step; // 低头 -> 前伸
//            else           Target_X -= move_step; // 抬头 -> 后缩
//        }

//        // Y轴 (Roll控制)
//        // 注意：这里 Roll 小于 50 度才会进来，所以要在 15~50 度之间控制 Y
//        if (my_abs(roll) > DEAD_ZONE) 
//        {
//            move_step = (my_abs(roll) - DEAD_ZONE) * SPEED_FACTOR;
//            
//            if (roll > 0) Target_Y += move_step; // 右翻 -> 右移
//            else          Target_Y -= move_step; // 左翻 -> 左移
//        }
//    }

//    // --- 范围限位 (防止超程) ---
//    if (Target_X > X_MAX) Target_X = X_MAX;
//    if (Target_X < X_MIN) Target_X = X_MIN;
//    
//    if (Target_Y > Y_MAX) Target_Y = Y_MAX;
//    if (Target_Y < Y_MIN) Target_Y = Y_MIN;
//    
//    if (Target_Z > Z_MAX) Target_Z = Z_MAX;
//    if (Target_Z < Z_MIN) Target_Z = Z_MIN;
//}
void Calculate_Coordinate_From_Attitude(float pitch, float roll, float yaw)
{
    float move_step = 0;
    
    // --- 侧立模式判断 ---
    // 初始状态 Roll 接近 90 或 -90。
    // 如果 Roll 回正到了水平面（比如 < 40度），我们进入 XY 平面模式
    // 否则（保持侧立），使用 Pitch 控制 X，用 Roll 的偏离量控制 Z
    
    if (my_abs(roll) > SIDE_MODE_THRESHOLD) // 保持侧立状态 (例如 THRESHOLD = 60)
    {
        // === 1. X 轴控制 (Pitch 前后倾斜) ===
        if (my_abs(pitch) > DEAD_ZONE) 
        {
            move_step = (my_abs(pitch) - DEAD_ZONE) * SPEED_FACTOR;
            // 侧立时，抬头/低头依然对应前后位移
            if (pitch > 0) Target_X += move_step; 
            else           Target_X -= move_step;
        }

        // === 2. Z 轴控制 (Roll 进一步侧倾或回正) ===
        // 假设 90 度是标准垂直，Roll 往 100 度走是升，往 80 度走是降
        float roll_offset = my_abs(roll) - 90.0f; 
        
        if (my_abs(roll_offset) > DEAD_ZONE)
        {
            move_step = (my_abs(roll_offset) - DEAD_ZONE) * SPEED_FACTOR;
            // 如果 Roll 绝对值变大（更斜了）则上升，变小（向水平靠拢）则下降
            if (roll_offset > 0) Target_Z -= move_step; 
            else                Target_Z += move_step;
        }
    }
    else 
    {
//        // === 3. Y 轴控制模式 (当手放平以后) ===
//        // 当 Roll 角度较小时，切换为控制 Y 轴（左右平移）
//        if (my_abs(yaw) > DEAD_ZONE)
//        {
//            move_step = (my_abs(roll) - DEAD_ZONE) * SPEED_FACTOR;
//            if (yaw > 0) Target_Y += move_step;
//            else          Target_Y -= move_step;
//        }
        
        // 此模式下 Pitch 依然可以控制 X
        if (my_abs(pitch) > DEAD_ZONE) 
        {
            move_step = (my_abs(pitch) - DEAD_ZONE) * SPEED_FACTOR;
            if (pitch > 0) Target_Y += move_step;
            else           Target_Y -= move_step;
        }
    }

    // --- 范围限位 (保持不变) ---
    if (Target_X > X_MAX) Target_X = X_MAX;
    if (Target_X < X_MIN) Target_X = X_MIN;
    if (Target_Y > Y_MAX) Target_Y = Y_MAX;
    if (Target_Y < Y_MIN) Target_Y = Y_MIN;
    if (Target_Z > Z_MAX) Target_Z = Z_MAX;
    if (Target_Z < Z_MIN) Target_Z = Z_MIN;
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
	      // 1. 高频计算 (保证手感丝滑)
    if (MPU_Check_And_Read()) 
    {
        Calculate_Coordinate_From_Attitude(Pitch, Roll, Yaw);
    }

    // 2. 低频发送 (解耦控制)
    // 只有当时间间隔 > 300ms 时才尝试发送
    if (HAL_GetTick() - last_send_time > 300)
    {
        // 3. 变化检测 (解决“一直发送”的问题)
        // 只有当坐标变化超过 2mm 时，才真正发给机械臂
        if (my_abs(Target_X - Last_Sent_X) > 2.0f || 
            my_abs(Target_Y - Last_Sent_Y) > 2.0f || 
            my_abs(Target_Z - Last_Sent_Z) > 2.0f)
        {
            // 更新发送时间
            last_send_time = HAL_GetTick();
            
            // 更新“上次坐标”
            Last_Sent_X = Target_X;
            Last_Sent_Y = Target_Y;
            Last_Sent_Z = Target_Z;
			
			printf("x%.1f y%.1f z%.1f;", Target_X, Target_Y, Target_Z);
        
			// 可选：翻转一个 LED 提示发送状态
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
            
            // 调试打印 (可选)
            // printf("Sent: X%.0f Y%.0f Z%.0f\r\n", Target_X, Target_Y, Target_Z);
        }
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
