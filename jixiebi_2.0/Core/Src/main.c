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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "stdio.h"
#include "string.h"
#include "servo.h"
#include "pca9685.h"
#include "Motor.h"
#include "Encoder.h"
#include "PID_M.h"
#include "MPU6050.h"

// #include "bldc_driver.h"
// #include "as5600.h"
// #include "pid.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#pragma pack(push, 1)
typedef struct {
    uint8_t header1;    // 0xAA
    uint8_t header2;    // 0x55
    uint8_t mode;       // 0, 1, 2
    float x;
    float y;
    float z;
    float wrist;     
    float gripper;//90~180度，大于90度闭合      
    uint8_t checksum;
    uint8_t tail;       // 0xEE
} K230_Packet_t;
#pragma pack(pop)

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

Arm_State ARM = {
    .mode = MODE_AUTO_REACH
};
float ThreeD_x = 100.0f;  
float ThreeD_y = 0.0f;
float ThreeD_z = 100.0f;
              int16_t dx;
              int16_t dy;

//USART1
char rx_buffer[64];  // 接收缓冲区
uint8_t rx_data;     // 存放接收的单个字符
uint8_t rx_index = 0; // 缓冲区索引
uint8_t rx_complete = 0; //接收完成标志位

//USART2
uint8_t rx2_data;              // 单个字节接收缓存
uint8_t rx2_buffer[11]; // 完整数据帧缓存
uint8_t rx2_index = 0;         // 接收索引
uint8_t rx2_state = 0;         // 状态机状态
volatile uint8_t k230_data_ready = 0; // K230数据就绪标志
K230_Packet_t k230_data;

//外部变量
extern IK_Result_t result;
extern float p_log;

////无刷电机
//extern PID_Controller angle_pid;// 1. 实例化并初始化 PID 控制器
//extern BLDC_Driver_t motor_driver;
//extern AS5600_t encoder;
//extern float final_start_angle;

FK_Result_t* FK_result; //正解结果指针

//底盘
Motor_Speed_t speed = {0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// 支持 printf 的重定向代码
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif
PUTCHAR_PROTOTYPE
{
   HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
   return ch;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM6) {	

//	  if(result.ready == 1)
//	  {
//		  servo_xyz();
//		  result.ready = 0;
//	  }
	  speed = Motor_GetPWM(result.j[0]);
	  Motor_SetSpeed_PWM(speed.left_pwm,speed.right_pwm);
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	
    if (huart->Instance == USART1)
    {
//      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); 
      switch(rx_data) {
          case 'a':
              ARM.mode = MODE_AUTO_REACH;
              printf("Switched to MODE_AUTO_REACH\r\n");
              break;
          case 'f':
              ARM.mode = MODE_GRAB_FLAT;
              printf("Switched to MODE_GRAB_FLAT\r\n");
              break;
          case 'd':
              ARM.mode = MODE_GRAB_DOWN;
              printf("Switched to MODE_GRAB_DOWN\r\n");
              break; 
          default:
                if (rx_data == '\n' || rx_data == '\r' || rx_data == ';') 
                {
                    rx_buffer[rx_index] = '\0';// 加上字符串结束符
                    rx_complete = 1;         
                    rx_index = 0;
                }
                else
                {
                    if (rx_index < 63)
                    {
                        rx_buffer[rx_index++] = rx_data;
                    }
                }
              break;
      }
      HAL_UART_Receive_IT(&huart1, &rx_data, 1);
    }

    if (huart->Instance == USART2)
    {
        // switch (rx2_state)
        // {
        //     case 0: // 找帧头 1
        //         if (rx2_data == 0xAA) 
        //         { 
        //           rx2_buffer[0] = rx2_data; 
        //           rx2_state = 1;
        //         }
        //         break;
        //     case 1: // 找帧头 2
        //         if (rx2_data == 0x55) 
        //         {
        //           rx2_buffer[1] = rx2_data;
        //           rx2_index = 2; 
        //           rx2_state = 2; 
        //         }
        //         else rx2_state = 0;
        //         break;
        //     case 2: // 接收数据体
        //         rx2_buffer[rx2_index++] = rx2_data;
        //         if (rx2_index >= sizeof(K230_Packet_t)) rx2_state = 3;
        //         break;
        // }

        // if (rx2_state == 3)
        // {
        //     if (rx2_buffer[sizeof(K230_Packet_t) - 1] == 0xEE)
        //     {
        //         uint8_t cal_checksum = 0;
        //         for (int i = 2; i < 23; i++) cal_checksum += rx2_buffer[i]; // 算校验和
        //         if (cal_checksum == rx2_buffer[15])
        //         {
        //             memcpy(&k230_data, rx2_buffer, sizeof(K230_Packet_t));
        //             k230_data_ready = 1; 
        //         }
        //     }
        //     rx2_state = 0;
        //     rx2_index = 0;
        // }

		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        switch (rx2_state)
        {
            case 0:
                if (rx2_data == 0xAA) 
                { 
                  rx2_buffer[0] = rx2_data; 
                  rx2_state = 1;
                }
                break;
            case 1:
                if (rx2_data == 0x55) 
                {
                  rx2_buffer[1] = rx2_data;
                  rx2_index = 2; 
                  rx2_state = 2; 
                }
                else rx2_state = 0;
                break;
            case 2:
                rx2_buffer[rx2_index++] = rx2_data;
                if (rx2_index >= 11) 
                {
                    rx2_state = 3;
                }
                break;
        }
        k230_data_ready = 1;
        HAL_UART_Receive_IT(&huart2, &rx2_data, 1);
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	
  // --- 新增：极速拉低 PA4 和 PA6 ---
  __HAL_RCC_GPIOA_CLK_ENABLE();
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN; // 软件强制下拉
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4 | GPIO_PIN_6, GPIO_PIN_RESET);

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
  MX_TIM2_Init();
  MX_I2C2_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  

//  BLDC_Init(&motor_driver, &htim2, 12.0f);
//  AS5600_Init(&encoder, &hi2c2);
//  BLDC_Enable();
//  PID_Init(&angle_pid, 1.5f, 0.2f, 1.0f, 3.0f);
//  align_sensor(final_start_angle);
  ARM.mode = MODE_GRAB_FLAT;
  
  MPU_Init();
  MPU_Calibrate_Gyro_Z(300); // 校准
  HAL_Delay(2000);
//  Encoder_Init();
	PCA9685_Init(50.0f);
  servo_init();
  Motor_Init_PWM();
  result.j[0] = 0.0f;
  
  HAL_Delay(200);
  HAL_TIM_Base_Start_IT(&htim6);
  HAL_UART_Receive_IT(&huart1, &rx_data, 1);
  HAL_UART_Receive_IT(&huart2, &rx_data, 1);
  printf("Ready!\r\n");
  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);
//  Motor_SetSpeed_PWM(0,500);
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
//	  HAL_Delay(200);
    if (rx2_state == 3)
    {
        if (rx2_buffer[11 - 1] == 0xBB)
        {
            uint8_t cal_checksum = 0;
            for (int i = 2; i < 9; i++) cal_checksum += rx2_buffer[i];
            if (cal_checksum == rx2_buffer[9])
            {
            dx = (int16_t)((rx2_buffer[4] << 8) | rx2_buffer[5]);
            dy = (int16_t)((rx2_buffer[6] << 8) | rx2_buffer[7]);
              uint8_t distance = (uint8_t)rx2_buffer[8] ;

              if (dx == 9999 || dy == 9999) 
              {
                  // 丢失目标
              }
              else
              {
                  float Kp = 0.0005f; 
                  ThreeD_y += (dx * Kp);
                  if(dx*Kp < 1.0f) 
                  {
                    ThreeD_z += (dy * Kp);
                  }
                  if(dx*Kp < 1.0f && dy*Kp < 1.0f)
                  {
                    ThreeD_z += distance * sinf(p_log * (M_PI / 180.0f)); 
                    ThreeD_x += distance * cosf(p_log * (M_PI / 180.0f));
                  }
				  
				if(ThreeD_x > 169.0f) ThreeD_x = 169.0f;
				if(ThreeD_x < -169.0f) ThreeD_x = -169.0f;
				if(ThreeD_y > 169.0f) ThreeD_y = 169.0f;
				if(ThreeD_y < -169.0f) ThreeD_y = -169.0f;
				if(ThreeD_z > 225.0f) ThreeD_z = 225.0f;
				if(ThreeD_z < -50.0f)  ThreeD_z = -50.0f;

                  result = IK_Get_Target_Angle(ThreeD_x, ThreeD_y, ThreeD_z, ARM.mode);
//                  result.j[4] = k230_data.wrist;
//                  result.j[5] = k230_data.gripper;
				  servo_xyz();
                  result.ready = 1;
              }
            }
        }
        rx2_state = 0;
        rx2_index = 0;
     }
//	servo_xyz();


     if (rx_complete)
     {
       rx_complete = 0;
       float target_x, target_y, target_z;
       if (sscanf(rx_buffer, "x%f y%f z%f", &target_x, &target_y, &target_z) == 3)
       {
         printf("Processing: [%s]\r\n", rx_buffer);
         printf("Target Received -> X:%.1f Y:%.1f Z:%.1f\r\n", target_x, target_y, target_z);
         IK_Result_t temporary = IK_Get_Target_Angle(target_x, target_y, target_z, ARM.mode);
         for(int i = 0; i < 6; i++) {
         result.j[i] = temporary.j[i];
       }
       result.ready = 1;
       printf("=== Servo Angles ===\r\nJ0: %.2f | J1: %.2f | J2: %.2f\r\nJ3: %.2f | J4: %.2f | J5: %.2f | P: %.2f \r\n--------------------\r\n", result.j[0], result.j[1], result.j[2], result.j[3], result.j[4], result.j[5], p_log);
       FK_result = FK_Solve_Core(result.j[0], result.j[1], result.j[2], result.j[3]);
       printf("%.2f|%.2f\n%.2f|%.2f\n%.2f|%.2f\n%.2f|%.2f\n--------------------\n", FK_result[0].x, FK_result[0].z, FK_result[1].x, FK_result[1].z, FK_result[2].x, FK_result[2].z, FK_result[3].x, FK_result[3].z);
 	  }
       else
       {
         printf("Error, Please use: x100 y50 z20\r\n");
       }
       // memset(rx_buffer, 0, sizeof(rx_buffer));
      
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
