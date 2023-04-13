/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "utils.h"
/* USER CODE END Includes */

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

/* USER CODE BEGIN PV */

float mag_r = 0, elecAngle = 0;
uint32_t t, tPrev, dt;
float dt_f;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void UVW_WritePinPwm(float u, float v, float w)
{
    // full, set pwm polarity
    // float: -1 to +1
    // wiring:
    // PB1(t3c4) - UH
    // PA7(t3c2) - VH
    // PA6(t3c1) - WH
    int PWM_MAX = 128-1;
    int u_i = (int)((u * 0.5f + 0.5f) * PWM_MAX + 0.5f);
    int v_i = (int)((v * 0.5f + 0.5f) * PWM_MAX + 0.5f);
    int w_i = (int)((w * 0.5f + 0.5f) * PWM_MAX + 0.5f);
    htim3.Instance->CCR1 = w_i;
    htim3.Instance->CCR2 = v_i;
    htim3.Instance->CCR4 = u_i;
}

void UVW_120deg(float theta, float strength)
{
    // theta: -pi to +pi
    // strength: 0 to 1
    UVW_WritePinPwm(sinFast(theta) * strength, sinFast(theta + PI / 3 * 2) * strength, sinFast(theta + PI / 3 * 4) * strength);
}

void UVW_force(float f) {
    // -1 ~ 1
    // clockwise = positive
    UVW_120deg(elecAngle + PI * 0.5f, f);
}

void stat0(float brightness) {
    htim1.Instance->CCR3 = (uint32_t)(clip01(brightness) * 255 + 0.5f); // stat0 outer
}

void stat1(float brightness) {
    htim1.Instance->CCR2 = (uint32_t)(clip01(brightness) * 255 + 0.5f); // stat1 inner
}

void smallDelay()
{
//    for (int i = 0; i < 10; ++i);
    __NOP();
}

void readMagSsi()
{
    unsigned int mag_d;
//    unsigned char mag_mg;
//    unsigned char mag_crc;
    mag_d = 0;
//    mag_mg = 0;
//    mag_crc = 0;
    unsigned char d;
    GPIOA->BRR = MAG_CSN_Pin;
    smallDelay();
    for (int i = 0; i < 14; i++)
    {
        GPIOA->BRR = MAG_CLK_Pin;
        smallDelay();
        GPIOA->BSRR = MAG_CLK_Pin;
        smallDelay();
        d = (GPIOA->IDR & MAG_DO_Pin) != 0;
        mag_d += d << (13 - i);
    }
    for (int i = 0; i < 4; i++)
    {
        GPIOA->BRR = MAG_CLK_Pin;
        smallDelay();
        GPIOA->BSRR = MAG_CLK_Pin;
        smallDelay();
        d = (GPIOA->IDR & MAG_DO_Pin) != 0;
//        mag_mg += d << (3 - i);
    }
    for (int i = 0; i < 6; i++)
    {
        GPIOA->BRR = MAG_CLK_Pin;
        smallDelay();
        GPIOA->BSRR = MAG_CLK_Pin;
        smallDelay();
        d = (GPIOA->IDR & MAG_DO_Pin) != 0;
//        mag_crc += d << (5 - i);
    }
    mag_r = mag_d / 16384.f;
//    smallDelay();
    GPIOA->BSRR = MAG_CSN_Pin;
//    elecAngle = convertAngleLUT(mag_r);
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
  MX_TIM3_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  // PWM for motor
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
  // PWM for led
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  stat1(0.3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      readMagSsi();
      elecAngle = -(mag_r - 0.60) * 7 * PI * 2;
      stat0(0.5 * sinFast(elecAngle) + 0.5);
      UVW_force(0.2);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

#ifdef  USE_FULL_ASSERT
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
