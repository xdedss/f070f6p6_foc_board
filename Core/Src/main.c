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
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// modify this ELEC_ANGLE_ZERO_POINT based on the angle of magnetic sensor relative to the motor
#define ELEC_ANGLE_ZERO_POINT 0.565
//#define MAG_MOUNTED_ON_TOP

// flags for debug code
//#define TEST_ELEC_ANGLE_ZERO_POINT
//#define TEST_SENDING_I2C

// velocity smoothing factor (0-1)
// closer to 0 = smoother
#define VEL_LERP 0.1

#define PID_MODE_FORCE 1
#define PID_MODE_VELOCITY 2
#define PID_MODE_POSITION 3

// I2C def:
// byte0 is a flag
// byte1-4 is a float
// flags:
#define I2C_UPDATE_FLAG_FORCE_TARGET 1
#define I2C_UPDATE_FLAG_VELOCITY_TARGET 2
#define I2C_UPDATE_FLAG_POSITION_TARGET 3
#define I2C_UPDATE_FLAG_VELOCITY_KP 4
#define I2C_UPDATE_FLAG_VELOCITY_KI 5
#define I2C_UPDATE_FLAG_POSITION_KP 6
#define I2C_UPDATE_FLAG_POSITION_KI 7
#define I2C_UPDATE_FLAG_POSITION_KD 8

typedef struct {
    float force_target;
    float velocity_target;
    float position_target;

    float velocity_integral;
    float velocity_integral_cap;
    float velocity_kp;
    float velocity_ki;

    float position_integral;
    float position_integral_cap;
    float position_kp;
    float position_ki;
    float position_kd;

    uint8_t pid_mode;

} PIDState;

typedef union {
    float f;
    uint8_t u4[4];
} Converter;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

float mag_r = 0, elecAngle = 0;
float realAngle = 0, realAnglePrev = 0, realVel = 0;
uint32_t t, tPrev, dt;
float dt_f;

// i2c rx data buffer
uint8_t RxData[8] = {0};

PIDState pid_state = {
    .force_target = 0.5,
    .velocity_target = 1.0,
    .position_target = 0.0,

    .velocity_integral = 0.0,
    .velocity_integral_cap = 0.1,
    .velocity_kp = 0.1,
    .velocity_ki = 10,

    .position_integral = 0.0,
    .position_integral_cap = 0.3,
    .position_kp = 2.0,
    .position_ki = 0.2,
    .position_kd = 0.02,

    .pid_mode = PID_MODE_POSITION,
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


extern void HAL_I2C_ListenCpltCallback (I2C_HandleTypeDef *hi2c)
{
    HAL_I2C_EnableListen_IT(hi2c);
}

extern void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    HAL_I2C_EnableListen_IT(hi2c);
}

extern void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode)
{
    if(TransferDirection == I2C_DIRECTION_TRANSMIT)  // if the master wants to transmit the data
    {
        HAL_I2C_Slave_Sequential_Receive_IT(hi2c, RxData, 5, I2C_FIRST_AND_LAST_FRAME);
        // this triggers HAL_I2C_SlaveRxCpltCallback when completed
    }
    else  // master requesting the data is not supported yet
    {
        Error_Handler();
    }
}

extern void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    Converter converter;
    uint8_t update_flag = RxData[0];
    memcpy(&converter, &RxData[1], 4);
    float converted = converter.f;
    switch (update_flag) {
        case I2C_UPDATE_FLAG_FORCE_TARGET:
            pid_state.force_target = converted;
            pid_state.pid_mode = PID_MODE_FORCE;
            break;
        case I2C_UPDATE_FLAG_VELOCITY_TARGET:
            pid_state.velocity_target = converted;
            pid_state.pid_mode = PID_MODE_VELOCITY;
            break;
        case I2C_UPDATE_FLAG_POSITION_TARGET:
            pid_state.position_target = converted;
            pid_state.pid_mode = PID_MODE_POSITION;
            break;
        case I2C_UPDATE_FLAG_VELOCITY_KP:
            pid_state.velocity_kp = converted;
            break;
        case I2C_UPDATE_FLAG_VELOCITY_KI:
            pid_state.velocity_ki = converted;
            break;
        case I2C_UPDATE_FLAG_POSITION_KP:
            pid_state.position_kp = converted;
            break;
        case I2C_UPDATE_FLAG_POSITION_KI:
            pid_state.position_ki = converted;
            break;
        case I2C_UPDATE_FLAG_POSITION_KD:
            pid_state.position_kd = converted;
            break;
    }
}

void sendI2CFloat(uint8_t flag, float f, uint8_t addr) {
    uint8_t buf[8];
    buf[0] = flag;
    memcpy(&buf[1], &f, 4);
    HAL_I2C_Master_Transmit(&hi2c1, addr << 1, buf, 5, 1000);
}


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
    f = clip(f, -1, 1);
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

    tPrev = HAL_GetTick();

    readMagSsi();
    realAngle = mag_r * 2 * PI;

#ifndef TEST_SENDING_I2C
    HAL_I2C_EnableListen_IT(&hi2c1);
#endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
        t = HAL_GetTick();
        dt = t - tPrev;
        if (dt == 0)
            continue;
        tPrev = t;
        dt_f = dt * 0.001;

        readMagSsi();
        // 3205 motor has 7 electrical periods per turn
#ifdef MAG_MOUNTED_ON_TOP
        elecANgle = (mag_r - ELEC_ANGLE_ZERO_POINT) * 7 * PI * 2;
#else
        elecAngle = -(mag_r - ELEC_ANGLE_ZERO_POINT) * 7 * PI * 2;
#endif
        realAngle = -mag_r * 2 * PI;
        realVel = realVel * (1 - VEL_LERP) + VEL_LERP * deltaAngle(realAngle, realAnglePrev) / dt_f;
        realAnglePrev = realAngle;

        // show elecAngle with LED
        stat0(0.5 * sinFast(elecAngle) + 0.5);

#ifdef TEST_ELEC_ANGLE_ZERO_POINT
        UVW_120deg(0, 1);
#else
#ifdef TEST_SENDING_I2C
        HAL_Delay(2000);
        // position -> 1.0
        sendI2CFloat(I2C_UPDATE_FLAG_POSITION_TARGET, 1.0, 7);
        HAL_Delay(2000);
        // position -> 0.0
        sendI2CFloat(I2C_UPDATE_FLAG_POSITION_TARGET, 0.0, 7);
        HAL_Delay(2000);
        // angular velocity -> 0.2
        sendI2CFloat(I2C_UPDATE_FLAG_VELOCITY_TARGET, 0.2, 7);
        HAL_Delay(2000);
        // angular velocity -> -0.2
        sendI2CFloat(I2C_UPDATE_FLAG_VELOCITY_TARGET, -0.2, 7);
        HAL_Delay(2000);
        // force -> 1.0
        sendI2CFloat(I2C_UPDATE_FLAG_FORCE_TARGET, 1.0, 7);
#else
        // control loop
        float diff;
        switch (pid_state.pid_mode)
        {
        case PID_MODE_FORCE:
            UVW_force(pid_state.force_target);
            break;
        case PID_MODE_VELOCITY:
            // vel ---P+I---> force
            diff = pid_state.velocity_target - realVel;
            pid_state.velocity_integral = clip(pid_state.velocity_integral + diff * dt_f, -pid_state.velocity_integral_cap, pid_state.velocity_integral_cap);
            UVW_force(pid_state.velocity_kp * diff + pid_state.velocity_ki * pid_state.velocity_integral);
            break;
        case PID_MODE_POSITION:
            // pos ---P+I+D---> force
            diff = deltaAngle(pid_state.position_target, realAngle);
            pid_state.position_integral = clip(pid_state.position_integral + diff * dt_f, -pid_state.position_integral_cap, pid_state.position_integral_cap);
            UVW_force(pid_state.position_kp * diff + pid_state.position_ki * pid_state.position_integral - pid_state.position_kd * realVel);
            break;
        }
#endif  // TEST_SENDING_I2C
#endif  // TEST_ELEC_ANGLE_ZERO_POINT
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
