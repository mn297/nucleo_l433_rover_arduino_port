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
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "dma.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "RoverArmMotor.h"
#include "AMT22.h"
// Standard includes
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <bitset>
#include <limits>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TEST_WRIST_ROLL_CYTRON 0
#define TEST_WRIST_PITCH_CYTRON 1
#define TEST_WAIST_SERVO 0
#define TEST_ELBOW_SERVO 0

#define MIN_FLOAT -std::numeric_limits<float>::infinity()
#define MAX_FLOAT std::numeric_limits<float>::infinity()
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// double aggKp=0.025, aggKi=0.019,  aggKd=0.0, elbaggKp=0.025, elbaggKi=0.019,  elbaggKd=0;
// double regKp=0.025, regKi=0.014, regKd=0, elbregKp=0.025, elbregKi=0.014,  elbregKd=0;
// double regKp=1, regKi=0.01, regKd=0.0, elbregKp=0.025, elbregKi=0,  elbregKd=0;  // PI for CYTRON

// WRIST_CYTRON
// double regKp_wrist = 0.4, regKi_wrist = 0.0, regKd_wrist = 0.0;

// WRIST_ROLL_TESTBENCH (DC)
double regKp_wrist_roll = 0.6, regKi_wrist_roll = 0.2, regKd_wrist_roll = 0.1;

// WRIST_PITCH_CYTRON
double regKp_wrist_pitch = 0.5, regKi_wrist_pitch = 0.2, regKd_wrist_pitch = 0.2;

// WAIST_SERVO
double regKp_waist = 1.8, regKi_waist = 1.0, regKd_waist = 0.5; // Vincent's configuration

// ELBOW (SERVO)
double regKp_elbow = 0.6, regKi_elbow = 0.2, regKd_elbow = 0.1; // Vincent's configuration
// double regKp_waist = 1.2, regKi_waist = 0.4, regKd_waist = 0.4; // Sam's configuration
// double regKp_waist = 2, regKi_waist = 0.4, regKd_waist = 0.4;  // Speed configuration

Pin dummy_pin;
double setpoint = 0;
int turn = 0;
int limit_set = 0;
int button_counter = 0;
int is_turning = 0;

/*---------------------UART---------------------*/
const int RX_BUFFER_SIZE = 32;
uint8_t rx_data[8]; // 1 byte
char rx_buffer[RX_BUFFER_SIZE];
uint32_t rx_index = 0;
char command_buffer[20];
double param1, param2, param3;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void print(const char *s)
{
  //	#ifdef PRINT
  HAL_StatusTypeDef code = HAL_UART_Transmit(&huart2, (uint8_t *)s, strlen(s), HAL_MAX_DELAY);
  //	#endif
}

int printf(const char *s, ...)
{
  char buffer[256];
  //	#ifdef PRINT
  va_list args;
  va_start(args, s);
  vsprintf(buffer, s, args);
  perror(buffer);
  print(buffer);
  va_end(args);
  //	#endif
  return strlen(buffer);
}

void delay_us(uint16_t us)
{
  __HAL_TIM_SET_COUNTER(&htim1, 0); // set the counter value a 0
  while (__HAL_TIM_GET_COUNTER(&htim1) < us)
    ; // wait for the counter to reach the us input in the parameter
}

static void print_MOTOR(char *msg, RoverArmMotor *pMotor)
{
  double current_angle_multi;
  double current_angle_sw;
  pMotor->get_current_angle_sw(&current_angle_sw);
  printf("%s setpoint %.2f, angle_sw %.2f, output %.2f\r\n",
         msg,
         pMotor->setpoint / pMotor->gearRatio,
         current_angle_sw / pMotor->gearRatio,
         pMotor->output);
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

Pin CYTRON_DIR_1(CYTRON_DIR_1_GPIO_Port, CYTRON_DIR_1_Pin);
Pin CYTRON_PWM_1(CYTRON_PWM_1_GPIO_Port, CYTRON_PWM_1_Pin, &htim2, TIM_CHANNEL_2);
Pin AMT22_1(GPIOC, GPIO_PIN_7);

/*---------------------WRIST_ROLL_CYTRON---------------------*/
#if TEST_WRIST_ROLL_CYTRON == 1
RoverArmMotor Wrist_Roll(&hspi1, CYTRON_PWM_1, CYTRON_DIR_1, AMT22_1, CYTRON, 0, 359.99f);
#endif

/*---------------------WRIST_PITCH_CYTRON---------------------*/
#if TEST_WRIST_PITCH_CYTRON == 1
RoverArmMotor Wrist_Pitch(&hspi1, CYTRON_PWM_1, CYTRON_DIR_1, AMT22_1, CYTRON, 0, 359.99f);
#endif

/*---------------------WAIST_SERVO DECLARATIONS---------------------*/
#if TEST_WAIST_SERVO == 1
Pin SERVO_PWM_1(CYTRON_PWM_1_GPIO_Port, CYTRON_PWM_1_Pin, &htim2, TIM_CHANNEL_2);
RoverArmMotor Waist(&hspi1, SERVO_PWM_1, dummy_pin, AMT22_1, BLUE_ROBOTICS, 0, 359.99f);
#endif

/*---------------------ELBOW_SERVO DECLARATIONS---------------------*/
#if TEST_ELBOW_SERVO == 1
Pin SERVO_PWM_2(CYTRON_PWM_1_GPIO_Port, CYTRON_PWM_1_Pin, &htim2, TIM_CHANNEL_2);
RoverArmMotor Elbow(&hspi1, SERVO_PWM_2, dummy_pin, AMT22_1, BLUE_ROBOTICS, 0, 359.99f);
#endif
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  //  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  uint16_t encoderData_1 = 99;
  uint16_t encoderData_2 = 99;
  uint16_t encoderData_3 = 99;
  uint16_t encoder_max = 0;
  uint16_t encoder_min = 4100;
  HAL_TIM_Base_Start(&htim1);

  /*---WAIST_SERVO setup---*/
#if TEST_WAIST_SERVO == 1
  Waist.wrist_waist = 1;
  Waist.setAngleLimits(MIN_FLOAT, MAX_FLOAT);
  Waist.reset_encoder();
  Waist.begin(regKp_waist, regKi_waist, regKd_waist);
#endif

  /* ELBOW_SERVO setup */
#if TEST_ELBOW_SERVO == 1
  Elbow.setAngleLimits(0, 120);
  Elbow.reset_encoder();
  Elbow.begin(regKp_elbow, regKi_elbow, regKd_elbow);
#endif

  /*---WRIST_ROLL_CYTRON setup---*/
#if TEST_WRIST_ROLL_CYTRON == 1
  int32_t CH2_DC = 0;
  Wrist_Roll.wrist_waist = 1;
  Wrist_Roll.setGearRatio(2.672222f);
  Wrist_Roll.setGearRatio(1);
  Wrist_Roll.setAngleLimits(MIN_FLOAT, MAX_FLOAT);
  Wrist_Roll.reset_encoder();
  Wrist_Roll.begin(regKp_wrist_roll, regKi_wrist_roll, regKd_wrist_roll);
#endif

/*---WRIST_PITCH_CYTRON setup---*/
#if TEST_WRIST_PITCH_CYTRON == 1
  Wrist_Pitch.wrist_waist = 0;
  Wrist_Pitch.setAngleLimits(-10, 120);
  Wrist_Pitch.reset_encoder();
  Wrist_Pitch.begin(regKp_wrist_pitch, regKi_wrist_pitch, regKd_wrist_pitch);
#endif

  while (!limit_set)
    ;

  HAL_TIM_Base_Start_IT(&htim6);

  /*---UART setup---*/
  HAL_UART_Receive_IT(&huart2, rx_data, 1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
#if TEST_WAIST_SERVO == 1
     print_MOTOR("SP Waist", &Waist);
     printf("forwardDistance: %f, backwardDistance: %f\r\n", Waist.forwardDistance, Waist.backwardDistance);
#endif

#if TEST_ELBOW_SERVO == 1
    print_MOTOR("SP Elbow", &Elbow);
    printf("forwardDistance: %f, backwardDistance: %f\r\n", Elbow.forwardDistance, Elbow.backwardDistance);
#endif

#if TEST_WRIST_ROLL_CYTRON == 1
    print_MOTOR("SP WRIST_ROLL_CYTRON", &Wrist_Roll);
    printf("forwardDistance: %f, backwardDistance: %f\r\n", Wrist_Roll.forwardDistance, Wrist_Roll.backwardDistance);
#endif

#if TEST_WRIST_PITCH_CYTRON == 1
    print_MOTOR("SP WRIST_PITCH_CYTRON", &Wrist_Pitch);
    printf("forwardDistance: %f, backwardDistance: %f\r\n", Wrist_Pitch.forwardDistance, Wrist_Pitch.backwardDistance);
#endif

    HAL_Delay(100);
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
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
// External Interrupt ISR Handler CallBackFun
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  // if(!limit_set) {
  if (GPIO_Pin == B1_Pin && HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET) // INT Source is pin A9
  {
    is_turning = !is_turning;
#if TEST_WRIST_ROLL_CYTRON == 1
    Wrist_Roll.stop();
    Wrist_Roll.set_zero_angle_sw();
    Wrist_Roll.newSetpoint(0.0);
#endif

#if TEST_WRIST_PITCH_CYTRON == 1
    Wrist_Pitch.stop();
    Wrist_Pitch.set_zero_angle_sw();
    Wrist_Pitch.newSetpoint(0.0);
#endif

#if TEST_WAIST_SERVO == 1
    Waist.stop();
    Waist.set_zero_angle_sw();
    Waist.newSetpoint(0.0);
#endif

#if TEST_ELBOW_SERVO == 1
    Elbow.stop();
    Elbow.set_zero_angle_sw();
    Elbow.newSetpoint(0.0);
#endif

    limit_set = 1;
  }

#if TEST_WRIST_PITCH_CYTRON == 1
   if (GPIO_Pin == LIMIT_WRIST_PITCH_MIN_Pin && HAL_GPIO_ReadPin(LIMIT_WRIST_PITCH_MIN_GPIO_Port, LIMIT_WRIST_PITCH_MIN_Pin) == GPIO_PIN_RESET)
   {
     Wrist_Pitch.stop();
     Wrist_Pitch.set_zero_angle_sw();
   }
  
   if (GPIO_Pin == LIMIT_WRIST_PITCH_MAX_Pin && HAL_GPIO_ReadPin(LIMIT_WRIST_PITCH_MAX_GPIO_Port, LIMIT_WRIST_PITCH_MAX_Pin) == GPIO_PIN_RESET)
   {
     Wrist_Pitch.stop();
     Wrist_Pitch.set_max_angle_sw();
   }
#endif

  button_counter++;
  return;
  // }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (rx_index < RX_BUFFER_SIZE - 1) // check if buffer is not full
  {
    rx_buffer[rx_index++] = (uint8_t)rx_data[0];  // add received byte to buffer
    if (rx_data[0] == '\n' || rx_data[0] == '\r') // check for Enter key
    {
      rx_buffer[rx_index] = '\0'; // add null terminator to make it a string
      rx_index = 0;               // reset buffer index

      //---------------------UART COMMANDS---------------------//
      // do something with the received data
      // sscanf(rx_buffer, "%s %lf %lf %lf", command_buffer, &param1, &param2, &param3);
      // // check if commmand_buffer is "pid"
      // if (strcmp(command_buffer, "pid") == 0)
      // {
      //   Wrist_Roll.set_PID_params(param1, param2, param3, param1, param2, param3);
      //   printf("set to Kp: %lf, Ki: %lf, Kd: %lf\r\n", param1, param2, param3);
      // }
      // else if (strcmp(command_buffer, "sp") == 0)
      // {
      //   Wrist_Roll.newSetpoint(param1);
      //   Waist.newSetpoint(param1); // TODO check this?
      //   printf("new Setpoint at %lf\r\n", param1);
      // }
      // else if (strcmp(command_buffer, "s") == 0)
      // {
      //   __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (int)param1); // set servo output
      // }
      //   else
      // {
      //   printf("invalid command %s\r\n", command_buffer);
      // }

      //---------------------FLOAT---------------------//
      sscanf(rx_buffer, "%lf", &param1);

#if TEST_WRIST_ROLL_CYTRON == 1
      bool is_sp_valid = Wrist_Roll.newSetpoint(param1);
#endif
#if TEST_WRIST_PITCH_CYTRON == 1
      bool is_sp_valid = Wrist_Pitch.newSetpoint(param1);
#endif
#if TEST_WAIST_SERVO == 1
      bool is_sp_valid = Waist.newSetpoint(param1);
#endif
#if TEST_ELBOW_SERVO == 1
      bool is_sp_valid = Elbow.newSetpoint(param1);
#endif

      if (is_sp_valid)
        printf("new Setpoint at %lf\r\n", param1);
      else
        printf("invalid Setpoint %lf\r\n", param1);
      // __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (int)param1); // set servo output
    }
  }
  else if (rx_index == RX_BUFFER_SIZE - 1) // buffer is full
  {
    rx_buffer[rx_index] = '\0'; // add null terminator to make it a string
    rx_index = 0;               // reset buffer index
    // do something with the received data
    sscanf(rx_buffer, "%s %lf %lf %lf", command_buffer, &param1, &param2, &param3);
    printf("set to Kp: %lf, Ki: %lf, Kd: %lf\r\n", param1, param2, param3);
  }
  // }
  HAL_UART_Receive_IT(&huart2, rx_data, 1); // start listening for next byte
}
// Callback: timer has rolled over
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  // Check which version of the timer triggered this callback and toggle LED
  if (htim == &htim6)
  {
#if TEST_WRIST_ROLL_CYTRON == 1
    Wrist_Roll.tick();
#endif
#if TEST_WRIST_PITCH_CYTRON == 1
    Wrist_Pitch.tick();
#endif
#if TEST_WAIST_SERVO == 1
    Waist.tick();
#endif
#if TEST_ELBOW_SERVO == 1
    Elbow.tick();
#endif
  }
}
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
