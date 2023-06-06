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
// double aggKp=0.025, aggKi=0.019,  aggKd=0.0, elbaggKp=0.025, elbaggKi=0.019,  elbaggKd=0;
// double regKp=0.025, regKi=0.014, regKd=0, elbregKp=0.025, elbregKi=0.014,  elbregKd=0;
// double regKp=1, regKi=0.01, regKd=0.0, elbregKp=0.025, elbregKi=0,  elbregKd=0;  // PI for CYTRON

// WRIST (DC)
//  double aggKp = 0.6, aggKi = 0.1, aggKd = 0.01, elbaggKp = 0.025, elbaggKi = 0, elbaggKd = 0;
//  double regKp = 0.4, regKi = 0.0, regKd = 0.0, elbregKp = 0.025, elbregKi = 0, elbregKd = 0;

// WAIST (SERVO)
double aggKp = 0.6, aggKi = 0.1, aggKd = 0.01, elbaggKp = 0.025, elbaggKi = 0, elbaggKd = 0;
double regKp = 0.8, regKi = 0.2, regKd = 0.2, elbregKp = 0.025, elbregKi = 0, elbregKd = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
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

/*---------------------CYTRON DECLARATIONS---------------------*/
double setpoint = 0;
int turn = 0;
int brakeSet = 0;
Pin CYTRON_DIR_1(CYTRON_DIR_1_GPIO_Port, CYTRON_DIR_1_Pin);
Pin CYTRON_PWM_1(CYTRON_PWM_1_GPIO_Port, CYTRON_PWM_1_Pin, &htim2, TIM_CHANNEL_2);
Pin AMT22_1(GPIOC, GPIO_PIN_7);
RoverArmMotor Wrist_Roll(&hspi1, CYTRON_PWM_1, CYTRON_DIR_1, AMT22_1, CYTRON, 0, 359.99f);
int button_counter = 0;

/*---------------------SERVO DECLARATIONS---------------------*/
Pin dummy_pin;
// Pin SERVO_PWM_1(SERVO_PWM_1_GPIO_Port, SERVO_PWM_1_Pin, &htim1, TIM_CHANNEL_2);
Pin SERVO_PWM_1(CYTRON_PWM_1_GPIO_Port, CYTRON_PWM_1_Pin, &htim2, TIM_CHANNEL_2);
RoverArmMotor Waist(&hspi1, SERVO_PWM_1, dummy_pin, AMT22_1, BLUE_ROBOTICS, 0, 359.99f);
int is_turning = 0;

/*---------------------HELPER---------------------*/
// static void print_MOTOR(char *msg, RoverArmMotor *pMotor)
// {
//   double current_angle = pMotor->get_current_angle();
//   double current_angle_multi = pMotor->get_current_angle_multi();
//   double current_angle_sw = pMotor->get_current_angle_sw();
//   int turn_count = pMotor->get_turn_count();
//   printf("%s turn_count %d, setpoint %.2f, angle_sw %.2f, zero_sw %.2f, angle_raw_multi %.2f, angle_raw %.2f, _outputSum %.2f, output_specific %.2f, output_raw %.2f\r\n",
//          msg,
//          turn_count,
//          pMotor->setpoint,
//          current_angle_sw,
//          pMotor->zero_angle_sw,
//          current_angle_multi,
//          current_angle,
//          pMotor->internalPIDInstance._outputSum,
//          (*(pMotor->internalPIDInstance._myOutput)) + 1500.0 - 1.0,
//         pMotor->output);
// }

static void print_MOTOR(char *msg, RoverArmMotor *pMotor)
{
  double current_angle_multi;
  double current_angle_sw;
  // int turn_count = pMotor->get_turn_count();

//  pMotor->get_current_angle_multi(&current_angle_multi);
  pMotor->get_current_angle_sw(&current_angle_sw);

  printf("%s setpoint %.2f, angle_sw %.2f, output %.2f, output_actual %.2f\r\n",
         msg,
         pMotor->setpoint,
         current_angle_sw,
         pMotor->output,
         pMotor->output + 1500 - 1);
}

/*---------------------UART---------------------*/
const int RX_BUFFER_SIZE = 32;
uint8_t rx_data[8]; // 1 byte
char rx_buffer[RX_BUFFER_SIZE];
uint32_t rx_index = 0;
char command_buffer[20];
double param1, param2, param3;

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

  /*---AMT22 setup---*/
  // resetAMT22(&hspi1, GPIOC, GPIO_PIN_7, &htim1);

  /*---SERVO setup---*/
  int32_t CH2_ESC = 1500 - 1;
  HAL_Delay(500);
  Waist.wrist_waist = 1;
  Waist.begin(aggKp, aggKi, aggKd, regKp, regKi, regKd);
  Waist.setAngleLimits(-999999.99f, 999999.99f); // TODO check good angle limits
  Waist.reset_encoder();
  HAL_TIM_Base_Start_IT(&htim6);

  /*---CYTRON setup---*/
  // int32_t CH2_DC = 0;
  // Wrist_Roll.wrist_waist = 1;
  // Wrist_Roll.begin(aggKp, aggKi, aggKd, regKp, regKi, regKd);
  // Wrist_Roll.setGearRatio(2.672222f);
  // Wrist_Roll.setAngleLimits(-359.99, 359.99f); // TODO check good angle limits

  // __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 30);
  // while (!brakeSet)
  // {
  //   print_MOTOR("BRAKE", &Waist);
  // }
  // __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);

  /*---UART setup---*/
  HAL_UART_Receive_IT(&huart2, rx_data, 1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /*--------------------------------------FREE DEBUG--------------------------------------*/
    // print_MOTOR("Wrist_Roll", &Waist);
    // print_MOTOR("Wrist_Roll", &Wrist_Roll);
    // HAL_Delay(100);

    /*--------------------------------------AMT22 test--------------------------------------*/
    // encoderData_1 = getPositionSPI(&hspi1, GPIOC, GPIO_PIN_7, 12, &htim1);
    // encoderData_2 = getPositionSPI(&hspi2, GPIOB, GPIO_PIN_6, 12, &htim1);
    // encoderData_3 = getPositionSPI(&hspi3, GPIOA, GPIO_PIN_8, 12, &htim1);
    // printf("encoder 1 gives %d\r\n", encoderData_1);
    // printf("encoder 2 gives %d\r\n", encoderData_2);
    // printf("encoder 3 gives %d\r\n", encoderData_3);
    // // int16_t* turn_count = (int16_t*)malloc(2*sizeof(int16_t));

    // int16_t turn_count[2];
    // int turn_count_WR = Wrist_Roll.get_turn_count();
    // getTurnCounterSPI(turn_count, &hspi1, GPIOC, GPIO_PIN_7, 12, &htim1);
    // printf("turn_count_WR is %d count is %d and %d\r\n", turn_count_WR, turn_count[0], turn_count[1]);
    // std::bitset<16> y(turn_count[1]);
    // printf("%s\r\n", y.to_string().c_str());
    // HAL_Delay(100);

    /*--------------------------------------SPI test--------------------------------------*/
    // uint8_t sendByte = 0x00;
    // uint8_t data = 0x00;
    // int err = HAL_SPI_TransmitReceive(&hspi1, &sendByte, &data, 1, 10);
    // uint8_t data2 = 0x00;

    /*--------------------------------------CYTRON test--------------------------------------*/
    // printf("0\r\n");
    // __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
    // HAL_Delay(1000);

    // printf("20\r\n");
    // __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 20);
    // HAL_Delay(1000);

    // printf("60\r\n");
    // __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 60);
    // HAL_Delay(1000);
    // current_angle = Wrist_Roll.get_current_angle();
    // printf("current angle is %f\r\n, current_angle");
    // HAL_Delay(50);
    // Wrist_Roll.newSetpoint(current_angle + 10);
    // Wrist_Roll.tick();
    // HAL_Delay(1); // safety delay

    /*--------------------------------------CYTRON print--------------------------------------*/
    // current_angle = Wrist_Roll.get_current_angle();
    // current_angle_sw = Wrist_Roll.get_current_angle_sw();
    // setpoint = Wrist_Roll.getSetpoint();
    // printf("current angle: %f, setpoint: %f, current_angle_sw: %f\r\n", current_angle, setpoint, current_angle_sw);

    /*--------------------------------------CYTRON angle limit test--------------------------------------*/
    // high first because we just set zero
    // Wrist_Roll.newSetpoint(Wrist_Roll.highestAngle);
    // while(!(Wrist_Roll.get_current_angle_sw() >= Wrist_Roll.highestAngle - 2.0)) {
    //     print_MOTOR("UP");
    //     Wrist_Roll.tick();
    // }
    // Wrist_Roll.stop();

    // Wrist_Roll.newSetpoint(Wrist_Roll.lowestAngle);
    // while(!(Wrist_Roll.get_current_angle_sw() <= Wrist_Roll.lowestAngle + 2.0)) {
    //     print_MOTOR("DOWN");
    //     Wrist_Roll.tick();
    // }
    // Wrist_Roll.stop();

    /*--------------------------------------CYTRON setpoint test--------------------------------------*/
    // print_MOTOR("SP Wrist_Roll", &Wrist_Roll);
    // Wrist_Roll.tick();

    /*--------------------------------------SERVO setpoint test--------------------------------------*/
    // print_MOTOR("SP Waist", &Waist);
    // Waist.tick();

    /*--------------------------------------CYTRON direction test--------------------------------------*/
    // Wrist_Roll.setpoint = 99999;  // to make sure turn in positive direction, should be CCW
    // print_MOTOR("SP Wrist_Roll", &Wrist_Roll);
    // Wrist_Roll.tick();

    /*--------------------------------------SERVO direction test--------------------------------------*/
    // Waist.setpoint = 99999;  // to make sure turn in positive direction, should be CCW
    // print_MOTOR("SP Waist", &Waist);
    print_MOTOR("SP Waist", &Waist);
    HAL_Delay(100);
    // __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 1600);

    /*--------------------------------------UART test loop--------------------------------------*/
    // HAL_UART_Receive(&huart2, rx_buffer, 4, 2000);
    // double kP, kI, kD;
    // char buffer[50];
    // sprintf(buffer, "0.1 0.2 0.3"); // Example string with three float values separated by spaces
    // sscanf(buffer, "%lf %lf %lf", &kP, &kI, &kD); // Parse the float values
    // printf("kP: %lf, kI: %lf, kD: %lf\r\n", kP, kI, kD); // Print the float values

    //  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13);
    //  HAL_Delay(200);

    /*--------------------------------------ESC test--------------------------------------*/
    // print_MOTOR("WAIST", &Waist);
    // printf("%d\r\n", is_turning);
    // __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 1500 - 1);
    // printf("1500\r\n");
    // HAL_Delay(2000);
    // __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 1550);
    // printf("1550\r\n");
    // HAL_Delay(2000);
    // HAL_Delay(1000);
    // __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 1500-1);
    // HAL_Delay(1000);
    // __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 1450-1);
    // printf("1450\r\n");

    /*--------------------------------------ESC sweep test--------------------------------------*/
    // while(CH2_ESC < 1600)
    // {
    //     // TIM2->CCR2 = CH2_DC;
    //   CH2_ESC += 1;
    // 	printf("current CH2_DC %d\r\n", CH2_ESC);
    //   __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, CH2_ESC); //this is the same as above
    //   HAL_Delay(50);
    // }
    // while(CH2_ESC > 1540)
    // {
    //     // TIM2->CCR2 = CH2_DC;
    //   CH2_ESC -= 1;
    // 	printf("current CH2_DC %d\r\n", CH2_ESC);
    //   __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, CH2_ESC); //this is the same as above
    //   HAL_Delay(50);
    // }

    // LEGACY CODE
    //  if(encoderData_3 > encoder_max && encoderData_3 != 65535) encoder_max = encoderData_3;
    //  if(encoderData_3 < encoder_min && encoderData_3 != 65535) encoder_min = encoderData_3;
    //  printf("encoder_max is %d\r\n", encoder_max);
    //  printf("encoder_min is %d\r\n", encoder_min);

    // PWM test
    //  while(CH2_DC < 65535)
    //  {
    //      // TIM2->CCR2 = CH2_DC;
    //  	printf("current CH2_DC %d\r\n", CH2_DC);
    //      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, CH2_DC); //this is the same as above
    //      CH2_DC += 70;
    //      HAL_Delay(10);
    //  }
    //  while(CH2_DC > 0)
    //  {
    //      // TIM2->CCR2 = CH2_DC;
    //  	printf("current CH2_DC %d\r\n", CH2_DC);
    //      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, CH2_DC); //this is the same as above
    //      CH2_DC -= 70;
    //      HAL_Delay(10);
    //  }

    // TIMER TEST
    //  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
    //  HAL_Delay(1000);
    //  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
    //  HAL_Delay(1000);

    // TIMER US TEST
    //	   HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_15);
    //	   delay_us(3);

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
  // if(!brakeSet) {
  if (GPIO_Pin == B1_Pin) // INT Source is pin A9
  {
    is_turning = !is_turning;

    Wrist_Roll.set_zero_angle_sw();
    Waist.set_zero_angle_sw();
    HAL_Delay(10);
    Wrist_Roll.newSetpoint(0.0); // TODO check this?
    Waist.newSetpoint(0.0);      // TODO check this?
    brakeSet = 1;
    button_counter++;
    return;
  }
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
      Waist.newSetpoint(param1);
      printf("new Setpoint at %lf\r\n", param1);
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
    Waist.tick();
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
