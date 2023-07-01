#include "main.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "dma.h"

#include "rover_arm.h"

// Drivers.
#include "RoverArmMotor.h"
#include "AMT22.h"

// Standard includes.
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <bitset>
#include <limits>

#define MIN_FLOAT -std::numeric_limits<float>::infinity()
#define MAX_FLOAT std::numeric_limits<float>::infinity()

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

// ELBOW_SERVO
double regKp_elbow = 0.6, regKi_elbow = 0.2, regKd_elbow = 0.1; // Vincent's configuration
// double regKp_waist = 1.2, regKi_waist = 0.4, regKd_waist = 0.4; // Sam's configuration
// double regKp_waist = 2, regKi_waist = 0.4, regKd_waist = 0.4;  // Speed configuration

// END_EFFECTOR_CYTRON
double regKp_end_effector = 0.4, regKi_end_effector = 0.0, regKd_end_effector = 0.0;

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

Pin AMT22_1(GPIOC, GPIO_PIN_7);
Pin CYTRON_DIR_1(CYTRON_DIR_1_GPIO_Port, CYTRON_DIR_1_Pin);
Pin CYTRON_PWM_1(CYTRON_PWM_1_GPIO_Port, CYTRON_PWM_1_Pin, &htim2, TIM_CHANNEL_2);
Pin SERVO_PWM_1(CYTRON_PWM_1_GPIO_Port, CYTRON_PWM_1_Pin, &htim2, TIM_CHANNEL_2);

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
RoverArmMotor Waist(&hspi1, SERVO_PWM_1, dummy_pin, AMT22_1, BLUE_ROBOTICS, 0, 359.99f);
#endif

/*---------------------ELBOW_SERVO DECLARATIONS---------------------*/
#if TEST_ELBOW_SERVO == 1
RoverArmMotor Elbow(&hspi1, SERVO_PWM_1, dummy_pin, AMT22_1, BLUE_ROBOTICS, 0, 359.99f);
#endif

/*---------------------END_EFFECTOR_CYTRON---------------------*/
#if TEST_END_EFFECTOR_CYTRON == 1
RoverArmMotor End_Effector(&hspi1, CYTRON_PWM_1, CYTRON_DIR_1, AMT22_1, CYTRON, 0, 359.99f);
#endif

void rover_arm_setup(void)
{
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

  /*---END_EFFECTOR_CYTRON setup---*/

#if TEST_END_EFFECTOR_CYTRON == 1
  End_Effector.wrist_waist = 0;
  End_Effector.setAngleLimits(MIN_FLOAT, MAX_FLOAT);
  End_Effector.reset_encoder();
  End_Effector.begin(regKp_end_effector, regKi_end_effector, regKd_end_effector);
  End_Effector.reverse(100);
  HAL_TIM_Base_Start_IT(&htim6);
#endif

  while (!limit_set)
    ;

  /*---TIM6 interrupt---*/
  HAL_TIM_Base_Start_IT(&htim6);

  /*---UART setup---*/
  HAL_UART_Receive_IT(&huart2, rx_data, 1);
}


void rover_arm_loop()
{
#if TEST_ENCODER == 1
  uint16_t encoderData_1 = 0;
  encoderData_1 = getPositionSPI(&hspi1, GPIOC, GPIO_PIN_7, 12, &htim1);
  printf("encoder 1 gives %d\r\n", encoderData_1);
#endif

#if TEST_WAIST_SERVO == 1
  print_MOTOR("SP Waist", &Waist);
#endif

#if TEST_ELBOW_SERVO == 1
  print_MOTOR("SP Elbow", &Elbow);
#endif

#if TEST_WRIST_ROLL_CYTRON == 1
  print_MOTOR("SP WRIST_ROLL_CYTRON", &Wrist_Roll);
#endif

#if TEST_WRIST_PITCH_CYTRON == 1
  print_MOTOR("SP WRIST_PITCH_CYTRON", &Wrist_Pitch);
#endif

#if TEST_END_EFFECTOR_CYTRON == 1
  print_MOTOR("SP END_EFFECTOR_CYTRON", &End_Effector);
#endif
  HAL_Delay(100);
}

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

#if TEST_END_EFFECTOR_CYTRON == 1
    End_Effector.stop();
    End_Effector.set_zero_angle_sw();
    End_Effector.newSetpoint(0.0);
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
      bool is_sp_valid;
#if TEST_WRIST_ROLL_CYTRON == 1
      is_sp_valid = Wrist_Roll.newSetpoint(param1);
#endif
#if TEST_WRIST_PITCH_CYTRON == 1
      is_sp_valid = Wrist_Pitch.newSetpoint(param1);
#endif
#if TEST_WAIST_SERVO == 1
      is_sp_valid = Waist.newSetpoint(param1);
#endif
#if TEST_ELBOW_SERVO == 1
      is_sp_valid = Elbow.newSetpoint(param1);
#endif
#if TEST_END_EFFECTOR_CYTRON == 1
      is_sp_valid = End_Effector.newSetpoint(param1);
#endif

      if (is_sp_valid)
        printf("new Setpoint at %lf\r\n", param1);
      else
        printf("invalid Setpoint %lf\r\n", param1);
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
#if TEST_END_EFFECTOR_CYTRON == 1
    // End_Effector.tick();
    print_MOTOR("SP END_EFFECTOR_CYTRON", &End_Effector);
#endif
  }
}
