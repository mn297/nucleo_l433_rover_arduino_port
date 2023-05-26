// #include <Arduino.h>
#include "RoverArmMotor.h"
#include "AMT22.h"
#include <cstdlib>
#include "tim.h"



// TODO: Test this class with the old code, remember to create backup beforehand!
// I'm very suspicious of the way I handled user defined pointers...

// The motor will not move until begin() is called!
/**
 * @brief  Constructor for RoverArmMotor class
 * @param  spi_handle: encoder SPI handle
 * @param  pwm_pin: pin for the PWM
 * @param  dir_pin: only used for Cytron ESC
 * @param  encoder_pin: pin for the encoder
 * @param  esc_type: CYTRON or BLUE_ROBOTICS
 * @param  minimum_angle: minimum angle of the motor
 * @param  maximum_angle: maximum angle of the motor
 * @param  brake_pin: pin for the brake or limit switch
 * @retval None
 */
RoverArmMotor::RoverArmMotor(SPI_HandleTypeDef* spi_handle, Pin pwm_pin, Pin dir_pin, Pin encoder_pin, int esc_type, double minimum_angle, double maximum_angle, Pin brake_pin)
                :internalPIDInstance(&input, &output, &setpoint, regularKp, regularKi, regularKd, _PID_CD_DIRECT)
                ,internalAveragerInstance(15){

    //constructor
    spi = spi_handle;
    pwm = pwm_pin;
    dir = dir_pin;
    encoder = encoder_pin;
    brake = brake_pin;
    escType = esc_type;
    lowestAngle = minimum_angle;
    highestAngle = maximum_angle;

    //clean up variables
    input = 0;
    output = 0;
    lastAngle = 0;
    useSwAngle = 1;  //default use software angle
    zero_angle_sw = 0;  // default no offset
    
}

void RoverArmMotor::begin(double aggP, double aggI, double aggD, double regP, double regI, double regD){


    /*------------------Initialize timers------------------*/ 
    HAL_TIM_PWM_Start(pwm.p_tim, pwm.tim_channel);
    HAL_Delay(500); // wait for the motor to start up
    if(escType == CYTRON){
        __HAL_TIM_SET_COMPARE(pwm.p_tim, pwm.tim_channel, 0); // stop motor
    }
    else if(escType == BLUE_ROBOTICS){
        __HAL_TIM_SET_COMPARE(pwm.p_tim, pwm.tim_channel, 1500-1); // astop motor
    }


    /*------------------set PID parameters------------------*/
    if(escType == CYTRON){
        internalPIDInstance.SetOutputLimits(-99, 99); // PWM duty cycle mn297 TOOD: check this
    }
    else if(escType == BLUE_ROBOTICS){
        internalPIDInstance.SetOutputLimits(-100, 100); // 1500 +- 400 for BlueRobotics ESC
    }
    
    /*------------------Initialize moving average------------------*/
    internalAveragerInstance.begin();

    /*------------------Initialize PID------------------*/
    internalPIDInstance.Init();
    internalPIDInstance.SetMode(_PID_MODE_AUTOMATIC);


    /*------------------Get setpoint------------------*/
    // Get current location and set it as setpoint. Essential to prevent jerkiness
    // as the microcontroller initializes.
    // adcResult = internalAveragerInstance.reading(analogRead(encoder));
    //after setup, currentAngle is same as setpoint
    currentAngle = get_current_angle();    // fix setpoint not equal to current angle
    setpoint = currentAngle;



    /*------------------Set PID parameters------------------*/
    regularKp = regP;
    regularKi = regI;
    regularKd = regD;
    aggressiveKp = aggP;
    aggressiveKi = aggI;
    aggressiveKd = aggD;

    internalPIDInstance.SetTunings(regularKp, regularKi, regularKd);
    // if(brake)  engageBrake(); //use brake if there is one
    if (brake.valid != 0)  engageBrake(); //use brake if there is one

    //initialize the multiplier bool to false and the multiplier to 1. 
    wrist_waist = false; 
    //multiplier = 1;
    gearRatio = 1;  //TODO check if this is correct

}

int positive_rezeros = 0;
double real_angle = 0;

// Needs to be called in each loop
void RoverArmMotor::tick(){ // worry about currentAngle and setpoint

    /*------------------Get current angle------------------*/
    // adcResult = internalAveragerInstance.reading(analogRead(encoder));
    if (useSwAngle) {
        currentAngle = get_current_angle_sw(); 
    }   else {
        currentAngle = get_current_angle(); 
    }

      // Measurement deadband - ignore sub-degree noise
    if(abs(currentAngle - lastAngle) < 1.0){
        currentAngle = lastAngle;
    }
    input = currentAngle; // range is R line



   //------------------Compute PID------------------//
    // Compute distance, retune PID if necessary. Less aggressive tuning params for small errors
    // Find the shortest from the current position to the set point
    double gap; //mn297 could be negative

    gap = setpoint - input;
    // if(wrist_waist){
    //     // (abs(setpoint-input) < abs((setpoint + 360.0f)-input)) ? 
    //     // gap = setpoint - input : gap = (setpoint + 360.0f) - input; 

    //     // if(abs(setpoint-input) < abs((setpoint + 360.0f)-input)) { 
    //     //     gap = setpoint - input; 
    //     // } else {
    //     //     gap = (setpoint + 360.0f) - input; 
    //     // }
    //     if(abs(setpoint-input) > abs((setpoint + 360.0f)-input)) { // TODO check if this is correct
    //         gap = input - (setpoint + 360.0f); 
    //     } else {
    //         gap = setpoint - input; 
    //     }
        
    // }
    // else{
    //     gap = setpoint - input;
    // }


    // Tone down P and I as the motor hones onto position
    if (abs(gap) < 10){
        internalPIDInstance.SetTunings(regularKp, regularKi, regularKd);
    }else{  // TODO agressive tuning params
        internalPIDInstance.SetTunings(regularKp, regularKi, regularKd);
        // internalPIDInstance.SetTunings(aggressiveKp, aggressiveKi, aggressiveKd);
    }
    internalPIDInstance.Compute(); // return value stored in output


    //------------------SAFETY------------------//
    if(currentAngle >= (highestAngle - 2) && currentAngle <= (lowestAngle + 2)) output = 0.0;


    //------------------Write to motor------------------//
    if(escType == CYTRON){

        // Interpret sign of the error signal as the direction pin value
        // (gap > 0) ? digitalWrite(dir, HIGH) : digitalWrite(dir, LOW); // invert if needed mn297
        if (output > 0) {
            HAL_GPIO_WritePin(dir.port, dir.pin, GPIO_PIN_SET); // B high
        }
        else {
            HAL_GPIO_WritePin(dir.port, dir.pin, GPIO_PIN_RESET); // A high
        }
        // Write to PWM pin
        //TODO port to HAL
        // analogWrite(pwm, abs(output)); //mn297 function execute quickly and jumps to next tick()
        double test_output = abs(output);     //smoothing
        __HAL_TIM_SET_COMPARE(pwm.p_tim, pwm.tim_channel, (int) test_output);

    }

    //TODO: Add support for other ESC types    
    else if(escType == BLUE_ROBOTICS){
        // This one is more straightforward since we already defined the output range
        // from 1100us to 1900us
        // internalServoInstance.writeMicroseconds(output);
        servo_dir = 1;  //TODO refactor
        __HAL_TIM_SET_COMPARE(pwm.p_tim, pwm.tim_channel, 1500-1 + output);
        HAL_Delay(100);

    }


    //------------------Update angle------------------//
    lastAngle = currentAngle;
    
}
void RoverArmMotor::stop(){
    __HAL_TIM_SET_COMPARE(pwm.p_tim, pwm.tim_channel, (int) 0);
}
void RoverArmMotor::set_PID_params(double aggP, double aggI, double aggD, double regP, double regI, double regD) {
    regularKp = regP;
    regularKi = regI;
    regularKd = regD;
    aggressiveKp = aggP;
    aggressiveKi = aggI;
    aggressiveKd = aggD;
    internalPIDInstance.SetTunings(regularKp, regularKi, regularKd);
}

bool RoverArmMotor::setMultiplierBool(bool mult, double ratio){
    wrist_waist = mult; 
    gearRatio = ratio; 
    //a bit redundant but just a sanity check of a second getter method
    if(getRatio() == ratio) return true; 
    return false; 
}

// For display purposes
double RoverArmMotor::getSetpoint(){
    return setpoint / gearRatio;
}

bool RoverArmMotor::newSetpoint(double angl){
    double setpoint_test = angl * gearRatio;
    if(setpoint_test >= lowestAngle && setpoint_test <= highestAngle){
        setpoint = setpoint_test;
        return true;
    }else{
        return false;
    }
}

int RoverArmMotor::getDirection(){
    // return (digitalRead(dir) == HIGH) ? FWD : REV;
    return (HAL_GPIO_ReadPin(dir.port, dir.pin) == GPIO_PIN_SET) ? FWD : REV; //mn297, TODO check if this is correct
}

void RoverArmMotor::setGearRatio(double ratio){
    gearRatio = ratio;
}

void RoverArmMotor::setAngleLimits(double lowest, double highest){
    lowestAngle = lowest * gearRatio;
    highestAngle = highest * gearRatio;
}

void RoverArmMotor::set_zero_angle(){
    setZeroSPI(spi, encoder.port, encoder.pin, nullptr); //timer not used, so nullptr
}
void RoverArmMotor::reset_encoder(){
    resetAMT22(spi, encoder.port, encoder.pin, nullptr); //timer not used, so nullptr
}
void RoverArmMotor::set_zero_angle_sw(){
    // zero_angle_sw = this->get_current_angle();
    zero_angle_sw = this->get_current_angle_multi();

}  //mn297 software zero angle
uint32_t RoverArmMotor::get_turns_encoder(){    //mn297
    uint32_t turns = get_turns_AMT22(spi, encoder.port, encoder.pin, 12, nullptr);
    return turns;
}


void RoverArmMotor::disengageBrake(){
    if(brake.valid != 0){
    //   digitalWrite(brake, LOW);  
    HAL_GPIO_WritePin(brake.port, brake.pin, GPIO_PIN_RESET); //mn297
    }
}

void RoverArmMotor::engageBrake(){
    if(brake.valid != 0){
    //    digitalWrite(brake, HIGH); 
    HAL_GPIO_WritePin(brake.port, brake.pin, GPIO_PIN_SET); //mn297
    }
}

double RoverArmMotor::get_current_angle_avg(){    //UNSUPPORTED
    // return currentAngle / gearRatio;
    uint16_t encoderData = getPositionSPI(spi, encoder.port, encoder.pin, 12, nullptr); //timer not used, so nullptr
    adcResult = internalAveragerInstance.reading(encoderData);  // implicit cast to int
    currentAngle = mapFloat((float) adcResult, MIN_ADC_VALUE, MAX_ADC_VALUE, 0, 359.99f); //mn297 potentiometer encoder
    return currentAngle / gearRatio;
}
double RoverArmMotor::get_current_angle(){    //mn297
    // return currentAngle / gearRatio;
    uint16_t encoderData = getPositionSPI(spi, encoder.port, encoder.pin, 12, nullptr); //timer not used, so nullptr
    currentAngle = mapFloat((float) encoderData, MIN_ADC_VALUE, MAX_ADC_VALUE, 0, 359.99f); //mn297 potentiometer encoder
    return currentAngle;
}

double RoverArmMotor::get_current_angle_multi(){    //mn297
    // return currentAngle / gearRatio;
    int16_t result_arr[2];
    getTurnCounterSPI(result_arr, spi, encoder.port, encoder.pin, 12, nullptr); //timer not used, so nullptr
    double angle_raw = mapFloat((float) result_arr[0], MIN_ADC_VALUE, MAX_ADC_VALUE, 0, 359.99f); //mn297 potentiometer encoder
    int turns = result_arr[1];
    if (turns > 0) {
        return angle_raw + 360 * turns;
    } else if (turns < 0) {
        return angle_raw + 360 * turns;
    } else {
        return angle_raw;
    }
}

double RoverArmMotor::get_current_angle_sw(){    //TODO mn297
    double angle_raw = get_current_angle_multi();
    double diff = angle_raw - zero_angle_sw;
    return diff;
}
double RoverArmMotor::get_current_angle_sw_multi(){ 
    //UNUSED
}
double RoverArmMotor::getCurrentOutput(){
    return output;
}

double RoverArmMotor::mapFloat(float x, float in_min, float in_max, float out_min, float out_max){
    double result = ((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);

    return result; 
    }

double RoverArmMotor::getRatio(){
    return gearRatio;
}

int RoverArmMotor::get_turn_count(){
    int16_t result_arr[2];
    getTurnCounterSPI(result_arr, spi, encoder.port, encoder.pin, 12, nullptr); //timer not used, so nullptr
    return result_arr[1];
    // return turn_count;
}


void RoverArmMotor::WatchdogISR(){
    // Get current angle

    // Set setpoint to that angle
}
