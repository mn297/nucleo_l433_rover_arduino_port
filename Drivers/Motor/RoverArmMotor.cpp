// #include <Arduino.h>
#include "RoverArmMotor.h"
#include <cstdlib>



// TODO: Test this class with the old code, remember to create backup beforehand!
// I'm very suspicious of the way I handled user defined pointers...

// The motor will not move until begin() is called!
RoverArmMotor::RoverArmMotor(Pin pwm_pin, Pin dir_pin, Pin encoder_pin, int esc_type, double minimum_angle, double maximum_angle, Pin brake_pin)
                :internalPIDInstance(&input, &output, &setpoint, regularKp, regularKi, regularKd, _PID_CD_DIRECT)
                ,internalAveragerInstance(15){

    //constructor
    pwm = pwm_pin;
    dir = dir_pin;
    encoder = encoder_pin;
    brake = brake_pin;
    escType = esc_type;
    lowestAngle = minimum_angle;
    highestAngle = maximum_angle;
    
}

void RoverArmMotor::begin(double aggP, double aggI, double aggD, double regP, double regI, double regD){

    // Initialize given pins
    // pinMode(encoder, INPUT); // not needed since we use HAL library
    // pinMode(pwm, OUTPUT);

    if(escType == CYTRON){
        // pinMode(dir, OUTPUT); // not needed since we use HAL library

        // Allow negative outputs, the sign will be interpreted as
        // the direction pin
        internalPIDInstance.SetOutputLimits(-100, 100); // max ADC write is 255 PWM mn297
    }
    //TODO: Add support for other ESC types
    // else if(escType == BLUE_ROBOTICS){
    //      // BlueRobotics ESC uses a servo-like control scheme where
    //     // 1100us is full speed reverse and 1900us is full speed forward
    //     internalPIDInstance.SetOutputLimits(1100, 1900); // 50Hz servo? mn297
    //     // internalServoInstance.attach(pwm);
    //     internalServoInstance.attach(pwm, 1100, 1900, 1500); // mn297
    // }
    
    // Initialize moving averager
    internalAveragerInstance.begin();

    // Set to auto
    internalPIDInstance.SetMode(_PID_MODE_AUTOMATIC);


    // TODO implement read encoder with new AMT22
    // Get current location and set it as setpoint. Essential to prevent jerkiness
    // as the microcontroller initializes.
    // adcResult = internalAveragerInstance.reading(analogRead(encoder));



    currentAngle = mapFloat((float) adcResult, MIN_ADC_VALUE, MAX_ADC_VALUE, 0, 359.0f);
    setpoint = currentAngle;

    // Set tuning params
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
    gearRatio = 1;

}

int positive_rezeros = 0;
double real_angle = 0;

// Needs to be called in each loop
void RoverArmMotor::tick(){

    //TODO implement read encoder with new AMT22
    // Get current angle
    // adcResult = internalAveragerInstance.reading(analogRead(encoder));



    // currentAngle = mapFloat((float) adcResult, MAX_ADC_VALUE, MIN_ADC_VALUE, 359.0f, 0.0f);
    currentAngle = mapFloat((float) adcResult, MIN_ADC_VALUE, MAX_ADC_VALUE, 0, 359.0f); //mn297 potentiometer encoder

      // Measurement deadband - ignore sub-degree noise
    if(abs(currentAngle - lastAngle) < 1.0){
        currentAngle = lastAngle;
    }

    input = currentAngle; // range [0,359]

    // Compute distance, retune PID if necessary. Less aggressive tuning params for small errors
    // Find the shortest from the current position to the set point
    double gap; //mn297 could be negative

    if(wrist_waist){
        (abs(setpoint-input) < abs((setpoint + 360.0f)-input)) ? //mn297 uesless cause always true
        gap = setpoint - input : gap = (setpoint + 360.0f) - input; 
    }else{
        gap = setpoint - input;
    }

    // If we are outside of angle bounds, make a setpoint intervention to bring the shaft to the midpoint
    if(input <= lowestAngle || input >= highestAngle){
        setpoint = gearRatio * (lowestAngle + highestAngle) / 2 ;
    }

    // Tone down P and I as the motor hones onto position
    if (abs(gap) < 10){
        internalPIDInstance.SetTunings(regularKp, regularKi, regularKd);
    }else{
        internalPIDInstance.SetTunings(aggressiveKp, aggressiveKi, aggressiveKd);
    }

    // Compute the next value
    internalPIDInstance.Compute(); // return value stored in output
    // Serial.print("current output is ");
    // Serial.println(output, 4); 


    // Refuse to go to setpoint if it is outside of boundaries
    // if(setpoint <= lowestAngle || setpoint >= highestAngle){
    //     output = 0;
    // }

    // Make sure we aren't snapping our tendons - move back a little bit if we are
    // if(currentAngle >= (highestAngle - 2) && currentAngle <= (lowestAngle + 2)) output = 0.0;

    // Interpret output data based on the ESC type defined in constructor
    if(escType == CYTRON){

        // Interpret sign of the error signal as the direction pin value
        // (gap > 0) ? digitalWrite(dir, HIGH) : digitalWrite(dir, LOW); // invert if needed mn297
        if (gap > 0) HAL_GPIO_WritePin(dir.port, dir.pin, GPIO_PIN_SET); //mn297
        else HAL_GPIO_WritePin(dir.port, dir.pin, GPIO_PIN_RESET); //mn297

        // Write to PWM pin
        //TODO port to HAL
        // analogWrite(pwm, abs(output)); //mn297 function execute quickly and jumps to next tick()

    }

    //TODO: Add support for other ESC types    
    // else if(escType == BLUE_ROBOTICS){
    //     // This one is more straightforward since we already defined the output range
    //     // from 1100us to 1900us
    //     internalServoInstance.writeMicroseconds(output);
    // }

    lastAngle = currentAngle;
    
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
    if(setpoint_test > lowestAngle && setpoint_test < highestAngle){
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

// void RoverArmMotor::setGearRatio(double ratio){
//     gearRatio = ratio;
// }

void RoverArmMotor::setAngleLimits(double lowest, double highest){
    lowestAngle = lowest * gearRatio;
    highestAngle = highest * gearRatio;
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

double RoverArmMotor::getCurrentAngle(){
    return currentAngle / gearRatio;
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

void RoverArmMotor::WatchdogISR(){
    // Get current angle

    // Set setpoint to that angle
}
