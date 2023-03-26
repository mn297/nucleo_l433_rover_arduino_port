/*
  AMT22.c - STM32 library for ATM22 series absolute encoders by CUI Devices.
  Created by Simone Di Blasi, December 2020.
*/

#include "AMT22.h"
#include "tim.h"
/* SPI commands */
#define AMT22_NOP       0x00
#define AMT22_RESET     0x60
#define AMT22_ZERO      0x70
#define AMT22_TURNS     0xA0

#define RES12           12
#define RES14           14

#define AMT22_DELAY     10

void setCSLine (GPIO_TypeDef* encoderPort, uint16_t encoderPin, GPIO_PinState csLine)
{
	HAL_GPIO_WritePin(encoderPort, encoderPin, csLine);
}

uint8_t spiWriteRead(SPI_HandleTypeDef *hspi, uint8_t sendByte, GPIO_TypeDef* encoderPort, uint16_t encoderPin, uint8_t releaseLine, TIM_HandleTypeDef *timer)
{
  //to hold received data  
  uint8_t data;

  //set cs low, cs may already be low but there's no issue calling it again except for extra time
  setCSLine(encoderPort, encoderPin , GPIO_PIN_RESET);

  //There is a minimum time requirement after CS goes low before data can be clocked out of the encoder.
//  delay(timer, 3);
  delay_us_AMT22(AMT22_DELAY);

  //send the command and receive response of the slave
  HAL_SPI_TransmitReceive(hspi, &sendByte, &data, 1, 10);

  //There is also a minimum time after clocking that CS should remain asserted before we release it
  //  delay(timer, 3);
  delay_us_AMT22(AMT22_DELAY);

  setCSLine(encoderPort, encoderPin, releaseLine); //if releaseLine is high set it high else it stays low

  return data;
}

uint16_t getPositionSPI(SPI_HandleTypeDef *hspi, GPIO_TypeDef* encoderPort, uint16_t encoderPin, uint8_t resolution, TIM_HandleTypeDef *timer)
{
  uint16_t currentPosition;       //16-bit response from encoder
  uint8_t binaryArray[16];        //after receiving the position we will populate this array and use it for calculating the checksum

  //get first byte which is the high byte, shift it 8 bits. don't release line for the first byte
  currentPosition = spiWriteRead(hspi, AMT22_NOP, encoderPort, encoderPin, 0, timer) << 8;

  //this is the time required between bytes as specified in the datasheet.
//  delay(timer, 3);
  delay_us_AMT22(AMT22_DELAY);

  //OR the low byte with the currentPosition variable. release line after second byte
  currentPosition |= spiWriteRead(hspi, AMT22_NOP, encoderPort, encoderPin, 1, timer);

  //run through the 16 bits of position and put each bit into a slot in the array so we can do the checksum calculation
  for(int i = 0; i < 16; i++) binaryArray[i] = (0x01) & (currentPosition >> (i));

  //using the equation on the datasheet we can calculate the checksums and then make sure they match what the encoder sent
 if ((binaryArray[15] == !(binaryArray[13] ^ binaryArray[11] ^ binaryArray[9] ^ binaryArray[7] ^ binaryArray[5] ^ binaryArray[3] ^ binaryArray[1]))
         && (binaryArray[14] == !(binaryArray[12] ^ binaryArray[10] ^ binaryArray[8] ^ binaryArray[6] ^ binaryArray[4] ^ binaryArray[2] ^ binaryArray[0])))
   {
     //we got back a good position, so just mask away the checkbits
     currentPosition &= 0x3FFF;
   }
 else
 {
   currentPosition = 0xFFFF; //bad position
 }
  // currentPosition &= 0x3FFF;
  //If the resolution is 12-bits, and wasn't 0xFFFF, then shift position, otherwise do nothing
  if ((resolution == RES12) && (currentPosition != 0xFFFF)) currentPosition = currentPosition >> 2;
  return currentPosition;
}
void getTurnCounterSPI(int16_t* returnArr, SPI_HandleTypeDef *hspi, GPIO_TypeDef* encoderPort, uint16_t encoderPin, uint8_t resolution, TIM_HandleTypeDef *timer)
{
	uint32_t position_raw, turns_raw;       //raw responses from encoder
	uint8_t binaryArray[16];        //after receiving the position and turn we will populate this array and use it for calculating the checksum
	int16_t position, turns;

	//get first byte of position which is the high byte, shift it 8 bits. don't release line for the first byte. then, get the lower byte and
	// or it with the variable to complete the read
	position_raw = (spiWriteRead(hspi, AMT22_NOP, encoderPort, encoderPin, 0, timer) << 8);
	delay_us_AMT22(AMT22_DELAY);
	position_raw |= (spiWriteRead(hspi, AMT22_TURNS, encoderPort, encoderPin, 0, timer));

	delay_us_AMT22(AMT22_DELAY);

	//same thing with the turn counter
	turns_raw = (spiWriteRead(hspi, AMT22_NOP, encoderPort, encoderPin, 0, timer) << 8);
	delay_us_AMT22(AMT22_DELAY);
	turns_raw |= (spiWriteRead(hspi, AMT22_NOP, encoderPort, encoderPin, 1, timer));

	//run through the 16 bits of position and put each bit into a slot in the array so we can do the checksum calculation
	for(int i = 0; i < 16; i++){binaryArray[i] = (0x01) & (position_raw >> (i));}

	//using the equation on the datasheet we can calculate the checksums and then make sure they match what the encoder sent
	if ((binaryArray[15] == !(binaryArray[13] ^ binaryArray[11] ^ binaryArray[9] ^ binaryArray[7] ^ binaryArray[5] ^ binaryArray[3] ^ binaryArray[1]))
		 && (binaryArray[14] == !(binaryArray[12] ^ binaryArray[10] ^ binaryArray[8] ^ binaryArray[6] ^ binaryArray[4] ^ binaryArray[2] ^ binaryArray[0])))
	{
	 //we got back a good position, so just mask away the checkbits
	 // bitstream &= 0x3FFF;
	 // position = bitstream;
		position = (position_raw & 0x3FFF);
	}
	else{position = 0xFFFF;} // bad position

  //if the resolution is 12-bits, and position wasn't 0xFFFF, then shift position, otherwise do nothing
  if ((resolution == RES12) && (position != 0xFFFF)) position = position >> 2;


  //repeat checksum calculation for the turn counter
  for(int i = 0; i < 16; i++){binaryArray[i] = (0x01) & (turns_raw >> (i));} // TODO check if this is necessary

  //using the equation on the datasheet we can calculate the checksums and then make sure they match what the encoder sent
	if ((binaryArray[15] == !(binaryArray[13] ^ binaryArray[11] ^ binaryArray[9] ^ binaryArray[7] ^ binaryArray[5] ^ binaryArray[3] ^ binaryArray[1]))
		 && (binaryArray[14] == !(binaryArray[12] ^ binaryArray[10] ^ binaryArray[8] ^ binaryArray[6] ^ binaryArray[4] ^ binaryArray[2] ^ binaryArray[0])))
	{
    //we got back a good position, so just mask away the checkbits
    turns_raw &= 0x3FFF;

    // the received 16 bit integer is supposed to be a 14-bit signed integer with two check bits as its
    // msb. once that is masked away, we still have to figure out its sign and extend it.

    // check bit 14
    if(turns_raw & 0x2000){
      // if number is negative, extend the sign by or'ing it with 1100 0000 0000 0000 and complete it to
      // an int16_t
      turns = (0xC000 | turns_raw);
    }else{
      // if number is positive, its 14-bit version will be equal to its int16_t version
      turns = turns_raw;
    }
	}
	else
	{
	  turns = 0xFFFF; //bad position
	}


	// populate return array
	returnArr[0] = position;
	returnArr[1] = turns;

}
uint32_t get_turns_AMT22(SPI_HandleTypeDef *hspi, GPIO_TypeDef* encoderPort, uint16_t encoderPin, uint8_t resolution, TIM_HandleTypeDef *timer)
{
  uint32_t currentTurns = 0;          //32-bit response from encoder
  uint16_t currentPosition;       //16-bit response from encoder
  uint8_t binaryArray[16];        //after receiving the position we will populate this array and use it for calculating the checksum

  //get first byte which is the high byte, shift it 8 bits. don't release line for the first byte
  currentPosition = spiWriteRead(hspi, AMT22_NOP, encoderPort, encoderPin, 0, timer) << 8;

  //this is the time required between bytes as specified in the datasheet.
//  delay(timer, 3);
  delay_us_AMT22(AMT22_DELAY);

  //OR the low byte with the currentPosition variable. release line after second byte
  currentPosition |= spiWriteRead(hspi, AMT22_TURNS, encoderPort, encoderPin, 0, timer);
  delay_us_AMT22(AMT22_DELAY);
  currentTurns = spiWriteRead(hspi, AMT22_NOP, encoderPort, encoderPin, 0, timer)  << 8;
  delay_us_AMT22(AMT22_DELAY);
  currentTurns |= spiWriteRead(hspi, AMT22_NOP, encoderPort, encoderPin, 1, timer);

  //run through the 16 bits of position and put each bit into a slot in the array so we can do the checksum calculation
  for(int i = 0; i < 16; i++) binaryArray[i] = (0x01) & (currentPosition >> (i));

  //using the equation on the datasheet we can calculate the checksums and then make sure they match what the encoder sent
 if ((binaryArray[15] == !(binaryArray[13] ^ binaryArray[11] ^ binaryArray[9] ^ binaryArray[7] ^ binaryArray[5] ^ binaryArray[3] ^ binaryArray[1]))
         && (binaryArray[14] == !(binaryArray[12] ^ binaryArray[10] ^ binaryArray[8] ^ binaryArray[6] ^ binaryArray[4] ^ binaryArray[2] ^ binaryArray[0])))
   {
     //we got back a good position, so just mask away the checkbits
     currentPosition &= 0x3FFF;
   }
 else
 {
   currentPosition = 0xFFFF; //bad position
 }
  // currentPosition &= 0x3FFF;
  //If the resolution is 12-bits, and wasn't 0xFFFF, then shift position, otherwise do nothing
  if ((resolution == RES12) && (currentPosition != 0xFFFF)) currentPosition = currentPosition >> 2;
  currentTurns |= currentPosition << 16;
  return currentTurns;
}

void setZeroSPI(SPI_HandleTypeDef *hspi, GPIO_TypeDef* encoderPort, uint16_t encoderPin, TIM_HandleTypeDef *timer)
{
  spiWriteRead(hspi, AMT22_NOP, encoderPort, encoderPin, 0, timer);

  //There is also a minimum time after clocking that CS should remain asserted before we release it
//  delay(timer, 3);
  delay_us_AMT22(3);

  spiWriteRead(hspi, AMT22_ZERO, encoderPort, encoderPin, 1, timer);


//  delay(timer, 250);
  // delay_us_AMT22(250);
  //power on delay is 200ms
  HAL_Delay(250);
}

void resetAMT22(SPI_HandleTypeDef *hspi, GPIO_TypeDef* encoderPort, uint16_t encoderPin, TIM_HandleTypeDef *timer)
{
  spiWriteRead(hspi, AMT22_NOP, encoderPort, encoderPin, 0, timer);

  //There is also a minimum time after clocking that CS should remain asserted before we release it
//  delay(timer, 3);
  delay_us_AMT22(AMT22_DELAY);

  spiWriteRead(hspi, AMT22_RESET, encoderPort, encoderPin, 1, timer);

//  delay(timer, 250);
  // delay_us_AMT22(250);
  //power on delay is 200ms
  HAL_Delay(250);
}

void delay(TIM_HandleTypeDef *timer, uint32_t delayTime){
	uint32_t startTime = __HAL_TIM_GET_COUNTER(timer); 	//reference point to count passed time
	uint32_t passedTime  = 0;

	while (passedTime<delayTime){
		passedTime = __HAL_TIM_GET_COUNTER(timer) - startTime;
		if (passedTime<0){
			passedTime +=timer->Init.Period;
		}
	}
}
void delay_us_AMT22 (uint16_t us)
{
	__HAL_TIM_SET_COUNTER(&htim1,0);  // set the counter value a 0
	while (__HAL_TIM_GET_COUNTER(&htim1) < us);  // wait for the counter to reach the us input in the parameter
}
