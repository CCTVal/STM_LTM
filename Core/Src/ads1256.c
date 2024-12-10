/*
 * ads1256.c
 *
 *  Created on: Dec 4, 2024
 *      Author: Vicente Saona Urmeneta <vicente.saona@usm.cl>
 */

#include <math.h>
#include "ads1256.h"

//Register-related variables
uint8_t _registerAddress; //Value holding the address of the register we want to manipulate
uint8_t _registerValueToWrite; //Value to be written on a selected register
uint8_t _registerValuetoRead; //Value read from the selected register

//Register values
uint8_t _DRATE; //Value of the DRATE register
uint8_t _ADCON; //Value of the ADCON register
uint8_t _MUX; //Value of the MUX register
uint8_t _PGA; //Value of the PGA (within ADCON)
uint8_t _GPIO; //Value of the GPIO register
uint8_t _STATUS; //Value of the status register
uint8_t _GPIOvalue; //GPIO value
uint8_t _ByteOrder; //Byte order

uint8_t _outputBuffer[3]; //3-byte (24-bit) buffer for the fast acquisition - Single-channel, continuous
long _outputValue; //Combined value of the _outputBuffer[3]
uint8_t _isAcquisitionRunning; //bool that keeps track of the acquisition (running or not)
uint8_t _cycle; //Tracks the cycles as the MUX is cycling through the input channels

#define convertSigned24BitToLong(value) ((value) & (1l << 23) ? (value) - 0x1000000 : value)

ADS125X_t ADS1256_InitializeADC(SPI_HandleTypeDef *hspix, uint16_t DRDY_pin, GPIO_TypeDef *DRDY_port,
		                        uint16_t SYNC_pin, GPIO_TypeDef *SYNC_port,
								uint16_t CS_pin, GPIO_TypeDef *CS_port, float VREF)
{
  ADS125X_t adc;
  adc.hspix = hspix;

  adc.drdyPin = DRDY_pin; // pinMode(DRDY_pin, INPUT);
  adc.drdyPort = DRDY_port;

//  if(RESET_pin !=0) {
//	adc.resetPin = RESET_pin; // pinMode(RESET_pin, OUTPUT);
//	adc.resetPort = RESET_port;
//  }

  if(SYNC_pin != 0) {
    adc.syncPin = SYNC_pin; // pinMode(_SYNC_pin, OUTPUT);
    adc.syncPort = SYNC_port;
  }

  adc.csPin = CS_pin; // pinMode(_CS_pin, OUTPUT);
  adc.csPort = CS_port;

  adc.vref = ADS1256_VREF;
  adc.pga = 0;

  HAL_GPIO_WritePin(adc.csPort, adc.csPin, 0);
//  if(RESET_pin !=0) {
//    HAL_GPIO_WritePin(adc.resetPort, adc.resetPin, 0);
//    HAL_Delay(200);
//    HAL_GPIO_WritePin(adc.resetPort, adc.resetPin, 1);
//    HAL_Delay(1000);
//  }
  if(SYNC_pin !=0) {
    HAL_GPIO_WritePin(adc.syncPort, adc.syncPin, 1);
  }
  // SPI.begin();
  HAL_Delay(200);

  ADS1256_writeRegister(&adc, ADS1256_STATUS_REG, 0b00110110); // BUFEN and ACAL enabled, Order is MSB, rest is read only
  HAL_Delay(200);
  ADS1256_writeRegister(&adc, ADS1256_MUX_REG, 0b00000001); // AIN0 - AIN1
  HAL_Delay(200);
  ADS1256_writeRegister(&adc, ADS1256_ADCON_REG, 0b00000000); // ADCON - CLK: OFF, SDCS: OFF, PGA = 0 (+/- 5 V)
  HAL_Delay(200);
  ADS1256_writeRegister(&adc, ADS1256_DRATE_REG, 0b10000010); // 100SPS
  HAL_Delay(200);
  ADS1256_sendDirectCommand(&adc, 0b11110000); // Offset and self-gain calibration
  HAL_Delay(200);

  // _isAcquisitionRunning = false;
  return(adc);
}

void waitForDRDY(ADS125X_t* adc)
{
  while(HAL_GPIO_ReadPin(adc->drdyPort, adc->drdyPin));
}

void stopConversion(ADS125X_t* adc)
{
  waitForDRDY(adc);
  uint8_t sdatac = 0b00001111;
  HAL_SPI_Transmit(adc->hspix, &sdatac, 1, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(adc->csPort, adc->csPin, 1);
  // SPI.endTransaction();

  // _isAcquisitionRunning = false;
}

//Read a register
long ADS1256_readRegister(ADS125X_t* adc, uint8_t registerAddress)
{
  waitForDRDY(adc);
  // SPI.beginTransaction(SPISettings(1920000, MSBFIRST, SPI_MODE1));
  // //SPI_MODE1 = output edge: rising, data capture: falling; clock polarity: 0, clock phase: 1.
  HAL_GPIO_WritePin(adc->csPort, adc->csPin, 0);
  uint8_t data = 0x10 | registerAddress;  // 0x10 = 0001000 = RREG - OR together the two numbers (command + address)
  HAL_SPI_Transmit(adc->hspix, &data, 1, HAL_MAX_DELAY);
  data = 0x00;
  HAL_SPI_Transmit(adc->hspix, &data, 1, HAL_MAX_DELAY); // 2nd (empty) command byte
  HAL_Delay(5); // see t6 in the datasheet
  HAL_SPI_Receive(adc->hspix, &data, 1, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(adc->csPort, adc->csPin, 1);
  // SPI.endTransaction();
  HAL_Delay(100);
  return(data);
}

//Write a register
void ADS1256_writeRegister(ADS125X_t* adc, uint8_t registerAddress, uint8_t registerValueToWrite)
{
  waitForDRDY(adc);
  // SPI.beginTransaction(SPISettings(1920000, MSBFIRST, SPI_MODE1));
  // //SPI_MODE1 = output edge: rising, data capture: falling; clock polarity: 0, clock phase: 1.
  HAL_GPIO_WritePin(adc->csPort, adc->csPin, 0);
  HAL_Delay(5);
  uint8_t data = 0x50 | registerAddress;
  HAL_SPI_Transmit(adc->hspix, &data, 1, HAL_MAX_DELAY);
  data = 0x00;
  HAL_SPI_Transmit(adc->hspix, &data, 1, HAL_MAX_DELAY);
  data = registerValueToWrite;
  HAL_SPI_Transmit(adc->hspix, &data, 1, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(adc->csPort, adc->csPin, 1);
  // SPI.endTransaction();
  HAL_Delay(100);
}

//Individual methods
void ADS1256_setDRATE(ADS125X_t* adc, uint8_t drate)
{
  ADS1256_writeRegister(adc, ADS1256_DRATE_REG, drate);
  // _DRATE = drate;
  HAL_Delay(200);
}

void ADS1256_setPGA(ADS125X_t* adc, uint8_t pga)
{
  adc->pga = pga;
  long adcon = ADS1256_readRegister(adc, ADS1256_ADCON_REG); //Read the most recent value of the register
  adcon = (adcon & 0b11111000) | (pga & 0b00000111); // Clearing and then setting bits 2-0 based on pga
  ADS1256_writeRegister(adc, ADS1256_ADCON_REG, adcon);
  HAL_Delay(200);
}
uint8_t ADS1256_getPGA(ADS125X_t* adc)
{
  uint8_t pgaValue = 0;
  pgaValue = ADS1256_readRegister(adc, ADS1256_ADCON_REG) & 0b00000111; //Reading the ADCON_REG and keeping the first three bits.
  return(pgaValue);
}
void ADS1256_setMUX(ADS125X_t* adc, uint8_t mux)
{
	ADS1256_writeRegister(adc, ADS1256_MUX_REG, mux);
	// _MUX = mux;
	HAL_Delay(200);
}
void ADS1256_setByteOrder(ADS125X_t* adc, uint8_t byteOrder)
{
  long status = ADS1256_readRegister(adc, ADS1256_STATUS_REG); //Read the most recent value of the register
  if(byteOrder == 0 || byteOrder == 1)
    status = (status & 0b11110111) | (byteOrder << 3);
  ADS1256_writeRegister(adc, ADS1256_STATUS_REG, status);
  HAL_Delay(100);
}

// TODO: Erase this, only works in Arduino testing context
void ADS1256_getByteOrder(ADS125X_t* adc)
{
  return;
}
void ADS1256_setBuffer(ADS125X_t* adc, uint8_t bufen)
{
  long status = ADS1256_readRegister(adc, ADS1256_STATUS_REG); //Read the most recent value of the register
  if(bufen == 0 || bufen == 1)
    status = (status & 0b11111101) | (bufen << 1);
  ADS1256_writeRegister(adc, ADS1256_STATUS_REG, status);
  HAL_Delay(100);
}

// TODO: Erase this. It only works in Arduino testing context.
void ADS1256_getBuffer(ADS125X_t* adc)
{
  return;
}
void ADS1256_setAutoCal(ADS125X_t* adc, uint8_t acal)
{
  long status = ADS1256_readRegister(adc, ADS1256_STATUS_REG); //Read the most recent value of the register
  if(acal == 0 || acal == 1)
    status = (status & 0b11111011) | (acal << 2);
  ADS1256_writeRegister(adc, ADS1256_STATUS_REG, status);
  HAL_Delay(100);
}

// TODO: Erase this, only works in Arduino testing context
void ADS1256_getAutoCal(ADS125X_t* adc)
{
  return;
}
void ADS1256_setGPIO(ADS125X_t* adc, uint8_t dir0, uint8_t dir1, uint8_t dir2, uint8_t dir3)
{
  long gpio = ADS1256_readRegister(adc, ADS1256_IO_REG); //Read the most recent value of the register
  gpio = (gpio & 0b01111111) | (dir3 << 7);
  gpio = (gpio & 0b10111111) | (dir2 << 6);
  gpio = (gpio & 0b11011111) | (dir1 << 5);
  gpio = (gpio & 0b11101111) | (dir0 << 4);
  ADS1256_writeRegister(adc, ADS1256_IO_REG, gpio);
  HAL_Delay(100);
}
void ADS1256_writeGPIO(ADS125X_t* adc, uint8_t dir0value, uint8_t dir1value, uint8_t dir2value, uint8_t dir3value)
{
  long gpio = ADS1256_readRegister(adc, ADS1256_IO_REG); //Read the most recent value of the register
  gpio = (gpio & 0b11110111) | (dir3value << 3);
  gpio = (gpio & 0b11111011) | (dir2value << 2);
  gpio = (gpio & 0b11111101) | (dir1value << 1);
  gpio = (gpio & 0b11111110) | (dir0value << 0);
  ADS1256_writeRegister(adc, ADS1256_IO_REG, gpio);
  HAL_Delay(100);
}
uint8_t ADS1256_readGPIO(ADS125X_t* adc, uint8_t gpioPin)
{
  long gpio = ADS1256_readRegister(adc, ADS1256_IO_REG); //Read the GPIO register
  return((gpio >> gpioPin) & 0x01);
}
void ADS1256_setCLKOUT(ADS125X_t* adc, uint8_t clkout)
{
  long adcon = ADS1256_readRegister(adc, ADS1256_ADCON_REG); //Read the most recent value of the register
  if(clkout == 0 || clkout == 1 || clkout == 2 || clkout == 3)
    adcon = (adcon & 0b10011111) | (clkout << 5);
  ADS1256_writeRegister(adc, ADS1256_ADCON_REG, adcon);
  HAL_Delay(100);
}
void ADS1256_setSDCS(ADS125X_t* adc, uint8_t sdcs)
{
  long adcon = ADS1256_readRegister(adc, ADS1256_ADCON_REG); //Read the most recent value of the register
  if(sdcs == 0 || sdcs == 1 || sdcs == 2 || sdcs == 3)
    adcon = (adcon & 0b11100111) | (sdcs << 3);
  ADS1256_writeRegister(adc, ADS1256_ADCON_REG, adcon);
  HAL_Delay(100);
}
void ADS1256_sendDirectCommand(ADS125X_t* adc, uint8_t directCommand)
{
  // SPI.beginTransaction(SPISettings(1920000, MSBFIRST, SPI_MODE1));
  HAL_GPIO_WritePin(adc->csPort, adc->csPin, 0);
  HAL_Delay(5);
  HAL_SPI_Transmit(adc->hspix, &directCommand, 1, HAL_MAX_DELAY);
  HAL_Delay(5);
  HAL_GPIO_WritePin(adc->csPort, adc->csPin, 1);
  // SPI.endTransaction();
}

//Get a single conversion
long ADS1256_readSingle(ADS125X_t* adc)
{
  // SPI.beginTransaction(SPISettings(1920000, MSBFIRST, SPI_MODE1));
  HAL_GPIO_WritePin(adc->csPort, adc->csPin, 0); // REF: P34: "CS must stay low during the entire command sequence"
  waitForDRDY(adc);
  uint8_t command = 0b00000001;
  HAL_SPI_Transmit(adc->hspix, &command, 1, HAL_MAX_DELAY); // Issue RDATA (0000 0001) command
  HAL_Delay(7); // Wait t6 time (~6.51 us) REF: P34, FIG:30.
  uint8_t data[3];
  HAL_SPI_Receive(adc->hspix, data, 3, HAL_MAX_DELAY);
  long outputValue = 0;
  outputValue = ((long)data[0]<<16) | ((long)data[1]<<8) | (data[2]);
  outputValue = convertSigned24BitToLong(outputValue);
  HAL_GPIO_WritePin(adc->csPort, adc->csPin, 1); // We finished the command sequence, so we set CS to HIGH
  // SPI:endTransaction();
  return(outputValue);
}

//Single input continuous reading
long ADS1256_readSingleContinuous(ADS125X_t* adc)
{
  // SPI.beginTransaction(SPISettings(1920000, MSBFIRST, SPI_MODE1));
  HAL_GPIO_WritePin(adc->csPort, adc->csPin, 0); // REF: P34: "CS must stay low during the entire command sequence"
  waitForDRDY(adc);
  uint8_t command = 0b00000011; //Issue RDATAC (0000 0011)
  HAL_SPI_Transmit(adc->hspix, &command, 1, HAL_MAX_DELAY);
  HAL_Delay(7);

  uint8_t data[3];
  HAL_SPI_Receive(adc->hspix, data, 3, HAL_MAX_DELAY);
  long outputValue = 0;
  outputValue = ((long)data[0]<<16) | ((long)data[1]<<8) | (data[2]);
  outputValue = convertSigned24BitToLong(outputValue);

  return(outputValue);
}

//Cycling through the single-ended inputs
long ADS1256_cycleSingle(ADS125X_t* adc)
{
  // update MUX
  static uint8_t channel = 0;

  ADS1256_setMUX(adc, channel);

  uint8_t command = 0b11111100; //SYNC
  HAL_SPI_Transmit(adc->hspix, &command, 1, HAL_MAX_DELAY);
  HAL_Delay(4);
  command = 0b11111111; //WAKEUP
  HAL_SPI_Transmit(adc->hspix, &command, 1, HAL_MAX_DELAY);
  channel = (channel + 1) % 8;
  return(ADS1256_readSingle(adc));
}

//Cycling through the differential inputs
long ADS1256_cycleDifferential(ADS125X_t* adc); //Ax + Ay

//Converts the reading into a voltage value
float ADS1256_convertToVoltage(ADS125X_t* adc, int32_t rawData)
{
  //return(((2 * adc->vref) / 8388608) * rawData / (pow(2, adc->pga)));
  return(((2 * adc->vref) / 8388607) * rawData / (pow(2, adc->pga)));
}

//Stop AD
void ADS1256_stopConversion(ADS125X_t* adc);
