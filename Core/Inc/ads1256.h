/*
 * ads1256.h
 *
 *  Created on: Dec 4, 2024
 *      Author: Vicente Saona Urmeneta <vicente.saona@usm.cl>
 */

#ifndef INC_ADS1256_H_
#define INC_ADS1256_H_

#include <stdio.h>
#include "main.h"

#define ADS1256_VREF (5.0f)
#define ADS1256_OSC_FREQ (7680000)

//PGA settings			  //Input voltage range
#define ADS1256_PGA_1 0b00000000  //± 5 V
#define ADS1256_PGA_2 0b00000001  //± 2.5 V
#define ADS1256_PGA_4 0b00000010  //± 1.25 V
#define ADS1256_PGA_8 0b00000011  //± 625 mV
#define ADS1256_PGA_16 0b00000100 //± 312.5 mV
#define ADS1256_PGA_32 0b00000101 //+ 156.25 mV
#define ADS1256_PGA_64 0b00000110 //± 78.125 mV

//Datarate						  //DEC
#define ADS1256_DRATE_30000SPS 0b11110000 //240
#define ADS1256_DRATE_15000SPS 0b11100000 //224
#define ADS1256_DRATE_7500SPS 0b11010000  //208
#define ADS1256_DRATE_3750SPS 0b11000000  //192
#define ADS1256_DRATE_2000SPS 0b10110000  //176
#define ADS1256_DRATE_1000SPS 0b10100001  //161
#define ADS1256_DRATE_500SPS 0b10010010   //146
#define ADS1256_DRATE_100SPS 0b10000010   //130
#define ADS1256_DRATE_60SPS 0b01110010    //114
#define ADS1256_DRATE_50SPS 0b01100011    //99
#define ADS1256_DRATE_30SPS 0b01010011    //83
#define ADS1256_DRATE_25SPS 0b01000011    //67
#define ADS1256_DRATE_15SPS 0b00110011    //51
#define ADS1256_DRATE_10SPS 0b00100011    //35
#define ADS1256_DRATE_5SPS 0b00010011     //19
#define ADS1256_DRATE_2SPS 0b00000011     //3

//Status register
#define ADS1256_BITORDER_MSB 0
#define ADS1256_BITORDER_LSB 1
#define ADS1256_ACAL_DISABLED 0
#define ADS1256_ACAL_ENABLED 1
#define ADS1256_BUFFER_DISABLED 0
#define ADS1256_BUFFER_ENABLED 1

//Register addresses
#define ADS1256_STATUS_REG 0x00
#define ADS1256_MUX_REG 0x01
#define ADS1256_ADCON_REG 0x02
#define ADS1256_DRATE_REG 0x03
#define ADS1256_IO_REG 0x04
#define ADS1256_OFC0_REG 0x05
#define ADS1256_OFC1_REG 0x06
#define ADS1256_OFC2_REG 0x07
#define ADS1256_FSC0_REG 0x08
#define ADS1256_FSC1_REG 0x09
#define ADS1256_FSC2_REG 0x0A

//Command definitions
#define ADS1256_WAKEUP 0b00000000
#define ADS1256_RDATA 0b00000001
#define ADS1256_RDATAC 0b00000011
#define ADS1256_SDATAC 0b00001111
#define ADS1256_RREG 0b00010000
#define ADS1256_WREG 0b01010000
#define ADS1256_SELFCAL 0b11110000
#define ADS1256_SELFOCAL 0b11110001
#define ADS1256_SELFGCAL 0b11110010
#define ADS1256_SYSOCAL 0b11110011
#define ADS1256_SYSGCAL 0b11110100
#define ADS1256_SYNC 0b11111100
#define ADS1256_STANDBY 0b11111101
#define ADS1256_RESET 0b11111110

// Struct "Object"
typedef struct {
    SPI_HandleTypeDef *hspix;
    float vref;
    uint8_t pga;
    float convFactor;
    uint32_t oscFreq;
    GPIO_TypeDef *csPort;
    uint16_t     csPin;
    GPIO_TypeDef *drdyPort;
    uint16_t     drdyPin;
    GPIO_TypeDef *syncPort;
    uint16_t     syncPin;
    GPIO_TypeDef *resetPort;
	uint16_t     resetPin;
} ADS125X_t;

//Constructor
ADS125X_t ADS1256_InitializeADC(SPI_HandleTypeDef *hspix,
		                        uint16_t DRDY_pin, GPIO_TypeDef *DRDY_port,
								uint16_t SYNC_pin, GPIO_TypeDef *SYNC_port,
								uint16_t CS_pin, GPIO_TypeDef *CS_port,
								float VREF);

//Read a register
long ADS1256_readRegister(ADS125X_t* adc, uint8_t registerAddress);

//Write a register
void ADS1256_writeRegister(ADS125X_t* adc, uint8_t registerAddress, uint8_t registerValueToWrite);

//Individual methods
void ADS1256_setDRATE(ADS125X_t* adc, uint8_t drate);
void ADS1256_setPGA(ADS125X_t* adc, uint8_t pga);
uint8_t ADS1256_getPGA(ADS125X_t* adc);
void ADS1256_setMUX(ADS125X_t* adc, uint8_t mux);
void ADS1256_setByteOrder(ADS125X_t* adc, uint8_t byteOrder);
void ADS1256_getByteOrder(ADS125X_t* adc);
void ADS1256_setBuffer(ADS125X_t* adc, uint8_t bufen);
void ADS1256_getBuffer(ADS125X_t* adc);
void ADS1256_setAutoCal(ADS125X_t* adc, uint8_t acal);
void ADS1256_getAutoCal(ADS125X_t* adc);
void ADS1256_setGPIO(ADS125X_t* adc, uint8_t dir0, uint8_t dir1, uint8_t dir2, uint8_t dir3);
void ADS1256_writeGPIO(ADS125X_t* adc, uint8_t dir0value, uint8_t dir1value, uint8_t dir2value, uint8_t dir3value);
uint8_t ADS1256_readGPIO(ADS125X_t* adc, uint8_t gpioPin);
void ADS1256_setCLKOUT(ADS125X_t* adc, uint8_t clkout);
void ADS1256_setSDCS(ADS125X_t* adc, uint8_t sdcs);
void ADS1256_sendDirectCommand(ADS125X_t* adc, uint8_t directCommand);

//Get a single conversion
long ADS1256_readSingle(ADS125X_t* adc);

//Single input continuous reading
long ADS1256_readSingleContinuous(ADS125X_t* adc);

//Cycling through the single-ended inputs
long ADS1256_cycleSingle(ADS125X_t* adc); //Ax + COM

//Cycling through the differential inputs
long ADS1256_cycleDifferential(ADS125X_t* adc); //Ax + Ay

//Converts the reading into a voltage value
float ADS1256_convertToVoltage(ADS125X_t* adc, int32_t rawData);

//Stop AD
void ADS1256_stopConversion(ADS125X_t* adc);

#endif /* INC_ADS1256_H_ */
