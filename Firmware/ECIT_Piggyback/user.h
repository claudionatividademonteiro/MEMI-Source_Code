/*
 * user.h
 *
 * Created: 09/01/2024 18:02:05
 *  Author: user
 */ 


#ifndef USER_H_
#define USER_H_


#include <Wire.h>

#include <HX711.h>
#include <TimerOne.h>
#include <I2C_Slave.h>


#define DEBUG_MODE
//#define DEBUG_I2C


//DEFINES I/O
#define HX711_1_DOUT			3			// Load cell pins for HX_1
#define HX711_1_SCK				2	
#define HX711_1_RATE			4			// Pin to choose sample rate for HX_1 (1=80Hz, 0=10Hz)
#define HX_1_4WIRE				A0			// Detection of loadcells of 4-wires or 6-wires

#define HX711_2_DOUT			6			// Load cell pins for HX_2
#define HX711_2_SCK				5			
#define HX711_2_RATE			7			// Pin to choose sample rate for HX_2 (1=80Hz, 0=10Hz)
#define HX_2_4WIRE				A1			// Detection of loadcells of 4-wires or 6-wires

#define RELAY_PIN				A2

#define GPS_EN					8

// States
#define STT_SETUP				0
#define STT_LOOP				1

// I2C
#define I2C_SLAVE_ADD		0x0A			// unused address (https://i2cdevices.org/addresses)

#define CONFIG_POS			0x00
#define STATE_POS			0x01
#define LC1_LSB0_POS		0x02
#define LC1_MSB0_POS		0x03
#define LC1_LSB1_POS		0x04
#define LC1_MSB1_POS		0x05
#define LC2_LSB0_POS		0x06
#define LC2_MSB0_POS		0x07
#define LC2_LSB1_POS		0x08
#define LC2_MSB1_POS		0x09

#define N_REGS				10

#define LC1_GAIN_MASK		0x01
#define LC1_RATE_MASK		0x02
#define LC2_GAIN_MASK		0x04
#define LC2_RATE_MASK		0x08
#define RELAY_STT_MASK		0x10
#define GPS_EN_MASK			0x20

// Commands
#define SET_CONFIG			0xAA
#define GET_CONFIG			0xAC
#define GET_STATE			0xAF
#define GET_LC_VAL			0xAE

#define OP_LC1_LC2			0
#define OP_LC1				1
#define OP_LC2				2


// system structure
typedef struct
{
	uint8_t factory = 0xFF;
	
	uint8_t time2sample = 0;
	uint8_t cnt2Sample10hz = 0;
	uint8_t cnt1s = 0;
	
	uint8_t state = STT_SETUP;
	uint8_t sttResume = 0;
	uint8_t confResume = 0;
	uint8_t relayStt = 0;
	uint8_t gpsEnable = 0;
	
	uint8_t regToProcess = 0;

} SYS_STRUCT;


typedef struct
{	
	uint8_t rxBuff[8] = { 0 };
	uint8_t rxBuffI = 0;
	uint8_t txBuff[32] = { 0 };
	uint8_t txBuffI = 0;

} I2C_STRUCT;


// Load cells processing structure
typedef struct
{
	uint8_t nr = 0;
	uint8_t ratePin = 0;
	uint8_t gain = 128;
	uint8_t rate = 80;
	uint8_t w4 = 1;
		
	int32_t rawVal = 0;
	int32_t oldRawVal = 0;
	
	uint8_t sps = 0;

} LC_STRUCT;


#endif /* USER_H_ */

