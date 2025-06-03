/*
 * user.h
 *
 * Created: 03/04/2023 
 * Author: user
 */


#ifndef USER_NODE_H_
#define USER_NODE_H_


#define USING_IGUS
#define USING_ATRONIA


//#define USING_BATTERY_CAL					// Define if is used calibration of the battery min and max values 

//	Define type of communications
#define USING_LTE			1					// Using LTE communications
#define USING_2G			2					// Using 2G communications
#define USING_WIFI			3					// Using WiFi communications

#define SSID_LEN			20					// SSID max length
#define ROUTER_PASS_LEN     20					// Router pass max length

#define WIFI_TIMEOUT		20000               // Timeout for WiFi connection attempt

// GPS definitions
#define GPS_MSG_LEN        1024					// Buffer length for GPS message
#define GPS_BAUDRATE	   9600
#define GPS_WAIT_MSG	   2000					// Time delay (ms) to receive GPS message	   

// Hardware defines
#define USING_LOADCELL

// WiFi connection, OTA updates and GPRS APN configuration
#include <Arduino.h>
#include <AutoConnect.h>
#include <WebServer.h>
#include <WiFi.h>
// Change in HTTPClient.h -> #define HTTPCLIENT_DEFAULT_TCP_TIMEOUT (10000)
#include <HTTPClient.h>

//Local peripherals and memory control
#include <esp_sleep.h>
#include <esp_system.h>
#include <Esp.h>
#include <esp_adc_cal.h>
#include <esp_task_wdt.h>
#include <esp_int_wdt.h>
#include <EEPROM.h>
#include <Wire.h>
#include <Adafruit_MCP9808.h>
#include <TimeLib.h>
#include <time.h>
#include <string>
#include <iostream>
#include <algorithm>
#include <ESP32Time.h>
#include <sys/time.h>
#include <MCP79412RTC.h>
#include <PCAL6416A.h>
#include <splash.h>
#include <Adafruit_SSD1306.h>
#include <gfxfont.h>
#include <Adafruit_SPITFT_Macros.h>
#include <Adafruit_SPITFT.h>
#include <Adafruit_GrayOLED.h>
#include <Adafruit_GFX.h>
#include <RHReliableDatagram.h>
#include <RH_RF95.h>
//#include "soc/soc.h"
//#include "soc/rtc_cntl_reg.h"

#ifdef USING_VOC
#include <MICS-VZ-89TE.h>
#endif


// Debug Defines
//#define DEBUG_MODE
//#define DEBUG_ATCMD_MODE
//#define DEBUG_MODBUS
//#define DEBUG_MODBUS_SENT
//#define DEBUG_MODBUS_READ
//#define DEEP_DEBUG_MODE
//#define DEBUG_FORCE_FS
//#define DEBUG_FORCE_FACTORY
//#define DEBUG_TIME_TO_SEND
//#define DEBUG_MODE_MEM
//#define DEBUG_ALARMS
//#define DEBUG_LOADCELL
//#define DEEP_DEBUG_LOADCELL


// State Machine definitions
#define PRINTTEL_TASK_RATE_MS		800
#define TICK_TASK_RATE_MS			5
#define RF_TASK_RATE_MS				130
#define LC_TASK_RATE_MS				100
#define COM_TASK_RATE_MS			275

#define SEC_IN_MS					1000
#define SEC_IN_MIN					60

// IGUS
#define TIMER_OVF					100000
#define MAX_VALID_TIME				2000
#define IGUS_MSG_ID					1
#define IGUS_MSG_ID_LC				2
#define DEFAULT_DIST				200 // Steel
#define DEFAULT_DIST_2				195	// Aluminum

#define PIN_LOW						0
#define PIN_HI						1
#define PIN_NO_STATE				2

#define INVALID_PASS				0
#define SENSOR_1					1
#define SENSOR_2					2
#define PUSH_DIR					SENSOR_1
#define PULL_DIR					SENSOR_2
#define CALCULATION_PASS			3

#define BOUNCE_DELAY				1

#define MODBUS_BAUDRATE			9600		
#define GSM_BAUDRATE			115200
#define DEBUG_BAUDRATE			9600		

//DEFINES I/O 
#define DIG_IN_1				26
#define DIG_IN_2				15

#define RF_NSS					5         
#define RF_RESET				27         
#define RF_INT					19		   

#define VCC_3_EN     			2          // GSM PowerKey

#define SCK_PIN					14
#define MISO_PIN				12
#define MOSI_PIN				13

#define SCL_PIN					22
#define SDA_PIN					21

#define BTN      				4

#define BAT_PLUS_MEASURE		34          

#define VUSB_PLUS_MEASURE       23
#define POUT_ADC				32
#define V_ADC					35
#define PV_PLUS_MEASURE			36
#define MA_ADC					39	

#define UART_TX					25          
#define UART_RX					33

#define RS485_TX_EN				18

#define ESP_TXD_2               17          
#define ESP_RXD_2               16   

// Expander IO definitions
#define GPS_EN                  0
#define DIG_OUT_EN_1            1
#define DIG_OUT_EN_2            2
#define BOOST_EN                3
#define VBAT_2_EN               4
#define POWERKEY                5
#define VCC_2_EN                6
#define INPUT_IV_SEL			11
#define USART_SEL               12
#define LED_B					13
#define LED_G					14
#define LED_R					15

// Peripheral and working flow defines
#ifdef USING_LOADCELL

#define PIGGYBACK_I2C_ADD	0x0A

#define CONFIG_REG			0x00
#define STATE_REG			0x01
#define LC1_LSB0			0x02
#define LC1_MSB0			0x03
#define LC1_LSB1			0x04
#define LC1_MSB1			0x05
#define LC2_LSB0			0x06
#define LC2_MSB0			0x07
#define LC2_LSB1			0x08
#define LC2_MSB1			0x09

// Commands
#define SET_CONFIG			0xAA
#define GET_CONFIG			0xAC
#define GET_STATE			0xAF
#define GET_LC_VAL			0xAE

#define OP_LC1_LC2			0
#define OP_LC1				1
#define OP_LC2				2

#define SYSTEM_STATE_POS	0x01
#define RELAY_STATE_POS		0x02
#define GPS_STATE_POS		0x04
#define W4LC1_STATE_POS		0x08
#define W4LC2_STATE_POS		0x10
#define UNDEFINED_STATE		0x80

#define N_PB_SAMPLES_BUFFER		10	
#define N_PB_MEAN_BUFFER		6	

#endif

#define DEFAULT_GW_UID				0xFFFFFFFFFFFF

#define RF95_FREQ					868.0
#define RF_TX_POWER					20			// Maximum Power
#define RF_ACK_LONG_TIMEOUT			1000		// Timeout to wait for an acknowledge
#define RF_ACK_SHORT_TIMEOUT		500			// Timeout to wait for an acknowledge
#define RF_ANS_LONG_TIMEOUT			1500		// Timeout to wait for an answer after acknowledge
#define RF_ANS_SHORT_TIMEOUT		1000		// Timeout to wait for an answer after acknowledge
#define RF_ANS_VERY_SHORT_TIMEOUT	100			// Timeout to wait for an answer after acknowledge

#define WAKEUP_BUTTON			1
#define WAKEUP_RTC				2

#define OFF						0
#define ON						1
#define DAMAGED_ABSENT			2				// Used for error contact codification	

// ADC defines
#define ADC_RAW_3V2				1780
#define ADC_RAW_4V05			2305
#define ADC_PV_0V5				0
#define ADC_PV_7V				1381

#define ADC_POUT_0V				10
#define ADC_POUT_12V			2390

#define N_SAMPLES				5
#define N_MEAN_SAMPLES			3
#define MED_TO_MEAN_0			1
#define MED_TO_MEAN_1			2
#define MED_TO_MEAN_2			3
#define BAT_MIN_CMD				"*batMin#\r"
#define BAT_MAX_CMD				"*batMax#\r"

// Sensors Errors
#define PIGGYBACK_ERROR				4

// LEDs defines
#define NO_COLOR					0
#define WHITE_CLR					1
#define RED							2
#define GREEN						3
#define BLUE						4
#define VIOLET						5
#define CYAN						6
#define ORANGE						7
#define FULL						8

// Portal
#define PORTAL_TIMER				6

// Radio
#define MAX_MSG_SIZE				64
#define RADIO_BUFFER_SIZE			MAX_MSG_SIZE+32

#define DEFAULT_CLIENT_ADDRESS		254	
#define REPEATER_ADDRESS			200
#define SERVER_ADDRESS				199
#define GW_ADDRESS					SERVER_ADDRESS

// RS485 commands length and positions
#define RS485_BUFFER_SIZE		128

// Defines DS1820
#define MAX_TEMP_DS1820				125.0
#define MIN_TEMP_DS1820				-55.0

/* Loadcells combinations
Channel 1 (2mV/V) 
Channel 2 (2mV/V)
W1 = Vishay Load Cell Model 616 (1t)
W2 = Vishay Load Cell Model 620 (2t)
W3 = ME Messsysteme Load Cell KD40s (2000kg)
D1 = DMS Force Sensor (small form)
D2 = DMS Force Sensor (large form)
*/
#define CH_NO_SENSOR			0
#define CH1_W1					1
#define CH1_W2					2
#define CH1_W3					3
#define CH1_D1					4
#define CH1_D2					5
#define CH2_W1					6
#define CH2_W2					7
#define CH2_W3					8
#define CH2_D1					9
#define CH2_D2					10

// Loadcells variables limits 
#define LC_NEWTONS_MIN			-2000000L
#define LC_NEWTONS_MAX			2000000L
#define LIMIT_COUNTER_MAX		255

// GSM Models
#define GSM_MODEL_QUECTEL_M95		1			// REPRESENTS Quectel M95
#define GSM_MODEL_QUECTEL_BG95M3	2			// REPRESENTS Quectel BG95-M3

#define GSM_MODEL_DEFAULT			GSM_MODEL_QUECTEL_M95

// Models Text
#define QUECTEL_M95					"Quectel_M95"
#define QUECTEL_BG95M3				"BG95-M3"

#define IMEI_SIZE					15
#define ICCID_SIZE					20

/* Result codes */
#define GSM_SUCCESS					0

#define	ERROR_ATE					111
#define	ERROR_MODEL					112
#define	ERROR_REPORT				113
#define ERROR_DISABLE_AUTOGPS		114
#define	ERROR_QOS					115

#define GSM_IMEI_ERROR				115
#define GSM_ICCID_ERROR				116

#define	ERROR_GPRS_ATTACH			2
#define	ERROR_GPRS_CONTEXT			3
#define	ERROR_GPRS_SET_APN			4
#define	ERROR_GPRS_START_TCPIP		5
#define	ERROR_GPRS_ACT_CONTEXT		6
#define	ERROR_GPRS_GET_IP			7
#define ERROR_GPRS_DNS_CONF			71
#define ERROR_WIFI_CON				72
#define	ERROR_GPRS_SET_TIMESERVER	8

#define	ERROR_HTTP_CONTTYPE1		10
#define ERROR_HTTP_CONTTYPE2		11
#define	ERROR_HTTP_SERVER			12

#define	ERROR_HTTP_SET_POST			13
#define	ERROR_HTTP_POST				14
#define	ERROR_HTTP_READ				15

#define	ERROR_NORMAL_MODE			29
#define	ERROR_LOW_CONSUMPTION_MODE  30

#define ERROR_HTTP_TERM				35

// GSM
#define SCAN_AUTO					0
#define SCAN_GSM_ONLY				1
#define SCAN_LTE_ONLY				3


#define TIME_OUT_READ_SERIAL		5000
#define TIME_OUT_AVAILABLE			100				/* Time out between bytes from serial buffer in miliseconds */

#define TIME_TO_SEND_TELEM			360				/* Time between the send of telemetries in minutes */

// EEPROM MAP - SIZE AND CHARACTERISTICS
#define EEPROM_SIZE				0x1000			// Memmory allocation in bytes - for data collection, json frames and telemetry -- 65536 from a total NVS of 131072 bytes

#define UINT8_SIZE				1
#define UINT16_SIZE				2
#define UINT32_SIZE				4
#define FLOAT_SIZE				4
#define NAME_SIZE				5
#define UID_SIZE				6
#define TIMESTAMP_SIZE			6

#define DEP_DATE_SIZE			12
#define CAL_DATE_SIZE			12
#define TELEMETRY_SIZE			86

#define TELEM_DATE_POS			0		// 12 Bytes
#define TELEM_HOUR_POS			12		// 9 Bytes
#define TELEM_S1_POS			21		// 4 Bytes
#define TELEM_S2_POS			25		// 4 Bytes
#define TELEM_S3_POS			29		// 4 Bytes
#define TELEM_S4_POS			33		// 4 Bytes

#define TELEM_ALR_I_POS			37		// 16 Bytes (4 Alarms)
#define TELEM_ALR_S_POS			53		// 16 Bytes (4 Alarms)
#define TELEM_BAT_POS			69		// 1 Byte
#define TELEM_PV_POS			70		// 4 Bytes
#define TELEM_P12V_POS			74		// 4 Bytes
#define TELEM_STP_POS			78		// 4 Bytes
// 82

#define START_ADD					0x00
#define FACTORY_ADD					START_ADD+UINT8_SIZE
#define UID_NODE_ADD				FACTORY_ADD+UINT8_SIZE			// 6 Byte
#define UID_GW_ADD					UID_NODE_ADD+UID_SIZE			// 6 Bytes
#define NODE_NR_ADD					UID_GW_ADD+UID_SIZE				// 1 Bytes
#define SENSOR_TYPE_ADD				NODE_NR_ADD+UINT8_SIZE
#define FS_ADD						SENSOR_TYPE_ADD+UINT8_SIZE
#define REFRESH_ADD					FS_ADD+UINT16_SIZE
#define APN_ADD						REFRESH_ADD+UINT16_SIZE
#define SERVER_ADD					APN_ADD+MAXIMUM_APN_LEN
#define CAL_DATE_ADD				SERVER_ADD+MAXIMUM_SERVER_LEN
#define ALR_I_ADD					CAL_DATE_ADD+CAL_DATE_SIZE
#define ALR_S_ADD					ALR_I_ADD+FLOAT_SIZE*4				// 4 alarms 
#define	ALRSET_IN_ADD				ALR_S_ADD+FLOAT_SIZE*4				// 4 alarms
#define	ALRCNT_ADD					ALRSET_IN_ADD+UINT8_SIZE
#define EXTERN_DS_ADD				ALRCNT_ADD+UINT8_SIZE*4
#define SEND_ERROR_ADD				EXTERN_DS_ADD+UINT8_SIZE
#define IMEI_ADD					SEND_ERROR_ADD+UINT8_SIZE
#define ICCID_ADD					IMEI_ADD+IMEI_SIZE+1
#define INIT_RESET_ADD				ICCID_ADD+ICCID_SIZE+1			//this adress is manually passed to OTA_LIB (portal), if this adress changes, reset flag in OTA updates is not raised
#define BAT_MIN_ADC_ADD				INIT_RESET_ADD+UINT32_SIZE
#define BAT_MAX_ADC_ADD				BAT_MIN_ADC_ADD+UINT16_SIZE
#define ALR_SEND_ORD_ADD			BAT_MAX_ADC_ADD+UINT16_SIZE
#define NEW_VERSION_ADD				ALR_SEND_ORD_ADD+UINT8_SIZE*4
/**********For loadcell**********/ 
#define LC1_SENSOR_ID_ADD			NEW_VERSION_ADD+UINT8_SIZE
#define LC1_ABS_UPPER_LIMIT_ADD		LC1_SENSOR_ID_ADD+UINT8_SIZE
#define LC1_UPPER_FORCE_OVS_ADD		LC1_ABS_UPPER_LIMIT_ADD+UINT32_SIZE
#define LC1_ABS_LOWER_LIMIT_ADD		LC1_UPPER_FORCE_OVS_ADD+UINT32_SIZE
#define LC1_LOWER_FORCE_OVS_ADD		LC1_ABS_LOWER_LIMIT_ADD+UINT32_SIZE
#define LC1_NR_UPPER_FORCE_OVS_ADD	LC1_LOWER_FORCE_OVS_ADD+UINT32_SIZE
#define LC1_NR_LOWER_FORCE_OVS_ADD	LC1_NR_UPPER_FORCE_OVS_ADD+UINT8_SIZE
#define LC1_FORCE_OFFSET_ADD		LC1_NR_LOWER_FORCE_OVS_ADD+UINT8_SIZE
#define LC1_FORCE_CAPACITY_ADD		LC1_FORCE_OFFSET_ADD+UINT32_SIZE
#define LC2_SENSOR_ID_ADD			LC1_FORCE_CAPACITY_ADD+UINT32_SIZE
#define LC2_ABS_UPPER_LIMIT_ADD		LC2_SENSOR_ID_ADD+UINT8_SIZE
#define LC2_UPPER_FORCE_OVS_ADD		LC2_ABS_UPPER_LIMIT_ADD+UINT32_SIZE
#define LC2_ABS_LOWER_LIMIT_ADD		LC2_UPPER_FORCE_OVS_ADD+UINT32_SIZE
#define LC2_LOWER_FORCE_OVS_ADD		LC2_ABS_LOWER_LIMIT_ADD+UINT32_SIZE
#define LC2_NR_UPPER_FORCE_OVS_ADD	LC2_LOWER_FORCE_OVS_ADD+UINT32_SIZE
#define LC2_NR_LOWER_FORCE_OVS_ADD	LC2_NR_UPPER_FORCE_OVS_ADD+UINT8_SIZE
#define LC2_FORCE_OFFSET_ADD		LC2_NR_LOWER_FORCE_OVS_ADD+UINT8_SIZE
#define LC2_FORCE_CAPACITY_ADD		LC2_FORCE_OFFSET_ADD+UINT32_SIZE
#define PB_RELAY_STATE_ADD			LC2_FORCE_CAPACITY_ADD+UINT32_SIZE
/********For wear sensors********/
#define SENSORS_DIST_ADD			PB_RELAY_STATE_ADD+UINT8_SIZE

#define CONN_TYPE_ADD				SENSORS_DIST_ADD+UINT16_SIZE
#define WIFI_SSID_ADD				CONN_TYPE_ADD+UINT8_SIZE
#define WIFI_PASSWORD_ADD			WIFI_SSID_ADD+UINT8_SIZE*SSID_LEN


// Telemetries memory block
#define IND_TELEM_ADD			WIFI_PASSWORD_ADD+UINT8_SIZE*ROUTER_PASS_LEN
#define TELEMETRY_BLOCK_ADD		IND_TELEM_ADD+FLOAT_SIZE

// Till the end of memory: 65512KB

// OLED
#define SCREEN_WIDTH		128		// OLED display width, in pixels
#define SCREEN_HEIGHT		32		// OLED display height, in pixels
#define OLED_RESET			-1		// Reset pin # (or -1 if sharing Arduino reset pin)

#define X_INIT_POS			10
#define Y_INIT_POS			0

#define N_LINES				4
#define MAX_CHARS			19
#define N_PIXELS_LINE		8

// Telemetry subset for EC.IT
#define DIR_POS						S1_POS+4			// 1 Byte
#define DELTA_TIME_POS				DIR_POS+1			// 4 Bytes
#define SPEED_POS					DELTA_TIME_POS+4	// 4 Bytes
#define DISPLACEMENT_POS			SPEED_POS+4			// 4 Bytes
#define BAT_POS						DISPLACEMENT_POS+4	// 1 Byte
#define JUMP_POS					BAT_POS+1			// 1 Byte
#define NAME_POS					JUMP_POS+1			// 5 Bytes
// + 5
// => Total 50 bytes

// Telemetry subset for Piggyback
#define SENSOR_ID_POS_PB					S1_POS+4						// 1 Byte
#define ABS_UPPER_LIMIT_POS_PB				SENSOR_ID_POS_PB+1				// 4 Bytes
#define UPPER_OVERSTEPPINS_POS_PB			ABS_UPPER_LIMIT_POS_PB+4		// 4 Bytes
#define MIN_ACT_FORCE_POS_PB				UPPER_OVERSTEPPINS_POS_PB+4		// 4 Bytes
#define MAX_ACT_FORCE_POS_PB				MIN_ACT_FORCE_POS_PB+4			// 4 Bytes
#define MEAN_ACT_FORCE_POS_PB				MAX_ACT_FORCE_POS_PB+4			// 4 Bytes
#define MEAN_ACT_POS_FORCE_POS_PB			MEAN_ACT_FORCE_POS_PB+4			// 4 Bytes
#define MEAN_ACT_NEG_FORCE_POS_PB			MEAN_ACT_POS_FORCE_POS_PB+4		// 4 Bytes
#define LOWER_OVERSTEPPINS_POS_PB			MEAN_ACT_NEG_FORCE_POS_PB+4		// 4 Bytes
#define ABS_LOWER_LIMIT_POS_PB				LOWER_OVERSTEPPINS_POS_PB+4		// 4 Bytes
#define NR_UPPER_OVERSTEPPINS_POS_PB		ABS_LOWER_LIMIT_POS_PB+4		// 1 Byte
#define NR_LOWER_OVERSTEPPINS_POS_PB		NR_UPPER_OVERSTEPPINS_POS_PB+1	// 1 Byte
#define FORCE_OFFSET_POS_PB					NR_LOWER_OVERSTEPPINS_POS_PB+1	// 4 Bytes
#define ABS_UPPER_LIMIT_COUNT_POS_PB		FORCE_OFFSET_POS_PB+4			// 1 Byte
#define UPPER_OVERSTEPPINS_COUNT_POS_PB		ABS_UPPER_LIMIT_COUNT_POS_PB+1		// 1 Byte
#define ABS_LOWER_LIMIT_COUNTER_POS_PB		UPPER_OVERSTEPPINS_COUNT_POS_PB+1	// 1 Byte
#define LOWER_OVERSTEPPINS_COUNT_POS_PB		ABS_LOWER_LIMIT_COUNTER_POS_PB+1	// 1 Byte
#define RELAY_STATE_POS_PB					LOWER_OVERSTEPPINS_COUNT_POS_PB+1	// 1 Byte
#define BAT_POS_PB							RELAY_STATE_POS_PB+1			// 1 Byte
#define JUMP_POS_PB							BAT_POS_PB+1					// 1 Byte
#define NAME_POS_PB							JUMP_POS_PB+1					// 5 Bytes
// + 5
// => Total 85 bytes


// LoRa Loadcell configurations positions
#define MSG_ID_POS							0									// 1 Byte
#define LC_CONF_UID_POS						MSG_ID_POS+1						// 6 Bytes
#define LC_CONF_SENSOR_ID_POS				LC_CONF_UID_POS+6					// 1 Byte
#define LC_CONF_ABS_UPPER_LIMIT_POS			LC_CONF_SENSOR_ID_POS+1				// 4 Bytes
#define LC_CONF_UPPER_OVERSTEPPINS_POS		LC_CONF_ABS_UPPER_LIMIT_POS+4		// 4 Bytes
#define LC_CONF_LOWER_OVERSTEPPINS_POS		LC_CONF_UPPER_OVERSTEPPINS_POS+4	// 4 Bytes
#define LC_CONF_ABS_LOWER_LIMIT_POS			LC_CONF_LOWER_OVERSTEPPINS_POS+4	// 4 Bytes
#define LC_CONF_NR_UPPER_OVERSTEPPINS_POS	LC_CONF_ABS_LOWER_LIMIT_POS+4		// 1 Byte
#define LC_CONF_NR_LOWER_OVERSTEPPINS_POS	LC_CONF_NR_UPPER_OVERSTEPPINS_POS+1	// 1 Byte
#define LC_CONF_FORCE_OFFSET_POS			LC_CONF_NR_LOWER_OVERSTEPPINS_POS+1	// 4 Bytes
#define LC_CONF_FORCE_CAPACITY_POS			LC_CONF_FORCE_OFFSET_POS+4			// 4 Bytes
#define LC_CONF_RELAY_STATE_POS				LC_CONF_FORCE_CAPACITY_POS+4		// 1 Byte
// + 1
// => Total 35 bytes


// RS485 Buffer
#define LC_RS485_BUFFER_LEN					50
#define LC_CONFIG_UID_POS					4
#define LC_CONFIG_SENSOR_TYPE_POS			11


// Set length of telemetry message according to the type and version of the sensors
#define TELEMETRY_MSG_SIZE			NAME_POS+5
#define TELEMETRY_MSG_SIZE_PB		NAME_POS_PB+5
#define LC_CONF_MSG_SIZE			LC_CONF_RELAY_STATE_POS+1

// Set Date and hour - Data Positions
#define DATE_HOUR_CFG_POS			NODE_NR_POS+1		// 6 Bytes
#define NAME_CFG_POS				NODE_NR_POS+1		// 4 Bytes
#define ALR0_CFG_POS				NODE_NR_POS+1		// 2 Bytes, follows the other alarms (1 and 2) - This position is valid for superior alarm and inferior
#define REFRESH_CFG_POS				NODE_NR_POS+1		// 2 Bytes
//#define RESET_STEPCNT_POS			NODE_NR_POS+1		// No bytes extra needed 

// Error codes for sending messages
#define SENDSTT_ERROR_TOTAL			99
#define SENDSTT_ERROR_1ST_TRY		97
#define SENDSTT_ERROR_2ND_TRY		98
#define SENDSTT_THR_GW				80
#define SENDSTT_THR_REP				70
#define SENDSTT_ACK_1ST_INC			5
#define SENDSTT_ACKCFG_1ST_INC		1
#define SENDSTT_ACK_2ND_INC			6
#define SENDSTT_ACKCFG_2ND_INC		2
#define SENDSTT_ERROR_VAL			90

// Specific Products
#define IGUS_ECIT					90
#define IGUS_ECIT_LOADCELL			91

#if defined( USING_LOADCELL )
#define SENSOR_TYPE					IGUS_ECIT_LOADCELL
#endif

#define ECIT_BUFFER					8

// Display structure
typedef struct
{
	char newLine[MAX_CHARS];
	char text[N_LINES][MAX_CHARS];
	uint8_t line = 0;
	uint8_t overlap = 0;

} DISPLAY_STRUCT;

// System structure
typedef struct
{
	uint8_t rfModemConf = 0;						// Bw125Cr45Sf128 => Configuration for LoRa communication (Bw, Cr, symbols)
	uint8_t rfRepeater = OFF;						// Enable/Disable the use of a repeater	
	
	uint8_t uID[UID_SIZE + 1] = { '\0' };
	uint8_t gWuID[UID_SIZE] = { 0xFF,0xFF,0xFF,0xFF,0xFF,0xFF };
	uint8_t nodeNr = DEFAULT_CLIENT_ADDRESS;

	uint8_t txBuf[RADIO_BUFFER_SIZE] = { 0 };
	uint8_t rxBuf[RADIO_BUFFER_SIZE] = { 0 };

	uint8_t msgID = 0;

	bool preferGateway = true;			//Flag to indicate the preference to the communication - GW or Repeater
	uint8_t serverAddr;
	uint8_t serverAddrBackup;
	uint8_t Jump = 0;					// TTL - Jumps in the trip from node to GW
	uint8_t ResendMsg = 0;				// Number of messages to send, after losing RF connection
	uint8_t arrayInd = 0;
	bool overlap = false;				// Flag to indicate an overflow in telems memory
	bool resending = false;				// Flag to indicate that the systems is sending telems in memory
	bool lock = false;
	uint8_t rf95RSSI;
	bool radioStt = false;
	uint8_t sendStt = 0;				// State of the message sending = status
	bool timeToSend = false;			// Flag to indicate the time to send	
	
	char json[JSON_BUFFER_SIZE];
	uint8_t sendJson;
	int16_t indTelem;
	uint8_t err=0;							// Errors in the communications
	uint8_t sendingError;
	uint8_t init;
	uint8_t gsmStt;
	uint8_t rssi;
	char date[11];
	char hour[6];
	char hhmmss[9];
	char imei[IMEI_SIZE+1];
	char iccid[ICCID_SIZE+1];
	uint16_t batMinADC = 1780;	//0xFFFF;
	uint16_t batMaxADC = 2734;	//0xFFFF;
	uint8_t externSensor;
	char sVA[4][5];
	float sens[4];
	uint8_t digIn;							// 4 bits to have the measures of the 4 inputs
	uint8_t nSensors;
	
	float alrI[4];
	float alrS[4];
	uint8_t alrSetIn[4];					// One variable for each sensor
	uint8_t alrCnt[4];						// To indicate an alarm sent and to indicate that the system isn't in alarm situation	
	uint8_t alrSendOrder[4];				// Flag to force the sending of jsons with the 

	uint8_t bat;
	float pv;
	float p12v;
	uint16_t refresh = 0;
	uint8_t fs;
	char name[5] = { '\0' };				// Used to transmit the FW version
	uint8_t ts[6] = { 0 };
	uint8_t typeSens;
	char typeSensStr[24];
	char calibStr[CAL_DATE_SIZE];
	char coords[25];
	uint8_t gsmModel = GSM_MODEL_DEFAULT;
	char gsmBuff[BUFFER_SIZE];
	uint16_t gsmBuffInd;
	uint16_t smsLen;
	uint8_t userNumLen;
	char apn[MAXIMUM_APN_LEN];
	//char server[MAXIMUM_SERVER_LEN];	
	uint8_t smsType;
	uint8_t factory;
	uint8_t wakeUp;
	uint8_t initAfterReset;
	uint8_t newFW;							// Flag to force write default values when a new version is released and has memory problems
	uint8_t connType;

	uint32_t telemCnt = 0;					// Counter for monitoring the sent of telemetries => it's incremented on each telemetry send/try to send 
	uint32_t telemStp = 0;					// Stamp for each telemetry current or from memory => it's incremented on every telemetry execution

	uint16_t sensorsDistance = DEFAULT_DIST;	// Steel = 200 mm 

} NODE_STRUCT;


typedef struct
{
	uint8_t msgID = 0;					// Msg ID of the configuration to send to the node

	uint8_t gWuID[UID_SIZE] = { 0xFF,0xFF,0xFF,0xFF,0xFF,0xFF };
	uint8_t uID[UID_SIZE] = { 0xFF,0xFF,0xFF,0xFF,0xFF,0xFF };
	uint8_t nodeNr = DEFAULT_CLIENT_ADDRESS;

	uint8_t fs;
	uint16_t refresh;

	uint8_t dateHour[6] = { 0 };												// year/month/day/hh/mm/ss
	char name[5] = { 'N','o','d','e','\0' };									// Sensor's name - 4 characters
	int16_t alrI[3] = { ND_ALARM,ND_ALARM,ND_ALARM };
	int16_t alrS[3] = { ND_ALARM,ND_ALARM,ND_ALARM };

} NODE_TOBE_CONF_STRUCT;


typedef struct
{
	float sens[2];					// Temperature PCB, resistance and step counter
	uint8_t digIn;					// 4 bits to have the measures of the 4 inputs
	uint8_t ts[6] = { 0 };

} TELEM_STRUCT;


#ifdef USING_LOADCELL

typedef struct
{
	uint8_t nr = 0;
	uint8_t gain = 128;
	double lsb = 3000.0 / 16777215.0;	// Vref / (2^24 - 1)
	uint8_t sr = 10;
	bool w4stt = false;

	int32_t valRaw = 0;
	int32_t oldValRaw = 0;
	int32_t sampleBuff[N_PB_SAMPLES_BUFFER] = { 0 };
	int32_t meanBuff[N_PB_MEAN_BUFFER] = { 0 };
	uint8_t sampleBuffIdx = 0;
	int32_t valMean = 0;

	double maxADCmV = 6.0;			// Vexcit = 3V, Sensibility = 2mV/V -> mVVsens = 2 * 3 = 6 mV;		
	double newtons = 0.0;

	uint8_t sensorId = CH_NO_SENSOR;
	int32_t absUpperForceLimit = 99999;
	int32_t upperForceOvesteppings = 99999;
	int32_t absLowerForceLimit = 99999;
	int32_t lowerForceOvesteppings = 99999;
	uint8_t nrUpperForceOversteppings = 1;
	uint8_t nrLowerForceOversteppings = 1;
	int32_t	forceOffset = 0;
	uint32_t Nmax = 10000;	// Maximum weight in N = 1000 Kg * 9.8066

	int32_t newtonsMeanSecAcc = 0;
	int32_t newtonsMeanMinAcc = 0;
	int32_t newtonsPosMeanSecAcc = 0;
	int32_t newtonsPosMeanMinAcc = 0;
	int32_t newtonsNegMeanSecAcc = 0;
	int32_t newtonsNegMeanMinAcc = 0;

	uint16_t secPosSampleCnt = 0;
	uint16_t secNegSampleCnt = 0;
	uint16_t minPosSampleCnt = 0;
	uint16_t minNegSampleCnt = 0;

} LOADCELL_STRUCT;


typedef struct
{
	LOADCELL_STRUCT lc1;
	LOADCELL_STRUCT lc2;
	
	uint16_t secSampleCnt = 0;
	uint16_t minSampleCnt = 0;

	uint8_t relayStt = OFF;
	bool	sendAlert = FALSE;
	uint8_t gpsEn = OFF;

} PIGGYBACK_STRUCT;


typedef struct
{
	uint8_t sensorId = CH1_W1;
	int32_t absUpperForceLimit = 99999;
	int32_t upperForceOvesteppings = 99999;

	// Limits counters - Every 1 second
	uint8_t absUpperLimitCounter = 0;
	uint8_t upperOversteppingsCounter = 0;
	uint8_t absLowerLimitCounter = 0;
	uint8_t lowerOversteppingsCounter = 0;

	// Limits counters - Every 1 minute
	uint8_t absUpperLimitCounterMin = 0;
	uint8_t upperOversteppingsCounterMin = 0;
	uint8_t absLowerLimitCounterMin = 0;
	uint8_t lowerOversteppingsCounterMin = 0;

	// For RS485 - Every 1 second
	int32_t	minActForce = 0;
	int32_t	maxActForce = 0;
	int32_t meanActForce = 0;
	int32_t	meanActPosForce = 0;
	int32_t	meanActNegForce = 0;

	// For LoRa - Every 1 minute
	int32_t	minActForceMin = 0;
	int32_t	maxActForceMin = 0;
	int32_t meanActForceMin = 0;
	int32_t	meanActPosForceMin = 0;
	int32_t	meanActNegForceMin = 0;

	int32_t lowerForceOvesteppings = -99999;
	int32_t absLowerForceLimit = -99999;
	uint8_t nrUpperForceOversteppings = 1;
	uint8_t nrLowerForceOversteppings = 1;
	int32_t	forceOffset = 0;	
	uint8_t relayState = OFF;

}TELEM_ECIT_PB_STRUCT;


typedef struct
{
	int32_t	minActForceSec = 0;
	int32_t	maxActForceSec = 0;
	int32_t	minActForceMin = 0;
	int32_t	maxActForceMin = 0;
	int32_t meanActForceSec = 0;	
	int32_t	meanActPosForceSec = 0;
	int32_t	meanActNegForceSec = 0;
	int32_t meanActForceMin = 0;
	int32_t meanActPosForceMin = 0;
	int32_t meanActNegForceMin = 0;
	volatile uint8_t absUpperLimitCounterSec = 0;
	volatile uint8_t upperOversteppingsCounterSec = 0;
	volatile uint8_t absLowerLimitCounterSec = 0;
	volatile uint8_t lowerOversteppingsCounterSec = 0;
	volatile uint8_t absUpperLimitCounterMin = 0;
	volatile uint8_t upperOversteppingsCounterMin = 0;
	uint8_t maxUpperOversteppingCountMin = 0;
	volatile uint8_t absLowerLimitCounterMin = 0;
	volatile uint8_t lowerOversteppingsCounterMin = 0;
	uint8_t maxLowerOversteppingCountMin = 0;
	volatile uint8_t relayState = OFF;

}DATA_BUFFER_PB_STRUCT;

#endif /* USING_LOADCELL */


typedef struct
{
	uint8_t dir = INVALID_PASS;
	uint32_t deltaSampleTime = 0;
	float sampleSpeed = 0.0;	
	float displacement = 0.0;

#ifdef USING_LOADCELL

	TELEM_ECIT_PB_STRUCT telemEcitLc1;
	TELEM_ECIT_PB_STRUCT telemEcitLc2;

#endif

}TELEM_ECIT_STRUCT;


typedef struct
{
	uint8_t dir = INVALID_PASS;
	uint32_t deltaTimer = 0;
	float speed = 0.0;
	uint32_t sampleTime = 0;

}DATA_BUFFER;


// Methods declaration
void IRAM_ATTR btnISR(void);
void IRAM_ATTR btn2ISR(void);
void PinSetup(void);
bool SYS_ConfExpIO(void);
void SetupDisplay(void);
void ShutDownDisplay(void);
void DisplaySplashScreen(void);
int8_t ConnectWifi(char* ssid, char* password);
int8_t CheckWifiConnection(void);
bool WiFi_GetTime(char* date, char* hr, char* hhmmss);
void ClearGPSBuffer(void);
void Read_GPS(void);
void Enable_GPS(bool action);
void Enable_DIG_OUT_1(bool action);
void Enable_DIG_OUT_2(bool action);
void Enable_BOOST(bool action);
void Enable_VBAT_2(bool action);
void Enable_VCC_2(bool action);
void Enable_VCC_3(bool action);
void Enable_POWERKEY(bool action);
void USART2_SelectSource(uint8_t source);
void Sys_ResetVerification(void);
uint16_t GetADCMeas(uint8_t pin);
void Calib_BatMeasure(void);
void Set_Color(uint8_t color);
void Blink_LED(uint8_t color, uint16_t len);
void ShortLEDblink_LED(uint8_t color);
uint8_t Init_Sensors(void);
uint8_t SetConfigReg(void);
uint8_t Init_Loadcell_PiggyBack(uint8_t i2cAddress);
uint8_t Get_LCvals(uint8_t i2cAddress, uint8_t op);
void GetLCmeanRaw(LOADCELL_STRUCT* lc);
double Calc_Newtons(LOADCELL_STRUCT* lc);
void LoadcellsAggregations(void);
void CheckLoadcellsAlerts(double measLc1, double measLc2);
void Set_Relay(void);
uint8_t CheckRelayState(uint8_t generalState);
uint8_t CheckLoadCellFail(uint8_t generalState2);
uint8_t CheckState(void);
uint8_t SetLcConfigs(char* buffer);
uint8_t Set_LcConfigs_Rf(uint8_t* buffer);
bool RF_Config(void);
void Radio_TxRx(NODE_STRUCT* node, NODE_TOBE_CONF_STRUCT* nodeWithCfg, TELEM_STRUCT* mem, uint8_t frameType, TELEM_ECIT_STRUCT* telem);
uint8_t FindAlarmsConf(char* buffer, uint16_t buffSize, float* alrAns);
uint8_t parseJSONResponse(char* buffer, uint16_t bufferSize, char* fsAns, char* refAns, float* aIans, float* aSans);
uint8_t GSM_SetScanMode(uint8_t mode);
uint8_t GSM_GetCellsAround(void);
uint8_t GSM_GetTime(char* date, char* hour, char* hhmmss);
void GSM_TurnON(void);
void GSM_TurnOFF(void);
uint8_t GSM_SetEchoOff(void);
uint8_t GSM_GetModel(uint8_t* model);
uint8_t  GSM_ReadSerial(uint32_t timeout, char* ans);
int16_t atrIndexOf(char* buffer, uint16_t buffSize, char* cmp, uint16_t bytes2cmp);
int16_t atrIndexOf(char* buffer, uint16_t iInd, uint16_t buffSize, char* cmp, uint16_t bytes2cmp);
uint8_t GSM_GetICCID(char* str);
uint8_t GSM_GetOperator(void);
uint8_t GSM_Init(void);
uint8_t GSM_WaitOperator(uint16_t tries);
void sendCmd(const String cmd);
uint8_t sendCmdAndWaitForResp(const String cmd, char* resp, unsigned timeout);
uint8_t waitForResp(char* resp, unsigned int timeout);
void cleanBuffer(char* buffer, int count);
uint8_t GSM_LteConnect(char* apnOP);
uint8_t GSM_GprsConnect(char* apnOP);
uint8_t GSM_GetLocalIP(uint8_t model);
uint8_t GSM_GetIMEI(char* imei);
int16_t GSM_GetRSSI(void);
uint8_t GSM_GetLoc(char* loc);
void ADC_Measures(void);
void BatMeasure(void);
void GSM_DisableSerial(void);
void RS485_DisableSerial(void);
void Print_WakeupReason(esp_sleep_wakeup_cause_t reason);
void Set_DefaultVals(void);
void EEPROM_SaveConfigs(void);
void EEPROM_ReadConfigs(void);
void EEPROM_SaveTelem(NODE_STRUCT* sysPt);
bool SYS_UpdateRTC(char* date, char* hhmmss);
void SYS_GetTimeRTC(char* date, char* hour, char* hhmmss);
double fmap(double x, double x1, double x2, double y1, double y2);
void sortInt(uint16_t* buff, uint8_t nSamples);
uint16_t meanInt(uint16_t* buffer, uint8_t nSamples);
void sortI32(int32_t* buff, uint8_t nSamples);
int32_t meanI32(int32_t* buffer, uint8_t nSamples);
double meanF(double* buffer, uint8_t nSamples);
void AddLine(char* line, uint8_t clear);
void SetLine(char* line);
void DisplayLines(void);
float CalculateDisplacement(void);

#endif /* USER_NODE_H_ */

