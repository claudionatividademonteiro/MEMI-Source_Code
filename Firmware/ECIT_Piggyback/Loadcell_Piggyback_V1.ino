
#include "user.h"


FUSES =
{
	.low = 0xE2,
	.high = 0xD1,
	.extended = 0xFD,
};


char fwVer[] = "1.10";
char hwVer[] = "1.00";

// System variables
SYS_STRUCT sys;

I2C_STRUCT i2c;

LC_STRUCT lc1Proc;
LC_STRUCT lc2Proc;

// Objects constructors

// Load cells
HX711 lc1;
HX711 lc2;


// callback to handle I2C commands upon receipt
void command_handler(uint8_t command, uint8_t value) 
{
	//uint8_t nBytes;
	//int32_t testBuff = 16909061;
	
	switch (command) 
	{
		case SET_CONFIG:
			if (value < 0x40)
			{	
				sys.confResume = value;
				ProcessConfigReg(sys.confResume);
#ifdef DEBUG_I2C				
				Serial.printf("Set configs to: 0x%02X\n", sys.confResume);
#endif				
			}
#ifdef DEBUG_I2C			 
			else 
			{
				Serial.printf("Invalid set for configs: 0x%02X\n", value);
			}
#endif			
			break;
		
		case GET_CONFIG:
			// Fill config register
			sys.confResume = SetConfigReg();
			Slave.writeRegisters(sys.confResume);
#ifdef DEBUG_I2C			
			Serial.println(F("Requested Config register 2 readings!"));
#endif			
			break;
		
		case GET_STATE:
			// Fill state resume register
			sys.sttResume = SetSttResumeReg();
			Slave.writeRegisters(sys.sttResume);
#ifdef DEBUG_I2C						
			Serial.printf("State: 0x%02X\n", sys.sttResume);
#endif
			break;
		
		case GET_LC_VAL:
			// store the selected device's temperature in the I2C registers
			if (value == OP_LC1_LC2) 
			{
				Slave.writeRegisters(i2c.txBuff+LC1_LSB0_POS, 8);						// write 8 bytes, 4 bytes for each load cell
#ifdef DEBUG_I2C				
				Serial.println(F("Requested loadcell 1 and 2 readings!\r\n"));
#endif
			} 
			else if (value == OP_LC1)
			{
				Slave.writeRegisters(i2c.txBuff+LC1_LSB0_POS, 4);
#ifdef DEBUG_I2C				
				Serial.println(F("Requested loadcell 1 readings!"));
#endif
			}
			else if (value == OP_LC2)
			{
				Slave.writeRegisters(i2c.txBuff+LC2_LSB0_POS, 4);
#ifdef DEBUG_I2C				
				Serial.println(F("Requested loadcell 2 readings"));
#endif
			}
#ifdef DEBUG_I2C			
			else 
			{
				Serial.println(F("Requested wrong option do loadcell readings!"));
			}
#endif			
		break;
	
		default:
			break;
	}
}


void setup()
{
	// Set I/O pins
	PinSetup();
	
	delay(10);
	
	Serial.begin(57600);
	Serial.printf("Loadcell Piggyback | FW: v%s | HW: v%s\r\n", fwVer, hwVer);
	
	// Start I2C communication
	Slave.begin(I2C_SLAVE_ADD);						// Initialize I2C (Slave Mode: address=0x79 )
	Slave.onCommand(command_handler);
	
	Serial.printf("I2C slave device initialized with address: %d\r\n", I2C_SLAVE_ADD);
	
	// Initialize HX
	
	// Initialize library with data output pin, clock input pin and gain factor.
	// Channel selection is made by passing the appropriate gain:
	// - With a gain factor of 64 or 128, channel A is selected
	// - With a gain factor of 32, channel B is selected
	// By omitting the gain factor parameter, the library
	// default "128" (Channel A) is used here.
	lc1.begin(HX711_1_DOUT, HX711_1_SCK);	
	lc2.begin(HX711_2_DOUT, HX711_2_SCK);		
	
	// Set number to loadcells procces
	lc1Proc.nr = 1;
	lc2Proc.nr = 2;
	
	Serial.println("Load cells initialized!");
	
	// Fill config register
	sys.confResume = SetConfigReg();
	
	// Config timer to get samples from load cells
	Timer1.initialize(12500);
	Timer1.attachInterrupt(setOrder2Sample); // blinkLED to run every 0.0125 seconds
	
	Serial.println("End setup!");
	
	sys.state = STT_LOOP;
}

void loop()
{
		
	// If there is order to sample, get samples
	if (sys.time2sample == 1)
	{
		sys.time2sample = 0;
		sys.cnt2Sample10hz++;
		sys.cnt1s++;
		
		if ( (lc1Proc.rate == 80) || ((lc1Proc.rate == 10) && (sys.cnt2Sample10hz > 7)) )
		{
			GetSamples(&lc1, &lc1Proc);
			lc1Proc.sps++;
			
			// Fill Tx buffer
			memcpy(i2c.txBuff+LC1_LSB0_POS, &lc1Proc.rawVal, 4);
		}

		if ( (lc2Proc.rate == 80) || ((lc2Proc.rate == 10) && (sys.cnt2Sample10hz > 7)) )
		{
			GetSamples(&lc2, &lc2Proc);
			lc2Proc.sps++;
			
			// Fill Tx buffer
			memcpy(i2c.txBuff+LC2_LSB0_POS, &lc2Proc.rawVal, 4);			
		}
		
		if (sys.cnt2Sample10hz > 7)
			sys.cnt2Sample10hz = 0;	
	}
	
	// 1s code
	if (sys.cnt1s > 80)
	{
		sys.cnt1s = 0;
	
		// Verify if the loadcell is 4W or 6W
		if (digitalRead(HX_1_4WIRE) == HIGH)
			lc1Proc.w4 = 0;
		else
			lc1Proc.w4 = 1;

		if (digitalRead(HX_2_4WIRE) == HIGH)
			lc2Proc.w4 = 0;
		else
			lc2Proc.w4 = 1;
		
#ifdef DEBUG_MODE		
		Serial.printf("\r\nConf | lc1 gain: %d | lc1 rate: %d | lc2 gain: %d | lc2 rate: %d | Relay: %d | GPS: %d\r\n", lc1Proc.gain, lc1Proc.rate, lc2Proc.gain, lc2Proc.rate, sys.relayStt, sys.gpsEnable);
		Serial.printf("State | system: %d | relay: %d | GPS: %d | 4W lc1: %d | 4W lc2: %d\r\n", sys.state, sys.relayStt, sys.gpsEnable, lc1Proc.w4, lc2Proc.w4);
		Serial.printf("LC1 val: %ld | LC2 val: %ld\r\n", lc1Proc.rawVal, lc2Proc.rawVal);
		Serial.printf("LC1 sps: %d |LC2 sps: %d\r\n", lc1Proc.sps, lc2Proc.sps);
		
		lc1Proc.sps = 0;
		lc2Proc.sps = 0;
#endif
	}
}

void PinSetup(void)
{
	// LC 1
	pinMode(HX711_1_DOUT, OUTPUT);
	digitalWrite(HX711_1_DOUT, LOW);
	pinMode(HX711_1_SCK, OUTPUT);
	digitalWrite(HX711_1_SCK, LOW);

	pinMode(HX711_1_RATE, OUTPUT);
	digitalWrite(HX711_1_RATE, HIGH);

	pinMode(HX_1_4WIRE, INPUT);
	
	// LC 2
	pinMode(HX711_2_DOUT, OUTPUT);
	digitalWrite(HX711_2_DOUT, LOW);
	pinMode(HX711_2_SCK, OUTPUT);
	digitalWrite(HX711_2_SCK, LOW);	

	pinMode(HX711_2_RATE, OUTPUT);
	digitalWrite(HX711_2_RATE, HIGH);
	
	pinMode(HX_2_4WIRE, INPUT);
	
	// Other functions
	
	pinMode(RELAY_PIN, OUTPUT);
	digitalWrite(RELAY_PIN, LOW);	

	pinMode(GPS_EN, OUTPUT);
	digitalWrite(GPS_EN, LOW);		
}

void setOrder2Sample(void)
{
	sys.time2sample = 1;
}

uint8_t GetSamples(HX711 *hx, LC_STRUCT * lc)
{
	lc->rawVal = hx->read(); 
	
#ifdef DEBUG_MODE
	if ( (lc->rawVal > 8000000) || (lc->rawVal < -8000000) || (abs(lc->oldRawVal - lc->rawVal) > 1000) )
		Serial.printf("ERROR on LC%d | Val: %ld | Old val: %ld\r\n", lc->nr, lc->rawVal, lc->oldRawVal);
#endif	
	
	lc->oldRawVal = lc->rawVal;
	
	return 1;
}

uint8_t SetGain(HX711 *hx, LC_STRUCT * lc, uint8_t gain)
{
	if ( (gain != 64) && (gain != 128) )
		return 0;	
	
	lc->gain = gain;
	hx->set_gain(lc->gain);	
	
	return 1;
}

uint8_t SetRate(LC_STRUCT * lc, uint8_t rate)
{
	lc->rate = rate;
	
	if (rate == 10)
		digitalWrite(lc->ratePin, LOW);
	else if (rate == 80)
		digitalWrite(lc->ratePin, HIGH);
	else
		return 0;		

	return 1;
}


uint8_t SetConfigReg(void)
{
	uint8_t configAux = 0;
	
	if (lc1Proc.gain == 64)
		configAux = 0;
	else
		configAux = 1;
	
	if (lc1Proc.rate == 10)
		configAux |= (0 << 1);
	else
		configAux |= (1 << 1);
		
	if (lc2Proc.gain == 64)
		configAux |= (0 << 2);
	else
		configAux |= (1 << 2);
		
	if (lc2Proc.rate == 10)
		configAux |= (0 << 3);
	else
		configAux |= (1 << 3);
		
	if (sys.relayStt == 0)
		configAux |= (0 << 4);
	else
		configAux |= (1 << 4);	
	
	if (sys.gpsEnable == 0)
		configAux |= (0 << 5);
	else
		configAux |= (1 << 5);
		
	return configAux;	
}


uint8_t ProcessConfigReg(uint8_t conf)
{
	uint8_t configAux = 0;
	
	// LC1 gain
	configAux = (conf & LC1_GAIN_MASK);
	
	if (configAux == 0)
	{
		lc1Proc.gain = 64;
		lc1.set_gain(lc1Proc.gain);
	}
	else
	{	
		lc1Proc.gain = 128;
		lc1.set_gain(lc1Proc.gain);
	}
	
	// LC1 rate
	configAux = (conf & LC1_RATE_MASK) >> 1;
	
	if (configAux == 0)
	{
		lc1Proc.rate = 10;
		digitalWrite(lc1Proc.ratePin, LOW);
	}
	else
	{
		lc1Proc.rate = 80;
		digitalWrite(lc1Proc.ratePin, HIGH);
	}	
	
	// LC2 gain
	configAux = (conf & LC2_GAIN_MASK) >> 2;
	
	if (configAux == 0)
	{
		lc2Proc.gain = 64;
		lc2.set_gain(lc2Proc.gain);
	}
	else
	{
		lc2Proc.gain = 128;
		lc2.set_gain(lc2Proc.gain);
	}
	
	// LC2 rate
	configAux = (conf & LC2_RATE_MASK) >> 3;
	
	if (configAux == 0)
	{
		lc2Proc.rate = 10;
		digitalWrite(lc2Proc.ratePin, LOW);
	}
	else
	{
		lc2Proc.rate = 80;
		digitalWrite(lc2Proc.ratePin, HIGH);
	}	
	
	// Relay set
	configAux = (conf & RELAY_STT_MASK) >> 4;
	
	sys.relayStt = configAux;
	digitalWrite(RELAY_PIN, sys.relayStt);

	// GPS power set
	configAux = (conf & RELAY_STT_MASK) >> 5;
	
	sys.gpsEnable = configAux;
	digitalWrite(GPS_EN, sys.gpsEnable);
	
	return 1;
}


uint8_t SetSttResumeReg(void)
{
	uint8_t sttResumeAux = 0;
	
	// System state
	sttResumeAux = sys.state;
	
	// Relay control
	if (digitalRead(RELAY_PIN) == LOW)
		sttResumeAux |= (0 << 1);
	else
		sttResumeAux |= (1 << 1);
	
	// GPS power ON control
	if (digitalRead(GPS_EN) == LOW)
		sttResumeAux |= (0 << 2);
	else
		sttResumeAux |= (1 << 2);

	// 4W detection	LC1
	sttResumeAux |= (lc1Proc.w4 << 3);
	
	// 4W detection	LC2
	sttResumeAux |= (lc2Proc.w4 << 4);	
	
	return sttResumeAux;
}


