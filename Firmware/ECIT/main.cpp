
#include "user.h"

 // Definition of firmware & hardware versions
char fwVersion[8] = { '\0' };
//String fwVersionStr = "7.90S";	// 5 chars maximum
String fwVersionStr = "7.90P";	// 5 chars maximum
char hwVersion[] = "2.10";
String version = "Igus_WD_V" + fwVersionStr + ".bin";

//version label to compare with remote OTA updates
const char local_password[] PROGMEM = "Atronia 2016";		//password protection of the sensorType config

// PCB temperature sensor
Adafruit_MCP9808 tIntSens = Adafruit_MCP9808();

// OLED SSD1306 object 
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Serial Port for Debug communication
HardwareSerial DebugSerial(0);

// Serial Port for RS485 communication
HardwareSerial EspSerial2(1);

// Serial Port for GSM communication
HardwareSerial GsmSerial(2);

// I/O Expander
PCAL6416A expander;

// Semaphore for I2C shared tasks - Shared I2C Resource Mutex
SemaphoreHandle_t xI2CSemaphore;

// Semaphore to handle configurations
SemaphoreHandle_t configSemaphore;

// Semaphore to handle tick count task
SemaphoreHandle_t tickSemaphore;

// Time structure
int16_t timeOffset = 0;		 // Offset on GMT time
ESP32Time rtc(timeOffset);   // Configure internal RTC
struct tm timeinfo;          // Structure for working time
bool rtcSet = false;         // Control if internal RTC already set
uint16_t wakeupTimerMinutes; // Variable to controlminutes to wakeup

// Node structures
NODE_STRUCT node;
NODE_TOBE_CONF_STRUCT node2BeCfg;

// UID
String macAddress;
char tempMacAddress[17] = { '\0' };
char tempUID[UID_SIZE + 1] = { '\0' };

// Telem from Memory Struct
TELEM_STRUCT telem;
TELEM_ECIT_STRUCT telemEcit[ECIT_BUFFER];
DATA_BUFFER data[ECIT_BUFFER];


#ifdef USING_LOADCELL
PIGGYBACK_STRUCT pb;
DATA_BUFFER_PB_STRUCT dataLc1;
DATA_BUFFER_PB_STRUCT dataLc2;
#endif

// Using LoRa radio
RH_RF95 rf95(RF_NSS, RF_INT);
bool    inTxRx = false;
uint8_t txFrame = IGUS_ECIT;

// Class to manage message delivery and receiving, using the driver declared above
RHReliableDatagram manager(rf95, node.nodeNr);

// IGUS timer and pins handling
//hw_timer_t* timer_1 = NULL;
volatile uint32_t timerTick = 0;
volatile uint32_t deltaTimerTick = 0;
volatile uint32_t sampleTime = 0;
volatile uint8_t firstPass = INVALID_PASS;
volatile uint8_t secondPass = INVALID_PASS;
volatile bool passEvent = false;
bool printOrder = false;
bool printOrderLc = false;
bool rfSendLc = false;
bool resumeLcConfRf = false;
float speed = 0.0;
bool toggleLED = false;
bool toggleBTN = false;
bool txFlag = false;
uint8_t dataIndex = 0;
bool printRS485test = false;
bool lcConfigFlag = false;
bool lc1OversteppingTrigger = false;
bool lc2OversteppingTrigger = false;
volatile uint8_t pbState = UNDEFINED_STATE;
uint8_t lcFail = 0;
bool pbPresent = false;
bool rs485PrintFlag = false;

// Tasks handle
TaskHandle_t taskHandleConn;
TaskHandle_t taskHandleRf;
TaskHandle_t taskHandlePrint;
TaskHandle_t taskHandle1s;
TaskHandle_t taskHandleLc;
TaskHandle_t taskHandleTicks;

void IRAM_ATTR btnISR(void)
{
	//DebugSerial.printf("\r\nS1\r\n");

	detachInterrupt(DIG_IN_1);
	detachInterrupt(DIG_IN_2);

	vTaskDelay(pdMS_TO_TICKS(BOUNCE_DELAY));

	if (digitalRead(DIG_IN_1) == LOW)
	{
		if (firstPass == INVALID_PASS)
		{
			firstPass = SENSOR_1;
			timerTick = millis();
		}
		else
		{
			if (firstPass != SENSOR_1)
			{
				secondPass = SENSOR_1;
				sampleTime = millis();
				if (sampleTime > timerTick)
				{
					deltaTimerTick = sampleTime - timerTick;	// Delta tick count inside ISR (millis dependant), so it is not influenced by task rate				
					passEvent = true;
					xSemaphoreGiveFromISR(tickSemaphore, NULL);
				}
				else
				{
					deltaTimerTick = 0;
					firstPass = INVALID_PASS;
					secondPass = INVALID_PASS;
				}
			}
		}		
	}

	attachInterrupt(DIG_IN_1, btnISR, FALLING);
	attachInterrupt(DIG_IN_2, btn2ISR, FALLING);
}

void IRAM_ATTR btn2ISR(void)
{
	//DebugSerial.printf("\r\nS2\r\n");

	detachInterrupt(DIG_IN_1);
	detachInterrupt(DIG_IN_2);
	
	vTaskDelay(pdMS_TO_TICKS(BOUNCE_DELAY));

	if (digitalRead(DIG_IN_2) == LOW)
	{		
		if (firstPass == INVALID_PASS)
		{
			firstPass = SENSOR_2;
			timerTick = millis();
		}
		else
		{
			if (firstPass != SENSOR_2)
			{
				secondPass = SENSOR_2;
				sampleTime = millis();
				if (sampleTime > timerTick)
				{
					deltaTimerTick = sampleTime - timerTick;	// Delta tick count inside ISR (millis dependant), so it is not influenced by task rate				
					passEvent = true;
					xSemaphoreGiveFromISR(tickSemaphore, NULL);
				}
				else
				{
					deltaTimerTick = 0;
					firstPass = INVALID_PASS;
					secondPass = INVALID_PASS;
				}
			}
		}		
	}

	attachInterrupt(DIG_IN_1, btnISR, FALLING);
	attachInterrupt(DIG_IN_2, btn2ISR, FALLING);
}


#ifdef USING_VOC
MICS_VZ_89TE voc = MICS_VZ_89TE();
#endif

// RS485 structure
RS485_SERIAL rs485;

DISPLAY_STRUCT lines;

// GPS variables
char gpsBuffer[GPS_MSG_LEN] = { '\0' };

// Deep sleep control variables
esp_sleep_wakeup_cause_t wakeupReason;

// WiFi Client variables
char routerSSID[SSID_LEN] = { '\0' };
char routerPassword[ROUTER_PASS_LEN] = { '\0' };
char* wifiHostIP;
int wifiHostPort;
int wifiStatus = 0;
int sendWifiJsonStatus = 0;
const char* serverName = "http://atronia.ddns.net:1880/v1.0/sensebluelight/measures/";
const char* ntpServer = "time.google.com";
const long  gmtOffset_sec = 0;
const int   daylightOffset_sec = 3600;

// Webserver variables
WebServer Server;
AutoConnect Portal(Server);
AutoConnectConfig Config;
uint8_t cnt = 0;
uint16_t insidePortal = 0;
bool portalInit = false;

//////////////////////////////////// PORTAL WebPages + Methods///////////////////////////////////

// Overrides default color scheme
//#define AUTOCONNECT_MENUCOLOR_TEXT        "#fff"		// Menu text foreground color
//#define AUTOCONNECT_MENUCOLOR_BACKGROUND  "#ffc000"	// Menu background color
//#define AUTOCONNECT_MENUCOLOR_ACTIVE      "#ff7200"	// Background color with active menu items and mouse hover

//SIM portal page
ACText(simtitle, "<h2>Sim card APN</h2>", "text-align:center;padding:10%;text-color:green");
ACText(currapn, "", "text-align:left;text-color:green");
ACElement(blank, "<hr>");			//multiuse
ACInput(newapn, "new apn", "Input new apn server and press Button to update!");
ACSubmit(apnbutton, "Save", "/upsim");
ACSubmit(retrnBtn, "Return", "/_ac");			//multiuse
AutoConnectAux simPage("/simcon", "Sim Configuration", false, { simtitle,currapn,blank,newapn,apnbutton,retrnBtn });

ACText(upcom, "APN Server updated!");
ACSubmit(apnsaved, "Main Menu", "/_ac");
ACSubmit(btnexit, "Exit", "/_ac/disc");
AutoConnectAux simsave("/upsim", "Apn Updated", false, { upcom,apnsaved, btnexit });

//Connection Type portal page
ACText(connTitle, "<h2>Connection Type</h2>", "text-align:center;padding:10%;text-color:green");
ACText(subtitle, "Choose Communication Type", "text-align:left;text-color:green");
ACRadio(radConnType, { "LTE", "2G", "WiFi" }, "", AC_Vertical, 1);
//ACText(wifiInfo, "For WiFi Connection Input SSID and Password", "text-align:left;text-color:green");
ACText(wifiInfo, "For WiFi Connection Set An Option In New WiFi Menu", "text-align:left;text-color:green");
ACInput(inputSSID, "", "SSID");
//ACInput(inputPass, "", "Password");
ACSubmit(connSaveButton, "Save", "/upconn");
ACSubmit(mainMenuBtn, "Main Menu", "/_ac");   // reusable
AutoConnectAux connPage("/conntype", "Connection Type", true, { connTitle, blank, subtitle, radConnType, blank, wifiInfo, inputSSID, connSaveButton, mainMenuBtn });
//AutoConnectAux connPage("/conntype", "Connection Type", true, { connTitle, blank, subtitle, radConnType, blank, wifiInfo, inputSSID, inputPass, connSaveButton, mainMenuBtn });

ACText(saveMsg, "Connection Type Updated");
ACSubmit(backMenuBtn, "Back", "/conntype");
AutoConnectAux saveConn("/upconn", "Connection Saved", false, { saveMsg, mainMenuBtn, backMenuBtn });

//Load cell configurations portal page
ACText(loadcellTitle, "<h2>CF.P Configurations</h2>", "text-align:center;padding:10%;text-color:green");
ACText(loadcellSubtitle, "Choose Load Cell Type", "text-align:left;text-color:green");
AutoConnectSelect selectLoadCell("selectLoadCell", { String("No Load Cell"), String("Vishay Load Cell Model 616 (1 Ton)"), String("Vishay Load Cell Model 620 (2 Ton)"), String("Load Cell KD40s (200 kg)")}, ""); // " " -> Menu title
//ACRadio(radLoadcellType, { "Vishay Load Cell Model 616 (1 Ton)", "Vishay Load Cell Model 620 (2 Ton)", "Load Cell KD40s (200 kg)" }, "", AC_Vertical, 1);
ACInput(inputUpperForceLimit, "", "Upper Force Limit [0-99999 N]:");
ACInput(inputLowerForceLimit, "", "Lower Force Limit [0-99999 N]:");
//ACInput(forceCapacity, "", "Force Capacity [0-99999 N]");
ACText(forceCapacity, "", "text-align:left;");	// color:#ffc000;
ACInput(inputOversteppingsUpperTolerance, "", "Overst. Upper Tolerance [1-99]:");
ACInput(inputOversteppingsLowerTolerance, "", "Overst. Lower Tolerance [1-99]:");
ACInput(inputAbsUpperForceLimit, "", "Abs. Upper Force Limit [0-99999 N]:");
ACInput(inputAbsLowerForceLimit, "", "Abs. Lower Force Limit [0-99999 N]:");
ACInput(forceOffset, "", "Force Offset [-99999/99999 N]:");

ACSubmit(loadcellSaveButton, "Save", "/uploadcell");
ACSubmit(tareButton, "Tare", "/tareloadcell");
ACSubmit(resetLoadCellButton, "Reset", "/resetloadcell");
ACSubmit(mainMenuBtn2, "Main Menu", "/_ac");   // reusable
AutoConnectAux loadcellPage("/loadcell", "CF.P", true, { loadcellTitle, blank, loadcellSubtitle, selectLoadCell, blank, inputUpperForceLimit, inputLowerForceLimit, blank, forceCapacity, blank, inputOversteppingsUpperTolerance, inputOversteppingsLowerTolerance, blank, inputAbsUpperForceLimit, inputAbsLowerForceLimit, blank, forceOffset, mainMenuBtn2, tareButton, resetLoadCellButton, loadcellSaveButton });
//AutoConnectAux loadcellPage("/loadcell", "Load Cell", true, { loadcellTitle, blank, loadcellSubtitle, radLoadcellType, blank, loadcellSaveButton, mainMenuBtn2 });

//ACText(loadcellSaveMsg, "Loadcell Updated");
ACText(loadcellSaveMsg, "");
ACSubmit(backMenuBtn2, "Back", "/loadcell");
AutoConnectAux saveLoadCell("/uploadcell", "Load Cell Configurations Saved", false, { loadcellSaveMsg, mainMenuBtn2, backMenuBtn2 });

ACText(tareLoadcellMsg, "Loadcell Tared");
ACSubmit(backMenuBtn3, "Back", "/loadcell");
ACSubmit(mainMenuBtn3, "Main Menu", "/_ac");   // reusable
AutoConnectAux tareLoadCell("/tareloadcell", "Tare Load Cell", false, { tareLoadcellMsg, mainMenuBtn3, backMenuBtn3 });

ACText(resetLoadcellMsg, "Error Contact Reseted");
ACSubmit(backMenuBtn4, "Back", "/loadcell");
ACSubmit(mainMenuBtn4, "Main Menu", "/_ac");   // reusable
AutoConnectAux resetLoadCell("/resetloadcell", "Load Cell Reset", false, { resetLoadcellMsg, backMenuBtn4, mainMenuBtn4 });

//Wear sensors distance selection
ACText(wearSensorTitle, "<h2>Wear Sensors Distance Selection</h2>", "text-align:center;padding:10%;text-color:green");
ACText(wearSensorSubtitle, "Choose Distance", "text-align:left;text-color:green");
ACRadio(radSensorsDistance, { "Steel - 200 mm", "Aluminum - 195 mm" }, "", AC_Vertical, 1);

ACSubmit(wearSensorsSaveButton, "Save", "/uploadwearsensors");
ACSubmit(mainMenuBtn5, "Main Menu", "/_ac");   // reusable
AutoConnectAux wearsensorsPage("/wearsensors", "Wear Sensors", true, { wearSensorTitle, blank, wearSensorSubtitle, radSensorsDistance, blank, wearSensorsSaveButton, mainMenuBtn5 });

ACText(saveDistanceMsg, "Wear Sensors Distance Updated");
ACSubmit(mainMenuBtn6, "Main Menu", "/_ac");
AutoConnectAux saveWearSensors("/uploadwearsensors", "Wear Sensors Distance Saved", false, { saveDistanceMsg, mainMenuBtn6 });

//Programmer Mode (password protected)
ACText(login, "<h2>Login</h2>", "text-align:center;padding:10%;text-color:violet");
ACInput(password, "", "Password: ", "", "password");		
ACSubmit(validate, "Validate", "/val");
AutoConnectAux passValid("/passVal", "Programmer Mode", true, { login, blank, password, validate, retrnBtn });		//Password input page

ACSubmit(proced, "OK", "/typeSens");
ACSubmit(failed, "Try again", "/passVal");
AutoConnectAux Validation("/val", "Validation", false, { proced ,failed });		//Password validate page

ACText(h2_header, "<h2>Change Sensor Type</h2>", "text-align:center;padding:10%;text-color:green");
#if defined( USING_AQUANETIX ) && defined( USING_PROBES_AQUALABO )
ACRadio(radSensor, { "Duo Temp", "Probe DO", "Probe SAL", "Probe PH", "Probe Conductivity" }, "Sensor Type:", AC_Vertical, 1);
#elif defined( USING_AQUANETIX ) && !defined( USING_PROBES_AQUALABO )
ACRadio(radSensor, { "Duo Temp", "Probe DO", "Probe SAL", "Probe PH" }, "Sensor Type:", AC_Vertical, 1);
#elif defined( USING_PROSENSOR ) || defined ( USING_ATRONIA ) || defined (USING_IGUS)
ACRadio(radSensor, { "Duo Temp", "Probe DO", "Probe SAL", "Probe PH", "Probe Conductivity", "Probe Soil (HR+Temp)", "Probe Level", "Probe Air (HR+Temp)", "Single Temp", "Photon Flow" }, "Sensor Type:", AC_Vertical, 1);
#endif

ACSubmit(procedsave, "Save", "/saveSens");
AutoConnectAux SensorType("/typeSens", "Sensor Type", false, { h2_header, radSensor, procedsave, retrnBtn });		//Sensor list page

ACText(lucom, "Sensor type updated");
AutoConnectAux Sensorsave("/saveSens", "Sensor saved", false, { lucom, apnsaved });			//Sensor type save page

//Factory mode force - RESET
ACText(ltlt, "Are you sure you want to reset probe?", "text-align:center;padding:10%;text-color:gray");
ACSubmit(procedres, "Yes", "/RESET");
AutoConnectAux SensorRes("/manReset", "Factory reset", true, { ltlt, procedres, retrnBtn });

AutoConnectAux finalRes("/RESET", "RESETING", false, {});


String dispApn(AutoConnectAux& aux, PageArgument& args)
{
	AutoConnectText& apnserver = aux["currapn"].as<AutoConnectText>();

	apnserver.value = "APN server:   " + String(node.apn);
	return String();
}

String getnewApn(AutoConnectAux& aux, PageArgument& args)
{
	String temp = simPage.getElement<AutoConnectInput>("newapn").value;

	temp.toCharArray(node.apn, temp.length() + 1);	//update real time node
	EEPROM.put(APN_ADD, node.apn);
	EEPROM.commit();

	DebugSerial.printf("New APN set: %s\n", node.apn);

	return String();
}

String validation(AutoConnectAux& aux, PageArgument& args)
{
	String pass = passValid.getElement<AutoConnectInput>("password").value;
	String local = String(local_password);
	AutoConnectSubmit& valid = aux.getElement<AutoConnectSubmit>("proced");
	AutoConnectSubmit& faile = aux.getElement<AutoConnectSubmit>("failed");

	if (local == pass)
	{
		faile.enable = false;
		valid.enable = true;
	}
	else
	{
		faile.enable = true;
		valid.enable = false;
	}

	return String();
}

String sensorListSetup(AutoConnectAux& aux, PageArgument& args) {
	AutoConnectRadio& radio_sens = aux.getElement<AutoConnectRadio>("radSensor");

	if (node.typeSens == PROBE_3T_MODBUS)
	{
		radio_sens.checked = 1;
	}
	else if (node.typeSens == PROBE_DO_2T)
	{
		radio_sens.checked = 2;
	}
	else if (node.typeSens == PROBE_SAL_2T)
	{
		radio_sens.checked = 3;
	}
	else if (node.typeSens == PROBE_PH_2T)
	{
		radio_sens.checked = 4;
	}
#if defined( USING_PROBES_AQUALABO ) || defined( USING_PROBES_DONGRUN_V2 ) || defined( USING_PROSENSOR )
	else if (node.typeSens == PROBE_COND_2T)
	{
		radio_sens.checked = 5;
	}
#endif
#if defined ( USING_PROSENSOR ) || defined ( USING_ATRONIA )
	else if (node.typeSens == PROBE_SOIL_MOIS)
	{
		radio_sens.checked = 6;
	}
	else if (node.typeSens == PROBE_LEVEL)
	{
		radio_sens.checked = 7;
	}
	else if (node.typeSens == PROBE_AIR_MOIS)
	{
		radio_sens.checked = 8;
	}
	else if (node.typeSens == PROBE_2T_MODBUS)
	{
		radio_sens.checked = 9;
	}
	else if (node.typeSens == PROBE_PHOTON)
	{
		radio_sens.checked = 10;
	}
#endif
	else
	{
		radio_sens.checked = 0;
	}
	return String();
}

String sensorSaving(AutoConnectAux& aux, PageArgument& args) {
	uint8_t ind = SensorType.getElement<AutoConnectRadio>("radSensor").checked;

	if (ind == 1)
	{
		node.typeSens = PROBE_3T_MODBUS;
	}
	else if (ind == 2)
	{
		node.typeSens = PROBE_DO_2T;
	}
	else if (ind == 3)
	{
		node.typeSens = PROBE_SAL_2T;
	}
	else if (ind == 4)
	{
		node.typeSens = PROBE_PH_2T;
	}
#if defined( USING_PROBES_AQUALABO ) || defined( USING_PROBES_DONGRUN_V2 ) || defined( USING_PROSENSOR )
	else if (ind == 5)
	{
		node.typeSens = PROBE_COND_2T;
	}
#endif
#if defined ( USING_PROSENSOR ) || defined ( USING_ATRONIA )
	else if (ind == 6)
	{
		node.typeSens = PROBE_SOIL_MOIS;
	}
	else if (ind == 7)
	{
		node.typeSens = PROBE_LEVEL;
	}
	else if (ind == 8)
	{
		node.typeSens = PROBE_AIR_MOIS;
	}
	else if (ind == 9)
	{
		node.typeSens = PROBE_2T_MODBUS;
	}
	else if (ind == 10)
	{
		node.typeSens = PROBE_PHOTON;
	}
#endif

	EEPROM.put(SENSOR_TYPE_ADD, node.typeSens);
	EEPROM.commit();

	return String();
}

String reset_func(AutoConnectAux& aux, PageArgument& args) {
	//In the next power up the board will have factory values - Save factory flag				
	EEPROM.put(FACTORY_ADD, 0xFF);
	EEPROM.put(INIT_RESET_ADD, 1);
	EEPROM.commit();

	// Set fs to default
	node.fs = 0;

	DebugSerial.println("Factory values!");

	// Restart ESP to set the factory values
	ESP.restart();

	return String();
}

String connListSetup(AutoConnectAux& aux, PageArgument& args)
{
	AutoConnectRadio& radio_connType = aux.getElement<AutoConnectRadio>("radConnType");

	if (node.connType == USING_LTE)
	{
		radio_connType.checked = 1;
	}
	else if (node.connType == USING_2G)
	{
		radio_connType.checked = 2;
	}
	else if (node.connType == USING_WIFI)
	{
		radio_connType.checked = 3;
	}
	else
	{
		radio_connType.checked = 0;
	}

	//connPage.setElementValue("inputSSID", routerSSID);
	connPage.setElementValue("inputSSID", WiFi.SSID());

	return String();
}


String SaveConnType(AutoConnectAux& aux, PageArgument& args)
{	
	uint8_t ind = connPage.getElement<AutoConnectRadio>("radConnType").checked;

	if (ind == 1)
	{
		node.connType = USING_LTE;
	}
	else if (ind == 2)
	{
		node.connType = USING_2G;
	}
	else if (ind == 3)
	{
		node.connType = USING_WIFI;
	}
	else
	{
		node.connType = USING_2G;
	}

	EEPROM.put(CONN_TYPE_ADD, node.connType);
	EEPROM.commit();
	vTaskDelay(pdMS_TO_TICKS(100));

	EEPROM.get(CONN_TYPE_ADD, node.connType);

	DebugSerial.printf("\r\nConnection Type Saved: %d\r\n", node.connType);

	return String();
}

String LoadCellListSetup(AutoConnectAux& aux, PageArgument& args)
{
	//AutoConnectRadio& radio_sensorId = aux.getElement<AutoConnectRadio>("radLoadcellType");
	AutoConnectSelect& selected_sensorId = aux.getElement<AutoConnectSelect>("selectLoadCell");

	switch (pb.lc1.sensorId)
	{
		case CH_NO_SENSOR:
			//radio_sensorId.checked = 1;
			selected_sensorId.selected = 1;
			break;
	
		case CH1_W1:
			//radio_sensorId.checked = 2;
			selected_sensorId.selected = 2;
			break;

		case CH1_W2:
			//radio_sensorId.checked = 3;
			selected_sensorId.selected = 3;
			break;

		case CH1_W3:
			//radio_sensorId.checked = 4;
			selected_sensorId.selected = 4;
			break;

		default:
			//radio_sensorId.checked = 1;
			selected_sensorId.selected = 1;
			break;
	}

	loadcellPage.setElementValue("inputUpperForceLimit", String(pb.lc1.upperForceOvesteppings));
	loadcellPage.setElementValue("inputLowerForceLimit", String(pb.lc1.lowerForceOvesteppings));
	//loadcellPage.setElementValue("forceCapacity", String(pb.lc1.Nmax));
	loadcellPage.setElementValue("forceCapacity", "Force Capacity [N]: " + String(pb.lc1.Nmax));
	loadcellPage.setElementValue("inputOversteppingsUpperTolerance", String(pb.lc1.nrUpperForceOversteppings));
	loadcellPage.setElementValue("inputOversteppingsLowerTolerance", String(pb.lc1.nrLowerForceOversteppings));
	loadcellPage.setElementValue("inputAbsUpperForceLimit", String(pb.lc1.absUpperForceLimit));
	loadcellPage.setElementValue("inputAbsLowerForceLimit", String(pb.lc1.absLowerForceLimit));
	loadcellPage.setElementValue("forceOffset", String(pb.lc1.forceOffset));	

	return String();
}

String SaveLoadCell(AutoConnectAux& aux, PageArgument& args)
{
	String tempString;

	int32_t tempUpperForceLimit, tempLowerForceLimit, tempAbsUpper, tempAbsLower, tempOffset;
	uint8_t tempUpperTolerance, tempLowerTolerance;
	bool entryError = false;
	
	//uint8_t ind = loadcellPage.getElement<AutoConnectRadio>("radLoadcellType").checked;
	uint8_t ind = loadcellPage.getElement<AutoConnectSelect>("selectLoadCell").selected;

	tempString = loadcellPage.getElement<AutoConnectInput>("inputUpperForceLimit").value;
	tempUpperForceLimit = tempString.toInt();
	tempString = loadcellPage.getElement<AutoConnectInput>("inputLowerForceLimit").value;
	tempLowerForceLimit = tempString.toInt();
	tempString = loadcellPage.getElement<AutoConnectInput>("inputOversteppingsUpperTolerance").value;
	tempUpperTolerance = (uint8_t)(tempString.toInt());
	tempString = loadcellPage.getElement<AutoConnectInput>("inputOversteppingsLowerTolerance").value;
	tempLowerTolerance = (uint8_t)(tempString.toInt());
	tempString = loadcellPage.getElement<AutoConnectInput>("inputAbsUpperForceLimit").value;
	tempAbsUpper = tempString.toInt();
	tempString = loadcellPage.getElement<AutoConnectInput>("inputAbsLowerForceLimit").value;
	tempAbsLower = tempString.toInt();
	tempString = loadcellPage.getElement<AutoConnectInput>("forceOffset").value;
	tempOffset = tempString.toInt();	

	if (((tempUpperForceLimit < 0) || (tempUpperForceLimit > 99999)) ||
		((tempLowerForceLimit < 0) || (tempLowerForceLimit > 99999)) ||
		((tempLowerForceLimit < 0) || (tempLowerForceLimit > 99999)) ||
		((tempUpperTolerance < 1) || (tempUpperTolerance > 99)) ||
		((tempLowerTolerance < 1) || (tempLowerTolerance > 99)) ||
		((tempAbsUpper < 0) || (tempAbsUpper > 99999)) ||
		((tempAbsLower < 0) || (tempAbsLower > 99999)) ||
		((tempOffset < -99999) || (tempOffset > 99999))) entryError = true;
	else entryError = false;

	if (!entryError)
	{
		pb.lc1.upperForceOvesteppings = tempUpperForceLimit;
		pb.lc1.lowerForceOvesteppings = tempLowerForceLimit;
		pb.lc1.nrUpperForceOversteppings = tempUpperTolerance;
		pb.lc1.nrLowerForceOversteppings = tempLowerTolerance;
		pb.lc1.absUpperForceLimit = tempAbsUpper;
		pb.lc1.absLowerForceLimit = tempAbsLower;
		pb.lc1.forceOffset = tempOffset;

		if (ind == 1)
		{
			pb.lc1.sensorId = CH_NO_SENSOR;
		}
		else if (ind == 2)
		{
			pb.lc1.sensorId = CH1_W1;
		}
		else if (ind == 3)
		{
			pb.lc1.sensorId = CH1_W2;
		}
		else if (ind == 4)
		{
			pb.lc1.sensorId = CH1_W3;
		}
		else
		{
			pb.lc1.sensorId = pb.lc1.sensorId;
		}

		EEPROM.put(LC1_SENSOR_ID_ADD, pb.lc1.sensorId);
		EEPROM.put(LC1_ABS_UPPER_LIMIT_ADD, pb.lc1.absUpperForceLimit);
		EEPROM.put(LC1_UPPER_FORCE_OVS_ADD, pb.lc1.upperForceOvesteppings);
		EEPROM.put(LC1_ABS_LOWER_LIMIT_ADD, pb.lc1.absLowerForceLimit);
		EEPROM.put(LC1_LOWER_FORCE_OVS_ADD, pb.lc1.lowerForceOvesteppings);
		EEPROM.put(LC1_NR_UPPER_FORCE_OVS_ADD, pb.lc1.nrUpperForceOversteppings);
		EEPROM.put(LC1_NR_LOWER_FORCE_OVS_ADD, pb.lc1.nrLowerForceOversteppings);
		EEPROM.put(LC1_FORCE_OFFSET_ADD, pb.lc1.forceOffset);
		EEPROM.commit();
		vTaskDelay(pdMS_TO_TICKS(100));

		EEPROM.get(LC1_SENSOR_ID_ADD, pb.lc1.sensorId);
		EEPROM.get(LC1_ABS_UPPER_LIMIT_ADD, pb.lc1.absUpperForceLimit);
		EEPROM.get(LC1_UPPER_FORCE_OVS_ADD, pb.lc1.upperForceOvesteppings);
		EEPROM.get(LC1_ABS_LOWER_LIMIT_ADD, pb.lc1.absLowerForceLimit);
		EEPROM.get(LC1_LOWER_FORCE_OVS_ADD, pb.lc1.lowerForceOvesteppings);
		EEPROM.get(LC1_NR_UPPER_FORCE_OVS_ADD, pb.lc1.nrUpperForceOversteppings);
		EEPROM.get(LC1_NR_LOWER_FORCE_OVS_ADD, pb.lc1.nrLowerForceOversteppings);
		EEPROM.get(LC1_FORCE_OFFSET_ADD, pb.lc1.forceOffset);

		DebugSerial.printf("\r\nLoad cell Type Saved: %d\r\n", pb.lc1.sensorId);

		Init_Loadcell_PiggyBack(PIGGYBACK_I2C_ADD);

		saveLoadCell.setElementValue("loadcellSaveMsg", "Load Cell Updated");

		loadcellPage.on(LoadCellListSetup, AC_EXIT_AHEAD);	// Refresh values
	}
	else
	{
		saveLoadCell.setElementValue("loadcellSaveMsg", "Entry Error - Observe Values Limits");
	}

	return String();
}

String TareLoadCell(AutoConnectAux& aux, PageArgument& args)
{
	pb.lc1.forceOffset = (int32_t)(-1.0 * pb.lc1.newtons);

	EEPROM.put(LC1_FORCE_OFFSET_ADD, pb.lc1.forceOffset);
	EEPROM.commit();
	vTaskDelay(pdMS_TO_TICKS(100));

	EEPROM.get(LC1_FORCE_OFFSET_ADD, pb.lc1.forceOffset);

	telemEcit[0].telemEcitLc1.forceOffset = pb.lc1.forceOffset;

	loadcellPage.on(LoadCellListSetup, AC_EXIT_AHEAD);	// Refresh values
	
	return String();
}

String ResetLoadCell(AutoConnectAux& aux, PageArgument& args)
{
	pb.relayStt = OFF;

	dataLc1.absUpperLimitCounterSec = 0;
	dataLc1.upperOversteppingsCounterSec = 0;
	dataLc1.absLowerLimitCounterSec = 0;
	dataLc1.lowerOversteppingsCounterSec = 0;
	dataLc1.absUpperLimitCounterMin = 0;
	dataLc1.upperOversteppingsCounterMin = 0;
	dataLc1.absLowerLimitCounterMin = 0;
	dataLc1.lowerOversteppingsCounterMin = 0;
	dataLc1.relayState = pb.relayStt;

	dataLc2.absUpperLimitCounterSec = 0;
	dataLc2.upperOversteppingsCounterSec = 0;
	dataLc2.absLowerLimitCounterSec = 0;
	dataLc2.lowerOversteppingsCounterSec = 0;
	dataLc2.absUpperLimitCounterMin = 0;
	dataLc2.upperOversteppingsCounterMin = 0;
	dataLc2.absLowerLimitCounterMin = 0;
	dataLc2.lowerOversteppingsCounterMin = 0;
	dataLc2.relayState = pb.relayStt;

	Set_Relay();	

	return String();
}


String WearSensorsListSetup(AutoConnectAux& aux, PageArgument& args)
{
	AutoConnectRadio& radio_wearSensorsDistance = aux.getElement<AutoConnectRadio>("radSensorsDistance");

	if (node.sensorsDistance == DEFAULT_DIST)
	{
		radio_wearSensorsDistance.checked = 1;
	}
	else if (node.sensorsDistance == DEFAULT_DIST_2)
	{
		radio_wearSensorsDistance.checked = 2;
	}
	else
	{
		radio_wearSensorsDistance.checked = 0;
	}

	return String();
}


String SaveWearSensorDistance(AutoConnectAux& aux, PageArgument& args)
{
	uint8_t ind = wearsensorsPage.getElement<AutoConnectRadio>("radSensorsDistance").checked;

	if (ind == 1)
	{
		node.sensorsDistance = DEFAULT_DIST;
	}
	else if (ind == 2)
	{
		node.sensorsDistance = DEFAULT_DIST_2;
	}
	else
	{
		node.sensorsDistance = DEFAULT_DIST;
	}

	EEPROM.put(SENSORS_DIST_ADD, node.sensorsDistance);
	EEPROM.commit();
	vTaskDelay(pdMS_TO_TICKS(100));

	EEPROM.get(SENSORS_DIST_ADD, node.sensorsDistance);

	DebugSerial.printf("\r\nSensors Distance Saved: %d\r\n", node.sensorsDistance);

	return String();
}

/////////////////////////////////////////////////////////////////////////////

// Union to process the float and uint16 values received from the probe
union {
	uint8_t b[4];
	float val;
} uF;

union {
	uint8_t b[2];
	int16_t val;
} uI;

union {
	uint8_t b[4];
	int32_t val;
} uI32;

// Default values
const char calDefault[] = "2023/04/03 ";

// GSM Hardware
char imeiDefault[] = "999999999999999";
char iccidTmp[] = "99999999999999999999";

// GPRS credentials
// Leave empty, if missing user or pass
char apnDefault[] = "iot.1nce.net";
//char apnDefault[] = "internet";				// APN MEO
//char apnDefault[] = "umts";					// APN NOS
const char user[] = "";
const char pass[] = "";

// Server details
char serverAtronia[] = "http://atronia.ddns.net:1880/sensorin/";
#endif

const char sendSms[] = "AT+CMGS=\"";
const char http_url[] = "AT+QHTTPURL=";
const char http_post[] = "AT+QHTTPPOST=";

const char DNSserver[] = "1.1.1.1";

// Control wakeup time
uint32_t deltaT;

bool temp;


void taskCom(void* pvParameters)
{
	TickType_t init;
	init = xTaskGetTickCount();

	char rs485Buffer[LC_RS485_BUFFER_LEN] = { '\0' };
	bool rs485RxEnd = false;
	uint8_t idx = 0;

	//UBaseType_t uxHighWaterMark;

	// Initialize watchdog timer
	esp_task_wdt_add(taskHandleConn);

	for (;;)
	{		
		if (EspSerial2.available() > 0)
		{
			memset(rs485Buffer, '\0', LC_RS485_BUFFER_LEN);			
			idx = 0;

			while (EspSerial2.available() > 0)
			{
				rs485Buffer[idx] = EspSerial2.read();

				if (rs485Buffer[idx] == '\r')
				{
					rs485RxEnd = true;
					break;
				}
				else if (idx < (LC_RS485_BUFFER_LEN - 2)) idx++;
				else break;
			}

			EspSerial2.flush();		// Flushs remaining characters if so
		}		

		if (rs485RxEnd)
		{
			if (memcmp(rs485Buffer, "test", 4) == 0) printRS485test = true;
			else if ( (rs485Buffer[0] == '!') && (rs485Buffer[2] == '2') ) // Configuration('!') for piggyback('2')
			{
				if (xSemaphoreTake(configSemaphore, (TickType_t)50) == pdTRUE)
				{
					SetLcConfigs(rs485Buffer);

					xSemaphoreGive(configSemaphore);
				}
			}

			rs485RxEnd = false;			
		}

		//uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
		//DebugSerial.printf("\r\nTaskComMem: %d\r\n", uxHighWaterMark);

		esp_task_wdt_reset();
		vTaskDelayUntil(&init, pdMS_TO_TICKS(COM_TASK_RATE_MS));
	}
}

void taskRF(void* pvParameters)
{
	TickType_t init;
	init = xTaskGetTickCount();

	//UBaseType_t uxHighWaterMark;

	// Initialize watchdog timer
	esp_task_wdt_add(taskHandleRf);

	for (;;)
	{
		if (node.radioStt)
		{
			Radio_TxRx(&node, &node2BeCfg, &telem, txFrame, &telemEcit[0]);			
		}

		//uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
		//DebugSerial.printf("\r\nTaskRfMem: %d\r\n", uxHighWaterMark);

		esp_task_wdt_reset();
		vTaskDelayUntil(&init, pdMS_TO_TICKS(RF_TASK_RATE_MS));
	}	
}

void taskTime2PrintTel(void* pvParameters)
{
	TickType_t init;
	init = xTaskGetTickCount();	

	//UBaseType_t uxHighWaterMark;
	
	// Initialize watchdog timer
	esp_task_wdt_add(taskHandlePrint);
	
	for (;;)
	{		
		if (printOrder)
		{
			// Test sensor detection
			//DebugSerial.printf("$,%d,%s,%d,%d,%.1f\r\n", IGUS_MSG_ID, node.uID, firstPass, deltaTimerTick, speed);

			if (!rs485PrintFlag)
			{
				rs485PrintFlag = true;

				RS485_TxMode();
				EspSerial2.printf("$,%d,%s,%d,%d,%.1f,%.1f,%s\r\n", IGUS_MSG_ID, node.uID, telemEcit[0].dir, telemEcit[0].deltaSampleTime, telemEcit[0].sampleSpeed, telemEcit[0].displacement, fwVersion);
				EspSerial2.flush();
				RS485_RxMode();

				rs485PrintFlag = false;
			}

			printOrder = false;
		}
		else if (printOrderLc || rfSendLc)
		{
			// Transmission by RS485
			if (!rs485PrintFlag)
			{
				rs485PrintFlag = true;

				RS485_TxMode();
				EspSerial2.printf("$,%d,%s,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%s\r\n", IGUS_MSG_ID_LC, node.uID, telemEcit[0].telemEcitLc1.sensorId, telemEcit[0].telemEcitLc1.absUpperForceLimit, telemEcit[0].telemEcitLc1.upperForceOvesteppings,	\
					telemEcit[0].telemEcitLc1.minActForce, telemEcit[0]. telemEcitLc1.maxActForce, telemEcit[0].telemEcitLc1.meanActForce, telemEcit[0].telemEcitLc1.meanActPosForce, telemEcit[0].telemEcitLc1.meanActNegForce, telemEcit[0].telemEcitLc1.lowerForceOvesteppings, telemEcit[0].telemEcitLc1.absLowerForceLimit,	\
					telemEcit[0].telemEcitLc1.nrUpperForceOversteppings, telemEcit[0].telemEcitLc1.nrLowerForceOversteppings, telemEcit[0].telemEcitLc1.forceOffset,	\
					telemEcit[0].telemEcitLc1.absUpperLimitCounter, telemEcit[0].telemEcitLc1.upperOversteppingsCounter, telemEcit[0].telemEcitLc1.absLowerLimitCounter, telemEcit[0].telemEcitLc1.lowerOversteppingsCounter, telemEcit[0].telemEcitLc1.relayState, fwVersion);
				EspSerial2.flush();
				RS485_RxMode();

				rs485PrintFlag = false;
			}

			// Enable RF transmission
			if (rfSendLc && !txFlag)
			{
				txFrame = IGUS_ECIT_LOADCELL;
				txFlag = true;
				rfSendLc = false;
			}

			// Reset variables
			printOrderLc = false;
		}

		if (printRS485test && !rs485PrintFlag)
		{
			rs485PrintFlag = true;
			
			RS485_TxMode();
			EspSerial2.printf("$,%d,%s,%d,%d,%.1f,%.1f,%s\r\n", IGUS_MSG_ID, node.uID, telemEcit[0].dir, telemEcit[0].deltaSampleTime, telemEcit[0].sampleSpeed, telemEcit[0].displacement, fwVersion);
			EspSerial2.flush();
			RS485_RxMode();			

			printRS485test = false;
			rs485PrintFlag = false;
		}

		//uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
		//DebugSerial.printf("\r\nTaskTimeToPrint: %d\r\n", uxHighWaterMark);

		esp_task_wdt_reset();
		vTaskDelayUntil(&init, pdMS_TO_TICKS(PRINTTEL_TASK_RATE_MS));
	}
}

void task1s(void* pvParameters)
{
	TickType_t init;
	init = xTaskGetTickCount();

	uint8_t temp_LMT = 0;
	uint8_t minuteCnt = 0;
	char rxData;
	uint8_t wifiCnt = 0;

	//UBaseType_t uxHighWaterMark;

	// Initialize watchdog timer
	esp_task_wdt_add(taskHandle1s);

	for (;;)
	{
		// Portal
		if ( (portalInit == false) && (node.bat >= 30) && (digitalRead(BTN) == LOW) )
		{	
			if (cnt % 2 == 0)
			{
				//ShortLEDblink_LED(VIOLET);

				wifiCnt++;

				if (xSemaphoreTake(xI2CSemaphore, (TickType_t)50) == pdTRUE)
				{
					display.clearDisplay();
					display.setCursor(X_INIT_POS + 15, Y_INIT_POS + 13);
					display.setTextSize(2);
					display.printf("WiFi:%d ", 4 - wifiCnt);
					display.display();

					xSemaphoreGive(xI2CSemaphore);
				}
			}

			if (cnt < PORTAL_TIMER)	cnt++;
		}
		else
		{
			if ( (cnt > 0) && (portalInit == false) )
			{
				cnt = 0;

				if (xSemaphoreTake(xI2CSemaphore, (TickType_t)50) == pdTRUE)
				{
					display.clearDisplay();
					DisplaySplashScreen();

					xSemaphoreGive(xI2CSemaphore);
				}
			}

			wifiCnt = 0;
		}
		 
		if ( (portalInit == false) && (cnt >= PORTAL_TIMER) )
		{
			//Set_Color(VIOLET);
			
			if (portalInit == false)
			{
				if (xSemaphoreTake(xI2CSemaphore, (TickType_t)50) == pdTRUE)
				{
					display.clearDisplay();
					display.setCursor(X_INIT_POS + 15, Y_INIT_POS + 13);
					display.setTextSize(2);
					display.write("WiFi ON");
					display.display();

					xSemaphoreGive(xI2CSemaphore);
				}
			}

			// Stop unused processes and WDT			
			detachInterrupt(DIG_IN_1);
			detachInterrupt(DIG_IN_2);
			esp_task_wdt_delete(taskHandle1s);
			esp_task_wdt_delete(taskHandleConn);
			esp_task_wdt_delete(taskHandleRf);
			esp_task_wdt_delete(taskHandlePrint);
			esp_task_wdt_delete(taskHandleLc);
			vTaskDelete(taskHandleConn);
			vTaskDelete(taskHandleRf);
			vTaskDelete(taskHandlePrint);
			vTaskDelete(taskHandleLc);
			vTaskDelete(taskHandleTicks);
			Enable_VCC_2(false);
			delay(50);
			
			insidePortal = 1;
			portalInit = true;
			
			DebugSerial.println("Portal begin!");			

			//External OTA update
			const char ap_title[] = "WiFi Portal";
			AutoConnectUpdate update("atronia.ddns.net", 3030, version, INIT_RESET_ADD, ap_title);

			simPage.on(dispApn, AC_EXIT_AHEAD);
			simsave.on(getnewApn, AC_EXIT_AHEAD);
			Validation.on(validation, AC_EXIT_AHEAD);
			SensorType.on(sensorListSetup, AC_EXIT_AHEAD);
			Sensorsave.on(sensorSaving, AC_EXIT_LATER);
			connPage.on(connListSetup, AC_EXIT_AHEAD);			
			saveConn.on(SaveConnType, AC_EXIT_AHEAD);
			loadcellPage.on(LoadCellListSetup, AC_EXIT_AHEAD);
			saveLoadCell.on(SaveLoadCell, AC_EXIT_AHEAD);
			tareLoadCell.on(TareLoadCell, AC_EXIT_AHEAD);
			resetLoadCell.on(ResetLoadCell, AC_EXIT_AHEAD);
			wearsensorsPage.on(WearSensorsListSetup, AC_EXIT_AHEAD);
			saveWearSensors.on(SaveWearSensorDistance, AC_EXIT_AHEAD);
			finalRes.on(reset_func, AC_EXIT_AHEAD);

			Portal.join({ simPage, simsave, Validation, SensorType, Sensorsave , connPage, saveConn, loadcellPage, saveLoadCell, SensorRes, tareLoadCell, resetLoadCell, wearsensorsPage, saveWearSensors, passValid, finalRes });

			/*************** Set configs for portal and begin portal ***************/
			Config.autoRise = true;
			Config.autoReconnect = false;
			Config.immediateStart = true;
			Config.title = "Igus Portal V" + fwVersionStr;
			Config.apid = "At-" + WiFi.macAddress();
			Config.psk = "";	//portal password
			Config.portalTimeout = 450000;
			Config.imei = "NA";
			Config.iccid = "NA";
			Config.retainPortal = false;
			Config.boundaryOffset = 0;

			Portal.config(Config);
			update.attach(Portal);

			if (Portal.begin())
			{
				DebugSerial.println("WiFi connected: " + WiFi.localIP().toString());
			}

			Portal.handleClient();
			vTaskDelay(pdMS_TO_TICKS(100));

			if (WiFi.SSID() != NULL)
			{
				String ssidName = WiFi.SSID();
				String ssidPass = WiFi.psk();

				ssidName.toCharArray(routerSSID, ssidName.length() + 1);
				ssidPass.toCharArray(routerPassword, ssidPass.length() + 1);

				EEPROM.put(WIFI_SSID_ADD, routerSSID);
				EEPROM.put(WIFI_PASSWORD_ADD, routerPassword);
				EEPROM.commit();
				vTaskDelay(pdMS_TO_TICKS(100));

				EEPROM.get(WIFI_SSID_ADD, routerSSID);
				EEPROM.get(WIFI_PASSWORD_ADD, routerPassword);

				DebugSerial.println("WiFi SSID: " + ssidName);
				DebugSerial.println("WiFi Password: " + ssidPass);
			}
			else
			{
				DebugSerial.println("No WiFi selected");
			}

			DebugSerial.println();

			insidePortal = 0;

			// Set default color
			Set_Color(BLUE);

			DebugSerial.printf("\r\nPortal exit\r\n");

			// Restart ESP to start to aply any changes on Portal
			ESP.restart();			
		}
		// End Portal

		// Verify battery level
		BatMeasure();

		if (node.bat < 30) ShortLEDblink_LED(RED);
		else ShortLEDblink_LED(BLUE);

#ifdef DEBUG_IGUS
		DebugSerial.printf("\r\nTicks: %d  %d", timerTickHi, timerTickLo);
#endif

		if (digitalRead(BTN) == LOW)	// For RF test purpose
		{
			if (!inTxRx && !toggleBTN)
			{
				txFrame = IGUS_ECIT;
				txFlag = true;

				// Transmission by RS485
				if (!rs485PrintFlag)
				{
					rs485PrintFlag = true;

					RS485_TxMode();
					EspSerial2.printf("$,%d,%s,%d,%d,%.1f,%.1f,%s\r\n", IGUS_MSG_ID, node.uID, telemEcit[0].dir, telemEcit[0].deltaSampleTime, telemEcit[0].sampleSpeed, telemEcit[0].displacement, fwVersion);
					EspSerial2.flush();
					RS485_RxMode();

					rs485PrintFlag = false;
				}
			}

			toggleBTN = true;
		}
		else
		{
			toggleBTN = false;
		}

#ifdef USING_LOADCELL

		// Checking for alert triggers
		if (pbPresent)
		{
			pbState = CheckState();
			CheckLoadcellsAlerts(pb.lc1.newtons, pb.lc2.newtons);
		}   
		
#ifdef DEBUG_LOADCELL
		DebugSerial.printf("\r\n----------------------------------------------------------------------------------------------------------------------\r\n");
		DebugSerial.printf("\r\n[task1s] > Loadcells Vals > Lc1: %d | Lc1Mean: %d | Lc2: %d | Lc2Mean: %d", pb.lc1.valRaw, pb.lc1.valMean, pb.lc2.valRaw, pb.lc2.valMean);
		DebugSerial.printf("\r\n[task1s] > Loadcells Newtons > Lc1: %.2f | Lc2: %.2f", pb.lc1.newtons, pb.lc2.newtons);
		DebugSerial.printf("\r\n[task1s] > Piggyback state: 0x%02X\r\n", pbState);
		DebugSerial.printf("\r\n----------------------------------------------------------------------------------------------------------------------\r\n");
#endif

#endif

		// Test communications from user
		//if (DebugSerial.available() > 0)
		//{
		//	while (DebugSerial.available() > 0)
		//	{
		//		rxData = DebugSerial.read();
		//		DebugSerial.print(rxData);
		//	}

		//	DebugSerial.println();
		//}		

		// Set wake up with the configured pin on the mask and through time
		//esp_sleep_enable_ext1_wakeup(BUTTON_PIN_BITMASK, ESP_EXT1_WAKEUP_ANY_HIGH);
		//esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);

		//uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
		//DebugSerial.printf("\r\nTask1S: %d\r\n", uxHighWaterMark);		

		esp_task_wdt_reset();
		vTaskDelayUntil(&init, pdMS_TO_TICKS(1000));

		//if (timeToSleep > 0) timeToSleep--;
		//else ShutDown();
	}
}

void taskTickCnt(void* pvParameters)
{
	TickType_t init;
	init = xTaskGetTickCount();	

	//UBaseType_t uxHighWaterMark;
	
	// Initialize watchdog timer
	//esp_task_wdt_add(NULL);

	for (;;)
	{
		xSemaphoreTake(tickSemaphore, portMAX_DELAY);
		
		if ( (deltaTimerTick > 0) && (deltaTimerTick <= MAX_VALID_TIME) ) // Avoids validation with unusual delta timer 
		{
			speed = (float)node.sensorsDistance / (deltaTimerTick / 1000.0);
		}
		else
		{
			speed = 0.0;
		}

		timerTick = 0;
		passEvent = false;

		// Test sensor detection
		//DebugSerial.printf("$,%d,%s,%d,%d,%.1f\r\n", IGUS_MSG_ID, node.uID, firstPass, deltaTimerTick, speed);

		data[dataIndex].dir = firstPass;
		data[dataIndex].deltaTimer = deltaTimerTick;
		data[dataIndex].speed = speed;
		data[dataIndex].sampleTime = sampleTime;

		if (dataIndex >= (ECIT_BUFFER - 1))
		{
			// Detach pins interrupts to avoid racing conditions
			detachInterrupt(DIG_IN_1);
			detachInterrupt(DIG_IN_2);			

			if (dataIndex == (ECIT_BUFFER - 1))	// Double check to prevent array index wrong access
			{
				telemEcit[0].displacement = CalculateDisplacement();
				printOrder = true;	// Transmission by RS485
				
				// Enable RF transmission
				if (!txFlag)
				{
					txFrame = IGUS_ECIT;
					txFlag = true;
				}
			}

			dataIndex = 0;

			// Attach pins interrupts again
			attachInterrupt(DIG_IN_1, btnISR, FALLING);
			attachInterrupt(DIG_IN_2, btn2ISR, FALLING);
		}
		else
		{
			if (dataIndex < (ECIT_BUFFER - 1)) dataIndex++;
		}

		// Before resetting variables, ensure RF data population
		deltaTimerTick = 0;
		firstPass = INVALID_PASS;
		secondPass = INVALID_PASS;
		
		
		//uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
		//DebugSerial.printf("\r\nTaskTickCnt: %d\r\n", uxHighWaterMark);

		//esp_task_wdt_reset();
		//vTaskDelayUntil(&init, pdMS_TO_TICKS(TICK_TASK_RATE_MS));
	}
}


#ifdef USING_LOADCELL

void taskLC(void* pvParameters)
{
	TickType_t init;
	init = xTaskGetTickCount();
	
	// Force offset for test purpose
	//pb.lc1.offset = 60.70;
	//pb.lc2.offset = 66.00;

	//UBaseType_t uxHighWaterMark;

	// Initialize watchdog timer
	esp_task_wdt_add(taskHandleLc);

	for (;;)
	{		
		// Fill buffers to then get the average	
		//if (xSemaphoreTake(configSemaphore, (TickType_t)50) == pdTRUE)
		//{
			Get_LCvals(PIGGYBACK_I2C_ADD, OP_LC1_LC2);
			GetLCmeanRaw(&pb.lc1);
			GetLCmeanRaw(&pb.lc2);

			pb.lc1.newtons = Calc_Newtons(&pb.lc1);
			pb.lc2.newtons = Calc_Newtons(&pb.lc2);

			// Set offset
			pb.lc1.newtons += pb.lc1.forceOffset;
			pb.lc2.newtons += pb.lc2.forceOffset;

#ifdef DEEP_DEBUG_LOADCELL
			DebugSerial.printf("\r\n[taskLC] > Loadcell valls | Lc1: %d | Lc2: %d\r\n", pb.lc1.valRaw, pb.lc2.valRaw);
			DebugSerial.printf("[taskLC] > Loadcell Mean valls | Lc1: %d | Lc2: %d\r\n", pb.lc1.valMean, pb.lc2.valMean);
			DebugSerial.printf("[taskLC] > Newtons | Lc1: %.2f | Lc2: %.2f\r\n", pb.lc1.newtons, pb.lc2.newtons);
#endif
			
			LoadcellsAggregations();

			//xSemaphoreGive(configSemaphore);
		//}

		//uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
		//DebugSerial.printf("\r\nTaskLC: %d\r\n", uxHighWaterMark);
	
		esp_task_wdt_reset();
		vTaskDelayUntil(&init, pdMS_TO_TICKS(LC_TASK_RATE_MS));
	}
}

#endif

void setup()
{
	//Pin Setup and Expander init
	PinSetup();
	vTaskDelay(pdMS_TO_TICKS(1000));
	temp = SYS_ConfExpIO();

	// Config Seril Port for Debug
	DebugSerial.begin(DEBUG_BAUDRATE);

	// Set fwVersion variable
	strcpy(fwVersion, fwVersionStr.c_str());

	char hw_type[] = "EC.IT";
	char clientType[] = "Igus";
	char ap_title[64];

	DebugSerial.printf("\nMonarch %s Started - %s | FW: %s | HW: %s\n", clientType, hw_type, fwVersion, hwVersion);
	sprintf(ap_title, "%s Monarch - V%s\r\n", hw_type, fwVersion);

	// Initialize Expander pins - I2C peripheral is started inside the library
	if (temp == false)
		DebugSerial.print("\r\nError initializing IO Expander!\r\n");
	else
		DebugSerial.print("\r\nIO Expander initialized!\r\n");

	// Check wakeup source and configure interrupts
	wakeupReason = esp_sleep_get_wakeup_cause();
	Print_WakeupReason(wakeupReason);	

	//attachInterrupt(BTN, BTN_isr, FALLING);		// If interrupt on pin needed. Be carefull if used to deep sleep wakeup
	esp_sleep_enable_ext0_wakeup(GPIO_NUM_4, LOW);  // External deep sleep wakeup on internal RTC pin

	// Verify waking up agent - redudancy
	if ( (digitalRead(BTN) == LOW) || (wakeupReason == ESP_SLEEP_WAKEUP_EXT0) || (wakeupReason == ESP_SLEEP_WAKEUP_UNDEFINED) )
		node.wakeUp = WAKEUP_BUTTON;
	else
		node.wakeUp = WAKEUP_RTC;

	setCpuFrequencyMhz(240);

	// Set semaphores 
	if (xI2CSemaphore == NULL)
	{
		xI2CSemaphore = xSemaphoreCreateMutex();
		if (xI2CSemaphore != NULL) xSemaphoreGive(xI2CSemaphore);
	}

	if (configSemaphore == NULL)
	{
		configSemaphore = xSemaphoreCreateMutex();
		if (configSemaphore != NULL) xSemaphoreGive(configSemaphore);
	}

	if (tickSemaphore == NULL)
	{
		tickSemaphore = xSemaphoreCreateBinary();
		if (tickSemaphore != NULL) xSemaphoreGive(tickSemaphore);
	}

	// Initialize Wire - for I2C communication
	Wire.begin();

#ifdef DEBUG_I2C
	// Show i2c devices
	//i2cScanner(&Wire, true, SDA_PIN, SCL_PIN);
	i2cScanner(&Wire, false, SDA_PIN, SCL_PIN);
#endif

	// Turn on display
	if (node.wakeUp == WAKEUP_BUTTON) SetupDisplay();

	// Display splash screen	
	if (node.wakeUp == WAKEUP_BUTTON)
	{
		display.clearDisplay();
		DisplaySplashScreen();
	}

	// Signiling LED
	Set_Color(NO_COLOR);


#ifdef DEBUG_MODE
	DebugSerial.print("\r\nDEBUG MODE ACTIVATED!\r\n");
#endif
#ifdef DEBUG_MODBUS 
	DebugSerial.print("\r\nDEBUG MODBUS ACTIVATED!\r\n");
#endif
#ifdef DEBUG_FORCE_FS
	DebugSerial.print("\r\nDEBUG FORCE FS ACTIVATED!\r\n");
#endif
#ifdef DEBUG_TIME_TO_SEND
	DebugSerial.print("\r\nDEBUG TIME TO SEND ACTIVATED!\r\n");
#endif
#ifdef DEBUG_FORCE_FACTORY
	DebugSerial.print("\r\nDEBUG FORCE FACTORY ACTIVATED!\r\n");
#endif
#ifdef DEBUG_MODE_MEM
	DebugSerial.print("\r\nDEBUG MODE MEM ACTIVATED!\r\n");
#endif
#ifdef DEBUG_ALARMS
	DebugSerial.print("\r\nDEBUG ALARM ACTIVATED!\r\n");
#endif	

	// Get time from RTC and verify if RTC is running
	SYS_GetTimeRTC(node.date, node.hour, node.hhmmss);

	AutoConnectUpdate update("atronia.ddns.net", 3030, version, INIT_RESET_ADD, ap_title);	//External OTA update

	// EEPROM initialization
	if (!EEPROM.begin(EEPROM_SIZE))
	{
		DebugSerial.println("\r\nFailed to initialise EEPROM");
		DebugSerial.println("\r\nRestarting...");
		vTaskDelay(pdMS_TO_TICKS(1000));
		ESP.restart();
	}

	// Verification of 1st turn ON after programming or wakeup from reset
	EEPROM.get(FACTORY_ADD, node.factory);
	EEPROM.get(NEW_VERSION_ADD, node.newFW);
	EEPROM.get(BAT_MIN_ADC_ADD, node.batMinADC);
	EEPROM.get(BAT_MAX_ADC_ADD, node.batMaxADC);

#ifdef DEBUG_FORCE_FACTORY
	node.factory = 0xFF;
	node.batMinADC = 0xFFFF;
	node.batMaxADC = 0xFFFF;
#endif

	if ((node.factory != 1) || (node.newFW != 128))
	{
		DebugSerial.println("\r\nSet Default values!\r\n");

		node.factory = 1;
		node.newFW = 128;

		// The user have to calibrate the battery measure - it is a blocking function
		// Only run when EEPROM is in default values
#ifdef USING_BATTERY_CAL
		if ((node.batMinADC == 0xFFFF) && (node.batMaxADC == 0xFFFF))
			Calib_BatMeasure();
#endif

		Set_DefaultVals();

		EEPROM_SaveConfigs();

		// Set flag to indicate a 1st power ON
		node.init = 1;
	}
	else
	{
		DebugSerial.println("\r\nRead memory");

		// Caso se tenha desligado, l? os valores gravados na EEPROM
		EEPROM_ReadConfigs();
		node.init = 0;
	}

	DebugSerial.printf("\r\nWear sensors distance: %d\r\n", node.sensorsDistance);

	// SPI initialization
	SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, RF_NSS);

#ifdef DEBUG_MODE
	Serial.printf("\r\nInitializing Node  ");
#endif

	// Radio and LoRa config
	inTxRx = false;

	node.radioStt = RF_Config();

	if (!node.radioStt)
	{
#ifdef DEBUG_MODE
		Serial.println("RF initialization failed!");
		Serial.println("Please restart !");
#endif
		// Error initializing RF module, reset the device
		Set_Color(RED);
		while (1);
	}

	// Get IMEI and ICCID
	DebugSerial.printf("\r\nGetting IMEI and ICCID ...\r\n");

	node.gsmStt = 0;	//Initialize GSM state
		
	GSM_TurnON();

	if (node.wakeUp == WAKEUP_BUTTON)
	{
		if (node.connType == USING_2G) AddLine("Waiting GSM...", 1);
		else AddLine("Waiting LTE...", 1);					
		DisplayLines();
	}

	// Needed to have stable initialization
	vTaskDelay(pdMS_TO_TICKS(5000));

	GSM_Init();

	// Get IMEI from GSM module
	node.gsmStt = GSM_GetIMEI(imeiDefault);

	if (node.gsmStt == GSM_SUCCESS)
	{
		// If success, save IMEI
		if (memcmp(node.imei, imeiDefault, IMEI_SIZE) != 0)
		{
			// It isn't the same, save it
			memcpy(node.imei, imeiDefault, IMEI_SIZE + 1);
			EEPROM.put(IMEI_ADD, node.imei);
			EEPROM.commit();
		}

		DebugSerial.printf("IMEI: \"%s\"\r\n", node.imei);
	}
	else
	{
		DebugSerial.printf("Failed to get IMEI\r\n");
	}
	
	// Get ICCID from the SIM Card installed
	node.gsmStt = GSM_GetICCID(iccidTmp);

	if (node.gsmStt == GSM_SUCCESS)
	{
		// If success, save ICCID
		if (memcmp(node.iccid, iccidTmp, ICCID_SIZE) != 0)
		{
			// It isn't the same, save it
			memcpy(node.iccid, iccidTmp, ICCID_SIZE + 1);
			EEPROM.put(ICCID_ADD, node.iccid);
			EEPROM.commit();
		}

		DebugSerial.printf("ICCID: \"%s\"\r\n", node.iccid);

		if (node.wakeUp == WAKEUP_BUTTON)
		{
			AddLine("ICCID OK!", 0);
			DisplayLines();
		}
	}
	else
	{
		DebugSerial.printf("Failed to get ICCID\r\n");
	}

	GSM_TurnOFF();

	display.clearDisplay();
	DisplaySplashScreen();	

	// Get UID from WiFi MAC
	uint8_t j = 0;
	macAddress = WiFi.macAddress();
	macAddress.toCharArray(tempMacAddress, macAddress.length() + 1);

	for (uint8_t i = 9; i < 17; i++)
	{
		if (tempMacAddress[i] != ':')
		{
			tempUID[j] = tempMacAddress[i];
			j++;
		}
	}

	memcpy(node.uID, tempUID, UID_SIZE+1);

	DebugSerial.printf("\r\nWiFi MAC: %s  UID: %s\r\n", tempMacAddress, node.uID);

	// Verify battery level
	BatMeasure();
	
	//Confirm connection type and credentials
	if ((node.connType == 0) || (node.connType > 3))
	{
		//Force WiFi as default connection type
		node.connType = USING_2G;
		EEPROM.put(CONN_TYPE_ADD, node.connType);
		EEPROM.commit();
		vTaskDelay(pdMS_TO_TICKS(100));
	}

	if (node.connType == USING_LTE)  DebugSerial.printf("\r\nConnection Type =  LTE\r\n");
	else if (node.connType == USING_2G) DebugSerial.printf("\r\nConnection Type =  2G\r\n");
	else if (node.connType == USING_WIFI) DebugSerial.printf("\r\nConnection Type =  WiFi\r\n");
	else DebugSerial.printf("\r\nNo Connection Type Selected !\r\n\n");

	EEPROM.get(WIFI_SSID_ADD, routerSSID);
	EEPROM.get(WIFI_PASSWORD_ADD, routerPassword);	

	// Initialize Sensors
	uint8_t errorInitSens = Init_Sensors();	

	if (errorInitSens > NO_SENSORS_ERRORS)
	{
		DebugSerial.printf("\r\nError Initializing Sensors: %d\n", errorInitSens);

		if (node.wakeUp == WAKEUP_BUTTON)
		{
			Set_Color(NO_COLOR);
			vTaskDelay(pdMS_TO_TICKS(250));
			Set_Color(ORANGE);
			vTaskDelay(pdMS_TO_TICKS(1000));
		}
	}
	else
	{
		if (node.wakeUp == WAKEUP_BUTTON)
		{
			Set_Color(NO_COLOR);
			vTaskDelay(pdMS_TO_TICKS(250));
			Set_Color(WHITE_CLR);
			vTaskDelay(pdMS_TO_TICKS(1000));
		}
	}

	DebugSerial.println("\r\nEnd Configs\r\n");

	// Battery, Solar Panel and USB read - after stabilization
	ADC_Measures();

	// Setup buttons interrupt
	attachInterrupt(DIG_IN_1, btnISR, FALLING);
	attachInterrupt(DIG_IN_2, btn2ISR, FALLING);


	// Task 1
	xTaskCreatePinnedToCore(
		task1s,				/* Function to implement the task */
		"task1s",			/* Name of the task */
		8192,				/* Stack size in words */
		NULL,				/* Task input parameter */
		4,					/* Priority of the task */
		&taskHandle1s,		/* Task handle. */
		1);					/* Core where the task should run */	

#ifdef USING_LOADCELL

	// Taks 2
	xTaskCreatePinnedToCore(
		taskCom,			/* Function to implement the task */
		"taskCom",			/* Name of the task */
		4096,				/* Stack size in words */
		NULL,				/* Task input parameter */
		2,					/* Priority of the task */
		&taskHandleConn,	/* Task handle. */
		0);					/* Core where the task should run */

	// Task 3
	if (pbPresent)	// Only if piggyback present it will do its tasks
	{		
		xTaskCreatePinnedToCore(
			taskLC,				/* Function to implement the task */
			"taskLC",			/* Name of the task */
			8192,				/* Stack size in words */
			NULL,				/* Task input parameter */
			3,					/* Priority of the task */
			&taskHandleLc,		/* Task handle. */
			1);					/* Core where the task should run */
	}

#endif	
	
	// Task 4
	xTaskCreatePinnedToCore(
		taskTickCnt,		/* Function to implement the task */
		"taskTickCnt",		/* Name of the task */
		1024,				/* Stack size in words */
		NULL,				/* Task input parameter */
		1,					/* Priority of the task */
		&taskHandleTicks,	/* Task handle. */
		0);					/* Core where the task should run */

	// Taks 5
	xTaskCreatePinnedToCore(
		taskRF,				/* Function to implement the task */
		"taskRF",			/* Name of the task */
		9216,				/* Stack size in words */
		NULL,				/* Task input parameter */
		1,					/* Priority of the task */
		&taskHandleRf,		/* Task handle. */
		1);					/* Core where the task should run */

	// Taks 6
	xTaskCreatePinnedToCore(
		taskTime2PrintTel,		/* Function to implement the task */
		"taskTime2PrintTel",	/* Name of the task */
		4096,					/* Stack size in words */
		NULL,					/* Task input parameter */
		2,						/* Priority of the task */
		&taskHandlePrint,		/* Task handle. */
		1);						/* Core where the task should run */

	// Initialize watchdog
	esp_err_t wdtRes = esp_task_wdt_init(5, true);	// Task watchdog timer
	esp_int_wdt_init();								// Interrupt watchdog timer
}


void loop()
{
}

//Declare reset function at address 0
void(*resetFunc) (void) = 0;

void PinSetup(void)
{
	pinMode(RS485_TX_EN, OUTPUT);				// MAX485_DE_RE
	digitalWrite(RS485_TX_EN, LOW);				// RS485 starts in receive mode

	pinMode(VCC_3_EN, OUTPUT);
	digitalWrite(VCC_3_EN, HIGH);				// Enable VCC 3

	pinMode(BAT_PLUS_MEASURE, INPUT);			// BAT+ MEASURE
	pinMode(MA_ADC, INPUT);
	pinMode(V_ADC, INPUT);

	pinMode(PV_PLUS_MEASURE, INPUT);			// PV+ MEASURE
	pinMode(POUT_ADC, INPUT);					// POUT_ADC
	pinMode(VUSB_PLUS_MEASURE, INPUT);			// VUSB_PLUS_MEASURE

	pinMode(BTN, INPUT);						// Pin to read a the power button press
	pinMode(DIG_IN_1, INPUT);
	pinMode(DIG_IN_2, INPUT);

	pinMode(RF_INT, INPUT);                     // LORA_INT - RF interrupt
	pinMode(RF_NSS, OUTPUT);					// LORA_NSS - RF chip select
	pinMode(RF_RESET, OUTPUT);                  // LORA_RESET - RF reset

	// ADC initialization
	adc1_config_width(ADC_WIDTH_12Bit);								// Voltages between 150mV -> 2450mV for 11dB attenuation
	adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11);		// PV_ADC	
	adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11);		// VBAT-ADC
	adc1_config_channel_atten(ADC1_CHANNEL_4, ADC_ATTEN_DB_11);		// POUT_ADC
}


// Initialize Expander Pins
bool SYS_ConfExpIO(void)
{
	// Initialize Expander IO
	if (expander.begin(0x20, SDA_PIN, SCL_PIN) == false) return false;

	// Disable outputs
	Enable_GPS(false);
	Enable_DIG_OUT_1(false);
	Enable_DIG_OUT_2(false);
	Enable_BOOST(false);		 // Disables RS485 12V for RS485 interface
	Enable_VBAT_2(false);
	Enable_VCC_2(false);
	Enable_POWERKEY(false);

	return true;
}


// Display Methods
void SetupDisplay(void)
{
	Enable_VCC_3(true);
	vTaskDelay(pdMS_TO_TICKS(1000));

	// SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
	if (display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
	{
		DebugSerial.println();
		DebugSerial.println("OLED init ok");
	}
	else
	{
		DebugSerial.println();
		DebugSerial.println("OLD init failed !");
	}

	// Clear the buffer
	display.clearDisplay();

	// Set defaults
	display.setTextSize(1);				// Normal 1:1 pixel scale
	display.setTextColor(WHITE);		// Draw white text
	display.setCursor(0, 0);			// Start at top-left corner
	display.cp437(true);				// Use full 256 char 'Code Page 437' font	

	display.clearDisplay();
}


void ShutDownDisplay(void)
{
	//DisplaySplashScreen();     // Keeps splash screen in memeory for next iteraction
	vTaskDelay(pdMS_TO_TICKS(2000));

	display.ssd1306_command(SSD1306_DISPLAYOFF);
	vTaskDelay(pdMS_TO_TICKS(100));
}


void DisplaySplashScreen(void)
{
	display.clearDisplay();
	display.setTextSize(2);
	display.setCursor(X_INIT_POS+12, Y_INIT_POS+1);
	display.println("ATRONIA");
	display.setTextSize(1);
	display.setCursor(X_INIT_POS+8, Y_INIT_POS+23);
	display.println("Tailored Sensing");
}


// Methods for WiFi management 
int8_t ConnectWifi(char* ssid, char* password)
{
	unsigned long initTime = millis();

	// WiFi Client connection
	DebugSerial.println();
	DebugSerial.printf("\r\n********** WiFi Client Connection **********");
	DebugSerial.printf("\r\nConnecting to ");
	DebugSerial.print(routerSSID);
	DebugSerial.println();

	WiFi.begin(ssid, password);

	while ((WiFi.status() != WL_CONNECTED) && ((millis() - initTime) < WIFI_TIMEOUT))
	{
		vTaskDelay(pdMS_TO_TICKS(500));
		DebugSerial.print(".");
	}

	if (WiFi.status() == WL_CONNECTED)
	{
		DebugSerial.println();
		DebugSerial.printf("\r\nWiFi connected");
		DebugSerial.printf("\r\nIP address: ");
		DebugSerial.print(WiFi.localIP());
		DebugSerial.println();
		return 1;
	}
	else
	{
		DebugSerial.printf("\r\nWiFi connection failed !\r\n");
		return -1;
	}
}


int8_t CheckWifiConnection(void)
{
	if (WiFi.status() == WL_CONNECTED) return 1;
	else return -1;
}


bool WiFi_GetTime(char* date, char* hr, char* hhmmss)
{
	char year[5];
	char month[3];
	char day[3];
	char h[3];
	char min[3];
	char sec[3];

	configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
	if (!getLocalTime(&timeinfo))
	{
		DebugSerial.printf("\r\nFailed to obtain time !\r\n");
		return false;
	}
	else
	{
		itoa(timeinfo.tm_year + 1900, year, 10);
		itoa(timeinfo.tm_mon + 1, month, 10);
		itoa(timeinfo.tm_mday, day, 10);
		itoa(timeinfo.tm_hour, h, 10);
		itoa(timeinfo.tm_min, min, 10);
		itoa(timeinfo.tm_sec, sec, 10);

		if (strlen(month) == 1)
		{
			month[1] = month[0];
			month[0] = '0';
		}

		if (strlen(day) == 1)
		{
			day[1] = day[0];
			day[0] = '0';
		}

		if (strlen(h) == 1)
		{
			h[1] = h[0];
			h[0] = '0';
		}

		if (strlen(min) == 1)
		{
			min[1] = min[0];
			min[0] = '0';
		}

		if (strlen(sec) == 1)
		{
			sec[1] = sec[0];
			sec[0] = '0';
		}

		memcpy(&date[0], year, 4);
		date[4] = char(47); // -> /
		memcpy(&date[5], month, 2);
		date[7] = char(47);
		memcpy(&date[8], day, 2);
		date[10] = '\0';

		memcpy(&hr[0], h, 2);
		hr[2] = char(58); // :
		memcpy(&hr[3], min, 2);
		hr[5] = '\0';

		memcpy(&hhmmss[0], h, 2);
		hhmmss[2] = char(58); // :
		memcpy(&hhmmss[3], min, 2);
		hhmmss[5] = char(58);
		memcpy(&hhmmss[6], sec, 2);
		hhmmss[8] = '\0';

		return true;
	}
}


// Methods to handle GPS messages
void ClearGPSBuffer(void)
{
	memset(gpsBuffer, '\0', GPS_MSG_LEN);
}


void Read_GPS(void)
{
	int cnt = 0;

	Enable_VCC_3(true);

	EspSerial2.begin(GPS_BAUDRATE, SERIAL_8N1, ESP_RXD_2, ESP_TXD_2);
	vTaskDelay(pdMS_TO_TICKS(100));
	EspSerial2.flush();

	DebugSerial.printf("\r\nReading GPS ...\r\n");

	Enable_GPS(true);
	vTaskDelay(pdMS_TO_TICKS(GPS_WAIT_MSG));	// Time limit to receive GPS message
	Enable_GPS(false);							// Ensures USART 2 selector to RS485 (default)

	cnt = EspSerial2.available();
	if ((cnt > 0) && (cnt < GPS_MSG_LEN))
	{
		EspSerial2.read(gpsBuffer, cnt);

		DebugSerial.printf("GPS message[%d]: \r\n", cnt);
		DebugSerial.printf("%s\r\n", gpsBuffer);
	}
	else
	{
		DebugSerial.printf("No message received from GPS\r\n");
	}

	DebugSerial.println();

	ClearGPSBuffer();

	EspSerial2.flush();

	EspSerial2.end();
}


// Methods to enable/disable modules through Expander IOs
// action = true -> Enable (high)
// action = false -> Disable (low)

void Enable_GPS(bool action)
{
	expander.pinMode(GPS_EN, OUTPUT);

	if (action == false)
	{
		expander.digitalWrite(GPS_EN, LOW);
		USART2_SelectSource(0);  // Selector on RS485 (default)
	}
	else
	{
		expander.digitalWrite(GPS_EN, HIGH);
		USART2_SelectSource(1);  // Change selector to GPS
	}
}


void Enable_DIG_OUT_1(bool action)
{
	expander.pinMode(DIG_OUT_EN_1, OUTPUT);

	if (action == false) expander.digitalWrite(DIG_OUT_EN_1, LOW);
	else expander.digitalWrite(DIG_OUT_EN_1, HIGH);
}


void Enable_DIG_OUT_2(bool action)
{
	expander.pinMode(DIG_OUT_EN_2, OUTPUT);

	if (action == false) expander.digitalWrite(DIG_OUT_EN_2, LOW);
	else expander.digitalWrite(DIG_OUT_EN_2, HIGH);
}


void Enable_BOOST(bool action)
{
	expander.pinMode(BOOST_EN, OUTPUT);

	if (action == false) expander.digitalWrite(BOOST_EN, LOW);
	else
	{
		expander.digitalWrite(BOOST_EN, HIGH);
		vTaskDelay(pdMS_TO_TICKS(1000));  // Time for BOOST stabilization
	}
}


void Enable_VBAT_2(bool action)
{
	expander.pinMode(VBAT_2_EN, OUTPUT);

	if (action == false) expander.digitalWrite(VBAT_2_EN, LOW);
	else expander.digitalWrite(VBAT_2_EN, HIGH);
}


void Enable_VCC_2(bool action)
{
	expander.pinMode(VCC_2_EN, OUTPUT);

	if (action == false) expander.digitalWrite(VCC_2_EN, LOW);
	else expander.digitalWrite(VCC_2_EN, HIGH);
}


void Enable_VCC_3(bool action)
{
	pinMode(VCC_3_EN, OUTPUT);

	if (action == false) digitalWrite(VCC_3_EN, LOW);
	else digitalWrite(VCC_3_EN, HIGH);
}


void Enable_POWERKEY(bool action)
{
	expander.pinMode(POWERKEY, OUTPUT);

	if (action == false) expander.digitalWrite(POWERKEY, LOW);
	else expander.digitalWrite(POWERKEY, HIGH);
}


// Method for USART 2 source selection
// 0 -> RS485
// 1 -> GPS

void USART2_SelectSource(uint8_t source)
{
	expander.pinMode(USART_SEL, OUTPUT);

	if (source == 0) expander.digitalWrite(USART_SEL, LOW);
	else expander.digitalWrite(USART_SEL, HIGH);
}


void Sys_ResetVerification(void)
{
	uint16_t ms1000Cnt = 0;

	// If button pressed - verify if is an order to reset
	if (digitalRead(BTN) == LOW)
	{
		// The user could be asking for reset
		while (digitalRead(BTN) == LOW)
		{
			delay(1000);
			ms1000Cnt++;

			// Manage LED indication
			if ((ms1000Cnt % 5) == 0)
			{
				Blink_LED(ORANGE, 100);
			}

			// An reset is going to be done - blinks twice in blue
			if (ms1000Cnt > BUTTON_PRESS_RST_TIME)
			{
				//In the next power up the board will have factory values - Save factory flag
				EEPROM.put(FACTORY_ADD, 0xFF);

				// Led indication
				Set_Color(NO_COLOR);
				delay(50);
				Blink_LED(CYAN, 100);
				Blink_LED(CYAN, 100);

				// Set fs to default
				node.fs = 0;

				DebugSerial.println("Factory values!");

				// Give time to the user take off the finger
				Set_Color(CYAN);
				delay(1000);

				// Restart ESP to set the factory values
				ESP.restart();
			}
		}

	} // END if (digitalRead(BTN_INT1) == 1)	
}


// Function to enable the battery measure and get the ADC value
uint16_t GetADCMeas(uint8_t pin)
{
	uint16_t rawAdc = 0;
	uint16_t rawBuffer[N_SAMPLES] = { 0 };
	uint16_t meanBuffer[N_MEAN_SAMPLES] = { 0 };

	//Enable_VBAT_2(true);   // Enables output VBAT_2 for VBat measurement
	//vTaskDelay(pdMS_TO_TICKS(100));

	// Get Raw Battery value
	for (uint8_t i = 0; i < N_SAMPLES; i++)
	{
		vTaskDelay(pdMS_TO_TICKS(50));
		rawBuffer[i] = analogRead(pin);
	}

	/* Set the order of the samples read */
	sortInt(rawBuffer, N_SAMPLES);

	// Set samples to make mean value
	meanBuffer[0] = rawBuffer[MED_TO_MEAN_0];
	meanBuffer[1] = rawBuffer[MED_TO_MEAN_1];
	meanBuffer[2] = rawBuffer[MED_TO_MEAN_2];

	// Get mean and final value
	rawAdc = meanInt(meanBuffer, N_MEAN_SAMPLES);

	//Enable_VBAT_2(false);   // Enables output VBAT_2 for VBat initial measurement

	return rawAdc;
}


void Calib_BatMeasure(void)
{
	uint8_t batCalib = 0;
	char batCmd[10] = { '\0' };
	uint8_t batCmdInd = 0;
	uint8_t batCmdRcv = 0;


	DebugSerial.println("Battery calibration Started!");

	//Enable_VBAT_2(true);				//Enables battery level to ADC
	//vTaskDelay(pdMS_TO_TICKS(1000));	//Time for level stabilization

	while (batCalib == 0)
	{
		// Get characters
		while (DebugSerial.available() > 0)
		{
			batCmd[batCmdInd] = (char)(DebugSerial.read());
			batCmdInd++;

			//DebugSerial.printf("\r\n %d = %c", batCmdInd - 1, batCmd[batCmdInd - 1]);

			if (((batCmdInd > 8) && (batCmd[batCmdInd - 2] == '#')) || (batCmdInd > 9))
			{
				batCmdRcv = 1;
				break;
			}
		}

		// Is there a new command ? Process it !
		if (batCmdRcv == 1)
		{
			if (strcmp(BAT_MIN_CMD, batCmd) == 0)
			{
				node.batMinADC = (GetADCMeas(BAT_PLUS_MEASURE)) - 10;

				DebugSerial.printf("Battery Minimum Value Set: %d\n", node.batMinADC);
			}
			else if (strcmp(BAT_MAX_CMD, batCmd) == 0)
			{
				node.batMaxADC = (GetADCMeas(BAT_PLUS_MEASURE)) + 10;

				DebugSerial.printf("Battery Maximum Value Set: %d\n", node.batMaxADC);
			}

			batCmdInd = 0;
			batCmdRcv = 0;
			memset(batCmd, 0x00, 10);
		}

		// Verify if the limits are already set
		if ((node.batMinADC != 0xFFFF) && (node.batMaxADC != 0xFFFF))
		{
			// Save values to memory
			EEPROM.put(BAT_MIN_ADC_ADD, node.batMinADC);
			EEPROM.put(BAT_MAX_ADC_ADD, node.batMaxADC);
			EEPROM.commit();

			batCalib = 1;
		}
	}

	// Already set - values are going to be saved in the next steps
	//Enable_VBAT_2(false);  //Disables battery level to ADC

	DebugSerial.println("Battery calibration Completed!");
}


void Set_Color(uint8_t color)
{
	expander.pinMode(LED_R, OUTPUT);
	expander.pinMode(LED_G, OUTPUT);
	expander.pinMode(LED_B, OUTPUT);

	switch (color)
	{
		// LOW=ON HIGH=OFF
		case NO_COLOR:
			expander.digitalWrite(LED_R, HIGH);
			expander.digitalWrite(LED_G, HIGH);
			expander.digitalWrite(LED_B, HIGH);
			break;
		case WHITE_CLR:
			expander.digitalWrite(LED_R, LOW);
			expander.digitalWrite(LED_G, LOW);
			expander.digitalWrite(LED_B, LOW);
			break;
		case BLUE:
			expander.digitalWrite(LED_R, HIGH);
			expander.digitalWrite(LED_G, HIGH);
			expander.digitalWrite(LED_B, LOW);
			break;
		case GREEN:
			expander.digitalWrite(LED_R, HIGH);
			expander.digitalWrite(LED_G, LOW);
			expander.digitalWrite(LED_B, HIGH);
			break;
		case RED:
			expander.digitalWrite(LED_R, LOW);
			expander.digitalWrite(LED_G, HIGH);
			expander.digitalWrite(LED_B, HIGH);
			break;
		case ORANGE:
			expander.digitalWrite(LED_R, LOW);
			expander.digitalWrite(LED_G, LOW);
			expander.digitalWrite(LED_B, HIGH);
			break;
		case CYAN:
			expander.digitalWrite(LED_R, HIGH);
			expander.digitalWrite(LED_G, LOW);
			expander.digitalWrite(LED_B, LOW);
			break;
		case VIOLET:
			expander.digitalWrite(LED_R, LOW);
			expander.digitalWrite(LED_G, HIGH);
			expander.digitalWrite(LED_B, LOW);
			break;
		default:
			expander.digitalWrite(LED_R, HIGH);
			expander.digitalWrite(LED_G, HIGH);
			expander.digitalWrite(LED_B, HIGH);
			break;
	}
}


void Blink_LED(uint8_t color, uint16_t len)
{
	Set_Color(color);
	vTaskDelay(pdMS_TO_TICKS(len));
	Set_Color(NO_COLOR);
	vTaskDelay(pdMS_TO_TICKS(len));
}


void ShortLEDblink_LED(uint8_t color)
{
	Set_Color(NO_COLOR);
	Set_Color(color);
	vTaskDelay(pdMS_TO_TICKS(50));
	Set_Color(NO_COLOR);
}


uint8_t Init_Sensors(void)
{
	uint8_t error = NO_SENSORS_ERRORS;
	
	// Turn On the RS485 power and set RX mode
	Enable_BOOST(true);

	Enable_VCC_3(true);

	USART2_SelectSource(0);

	RS485_RxMode();

	EspSerial2.begin(MODBUS_BAUDRATE, SERIAL_8N1, ESP_RXD_2, ESP_TXD_2);

	// Force pullup on RX
	pinMode(ESP_RXD_2, INPUT_PULLUP);

	// On-board temperature sensor
	if (!tIntSens.begin(INTERNAL_MCP_ADD)) error += MCP9808_INT_ERROR;
	else tIntSens.setResolution(3);

#ifdef USING_LOADCELL

	// Initialize loadcell piggyback
	if (Init_Loadcell_PiggyBack(PIGGYBACK_I2C_ADD) == 1)
	{
		DebugSerial.printf("\r\nPiggyback initialized !\r\n");
		pbPresent = true;
	}
	else
	{
		DebugSerial.printf("\r\nERROR initializing Piggyback !!!\r\n");
		error += PIGGYBACK_ERROR;
	}

#endif // USING_LOADCELL	

	// Successful initialization
	return error;
}


#ifdef USING_LOADCELL

uint8_t SetConfigReg(void)
{
	uint8_t configAux = 0x00;

	if (pb.lc1.gain == 64) configAux = 0;
	else configAux = 1;

	if (pb.lc1.sr == 10) configAux |= (0 << 1);
	else configAux |= (1 << 1);

	if (pb.lc2.gain == 64) configAux |= (0 << 2);
	else configAux |= (1 << 2);

	if (pb.lc2.sr == 10) configAux |= (0 << 3);
	else configAux |= (1 << 3);

	if (pb.relayStt == OFF) configAux |= (0 << 4);
	else configAux |= (1 << 4);

	if (pb.gpsEn == OFF) configAux |= (0 << 5);
	else configAux |= (1 << 5);

#ifdef DEBUG_LOADCELL
	DebugSerial.printf("\r\nConfig register to config piggyback: 0x%02X", configAux);
#endif

	return configAux;
}


// Initialize loadcell piggyback
uint8_t Init_Loadcell_PiggyBack(uint8_t i2cAddress)
{
	// Define number of loadcells 
	pb.lc1.nr = 1;
	pb.lc2.nr = 2;

	
	switch (pb.lc1.sensorId)
	{
		case CH1_W1:
			pb.lc1.w4stt = false;
			pb.lc1.gain = 128;
			pb.lc1.sr = 10;
			//pb.lc1.sr = 80;
			pb.lc1.maxADCmV = 3.0 * 2.0;	// 2mV/V
			pb.lc1.Nmax = 10000;

			break;

		case CH1_W2:
			pb.lc1.w4stt = false;
			pb.lc1.gain = 128;
			pb.lc1.sr = 10;
			//pb.lc1.sr = 80;
			pb.lc1.maxADCmV = 3.0 * 2.0;	// 2mV/V
			pb.lc1.Nmax = 20000;

			break;

		case CH1_W3:
			pb.lc1.w4stt = true;
			pb.lc1.gain = 128;
			pb.lc1.sr = 10;
			//pb.lc1.sr = 80;
			pb.lc1.maxADCmV = 3.0 * 2.0;	// 2mV/V
			pb.lc1.Nmax = 2000;

			break;

		case CH1_D1:
			pb.lc1.w4stt = true;
			pb.lc1.gain = 128;
			pb.lc1.sr = 10;
			//pb.lc1.sr = 80;
			pb.lc1.maxADCmV = 3.0 * 2.0;


			break;

		case CH1_D2:
			pb.lc1.w4stt = true;
			pb.lc1.gain = 128;
			pb.lc1.sr = 10;
			//pb.lc1.sr = 80;
			pb.lc1.maxADCmV = 3.0 * 2.0;

			break;

		case CH_NO_SENSOR:
			pb.lc1.w4stt = true;
			pb.lc1.gain = 128;
			pb.lc1.sr = 10;
			//pb.lc1.sr = 80;
			pb.lc1.maxADCmV = 3.0 * 2.0;

			break;


		default:

			break;
	}

	switch (pb.lc2.sensorId)
	{
		case CH2_W1:
			pb.lc2.w4stt = false;
			pb.lc2.gain = 128;
			pb.lc2.sr = 10;
			//pb.lc2.sr = 80;
			pb.lc2.maxADCmV = 3.0 * 2.0;
			pb.lc2.Nmax = 10000;

			break;

		case CH2_W2:
			pb.lc2.w4stt = false;
			pb.lc2.gain = 128;
			pb.lc2.sr = 10;
			//pb.lc2.sr = 80;
			pb.lc2.maxADCmV = 3.0 * 2.0;
			pb.lc2.Nmax = 20000;

			break;

		case CH2_W3:
			pb.lc2.w4stt = true;
			pb.lc2.gain = 128;
			pb.lc2.sr = 10;
			//pb.lc2.sr = 80;
			pb.lc2.maxADCmV = 3.0 * 2.0;
			pb.lc2.Nmax = 2000;

			break;

		case CH2_D1:
			pb.lc2.w4stt = true;
			pb.lc2.gain = 128;
			pb.lc2.sr = 10;
			//pb.lc2.sr = 80;
			pb.lc2.maxADCmV = 3.0 * 2.0;

			break;

		case CH2_D2:
			pb.lc2.w4stt = true;
			pb.lc2.gain = 128;
			pb.lc2.sr = 10;
			//pb.lc2.sr = 80;
			pb.lc2.maxADCmV = 3.0 * 2.0;

			break;
		
		case CH_NO_SENSOR:
			pb.lc2.w4stt = true;
			pb.lc2.gain = 128;
			pb.lc2.sr = 10;
			//pb.lc1.sr = 80;
			pb.lc2.maxADCmV = 3.0 * 2.0;

			break;


		default:

			break;
	}

	// Loadcells telemtry structure initialization
	telemEcit[0].telemEcitLc1.sensorId = pb.lc1.sensorId;
	telemEcit[0].telemEcitLc1.absUpperForceLimit = pb.lc1.absUpperForceLimit;
	telemEcit[0].telemEcitLc1.upperForceOvesteppings = pb.lc1.upperForceOvesteppings;
	telemEcit[0].telemEcitLc1.lowerForceOvesteppings = pb.lc1.lowerForceOvesteppings;
	telemEcit[0].telemEcitLc1.absLowerForceLimit = pb.lc1.absLowerForceLimit;
	telemEcit[0].telemEcitLc1.nrUpperForceOversteppings = pb.lc1.nrUpperForceOversteppings;
	telemEcit[0].telemEcitLc1.nrLowerForceOversteppings = pb.lc1.nrLowerForceOversteppings;
	telemEcit[0].telemEcitLc1.forceOffset = pb.lc1.forceOffset;
	telemEcit[0].telemEcitLc1.relayState = pb.relayStt;

	telemEcit[0].telemEcitLc2.sensorId = pb.lc2.sensorId;
	telemEcit[0].telemEcitLc2.absUpperForceLimit = pb.lc2.absUpperForceLimit;
	telemEcit[0].telemEcitLc2.upperForceOvesteppings = pb.lc2.upperForceOvesteppings;
	telemEcit[0].telemEcitLc2.lowerForceOvesteppings = pb.lc2.lowerForceOvesteppings;
	telemEcit[0].telemEcitLc2.absLowerForceLimit = pb.lc2.absLowerForceLimit;
	telemEcit[0].telemEcitLc2.nrUpperForceOversteppings = pb.lc2.nrUpperForceOversteppings;
	telemEcit[0].telemEcitLc2.nrLowerForceOversteppings = pb.lc2.nrLowerForceOversteppings;
	telemEcit[0].telemEcitLc2.forceOffset = pb.lc2.forceOffset;
	telemEcit[0].telemEcitLc2.relayState = pb.relayStt;

	// Build config settings 
	uint8_t conf = SetConfigReg();

	if (xSemaphoreTake(xI2CSemaphore, (TickType_t)50) == pdTRUE)
	{
		Wire.beginTransmission(i2cAddress);		// Begin I2C transmission with Address device

		// Receive 0 = success (ACK response) 
		if (Wire.endTransmission() != 0)
		{
			xSemaphoreGive(xI2CSemaphore);
			return 0;
		}

		// Set piggyback settings 
		Wire.beginTransmission(i2cAddress);
		Wire.write(SET_CONFIG);					/* Command byte */
		Wire.write(conf);						/* Data bytes */
		Wire.endTransmission();

		xSemaphoreGive(xI2CSemaphore);
	}

	return 1;
}


uint8_t Get_LCvals(uint8_t i2cAddress, uint8_t op)
{
	uint8_t buff[9];
	uint8_t i = 0;
	uint8_t result = 0;
	bool overrun = false;

	if (xSemaphoreTake(xI2CSemaphore, (TickType_t)50) == pdTRUE)
	{
		/* Set the register address to read */
		Wire.beginTransmission(i2cAddress);
		Wire.write(GET_LC_VAL);						// Command byte
		Wire.write(op);								// Ask for the numeber of loadcells desired
		Wire.endTransmission();

		if (op == OP_LC1_LC2)
			Wire.requestFrom(i2cAddress, 8);		// 4 bytes for each val for loadcell
		else
			Wire.requestFrom(i2cAddress, 4);		// 4 bytes for each val for loadcell

#ifdef DEEP_DEBUG_LOADCELL
		DebugSerial.println("Bytes received:");
#endif

		while (Wire.available() > 0)					
		{
			// Protect overun error
			if (i >= 8)
			{
				xSemaphoreGive(xI2CSemaphore);
				return 0;
			}

			buff[i] = (uint8_t)(Wire.read());

#ifdef DEEP_DEBUG_LOADCELL
			DebugSerial.printf("%02X ", buff[i]);
#endif
			i++;
		}

//		while (Wire.available() > 0)
//		{
//			// Protect overun error
//			if (i >= 8)
//			{
//				overrun = true; 
//				break;
// 		    }
//
//			buff[i] = (uint8_t)(Wire.read());
//
//#ifdef DEEP_DEBUG_LOADCELL
//			DebugSerial.printf("%02X ", buff[i]);
//#endif
//			i++;
//		}

#ifdef DEEP_DEBUG_LOADCELL
		DebugSerial.println("\r\nEnd receiving bytes");
#endif

		xSemaphoreGive(xI2CSemaphore); // Now free or "Give" the Serial Port for others.

		//if (overrun) return 0;
	}

	// Verify bytes received
	switch (op)
	{
	case OP_LC1_LC2:
		if (i == 8)
		{
			// Loadcell 1 val
			uI32.b[0] = buff[0];						/* LSB first */
			uI32.b[1] = buff[1];
			uI32.b[2] = buff[2];
			uI32.b[3] = buff[3];
			pb.lc1.valRaw = uI32.val;

#ifdef DEEP_DEBUG_LOADCELL
			DebugSerial.printf("\r\nBytes received LC1: %02X %02X %02X %02X\r\n", uI32.b[0], uI32.b[1], uI32.b[2], uI32.b[3]);
#endif
			// Loadcell 2 val
			uI32.b[0] = buff[4];						/* LSB first */
			uI32.b[1] = buff[5];
			uI32.b[2] = buff[6];
			uI32.b[3] = buff[7];
			pb.lc2.valRaw = uI32.val;

#ifdef DEEP_DEBUG_LOADCELL
			DebugSerial.printf("Bytes received LC2: %02X %02X %02X %02X\r\n", uI32.b[0], uI32.b[1], uI32.b[2], uI32.b[3]);
#endif				
			result = 1;
		}
		else
		{
			result = 0;
		}
		break;
	case OP_LC1:
		if (i == 4)
		{
			// Loadcell 1 val
			uI32.b[0] = buff[0];						/* LSB first */
			uI32.b[1] = buff[1];
			uI32.b[2] = buff[2];
			uI32.b[3] = buff[3];
			pb.lc1.valRaw = uI32.val;

#ifdef DEEP_DEBUG_LOADCELL				
			DebugSerial.printf("\r\nBytes received LC1: %02X %02X %02X %02X\r\n", uI32.b[0], uI32.b[1], uI32.b[2], uI32.b[3]);
#endif				
			result = 1;
		}
		else
		{
			result = 0;
		}
		break;
	case OP_LC2:
		if (i == 4)
		{
			// Loadcell 2 val
			uI32.b[0] = buff[0];						/* LSB first */
			uI32.b[1] = buff[1];
			uI32.b[2] = buff[2];
			uI32.b[3] = buff[3];
			pb.lc2.valRaw = uI32.val;

#ifdef DEEP_DEBUG_LOADCELL
			DebugSerial.printf("Bytes received LC2: %02X %02X %02X %02X\r\n", uI32.b[0], uI32.b[1], uI32.b[2], uI32.b[3]);
#endif
			result = 1;
		}
		else
		{
			result = 0;
		}
		break;
	default:
		result = 0;
		break;
	}

	return result;
}


void GetLCmeanRaw(LOADCELL_STRUCT* lc)
{
	// Save sample to buffer
	lc->sampleBuff[lc->sampleBuffIdx] = lc->valRaw;

#ifdef DEEP_DEBUG_LOADCELL
	DebugSerial.printf("LC%d | Val %d mean buff: %d\r\n", lc->nr, lc->sampleBuffIdx, lc->sampleBuff[lc->sampleBuffIdx]);

	if ((lc->valRaw > 8000000) || (lc->valRaw < -8000000) || (abs(lc->oldValRaw - lc->valRaw) > 1000))
	{
		DebugSerial.printf("ERROR on LC%d | Val: %d | Old val: %d\r\n", lc->nr, lc->valRaw, lc->oldValRaw);

		DebugSerial.println(" Sample Buffer: ");

		for (uint8_t i = 0; i < N_PB_SAMPLES_BUFFER; i++)
		{
			DebugSerial.printf("%d | ", lc->sampleBuff[i]);
		}

		DebugSerial.println("\r\nEnd buffer");
	}
#endif	

	// Save last sample
	lc->oldValRaw = lc->valRaw;

	// Increment mean buff index - Circular buffer to filter signal
	if (lc->sampleBuffIdx < (N_PB_SAMPLES_BUFFER - 1)) lc->sampleBuffIdx++;
	else lc->sampleBuffIdx = 0;


	// Sort and get mean value
	sortI32(lc->sampleBuff, N_PB_SAMPLES_BUFFER);

	// Copy samples to mean buffer
	for (uint8_t i = 0; i < N_PB_MEAN_BUFFER; i++)
	{
		lc->meanBuff[i] = lc->sampleBuff[i + 2];
	}

	lc->valMean = meanI32(lc->meanBuff, N_PB_MEAN_BUFFER);

#ifdef DEEP_DEBUG_LOADCELL
	DebugSerial.printf("Val mean: %d | Mean Buffer:\r\n", lc->valMean);

	for (uint8_t i = 0; i < N_PB_MEAN_BUFFER; i++)
	{
		DebugSerial.printf("%d | ", lc->meanBuff[i]);
	}

	DebugSerial.println("\r\nEnd buffer");
#endif	
}


double Calc_Newtons(LOADCELL_STRUCT* lc)
{
	double mV;
	double result;
	
	mV = ( lc->lsb * ((double)lc->valMean) ) / ( (double)lc->gain );	// mV = (lsb * rawADC) / adcGain	
	result = (((double)lc->Nmax) * mV) / lc->maxADCmV;							// result = (Nmax * mV) / maxADCmV

	return result;
}


void LoadcellsAggregations(void)
{
	pb.secSampleCnt++;
	
	if (pb.lc1.newtons != 0.0)	// Discard "zero" samples
	{
		if (pb.lc1.newtons > 0.0)
		{
			pb.lc1.newtonsPosMeanSecAcc += (int32_t)pb.lc1.newtons;
			pb.lc1.secPosSampleCnt++;
		}
		else
		{
			pb.lc1.newtonsNegMeanSecAcc += (int32_t)pb.lc1.newtons;
			pb.lc1.secNegSampleCnt++;
		}

		if ((int32_t)pb.lc1.newtons < dataLc1.minActForceSec) dataLc1.minActForceSec = (int32_t)pb.lc1.newtons;
		if ((int32_t)pb.lc1.newtons > dataLc1.maxActForceSec) dataLc1.maxActForceSec = (int32_t)pb.lc1.newtons;

		if ((int32_t)pb.lc1.newtons < dataLc1.minActForceMin) dataLc1.minActForceMin = (int32_t)pb.lc1.newtons;
		if ((int32_t)pb.lc1.newtons > dataLc1.maxActForceMin) dataLc1.maxActForceMin = (int32_t)pb.lc1.newtons;
	}
	
	if (pb.lc2.newtons != 0.0)	// Discard "zero" samples
	{
		if (pb.lc2.newtons > 0.0)
		{
			pb.lc2.newtonsPosMeanSecAcc += (int32_t)pb.lc2.newtons;
			pb.lc2.secPosSampleCnt++;
		}
		else
		{
			pb.lc2.newtonsNegMeanSecAcc += (int32_t)pb.lc2.newtons;
			pb.lc2.secNegSampleCnt++;
		}

		if ((int32_t)pb.lc2.newtons < dataLc2.minActForceSec) dataLc2.minActForceSec = (int32_t)pb.lc2.newtons;
		if ((int32_t)pb.lc2.newtons > dataLc2.maxActForceSec) dataLc2.maxActForceSec = (int32_t)pb.lc2.newtons;

		if ((int32_t)pb.lc2.newtons < dataLc2.minActForceMin) dataLc2.minActForceMin = (int32_t)pb.lc2.newtons;
		if ((int32_t)pb.lc2.newtons > dataLc2.maxActForceMin) dataLc2.maxActForceMin = (int32_t)pb.lc2.newtons;
	}

	pb.lc1.newtonsMeanSecAcc += (int32_t)pb.lc1.newtons;
	pb.lc2.newtonsMeanSecAcc += (int32_t)pb.lc2.newtons;

	/******************************************** 1 sec interval aggregation ******************************************************/	
	if ( (pb.secSampleCnt >= (SEC_IN_MS / LC_TASK_RATE_MS)) || ( (pb.relayStt >= ON) && (pb.sendAlert == FALSE ) ) )
	{
		// Loadcell 1
		if (pb.secSampleCnt > 0) dataLc1.meanActForceSec = pb.lc1.newtonsMeanSecAcc / pb.secSampleCnt;
		else dataLc1.meanActForceSec = 0;
		if (pb.lc1.secPosSampleCnt > 0) dataLc1.meanActPosForceSec = pb.lc1.newtonsPosMeanSecAcc / pb.lc1.secPosSampleCnt;
		else dataLc1.meanActPosForceSec = 0;		
		if (pb.lc1.secNegSampleCnt > 0) dataLc1.meanActNegForceSec = pb.lc1.newtonsNegMeanSecAcc / pb.lc1.secNegSampleCnt;
		else dataLc1.meanActNegForceSec = 0;

		if (dataLc1.upperOversteppingsCounterSec > dataLc1.maxUpperOversteppingCountMin) dataLc1.maxUpperOversteppingCountMin = dataLc1.upperOversteppingsCounterSec;
		if (dataLc1.lowerOversteppingsCounterSec > dataLc1.maxLowerOversteppingCountMin) dataLc1.maxLowerOversteppingCountMin = dataLc1.lowerOversteppingsCounterSec;

		telemEcit[0].telemEcitLc1.minActForce = dataLc1.minActForceSec;
		telemEcit[0].telemEcitLc1.maxActForce = dataLc1.maxActForceSec;
		telemEcit[0].telemEcitLc1.meanActForce = dataLc1.meanActForceSec;
		telemEcit[0].telemEcitLc1.meanActPosForce = dataLc1.meanActPosForceSec;
		telemEcit[0].telemEcitLc1.meanActNegForce = dataLc1.meanActNegForceSec;
		telemEcit[0].telemEcitLc1.absUpperLimitCounter = dataLc1.absUpperLimitCounterSec;
		telemEcit[0].telemEcitLc1.upperOversteppingsCounter = dataLc1.upperOversteppingsCounterSec;
		telemEcit[0].telemEcitLc1.absLowerLimitCounter = dataLc1.absLowerLimitCounterSec;
		telemEcit[0].telemEcitLc1.lowerOversteppingsCounter = dataLc1.lowerOversteppingsCounterSec;
		telemEcit[0].telemEcitLc1.relayState = pb.relayStt;

		dataLc1.minActForceSec = LC_NEWTONS_MAX;
		dataLc1.maxActForceSec = LC_NEWTONS_MIN;
		dataLc1.upperOversteppingsCounterSec = 0;
		dataLc1.lowerOversteppingsCounterSec = 0;
		pb.lc1.newtonsMeanSecAcc = 0;
		pb.lc1.newtonsPosMeanSecAcc = 0;
		pb.lc1.newtonsNegMeanSecAcc = 0;
		pb.lc1.secPosSampleCnt = 0;
		pb.lc1.secNegSampleCnt = 0;

		// Loadcell 2
		if (pb.secSampleCnt > 0) dataLc2.meanActForceSec = pb.lc2.newtonsMeanSecAcc / pb.secSampleCnt;
		else dataLc2.meanActForceSec = 0;
		if (pb.lc2.secPosSampleCnt > 0) dataLc2.meanActPosForceSec = pb.lc2.newtonsPosMeanSecAcc / pb.lc2.secPosSampleCnt;
		else dataLc2.meanActPosForceSec = 0;		
		if (pb.lc2.secNegSampleCnt > 0) dataLc2.meanActNegForceSec = pb.lc2.newtonsNegMeanSecAcc / pb.lc2.secNegSampleCnt;
		else dataLc2.meanActNegForceSec = 0;

		if (dataLc2.upperOversteppingsCounterSec > dataLc2.maxUpperOversteppingCountMin) dataLc2.maxUpperOversteppingCountMin = dataLc2.upperOversteppingsCounterSec;
		if (dataLc2.lowerOversteppingsCounterSec > dataLc2.maxLowerOversteppingCountMin) dataLc2.maxLowerOversteppingCountMin = dataLc2.lowerOversteppingsCounterSec;

		telemEcit[0].telemEcitLc2.minActForce = dataLc2.minActForceSec;
		telemEcit[0].telemEcitLc2.maxActForce = dataLc2.maxActForceSec;
		telemEcit[0].telemEcitLc2.meanActForce = dataLc2.meanActForceSec;
		telemEcit[0].telemEcitLc2.meanActPosForce = dataLc2.meanActPosForceSec;
		telemEcit[0].telemEcitLc2.meanActNegForce = dataLc2.meanActNegForceSec;
		telemEcit[0].telemEcitLc2.absUpperLimitCounter = dataLc2.absUpperLimitCounterSec;
		telemEcit[0].telemEcitLc2.upperOversteppingsCounter = dataLc2.upperOversteppingsCounterSec;
		telemEcit[0].telemEcitLc2.absLowerLimitCounter = dataLc2.absLowerLimitCounterSec;
		telemEcit[0].telemEcitLc2.lowerOversteppingsCounter = dataLc2.lowerOversteppingsCounterSec;
		telemEcit[0].telemEcitLc2.relayState = pb.relayStt;

		dataLc2.minActForceSec = LC_NEWTONS_MAX;
		dataLc2.maxActForceSec = LC_NEWTONS_MIN;
		dataLc2.upperOversteppingsCounterSec = 0;
		dataLc2.lowerOversteppingsCounterSec = 0;
		pb.lc2.newtonsMeanSecAcc = 0;
		pb.lc2.newtonsPosMeanSecAcc = 0;
		pb.lc2.newtonsNegMeanSecAcc = 0;
		pb.lc2.secPosSampleCnt = 0;
		pb.lc2.secNegSampleCnt = 0;
		
		pb.secSampleCnt = 0;	// Reset general counter		
		printOrderLc = true;	// Order to print by RS485

		pb.minSampleCnt++;

		pb.lc1.newtonsMeanMinAcc += dataLc1.meanActForceSec;
		pb.lc2.newtonsMeanMinAcc += dataLc2.meanActForceSec;

		if ( (dataLc1.meanActPosForceSec != 0) || (dataLc1.meanActNegForceSec != 0) )
		{
			if (dataLc1.meanActPosForceSec != 0)
			{
				pb.lc1.newtonsPosMeanMinAcc += dataLc1.meanActPosForceSec;
				pb.lc1.minPosSampleCnt++;
			}

			if (dataLc1.meanActNegForceSec != 0)
			{
				pb.lc1.newtonsNegMeanMinAcc += dataLc1.meanActNegForceSec;
				pb.lc1.minNegSampleCnt++;
			}			
		}

		if ( (dataLc2.meanActPosForceSec != 0) || (dataLc2.meanActNegForceSec != 0) )
		{
			if (dataLc2.meanActPosForceSec != 0)
			{
				pb.lc2.newtonsPosMeanMinAcc += dataLc2.meanActPosForceSec;
				pb.lc2.minPosSampleCnt++;
			}

			if (dataLc2.meanActNegForceSec != 0)
			{
				pb.lc2.newtonsNegMeanMinAcc += dataLc2.meanActNegForceSec;
				pb.lc2.minNegSampleCnt++;
			}
		}

		/******************************************* 1 minute interval aggregation ************************************************/
		if ( (pb.minSampleCnt >= SEC_IN_MIN) || ( (pb.relayStt >= ON) && (pb.sendAlert == FALSE) ) || (resumeLcConfRf && !inTxRx) )
		{
			// Loadcell 1
			if (pb.minSampleCnt > 0) dataLc1.meanActForceMin = pb.lc1.newtonsMeanMinAcc / pb.minSampleCnt;
			else dataLc1.meanActForceMin = 0;
			if (pb.lc1.minPosSampleCnt > 0) dataLc1.meanActPosForceMin = pb.lc1.newtonsPosMeanMinAcc / pb.lc1.minPosSampleCnt;
			else dataLc1.meanActPosForceMin = 0;
			if (pb.lc1.minNegSampleCnt > 0) dataLc1.meanActNegForceMin = pb.lc1.newtonsNegMeanMinAcc / pb.lc1.minNegSampleCnt;
			else dataLc1.meanActNegForceMin = 0;

			telemEcit[0].telemEcitLc1.minActForceMin = dataLc1.minActForceMin;
			telemEcit[0].telemEcitLc1.maxActForceMin = dataLc1.maxActForceMin;
			telemEcit[0].telemEcitLc1.meanActForceMin = dataLc1.meanActForceMin;
			telemEcit[0].telemEcitLc1.meanActPosForceMin = dataLc1.meanActPosForceMin;
			telemEcit[0].telemEcitLc1.meanActNegForceMin = dataLc1.meanActNegForceMin;
			telemEcit[0].telemEcitLc1.absUpperLimitCounterMin = dataLc1.absUpperLimitCounterSec;
			telemEcit[0].telemEcitLc1.upperOversteppingsCounterMin = dataLc1.maxUpperOversteppingCountMin;
			telemEcit[0].telemEcitLc1.absLowerLimitCounterMin = dataLc1.absLowerLimitCounterSec;
			telemEcit[0].telemEcitLc1.lowerOversteppingsCounterMin = dataLc1.maxLowerOversteppingCountMin;
			telemEcit[0].telemEcitLc1.relayState = pb.relayStt;

			dataLc1.minActForceMin = LC_NEWTONS_MAX;
			dataLc1.maxActForceMin = LC_NEWTONS_MIN;
			dataLc1.maxUpperOversteppingCountMin = 0;
			dataLc1.maxLowerOversteppingCountMin = 0;
			pb.lc1.newtonsMeanMinAcc = 0;
			pb.lc1.newtonsPosMeanMinAcc = 0;
			pb.lc1.newtonsNegMeanMinAcc = 0;
			pb.lc1.minPosSampleCnt = 0;
			pb.lc1.minNegSampleCnt = 0;

			// Loadcell 2
			if (pb.minSampleCnt > 0) dataLc2.meanActForceMin = pb.lc2.newtonsMeanMinAcc / pb.minSampleCnt;
			else dataLc2.meanActForceMin = 0;
			if (pb.lc2.minPosSampleCnt > 0) dataLc2.meanActPosForceMin = pb.lc2.newtonsPosMeanMinAcc / pb.lc2.minPosSampleCnt;
			else dataLc2.meanActPosForceMin = 0;
			if (pb.lc2.minNegSampleCnt > 0) dataLc2.meanActNegForceMin = pb.lc2.newtonsNegMeanMinAcc / pb.lc2.minNegSampleCnt;
			else dataLc2.meanActNegForceMin = 0;

			telemEcit[0].telemEcitLc2.minActForceMin = dataLc2.minActForceMin;
			telemEcit[0].telemEcitLc2.maxActForceMin = dataLc2.maxActForceMin;
			telemEcit[0].telemEcitLc2.meanActForceMin = dataLc2.meanActForceMin;
			telemEcit[0].telemEcitLc2.meanActPosForceMin = dataLc2.meanActPosForceMin;
			telemEcit[0].telemEcitLc2.meanActNegForceMin = dataLc2.meanActNegForceMin;
			telemEcit[0].telemEcitLc2.absUpperLimitCounterMin = dataLc2.absUpperLimitCounterSec;
			telemEcit[0].telemEcitLc2.upperOversteppingsCounterMin = dataLc2.maxUpperOversteppingCountMin;
			telemEcit[0].telemEcitLc2.absLowerLimitCounterMin = dataLc2.absLowerLimitCounterSec;
			telemEcit[0].telemEcitLc2.lowerOversteppingsCounterMin = dataLc2.maxLowerOversteppingCountMin;
			telemEcit[0].telemEcitLc2.relayState = pb.relayStt;

			dataLc2.minActForceMin = LC_NEWTONS_MAX;
			dataLc2.maxActForceMin = LC_NEWTONS_MIN;
			dataLc2.maxUpperOversteppingCountMin = 0;
			dataLc2.maxLowerOversteppingCountMin = 0;
			pb.lc2.newtonsMeanMinAcc = 0;
			pb.lc2.newtonsPosMeanMinAcc = 0;
			pb.lc2.newtonsNegMeanMinAcc = 0;
			pb.lc2.minPosSampleCnt = 0;
			pb.lc2.minNegSampleCnt = 0;	
			
			rfSendLc = true;							// Order for RF transmission
			resumeLcConfRf = false;
			if (pb.relayStt >= ON) pb.sendAlert = TRUE;	// Alert order set
			else pb.sendAlert = FALSE;
			pb.minSampleCnt = 0;						// Reset general counter			
		}
	}
}


void CheckLoadcellsAlerts(double measLc1, double measLc2)
{
	if (measLc1 > pb.lc1.upperForceOvesteppings)	// Loadcell 1
	{
		if ( (dataLc1.upperOversteppingsCounterSec < LIMIT_COUNTER_MAX) && (!lc1OversteppingTrigger) )
		{
			dataLc1.upperOversteppingsCounterSec++;
			lc1OversteppingTrigger = true;
		}

		if ( (dataLc1.upperOversteppingsCounterSec > pb.lc1.nrUpperForceOversteppings) || (measLc1 > pb.lc1.absUpperForceLimit) )
		{
			dataLc1.relayState = ON;
			if ((measLc1 > pb.lc1.absUpperForceLimit) && (dataLc1.absUpperLimitCounterSec < LIMIT_COUNTER_MAX)) dataLc1.absUpperLimitCounterSec = 1;
		}
	}
	else if (measLc1 < (-1 * pb.lc1.lowerForceOvesteppings))
	{
		if ( (dataLc1.lowerOversteppingsCounterSec < LIMIT_COUNTER_MAX) && (!lc1OversteppingTrigger) )
		{
			dataLc1.lowerOversteppingsCounterSec++;
			lc1OversteppingTrigger = true;
		}

		if ( (dataLc1.lowerOversteppingsCounterSec > pb.lc1.nrLowerForceOversteppings) || (measLc1 < (-1 * pb.lc1.absLowerForceLimit)) )
		{
			dataLc1.relayState = ON;
			if ( (measLc1 < (-1 * pb.lc1.absLowerForceLimit)) && (dataLc1.absLowerLimitCounterSec < LIMIT_COUNTER_MAX) ) dataLc1.absUpperLimitCounterSec = 1;
		}
	}
	else
	{
		lc1OversteppingTrigger = false;
	}

	if (measLc2 > pb.lc2.upperForceOvesteppings)	// Loadcell 2
	{
		if ( (dataLc2.upperOversteppingsCounterSec < LIMIT_COUNTER_MAX) && (!lc2OversteppingTrigger) )
		{
			dataLc2.upperOversteppingsCounterSec++;
			lc2OversteppingTrigger = true;
		}

		if ( (dataLc2.upperOversteppingsCounterSec > pb.lc2.nrUpperForceOversteppings) || (measLc2 > pb.lc2.absUpperForceLimit) )
		{
			dataLc2.relayState = ON;
			if ( (measLc2 > pb.lc2.absUpperForceLimit) && (dataLc2.absUpperLimitCounterSec < LIMIT_COUNTER_MAX) ) dataLc2.absUpperLimitCounterSec = 1;
		}
	}
	else if (measLc2 < (-1 * pb.lc2.lowerForceOvesteppings))
	{
		if ((dataLc2.lowerOversteppingsCounterSec < LIMIT_COUNTER_MAX) && (!lc2OversteppingTrigger))
		{
			dataLc2.lowerOversteppingsCounterSec++;
			lc2OversteppingTrigger = true;
		}

		if ( (dataLc2.lowerOversteppingsCounterSec > pb.lc2.nrLowerForceOversteppings) || (measLc2 < (-1 * pb.lc2.absLowerForceLimit)) )
		{
			dataLc2.relayState = ON;
			if ( (measLc2 < (-1 * pb.lc2.absLowerForceLimit)) && (dataLc2.absLowerLimitCounterSec < LIMIT_COUNTER_MAX) ) dataLc2.absUpperLimitCounterSec = 1;
		}
	}
	else
	{
		lc2OversteppingTrigger = false;
	}

	lcFail = CheckLoadCellFail(pbState);
	
	// Error contact set/reset
	if ( (dataLc1.relayState == ON) || (dataLc2.relayState == ON) || (lcFail == 1) )
	{
		if (pb.relayStt == OFF)
		{
			pb.relayStt = ON;
			Set_Relay();	// Method uses realyStt variable				

#ifdef DEBUG_LOADCELL
			DebugSerial.printf("\r\nSet relay to: %d\r\n", pb.relayStt);
#endif
		}
	}
	else if (pb.relayStt >= ON)
	{
		pb.relayStt = OFF;		
		Set_Relay();		// Method uses realyStt variable
	}

	if (lcFail == 1) pb.relayStt = 2;	// Overrides status in case of relay is laready on an
}


void Set_Relay(void)
{
	uint8_t retry = 0;
	uint8_t conf = SetConfigReg();	// Build configs register	
	
	// Set piggyback settings
	if (xSemaphoreTake(xI2CSemaphore, (TickType_t)50) == pdTRUE)
	{
		Wire.beginTransmission(PIGGYBACK_I2C_ADD);
		Wire.write(SET_CONFIG);		// Command byte 
		Wire.write(conf);			// Data bytes 
		Wire.endTransmission();

		xSemaphoreGive(xI2CSemaphore);
	}

	EEPROM.put(PB_RELAY_STATE_ADD, pb.relayStt);
	EEPROM.commit();

	vTaskDelay(pdMS_TO_TICKS(10));	// Time to permit a consecutive reading of relay state from piggyback's register
}


uint8_t CheckRelayState(uint8_t generalState)
{
	uint8_t state = generalState & RELAY_STATE_POS;

	if (state > 0) return ON;
	else return OFF;
}


uint8_t CheckLoadCellFail(uint8_t generalState2)
{
	uint8_t lc1W4Stt = generalState2 & W4LC1_STATE_POS;
	uint8_t lc2W4Stt = generalState2 & W4LC2_STATE_POS;

	if ( ((lc1W4Stt > 0) && (!pb.lc1.w4stt)) || ((lc2W4Stt > 0) && (!pb.lc2.w4stt)) || (generalState2 == UNDEFINED_STATE) ) return 1;
	else return 0;
}


uint8_t CheckState(void)
{
	uint8_t state = 0x00;
	int iterator = 1;

	if (xSemaphoreTake(xI2CSemaphore, (TickType_t)50) == pdTRUE)
	{
		Wire.beginTransmission(PIGGYBACK_I2C_ADD);
		Wire.write(GET_STATE);				// Command byte
		Wire.write(0x00);
		Wire.endTransmission();

		Wire.requestFrom(PIGGYBACK_I2C_ADD, 1);

		if (Wire.available() > 0)
		{
			state = (uint8_t)(Wire.read());
			iterator--;
		}

		xSemaphoreGive(xI2CSemaphore);
	}

	if (iterator == 0) return state;
	else return UNDEFINED_STATE;
}


uint8_t SetLcConfigs(char* buffer)
{
	uint8_t sepPos[20] = { 0 };
	uint8_t sepIdx = 0;
	uint8_t sensorType = CH_NO_SENSOR;	// Used to filter wich loadcell to apply configs
	LOADCELL_STRUCT* lc;
	uint8_t result = 0;

	lcConfigFlag = true;

	for (uint8_t i = 0; i < LC_RS485_BUFFER_LEN; i++)	// turns ',' into '\0' to use string methods
	{
		if (buffer[i] == '\0') break;
		
		if (buffer[i] == ',')
		{
			buffer[i] = '\0';			
			sepPos[sepIdx] = i;
			sepIdx++;			
		}		
	}

	sensorType = atoi(&buffer[sepPos[2] + 1]);
	
	if ((sensorType >= CH_NO_SENSOR) && (sensorType <= CH1_D2)) lc = &pb.lc1;
	else lc = &pb.lc2;

	
	lc->sensorId = sensorType;
	lc->absUpperForceLimit = atoi(&buffer[sepPos[3] + 1]);
	lc->upperForceOvesteppings = atoi(&buffer[sepPos[4] + 1]);
	lc->lowerForceOvesteppings = atoi(&buffer[sepPos[5] + 1]);
	lc->absLowerForceLimit = atoi(&buffer[sepPos[6] + 1]);
	lc->nrUpperForceOversteppings = atoi(&buffer[sepPos[7] + 1]);
	lc->nrLowerForceOversteppings = atoi(&buffer[sepPos[8] + 1]);
	lc->forceOffset = atoi(&buffer[sepPos[9] + 1]);
	lc->Nmax = atoi(&buffer[sepPos[10] + 1]);
	pb.relayStt = atoi(&buffer[sepPos[11] + 1]);

	result = Init_Loadcell_PiggyBack(PIGGYBACK_I2C_ADD);

	if (result == 1)
	{
		if ( (pb.relayStt == OFF) && ( (dataLc1.relayState == ON) || (dataLc2.relayState == ON) || (lcFail == 1) ) )	// Received order to reset error contact
		{
			dataLc1.absUpperLimitCounterSec = 0;
			dataLc1.upperOversteppingsCounterSec = 0;
			dataLc1.absLowerLimitCounterSec = 0;
			dataLc1.lowerOversteppingsCounterSec = 0;
			dataLc1.absUpperLimitCounterMin = 0;
			dataLc1.upperOversteppingsCounterMin = 0;
			dataLc1.absLowerLimitCounterMin = 0;
			dataLc1.lowerOversteppingsCounterMin = 0;
			dataLc1.relayState = pb.relayStt;

			dataLc2.absUpperLimitCounterSec = 0;
			dataLc2.upperOversteppingsCounterSec = 0;
			dataLc2.absLowerLimitCounterSec = 0;
			dataLc2.lowerOversteppingsCounterSec = 0;
			dataLc2.absUpperLimitCounterMin = 0;
			dataLc2.upperOversteppingsCounterMin = 0;
			dataLc2.absLowerLimitCounterMin = 0;
			dataLc2.lowerOversteppingsCounterMin = 0;
			dataLc2.relayState = pb.relayStt;

			lcFail = 0;
		}

		EEPROM_SaveConfigs();
	}
	
	lcConfigFlag = false;

	return result;
}


uint8_t Set_LcConfigs_Rf(uint8_t* buffer)
{
	uint8_t sensorType = CH_NO_SENSOR;	// Used to filter wich loadcell to apply configs
	LOADCELL_STRUCT* lc;
	uint8_t result = 0;

	lcConfigFlag = true;

	sensorType = buffer[LC_CONF_SENSOR_ID_POS];

	if ((sensorType >= CH_NO_SENSOR) && (sensorType <= CH1_D2)) lc = &pb.lc1;
	else lc = &pb.lc2;

	lc->sensorId = sensorType;
	memcpy(&lc->absUpperForceLimit, &buffer[LC_CONF_ABS_UPPER_LIMIT_POS], UINT32_SIZE);
	memcpy(&lc->upperForceOvesteppings, &buffer[LC_CONF_UPPER_OVERSTEPPINS_POS], UINT32_SIZE);
	memcpy(&lc->lowerForceOvesteppings, &buffer[LC_CONF_LOWER_OVERSTEPPINS_POS], UINT32_SIZE);
	memcpy(&lc->absLowerForceLimit, &buffer[LC_CONF_ABS_LOWER_LIMIT_POS], UINT32_SIZE);
	lc->nrUpperForceOversteppings = buffer[LC_CONF_NR_UPPER_OVERSTEPPINS_POS];
	lc->nrLowerForceOversteppings = buffer[LC_CONF_NR_LOWER_OVERSTEPPINS_POS];
	memcpy(&lc->forceOffset, &buffer[LC_CONF_FORCE_OFFSET_POS], UINT32_SIZE);
	memcpy(&lc->Nmax, &buffer[LC_CONF_FORCE_CAPACITY_POS], UINT32_SIZE);
	pb.relayStt = buffer[LC_CONF_RELAY_STATE_POS];

	result = Init_Loadcell_PiggyBack(PIGGYBACK_I2C_ADD);

	if (result == 1)
	{
		if ( (pb.relayStt == OFF) && ( (dataLc1.relayState == ON) || (dataLc2.relayState == ON) || (lcFail == 1) ) )	// Received order to reset error contact
		{
			dataLc1.absUpperLimitCounterSec = 0;
			dataLc1.upperOversteppingsCounterSec = 0;
			dataLc1.absLowerLimitCounterSec = 0;
			dataLc1.lowerOversteppingsCounterSec = 0;
			dataLc1.absUpperLimitCounterMin = 0;
			dataLc1.upperOversteppingsCounterMin = 0;
			dataLc1.absLowerLimitCounterMin = 0;
			dataLc1.lowerOversteppingsCounterMin = 0;
			dataLc1.relayState = pb.relayStt;

			dataLc2.absUpperLimitCounterSec = 0;
			dataLc2.upperOversteppingsCounterSec = 0;
			dataLc2.absLowerLimitCounterSec = 0;
			dataLc2.lowerOversteppingsCounterSec = 0;
			dataLc2.absUpperLimitCounterMin = 0;
			dataLc2.upperOversteppingsCounterMin = 0;
			dataLc2.absLowerLimitCounterMin = 0;
			dataLc2.lowerOversteppingsCounterMin = 0;
			dataLc2.relayState = pb.relayStt;

			lcFail = 0;
		}

		EEPROM_SaveConfigs();

		resumeLcConfRf = true;
	}

	lcConfigFlag = false;

	return result;
}

#endif // USING_LOADCELL


bool RF_Config(void)
{
	// Turn radio on
	Enable_VCC_2(true);
	delay(50);	
	
	// Reset RF module
	digitalWrite(RF_RESET, LOW);
	delay(10);
	digitalWrite(RF_RESET, HIGH);
	delay(10);

	if (!manager.init())
	{
#ifdef DEBUG_MODE
		DebugSerial.printf("\r\nRF Init failed\r\n");
#endif
		return false;
	}
	else
	{
#ifdef DEBUG_MODE
		DebugSerial.printf("\r\nRF Init done\r\n");  // Defaults after init are 434.0MHz, 0.05MHz AFC pull-in, modulation FSK_Rb2_4Fd36
#endif
	}	

	rf95.setTxPower(RF_TX_POWER, false); // False -> output is on PA_BOOST, power from 2 to 20 dBm, use this setting for high power real usage
	//rf95.setTxPower(1, true); // True -> output is on RFO, power from 0 to 15 dBm, use this setting for low power
	rf95.setFrequency(RF95_FREQ);
	rf95.setCADTimeout(1000);

	boolean longRange = false;

	if (longRange)
	{
		// custom configuration
		RH_RF95::ModemConfig modem_config = {
		  0x78, // Reg 0x1D: BW=125kHz, Coding=4/8, Header=explicit
		  0xC4, // Reg 0x1E: Spread=4096chips/symbol, CRC=enable
		  0x08  // Reg 0x26: LowDataRate=On, Agc=Off.  0x0C is LowDataRate=ON, ACG=ON
		};
		rf95.setModemRegisters(&modem_config);
	}
	else
	{
		// Predefined configurations( bandwidth, coding rate, spread factor ):
		// Bw125Cr45Sf128     Bw = 125 kHz, Cr = 4/5, Sf = 128 chips/symbol, CRC on -> Default medium range
		// Bw500Cr45Sf128     Bw = 500 kHz, Cr = 4/5, Sf = 128 chips/symbol, CRC on -> Fast + short range
		// Bw31_25Cr48Sf512   Bw = 31.25 kHz, Cr = 4/8, Sf = 512 chips/symbol, CRC on -> Slow + long range
		// Bw125Cr48Sf4096    Bw = 125 kHz, Cr = 4/8, Sf = 4096 chips/symbol, Low data rate, CRC on -> Slow + long range
		// Bw125Cr45Sf2048    Bw = 125 kHz, Cr = 4/5, Sf = 2048 chips/symbol, CRC on -> Slow + long range

		node.rfModemConf = rf95.Bw125Cr45Sf128;

		if (!rf95.setModemConfig(RH_RF95::Bw125Cr45Sf128))
		{
#ifdef DEBUG_MODE
			DebugSerial.printf("\r\nSet LoRa config failed\r\n");
#endif
			return false;
		}
	}

#ifdef DEBUG_MODE
	DebugSerial.printf("\r\nRF95 ready\r\n");
#endif

	// Max retries
	//manager.setRetries(3);
	manager.setRetries(0);

	// Set ACK timeout
	manager.setTimeout(RF_ACK_LONG_TIMEOUT);

	// Change Mesh address to node number in memory
	manager.setThisAddress(node.nodeNr);

#ifdef DEBUG_MODE
	DebugSerial.printf("\r\nRF Config Success!");
	DebugSerial.printf("\r\nMode: %d | Freq (MHz): %.1f | P (dBm): %d | Node Nr: %d\r\n", node.rfModemConf, RF95_FREQ, RF_TX_POWER, node.nodeNr);
#endif	

	return true;
}


void Radio_TxRx(NODE_STRUCT* node, NODE_TOBE_CONF_STRUCT* nodeWithCfg, TELEM_STRUCT* mem, uint8_t frameType, TELEM_ECIT_STRUCT* telem)
{
	uint8_t len = MAX_MSG_SIZE;
	uint8_t from;
	uint8_t to;
	uint8_t message_id;

	inTxRx = true;
	memset(node->rxBuf, '\0', RADIO_BUFFER_SIZE);

	if (txFlag)
	{
		memset(node->txBuf, '\0', RADIO_BUFFER_SIZE);

		node->txBuf[0] = MSG_TELEMETRY;
		memcpy(node->txBuf + UID_GW_POS, node->gWuID, UID_SIZE);
		memcpy(node->txBuf + UID_NODE_POS, node->uID, UID_SIZE);
		node->txBuf[NODE_NR_POS] = node->nodeNr;
		node->txBuf[FS_POS] = node->fs;
		memcpy(node->txBuf + REF_POS, &node->refresh, UINT16_SIZE);
		node->txBuf[TYPE_POS] = frameType;
		node->txBuf[NR_SENSORS_POS] = N_SENSORS + 1;
		node->txBuf[WAKEUP_POS] = node->wakeUp;
		memcpy(node->txBuf + TS_POS, node->ts, TIMESTAMP_SIZE);
		memcpy(node->txBuf + S1_POS, &node->sens[T_SENSOR], FLOAT_SIZE);

		switch (frameType)
		{
		case IGUS_ECIT:
			node->txBuf[DIR_POS] = telem->dir;
			memcpy(node->txBuf + DELTA_TIME_POS, &telem->deltaSampleTime, UINT32_SIZE);
			memcpy(node->txBuf + SPEED_POS, &telem->sampleSpeed, FLOAT_SIZE);
			memcpy(node->txBuf + DISPLACEMENT_POS, &telem->displacement, FLOAT_SIZE);
			node->txBuf[BAT_POS] = node->bat;
			node->txBuf[JUMP_POS] = node->Jump;
			memcpy(node->name, fwVersion, NAME_SIZE);
			memcpy(node->txBuf + NAME_POS, node->name, NAME_SIZE);

			len = TELEMETRY_MSG_SIZE;

			break;

		case IGUS_ECIT_LOADCELL:
			node->txBuf[SENSOR_ID_POS_PB] = telem->telemEcitLc1.sensorId;
			memcpy(node->txBuf + ABS_UPPER_LIMIT_POS_PB, &telem->telemEcitLc1.absUpperForceLimit, UINT32_SIZE);
			memcpy(node->txBuf + UPPER_OVERSTEPPINS_POS_PB, &telem->telemEcitLc1.upperForceOvesteppings, UINT32_SIZE);
			memcpy(node->txBuf + MIN_ACT_FORCE_POS_PB, &telem->telemEcitLc1.minActForceMin, UINT32_SIZE);
			memcpy(node->txBuf + MAX_ACT_FORCE_POS_PB, &telem->telemEcitLc1.maxActForceMin, UINT32_SIZE);
			memcpy(node->txBuf + MEAN_ACT_FORCE_POS_PB, &telem->telemEcitLc1.meanActForceMin, UINT32_SIZE);
			memcpy(node->txBuf + MEAN_ACT_POS_FORCE_POS_PB, &telem->telemEcitLc1.meanActPosForceMin, UINT32_SIZE);
			memcpy(node->txBuf + MEAN_ACT_NEG_FORCE_POS_PB, &telem->telemEcitLc1.meanActNegForceMin, UINT32_SIZE);
			memcpy(node->txBuf + LOWER_OVERSTEPPINS_POS_PB, &telem->telemEcitLc1.lowerForceOvesteppings, UINT32_SIZE);
			memcpy(node->txBuf + ABS_LOWER_LIMIT_POS_PB, &telem->telemEcitLc1.absLowerForceLimit, UINT32_SIZE);
			node->txBuf[NR_UPPER_OVERSTEPPINS_POS_PB] = telem->telemEcitLc1.nrUpperForceOversteppings;
			node->txBuf[NR_LOWER_OVERSTEPPINS_POS_PB] = telem->telemEcitLc1.nrLowerForceOversteppings;
			memcpy(node->txBuf + FORCE_OFFSET_POS_PB, &telem->telemEcitLc1.forceOffset, UINT32_SIZE);
			node->txBuf[ABS_UPPER_LIMIT_COUNT_POS_PB] = telem->telemEcitLc1.absUpperLimitCounterMin;
			node->txBuf[UPPER_OVERSTEPPINS_COUNT_POS_PB] = telem->telemEcitLc1.upperOversteppingsCounterMin;
			node->txBuf[ABS_LOWER_LIMIT_COUNTER_POS_PB] = telem->telemEcitLc1.absLowerLimitCounterMin;
			node->txBuf[LOWER_OVERSTEPPINS_COUNT_POS_PB] = telem->telemEcitLc1.lowerOversteppingsCounterMin;
			node->txBuf[RELAY_STATE_POS_PB] = telem->telemEcitLc1.relayState;
			node->txBuf[BAT_POS_PB] = node->bat;
			node->txBuf[JUMP_POS_PB] = node->Jump;
			memcpy(node->name, fwVersion, NAME_SIZE);
			memcpy(node->txBuf + NAME_POS_PB, node->name, NAME_SIZE);

			len = TELEMETRY_MSG_SIZE_PB;

			break;

		default:

			break;
		}

		//node->serverAddr = SERVER_ADDRESS;
		node->serverAddr = RH_BROADCAST_ADDRESS;

#ifdef DEEP_DEBUG_MODE
		DebugSerial.printf("\r\nBuffer Sent [%d]: ", len);
		for (uint8_t i = 0; i < len; i++)
		{
			DebugSerial.printf("%02X ", *(node->txBuf + i));
		}
		DebugSerial.print("\r\n");
#endif

		manager.sendtoWait(node->txBuf, len, node->serverAddr);

		txFlag = false;
	}
	else if (manager.recvfrom(node->rxBuf, &len, &from, &to, &message_id))
	{
		rf95.setModeIdle();

#ifdef DEEP_DEBUG_MODE
		DebugSerial.printf("\r\nReceived from: %d | To: %d | Len: %d | Msg_Id: %d\r\n", from, to, len, message_id);
		
		DebugSerial.printf("\r\nBuffer Received [%d]: ", len);

		for (uint8_t i = 0; i < len; i++)
		{
			DebugSerial.printf("%02X ", *(node->rxBuf + i));
		}
		DebugSerial.print("\r\n");
#endif

		// Get the header to verify if it is a valid message
		if ( (node->rxBuf[MSG_ID_POS] == MSG_LC_CONFIG) && (memcmp(&node->rxBuf[LC_CONF_UID_POS], node->uID, UID_SIZE) == 0) && (len == LC_CONF_MSG_SIZE) )
		{			
			if (xSemaphoreTake(configSemaphore, (TickType_t)50) == pdTRUE)
			{
				Set_LcConfigs_Rf(node->rxBuf);

				xSemaphoreGive(configSemaphore);
			}
		}
#ifdef DEBUG_MODE
		else
		{
			DebugSerial.printf("\r\nMessage not valid !\r\n");			
		}

		rf95.setModeRx();
#endif
	}

	// Prevents radio from stucking in continuos interrupts
	if (digitalRead(RF_INT) == HIGH)
	{
		Enable_VCC_2(false);
		delay(50);

		node->radioStt = RF_Config();
	}

	inTxRx = false;
}


// Set scan mode to only LTE
// <scan_mode> Integer type.RAT(s) to be searched for.
//		0 Automatic(GSM and LTE)
//		1 GSM only
//		3 LTE only
//
// <effect> Integer type.When to take effect.
//		0 Take effect after rebooting
//		1 Take effect immediately
uint8_t GSM_SetScanMode(uint8_t mode)
{
	char confScanMode[32] = { '\0' };
	sprintf(confScanMode, "AT+QCFG=\"nwscanmode\",%d,1\r\n", mode);

	// Set Report Mobile Equipment Error
	if (sendCmdAndWaitForResp(confScanMode, OK, DEFAULT_TIMEOUT) == 0)
		return 0;

	// Reaching here, there success
	return 1;
}


uint8_t GSM_GetCellsAround(void)
{
	uint8_t result = 0;

	// Check Cells around	
	sendCmd("AT+QENG=\"neighbourcell\"\r\n");

	result = GSM_ReadSerial(DEFAULT_TIMEOUT, OK);

	// Show results if there is cells around
	int8_t ind = atrIndexOf(node.gsmBuff, BUFFER_SIZE, "+QENG:", 6);

	if (ind > 0)
		DebugSerial.println(node.gsmBuff);

	return result;
}


uint8_t GSM_GetTime(char* date, char* hour, char* hhmmss)
{
	sendCmd("AT+CCLK?\r\n");

	GSM_ReadSerial(DEFAULT_TIMEOUT, OK);

#ifdef DEBUG_ATCMD_MODE
	DebugSerial.println(node.gsmBuff);
#endif	

	// Search for the OK
	if (atrIndexOf(node.gsmBuff, BUFFER_SIZE, OK, 4) == -1)
	{
		return 0;
	}
	else
	{
		int8_t ind = atrIndexOf(node.gsmBuff, BUFFER_SIZE, "\"", 1);

		if (ind > 0)
		{
			ind++;

			memcpy(date + 2, node.gsmBuff + ind, 8);
			date[0] = '2';
			date[1] = '0';
			date[10] = '\0';

			memcpy(hour, node.gsmBuff + ind + 9, 5);
			hour[5] = '\0';

			memcpy(hhmmss, node.gsmBuff + ind + 9, 8);
			hhmmss[8] = '\0';

			return 1;
		}
		else
		{
			return 0;
		}

	}
}


//	GSM Handling functions
void GSM_TurnON(void)
{
	// Initialize UART for GSM module
	GsmSerial.begin(GSM_BAUDRATE, SERIAL_8N1, UART_RX, UART_TX);

	// Enable GSM power supply
	//digitalWrite(RF_POWER_EN, HIGH);
	Enable_VBAT_2(true);
	vTaskDelay(pdMS_TO_TICKS(1000));

	Enable_POWERKEY(true);
	vTaskDelay(pdMS_TO_TICKS(1100));
	Enable_POWERKEY(false);
}


void GSM_TurnOFF(void)
{
	// Disable uart pins and force them LOW
	GSM_DisableSerial();

	// Turn Off GSM
	Enable_POWERKEY(true);
	vTaskDelay(pdMS_TO_TICKS(700));
	Enable_POWERKEY(false);

	// Disable GSM power supply and force the capacitors to discharge
	Enable_POWERKEY(true);
	//digitalWrite(RF_POWER_EN, LOW);
	Enable_VBAT_2(false);

	// Clear Buffer
	memset(node.gsmBuff, '\0', BUFFER_SIZE);
	node.gsmBuffInd = 0;
}


uint8_t GSM_SetEchoOff(void)
{
	//wdt_reset();

	sendCmd("ATE0\r\n");

	GSM_ReadSerial(DEFAULT_TIMEOUT, OK);

#ifdef DEBUG_ATCMD_MODE
	DebugSerial.println(node.gsmBuff);
#endif

	if (atrIndexOf(node.gsmBuff, BUFFER_SIZE, OK, 4) == -1)
		return 0;
	else
		return 1;

	// Error found, return 0
	// Error NOT found, return 1		
}


// Get Module version
uint8_t GSM_GetModel(uint8_t* model)
{
	sendCmd("AT+CGMM\r\n");

	GSM_ReadSerial(DEFAULT_TIMEOUT, OK);

#ifdef DEBUG_ATCMD_MODE
	DebugSerial.println(node.gsmBuff);
#endif

	if (atrIndexOf(node.gsmBuff, BUFFER_SIZE, QUECTEL_M95, 11) > -1)
	{
		*model = GSM_MODEL_QUECTEL_M95;
		DebugSerial.println("GSM Model detected: Quectel M95");
	}
	else if (atrIndexOf(node.gsmBuff, BUFFER_SIZE, QUECTEL_BG95M3, 7) > -1)
	{
		*model = GSM_MODEL_QUECTEL_BG95M3;
		DebugSerial.println("GSM Model detected: Quectel BG95-M3");
	}
	else
	{
		return 0;
	}

	return 1;
}


uint8_t  GSM_ReadSerial(uint32_t timeout, char* ans)
{
	uint8_t timeOutFlag = 0;
	uint64_t timeOld = millis();
	uint8_t sum = 0;
	uint8_t sumError = 0;
	uint8_t len = strlen(ans);

	//DebugSerial.printf("Ans: %s | len: %d\r\n", ans, len);

	// Check for desired answer
	if (len == 0)
		return 0;

	//wdt_reset();

	// Waits for the transmission of outgoing serial data to complete
	GsmSerial.flush();

	// Clear Buffer
	memset(node.gsmBuff, '\0', BUFFER_SIZE);
	node.gsmBuffInd = 0;

	while (!GsmSerial.available() && !(millis() > timeOld + timeout))
	{
		//wdt_reset();
		vTaskDelay(pdMS_TO_TICKS(1));
	}

	// Restart time flag
	timeOld = millis();

	// While available and wait for timeout
	while (timeOutFlag == 0)
	{
		//wdt_reset();

		if (GsmSerial.available() > 0)
		{
			node.gsmBuff[node.gsmBuffInd] = (char)(GsmSerial.read());

			// Check for specific answer to be received
			sum = (node.gsmBuff[node.gsmBuffInd] == ans[sum] || ans[sum] == 'X') ? sum + 1 : 0;
			//DebugSerial.printf("Sum: %d\r\n", sum);
			if (sum == len)
				return 1;

			// Check for error answer
			sumError = (node.gsmBuff[node.gsmBuffInd] == ERROR[sumError]) ? sumError + 1 : 0;
			//DebugSerial.printf("SumError: %d\r\n", sumError);
			if (sumError == 5)
				return 0;

			node.gsmBuffInd++;
			timeOld = millis();
		}
		else if (millis() > timeOld + timeout)
		{
			timeOutFlag = 1;
		}
	}

	return 0;
}


/* Auxiliary functions */
int16_t atrIndexOf(char* buffer, uint16_t buffSize, char* cmp, uint16_t bytes2cmp)
{
	for (uint16_t i = 0; i < buffSize; i++)
	{
		if (memcmp(buffer + i, cmp, bytes2cmp) == 0)
			return i;
	}

	return -1;
}

int16_t atrIndexOf(char* buffer, uint16_t iInd, uint16_t buffSize, char* cmp, uint16_t bytes2cmp)
{
	for (uint16_t i = iInd; i < buffSize; i++)
	{
		if (memcmp(buffer + i, cmp, bytes2cmp) == 0)
			return i;
	}

	return -1;
}


uint8_t GSM_GetICCID(char* str)
{
	uint8_t result = GSM_ICCID_ERROR;

	sendCmd("AT+QCCID\r\n");

	GSM_ReadSerial(DEFAULT_TIMEOUT, OK);

#ifdef DEBUG_ATCMD_MODE
	DebugSerial.println(node.gsmBuff);
#endif

	// The answer is just the iccid (20 numbers) and the ok
	int8_t ind = atrIndexOf(node.gsmBuff, BUFFER_SIZE, OK, 4);

	if (ind > 24)
	{
		ind -= 24;

		memcpy(str, node.gsmBuff + ind, 20);
		str[20] = '\0';
		result = GSM_SUCCESS;
	}
	else
	{
		memset(str, '\0', 21);
		result = GSM_ICCID_ERROR;
	}

	return result;
}


uint8_t GSM_GetOperator(void)
{
	//wdt_reset();

	// Verify registation on network
	sendCmd("AT+COPS?\r\n");

	GSM_ReadSerial(DEFAULT_TIMEOUT, OK);

#ifdef DEBUG_ATCMD_MODE
	DebugSerial.println(node.gsmBuff);
#endif

	// when the modem is configured in a network, there is a comma separating the network info
	if (atrIndexOf(node.gsmBuff, BUFFER_SIZE, ",", 1) == -1)
		return 0;
	else
		return 1;

	// Operator NOT found, return 0
	// Operator found, return 1
}


uint8_t GSM_Init(void)
{
	uint8_t tries = 0;

	while (tries < 5)
	{
		if (GSM_SetEchoOff() == 0)
		{
#ifdef DEBUG_ATCMD_MODE
			DebugSerial.printf("Echo error, try: %d\n", tries);
#endif		
			vTaskDelay(pdMS_TO_TICKS(1000));
		}
		else
		{
			break;
		}
		tries++;
	}

	if (tries == 5)
	{
#ifdef DEBUG_ATCMD_MODE
		DebugSerial.println("Echo error!");
#endif
		return ERROR_ATE;
	}

	// Get Module version
	if (GSM_GetModel(&node.gsmModel) == 0)
		return ERROR_MODEL;
	else
		DebugSerial.printf("Model: %d\n", node.gsmModel);

	// Set Report Mobile Equipment Error
	if (sendCmdAndWaitForResp("AT+CMEE=2\r\n", OK, DEFAULT_TIMEOUT) == 0)
		return ERROR_REPORT;

	//vTaskDelay(pdMS_TO_TICKS(1000));	

	// Minimum Quality of the Service Requested - GPRS exclusive command
	if (node.gsmModel == GSM_MODEL_QUECTEL_M95)
	{
		if (sendCmdAndWaitForResp("AT+CGQMIN=1,0,0,0,0,0\r\n", OK, DEFAULT_TIMEOUT) == 0)
			return ERROR_QOS;
	}
	else if (node.gsmModel == GSM_MODEL_QUECTEL_BG95M3)
	{
		// Disable autorun of the GPS
		if (sendCmdAndWaitForResp("AT+QGPSCFG=\"autogps\",0\r\n", OK, DEFAULT_TIMEOUT) == 0)
			return ERROR_DISABLE_AUTOGPS;
	}

	return 1;
}


uint8_t GSM_WaitOperator(uint16_t tries)
{
	uint8_t result = 0;

	if ((tries <= 0) || (tries > INIT_MAX_TRIES))
		tries = 6;								// Default minimum tries = 5*6 = 30s

	uint8_t n = 0;
	uint32_t timeStart = millis();
	uint32_t timeInt = 0;
	while ( (n < tries) || (timeInt < tries*5000) )
	{
		if (node.gsmModel == GSM_MODEL_QUECTEL_BG95M3)
		{
			// Verify which cells are around 
			GSM_GetCellsAround();					// DEBUG
		}

		// Verify RSSI
		node.rssi = GSM_GetRSSI();

		DebugSerial.printf("RSSI: %d, try: %d\n", node.rssi, n);

		if (node.wakeUp == WAKEUP_BUTTON)
		{
			sprintf(lines.newLine, "RSSI: %d, try: %d", node.rssi, n+1);
			if (n == 0)
				AddLine(lines.newLine, 0);
			else
				SetLine(lines.newLine);
			DisplayLines();
		}

		result = GSM_GetOperator();

		vTaskDelay(pdMS_TO_TICKS(5000));

		// Notify user for GSM waiting
		if (node.wakeUp == WAKEUP_BUTTON)
		{
			Set_Color(NO_COLOR);
			vTaskDelay(pdMS_TO_TICKS(100));
			Set_Color(BLUE);
		}

		if (result == 1)
			break;

		// Update controll variables
		n++;
		timeInt = millis() - timeStart;
	}

	return result;
}


void sendCmd(const String cmd)
{
	GsmSerial.print(cmd);
}


uint8_t sendCmdAndWaitForResp(const String cmd, char* resp, unsigned timeout)
{
	//wdt_reset();
	vTaskDelay(pdMS_TO_TICKS(100));

	sendCmd(cmd);

#ifdef DEBUG_ATCMD_MODE
	DebugSerial.println(cmd);
#endif	

	return waitForResp(resp, timeout);
}

uint8_t waitForResp(char* resp, unsigned int timeout)
{
	int len = strlen(resp);
	int sum = 0;
	unsigned long timerStart, timerEnd;
	timerStart = millis();

	// Waits for the transmission of outgoing serial data to complete
	GsmSerial.flush();

	while (1)
	{
		//wdt_reset();
		if (GsmSerial.available())
		{
			char c = GsmSerial.read();

#ifdef DEBUG_ATCMD_MODE
			DebugSerial.print(c);
#endif

			vTaskDelay(pdMS_TO_TICKS(1));

			sum = (c == resp[sum] || resp[sum] == 'X') ? sum + 1 : 0;
			if (sum == len)
				break;
		}

		timerEnd = millis();

		if (timerEnd - timerStart > timeout)
		{
			return 0;
		}
	}

	while (GsmSerial.available())
	{
		//wdt_reset();
		GsmSerial.read();
	}

	return 1;
}


void cleanBuffer(char* buffer, int count)
{
	memset(buffer, 0x00, count);
}


// BG95-M3 module function for LTE configuration
uint8_t GSM_LteConnect(char* apnOP)
{
	//wdt_disable();

	// Attacth to GPRS Service
	if (sendCmdAndWaitForResp("AT+CGATT=1\r\n", OK, 60000L) != 1)
		return ERROR_GPRS_ATTACH;

	// Set the APN
	char cmd[64] = { '\0' };
	sprintf(cmd, "AT+QICSGP=1,1,\"%s\"\r\n", apnOP);
	if (sendCmdAndWaitForResp(cmd, OK, DEFAULT_TIMEOUT) != 1)
		return ERROR_GPRS_SET_APN;

	// Activate PDP Context
	if (sendCmdAndWaitForResp("AT+QIACT=1\r\n", OK, 60000L) != 1)
		return ERROR_GPRS_ACT_CONTEXT;

	// Get Local IP Address, only assigned after connection
	if (GSM_GetLocalIP(GSM_MODEL_QUECTEL_BG95M3) == 0)
		return ERROR_GPRS_GET_IP;

	// Configure DNS server
	sprintf(cmd, "AT+QIDNSCFG=1,\"%s\"\r\n", DNSserver);
	if (sendCmdAndWaitForResp(cmd, OK, DEFAULT_TIMEOUT) != 1)
		return ERROR_GPRS_DNS_CONF;

	if (sendCmdAndWaitForResp("AT+QNTP=1,\"time.cloudflare.com\"\r\n", "+QNTP: 0", 60000L) != 1)
		return ERROR_GPRS_SET_TIMESERVER;

	//wdt_enable(WDTO_8S);

	return GSM_SUCCESS;
}


// M95 module function for GPRS configuration
uint8_t GSM_GprsConnect(char* apnOP)
{
	//wdt_disable();

	// Attacth to GPRS Service
	if (sendCmdAndWaitForResp("AT+CGATT=1\r\n", OK, 60000L) != 1)
		return ERROR_GPRS_ATTACH;

	// Select a Context as Foreground Context
	if (sendCmdAndWaitForResp("AT+QIFGCNT=0\r\n", OK, DEFAULT_TIMEOUT) != 1)
		return ERROR_GPRS_CONTEXT;

	// Set the APN
	char cmd[64] = { '\0' };
	sprintf(cmd, "AT+QICSGP=1,\"%s\"\r\n", apnOP);
	if (sendCmdAndWaitForResp(cmd, OK, DEFAULT_TIMEOUT) != 1)
		return ERROR_GPRS_SET_APN;

	// Start TCPIP Task
	if (sendCmdAndWaitForResp("AT+QIREGAPP\r\n", OK, DEFAULT_TIMEOUT) != 1)
		return ERROR_GPRS_START_TCPIP;

	// Activate GPRS Context
	if (sendCmdAndWaitForResp("AT+QIACT\r\n", OK, 60000L) != 1)
		return ERROR_GPRS_ACT_CONTEXT;

	// Get Local IP Address, only assigned after connection
	if (GSM_GetLocalIP(GSM_MODEL_QUECTEL_M95) == 0)
		return ERROR_GPRS_GET_IP;

	// Configure DNS server
	sprintf(cmd, "AT+QIDNSCFG=\"%s\"\r\n", DNSserver);
	if (sendCmdAndWaitForResp(cmd, OK, DEFAULT_TIMEOUT) != 1)
		return ERROR_GPRS_DNS_CONF;

	// Set NTP server
	if (sendCmdAndWaitForResp("AT+QNTP=\"time.cloudflare.com\"\r\n", "+QNTP: 0", 60000L) != 1)
		return ERROR_GPRS_SET_TIMESERVER;

	//wdt_enable(WDTO_8S);

	return GSM_SUCCESS;
}


uint8_t GSM_GetLocalIP(uint8_t model)
{
	//wdt_disable();

	if (model == GSM_MODEL_QUECTEL_BG95M3)
		sendCmd("AT+QIACT?\r\n");
	else if (model == GSM_MODEL_QUECTEL_M95)
		sendCmd("AT+QILOCIP\r\n");

	GSM_ReadSerial(LONG_TIMEOUT, "\"\r\n");

	//wdt_enable(WDTO_8S);

#ifdef DEBUG_ATCMD_MODE
	DebugSerial.println(node.gsmBuff);
#endif

	// when the modem is configured in a network, there is a . separating the IP info
	if (atrIndexOf(node.gsmBuff, BUFFER_SIZE, ".", 1) == -1)
		return 0;
	else
		return 1;

	// Operator NOT found, return 0
	// Operator found, return 1
}


uint8_t GSM_GetIMEI(char* imei)
{
	uint8_t result = GSM_IMEI_ERROR;

	sendCmd("AT+GSN\r\n");

	GSM_ReadSerial(DEFAULT_TIMEOUT, OK);

#ifdef DEBUG_ATCMD_MODE
	DebugSerial.println(node.gsmBuff);
#endif

	int8_t ind = atrIndexOf(node.gsmBuff, BUFFER_SIZE, OK, 4);

	if (ind > 19)
	{
		ind -= 19;

		memcpy(imei, node.gsmBuff + ind, 15);
		imei[15] = '\0';
		result = GSM_SUCCESS;
	}
	else
	{
		memset(imei, '\0', 16);
		result = GSM_IMEI_ERROR;
	}

	return result;
}


int16_t GSM_GetRSSI(void)
{
	char rssiStr[4] = { '\0','\0','\0','\0' };
	int16_t rssi = 0;

	sendCmd("AT+CSQ\r\n");

	GSM_ReadSerial(DEFAULT_TIMEOUT, OK);

#ifdef DEBUG_ATCMD_MODE
	DebugSerial.println(node.gsmBuff);
#endif

	// Example: +CSQ: 17,0
	int8_t ind = atrIndexOf(node.gsmBuff, BUFFER_SIZE, "+CSQ: ", 6);

	if (ind > 0)
	{
		// Go to the begining of the RSSI value
		ind += 6;

		int8_t indF = atrIndexOf(node.gsmBuff, ind, BUFFER_SIZE, ",", 1);
		if (indF > 0)
		{
			memcpy(rssiStr, node.gsmBuff + ind, indF - ind);
			rssi = atoi(rssiStr);

			if ((rssi < 0) || (rssi > 199))
				return -1;
			else
				return rssi;
		}
		else
		{
			return -1;
		}
	}

	return -1;
}


uint8_t GSM_GetLoc(char* loc)
{
	sendCmd("AT+QCELLLOC=1\r\n");

	GSM_ReadSerial(VERY_LONG_TIMEOUT, OK);

#ifdef DEBUG_ATCMD_MODE
	DebugSerial.println(node.gsmBuff);
#endif

	// +QCELLLOC: <longitude>,<latitude>
	int8_t ind = atrIndexOf(node.gsmBuff, BUFFER_SIZE, "+QCELLLOC: ", 11);

	if (ind > 0)
	{
		int8_t ind1 = atrIndexOf(node.gsmBuff + 11 + ind, BUFFER_SIZE, ",", 1);

		if (ind1 > 0)
		{
			// Get coordenates
			char coords[32] = { '\0' };
			int8_t indF = atrIndexOf(node.gsmBuff + 11 + ind, BUFFER_SIZE, "\r", 1);

			if (indF > 0)
			{
				memcpy(coords, node.gsmBuff + 11 + ind, indF);

				uint8_t latLen = indF - ind1 - 1;
				uint8_t lonLen = ind1;

				if ((latLen > 0) && (lonLen > 0))
				{
					memcpy(loc, coords + ind1 + 1, latLen);
					loc[latLen] = ',';
					memcpy(loc + latLen + 1, coords, lonLen);
					loc[latLen + lonLen + 1] = '\0';

#ifdef DEBUG_MODE
					DebugSerial.printf("Loc: %s\n", loc);
#endif

					return 1;
				}
				else
				{
					return 0;
				}
			}
			else
			{
				return 0;
			}
		}
		else
		{
			return 0;
		}
	}
	else
	{
		return 0;
	}

	return 0;
}


void ADC_Measures(void)
{
	uint16_t VBat = 0, Vpv = 0, Vusb = 0, Vpout = 0;

	//Enable_VBAT_2(true);   // Enables output VBAT_2 for VBat measurement

	Enable_BOOST(true);    // Enables output POUT_ADC

	vTaskDelay(pdMS_TO_TICKS(250));

	// Get battery level
	BatMeasure();

	// Get solar panel voltage
	Vpv = GetADCMeas(PV_PLUS_MEASURE);

	//DebugSerial.printf("Vpv Raw: %d | ", Vpv);

	if (Vpv < ADC_PV_0V5)
		Vpv = ADC_PV_0V5;
	else if (Vpv > ADC_PV_7V)
		Vpv = ADC_PV_7V;

	node.pv = (float)(map(Vpv, ADC_PV_0V5, ADC_PV_7V, 5, 70)) / 10.0;
	if (node.pv < 0.6)
		node.pv = 0;

	//DebugSerial.printf("PV: %.1f\r\n", node.pv);

	Vusb = GetADCMeas(VUSB_PLUS_MEASURE);
	//DebugSerial.printf("V_USB raw: %d\r\n", Vusb);

	Vpout = GetADCMeas(POUT_ADC);
	//DebugSerial.printf("V_POUT_ADC raw: %d | ", Vpout);

	if (Vpout < ADC_POUT_0V)
		Vpout = ADC_POUT_0V;
	else if (Vpout > ADC_POUT_12V)
		Vpout = ADC_POUT_12V;

	node.p12v = (float)(map(Vpout, ADC_POUT_0V, ADC_POUT_12V, 0, 120)) / 10.0;

	//DebugSerial.printf("p12V: %.1f\r\n", node.p12v);

	//Enable_VBAT_2(false);  // Disables VBAT_2 output
	//Enable_BOOST(false);   // Disables POUT_ADC output

	//vTaskDelay(pdMS_TO_TICKS(250));
}

void BatMeasure(void)
{
	uint16_t VBat = 0;

	VBat = GetADCMeas(BAT_PLUS_MEASURE);

#ifdef USING_BATTERY_CAL
	//DebugSerial.printf("BatMin: %d | BatMax: %d\n", node.batMinADC, node.batMaxADC);
#else
	//DebugSerial.printf("BatMin: %d | BatMax: %d\n", ADC_RAW_3V2, ADC_RAW_4V05);
#endif

	//DebugSerial.printf("Vbat: %d | ", VBat);

	// Specific case rechargable li-ion batteries
#ifdef USING_BATTERY_CAL	
	if ((node.batMinADC != 0xFFFF) && (node.batMaxADC != 0xFFFF))
	{
		if (VBat < node.batMinADC)			// ADC value for 3.2V
			VBat = node.batMinADC;
		else if (VBat > node.batMaxADC)
			VBat = node.batMaxADC;			// ADC value for 4.05V

		node.bat = map(VBat, node.batMinADC, node.batMaxADC, 0, 100);
	}
	else
	{
		if (VBat < ADC_RAW_3V2)				// ADC default value
			VBat = ADC_RAW_3V2;
		else if (VBat > ADC_RAW_4V05)
			VBat = ADC_RAW_4V05;			// ADC default value

		node.bat = map(VBat, ADC_RAW_3V2, ADC_RAW_4V05, 0, 100);
	}
#else
	if (VBat < ADC_RAW_3V2)				// ADC default value
		VBat = ADC_RAW_3V2;
	else if (VBat > ADC_RAW_4V05)
		VBat = ADC_RAW_4V05;			// ADC default value

	node.bat = map(VBat, ADC_RAW_3V2, ADC_RAW_4V05, 0, 100);
#endif

	//DebugSerial.printf("Bat: %u\n", node.bat);

	if (node.bat < 0)
		node.bat = 0;
	else if (node.bat > 100)
		node.bat = 100;

	if (node.bat < 0)
		node.bat = 0;
	else if (node.bat >= 95)
		node.bat = 100;
	else if (node.bat >= 85)
		node.bat = 90;
	else if (node.bat >= 75)
		node.bat = 80;
	else if (node.bat >= 65)
		node.bat = 70;
	else if (node.bat >= 55)
		node.bat = 60;
	else if (node.bat >= 45)
		node.bat = 50;
	else if (node.bat >= 35)
		node.bat = 40;
	else if (node.bat >= 25)
		node.bat = 30;
	else if (node.bat >= 15)
		node.bat = 20;
	else if (node.bat >= 5)
		node.bat = 10;
	else
		node.bat = 1;
}


void GSM_DisableSerial(void)
{
	GsmSerial.end();
}


void RS485_DisableSerial(void)
{
	EspSerial2.end();
}


void Print_WakeupReason(esp_sleep_wakeup_cause_t reason)
{
	switch (reason)
	{
	case ESP_SLEEP_WAKEUP_EXT0: DebugSerial.println("\r\nWakeup caused by external signal using RTC_IO\r\n"); break;
	case ESP_SLEEP_WAKEUP_EXT1: DebugSerial.println("\r\nWakeup caused by external signal using RTC_CNTL\r\n"); break;
	case ESP_SLEEP_WAKEUP_TIMER: DebugSerial.println("\r\nWakeup caused by timer\r\n"); break;
	case ESP_SLEEP_WAKEUP_TOUCHPAD: DebugSerial.println("\r\nWakeup caused by touchpad\r\n"); break;
	case ESP_SLEEP_WAKEUP_ULP: DebugSerial.println("\r\nWakeup caused by ULP program\r\n"); break;
	default: 
		wakeupReason = ESP_SLEEP_WAKEUP_UNDEFINED;
		DebugSerial.printf("\r\nWakeup was not caused by deep sleep: %d\r\n", wakeupReason); 
		break;
	}
}


void Set_DefaultVals(void)
{
	// Set configs to comunications
	memcpy(node.apn, apnDefault, sizeof(apnDefault));
	//memcpy(node.server, serverDefault, sizeof(serverDefault));

	memcpy(node.imei, imeiDefault, IMEI_SIZE + 1);
	memcpy(node.iccid, iccidTmp, ICCID_SIZE + 1);
	node.nodeNr = DEFAULT_CLIENT_ADDRESS;

	node.fs = 0;
	node.refresh = 360;

#ifdef USING_LOADCELL
	pb.lc1.sensorId = CH1_W1;
	pb.lc1.absUpperForceLimit = 99999;
	pb.lc1.upperForceOvesteppings = 99999;
	pb.lc1.absLowerForceLimit = 99999;
	pb.lc1.lowerForceOvesteppings = 99999;
	pb.lc1.nrUpperForceOversteppings = 1;
	pb.lc1.nrLowerForceOversteppings = 1;
	pb.lc1.forceOffset = 0;
	pb.lc1.Nmax = 10000;
	pb.lc2.sensorId = CH_NO_SENSOR;
	pb.lc2.absUpperForceLimit = 99999;
	pb.lc2.upperForceOvesteppings = 99999;
	pb.lc2.absLowerForceLimit = 99999;
	pb.lc2.lowerForceOvesteppings = 99999;
	pb.lc2.nrUpperForceOversteppings = 1;
	pb.lc2.nrLowerForceOversteppings = 1;
	pb.lc2.forceOffset = 0;
	pb.lc2.Nmax = 10000;
	pb.relayStt = 0;
#endif

	node.sensorsDistance = DEFAULT_DIST;
}


void EEPROM_SaveConfigs(void)
{
	//Save values to EEPROM
	EEPROM.put(FACTORY_ADD, node.factory);
	EEPROM.put(NEW_VERSION_ADD, node.newFW);

	EEPROM.put(SENSOR_TYPE_ADD, node.typeSens);
	EEPROM.put(UID_NODE_ADD, node.uID);							// UID is given by the RTC
	EEPROM.put(UID_GW_ADD, node.gWuID);
	EEPROM.put(NODE_NR_ADD, node.nodeNr);
	//EEPROM.put(PREFER_GW_ADD, node.preferGateway);
	//EEPROM.put(NAME_ADD, node.name);
	EEPROM.put(FS_ADD, node.fs);
	EEPROM.put(REFRESH_ADD, node.refresh);

	EEPROM.put(APN_ADD, node.apn);
	EEPROM.put(IMEI_ADD, node.imei);
	EEPROM.put(ICCID_ADD, node.iccid);

#ifdef USING_LOADCELL
	EEPROM.put(LC1_SENSOR_ID_ADD, pb.lc1.sensorId);
	EEPROM.put(LC1_ABS_UPPER_LIMIT_ADD, pb.lc1.absUpperForceLimit);
	EEPROM.put(LC1_UPPER_FORCE_OVS_ADD, pb.lc1.upperForceOvesteppings);
	EEPROM.put(LC1_ABS_LOWER_LIMIT_ADD, pb.lc1.absLowerForceLimit);
	EEPROM.put(LC1_LOWER_FORCE_OVS_ADD, pb.lc1.lowerForceOvesteppings);
	EEPROM.put(LC1_NR_UPPER_FORCE_OVS_ADD, pb.lc1.nrUpperForceOversteppings);
	EEPROM.put(LC1_NR_LOWER_FORCE_OVS_ADD, pb.lc1.nrLowerForceOversteppings);
	EEPROM.put(LC1_FORCE_OFFSET_ADD, pb.lc1.forceOffset);
	EEPROM.put(LC1_FORCE_CAPACITY_ADD, pb.lc1.Nmax);
	EEPROM.put(LC2_SENSOR_ID_ADD, pb.lc2.sensorId);
	EEPROM.put(LC2_ABS_UPPER_LIMIT_ADD, pb.lc2.absUpperForceLimit);
	EEPROM.put(LC2_UPPER_FORCE_OVS_ADD, pb.lc2.upperForceOvesteppings);
	EEPROM.put(LC2_ABS_LOWER_LIMIT_ADD, pb.lc2.absLowerForceLimit);
	EEPROM.put(LC2_LOWER_FORCE_OVS_ADD, pb.lc2.lowerForceOvesteppings);
	EEPROM.put(LC2_NR_UPPER_FORCE_OVS_ADD, pb.lc2.nrUpperForceOversteppings);
	EEPROM.put(LC2_NR_LOWER_FORCE_OVS_ADD, pb.lc2.nrLowerForceOversteppings);
	EEPROM.put(LC2_FORCE_OFFSET_ADD, pb.lc2.forceOffset);
	EEPROM.put(LC2_FORCE_CAPACITY_ADD, pb.lc2.Nmax);
	EEPROM.put(PB_RELAY_STATE_ADD, pb.relayStt);
#endif

	EEPROM.put(SENSORS_DIST_ADD, node.sensorsDistance);
	EEPROM.put(CONN_TYPE_ADD, node.connType);

	EEPROM.commit();
}


void EEPROM_ReadConfigs(void)
{
	//Read values from EEPROM
	EEPROM.get(SENSOR_TYPE_ADD, node.typeSens);
	EEPROM.get(UID_NODE_ADD, node.uID);							
	EEPROM.get(UID_GW_ADD, node.gWuID);
	EEPROM.get(NODE_NR_ADD, node.nodeNr);
	EEPROM.get(FS_ADD, node.fs);
	if (node.fs > 254)
		node.fs = 0;
	EEPROM.get(REFRESH_ADD, node.refresh);
	if ((node.refresh > 1440) || (node.refresh < 1))
		node.refresh = 360;

	EEPROM.get(APN_ADD, node.apn);	
	
	EEPROM.get(IMEI_ADD, node.imei);
	EEPROM.get(ICCID_ADD, node.iccid);

#ifdef USING_LOADCELL
	EEPROM.get(LC1_SENSOR_ID_ADD, pb.lc1.sensorId);
	EEPROM.get(LC1_ABS_UPPER_LIMIT_ADD, pb.lc1.absUpperForceLimit);
	EEPROM.get(LC1_UPPER_FORCE_OVS_ADD, pb.lc1.upperForceOvesteppings);
	EEPROM.get(LC1_ABS_LOWER_LIMIT_ADD, pb.lc1.absLowerForceLimit);
	EEPROM.get(LC1_LOWER_FORCE_OVS_ADD, pb.lc1.lowerForceOvesteppings);
	EEPROM.get(LC1_NR_UPPER_FORCE_OVS_ADD, pb.lc1.nrUpperForceOversteppings);
	EEPROM.get(LC1_NR_LOWER_FORCE_OVS_ADD, pb.lc1.nrLowerForceOversteppings);
	EEPROM.get(LC1_FORCE_OFFSET_ADD, pb.lc1.forceOffset);
	EEPROM.get(LC1_FORCE_CAPACITY_ADD, pb.lc1.Nmax);
	EEPROM.get(LC2_SENSOR_ID_ADD, pb.lc2.sensorId);
	EEPROM.get(LC2_ABS_UPPER_LIMIT_ADD, pb.lc2.absUpperForceLimit);
	EEPROM.get(LC2_UPPER_FORCE_OVS_ADD, pb.lc2.upperForceOvesteppings);
	EEPROM.get(LC2_ABS_LOWER_LIMIT_ADD, pb.lc2.absLowerForceLimit);
	EEPROM.get(LC2_LOWER_FORCE_OVS_ADD, pb.lc2.lowerForceOvesteppings);
	EEPROM.get(LC2_NR_UPPER_FORCE_OVS_ADD, pb.lc2.nrUpperForceOversteppings);
	EEPROM.get(LC2_NR_LOWER_FORCE_OVS_ADD, pb.lc2.nrLowerForceOversteppings);
	EEPROM.get(LC2_FORCE_OFFSET_ADD, pb.lc2.forceOffset);
	EEPROM.get(LC2_FORCE_CAPACITY_ADD, pb.lc2.Nmax);
	EEPROM.get(PB_RELAY_STATE_ADD, pb.relayStt);
#endif

	EEPROM.get(SENSORS_DIST_ADD, node.sensorsDistance);

	EEPROM.get(CONN_TYPE_ADD, node.connType);

	EEPROM.get(WIFI_SSID_ADD, routerSSID);
	EEPROM.get(WIFI_PASSWORD_ADD, routerPassword);
}


bool SYS_UpdateRTC(char* date, char* hhmmss)
{
	uint16_t yyyy;
	uint8_t mm, dd, hh, mn, ss;
	char strDate[5];

	memcpy(strDate, date, 4);
	strDate[4] = '\0';
	yyyy = atoi(strDate);

	memcpy(strDate, date + 5, 2);
	strDate[2] = '\0';
	mm = atoi(strDate);

	memcpy(strDate, date + 8, 2);
	strDate[2] = '\0';
	dd = atoi(strDate);

	memcpy(strDate, hhmmss, 2);
	strDate[2] = '\0';
	hh = atoi(strDate);

	memcpy(strDate, hhmmss + 3, 2);
	strDate[2] = '\0';
	mn = atoi(strDate);

	memcpy(strDate, hhmmss + 6, 2);
	strDate[2] = '\0';
	ss = atoi(strDate);

	// update rtc with new date and hour
	if ((yyyy > 2019) && (yyyy < 2050) && (mm > 0) && (mm < 13) && (dd > 0) && (dd < 32) && (hh >= 0) && (hh < 24) && (mn >= 0) && (mn < 60) && (ss >= 0) && (ss < 60))
	{
		rtc.setTime(ss, mn, hh, dd, mm, yyyy);
		vTaskDelay(pdMS_TO_TICKS(1000));

		DebugSerial.printf("\r\nTime Updated: ");
		//DebugSerial.println(rtc.getTime("%A, %B %d %Y %H:%M:%S"));
		DebugSerial.printf("%02d:%02d:%02d  %02d/%02d/%04d\r\n", \
			rtc.getHour(true), rtc.getMinute(), rtc.getSecond(), \
			rtc.getDay(), rtc.getMonth() + 1, rtc.getYear());

		return true;
	}
	else
	{
		// Date not valid
		DebugSerial.printf("Not valid date: %04u/%02u/%02u %02u:%02u:%02u\n", yyyy, mm, dd, hh, mn, ss);
		return false;
	}
}


void SYS_GetTimeRTC(char* date, char* hour, char* hhmmss)
{
	// date = "2023/11/28", hour = "9:30"

	if ((rtc.getYear() < 2023) || (rtc.getYear() > 2100))
	{
		DebugSerial.println("\r\nRTC not initialized - Setting dummy time !");
		rtc.setTime(00, 30, 9, 28, 11, 2023);
		vTaskDelay(pdMS_TO_TICKS(1000));
	}

	// Set time to local variables - used for telemetries saved to memory
	sprintf(date, "%04u/%02u/%02u", rtc.getYear(), rtc.getMonth() + 1, rtc.getDay());
	sprintf(hour, "%02u:%02u", rtc.getHour(true), rtc.getMinute());
	//sprintf(hhmmss, "%02u:%02u:%02u", rtc.getHour(true), rtc.getMinute(), rtc.getSecond());
	sprintf(hhmmss, "%02u:%02u:%02u", rtc.getHour(true), rtc.getMinute(), 9);						// Force seconds value

	DebugSerial.printf("RTC timestamp | date: %s | hhmm: %s | hhmmss: %s\r\n", date, hour, hhmmss);
}


double fmap(double x, double x1, double x2, double y1, double y2)
{
	double y, yAux1, yAux2, yAux3;

	// y = ((y2-y1)/(x2-x1))*(x-x1) + y1;

	yAux1 = (y2 - y1);
	yAux2 = (x2 - x1);
	yAux3 = yAux1 / yAux2;
	yAux3 = yAux3 * (x - x1);

	y = yAux3 + y1;

	//Serial.printf("Valores %f | %f | %f | %f\n", yAux1, yAux2, yAux3, y);

	return y;
}


/* Function to sort an uint16 buffer */
void sortInt(uint16_t* buff, uint8_t nSamples)
{
	uint16_t aux;
	uint8_t n, k;

	for (n = 0; n < nSamples - 1; n++)
	{
		for (k = 0; k < nSamples - (n + 1); k++)
		{
			if (buff[k] > buff[k + 1])
			{
				aux = buff[k];
				buff[k] = buff[k + 1];
				buff[k + 1] = aux;
			}
		}
	}
}


uint16_t meanInt(uint16_t* buffer, uint8_t nSamples)
{
	uint8_t i;
	uint32_t sum = 0;

	for (i = 0; i < nSamples; i++)
	{
		sum += *(buffer + i);
	}

	return (uint16_t)(sum / nSamples);
}


void sortI32(int32_t* buff, uint8_t nSamples)
{
	int32_t aux;
	uint8_t n, k;

	for (n = 0; n < nSamples - 1; n++)
	{
		for (k = 0; k < nSamples - (n + 1); k++)
		{
			if (buff[k] > buff[k + 1])
			{
				aux = buff[k];
				buff[k] = buff[k + 1];
				buff[k + 1] = aux;
			}
		}
	}
}


int32_t meanI32(int32_t* buffer, uint8_t nSamples)
{
	uint8_t i;
	int64_t sum = 0;

	for (i = 0; i < nSamples; i++)
	{
		sum += *(buffer + i);
	}

	return (int32_t)(sum / nSamples);
}


double meanF(double* buffer, uint8_t nSamples)
{
	uint8_t i;
	double sum = 0;

	for (i = 0; i < nSamples; i++)
	{
		sum += (*(buffer + i));
	}

	return (sum / (double)(nSamples));
}


// Auxliary functions do display lines
// Add line to the list of lines, in case of overlapping, scroll lines from bottom to up
// If user wants to clear all messages, set clear
void AddLine(char* line, uint8_t clear)
{
	uint8_t lineToWrite;
	uint8_t lineLen = strlen(line);

	if (lineLen > MAX_CHARS)
		lineLen = MAX_CHARS;
	
	if (clear == 1)
	{
		memset(&lines.text[0][0], 0x00, N_LINES*MAX_CHARS);
		lines.line = 0;
		lines.overlap = 0;

		// Copy new line
		lines.line++;
		memcpy(&lines.text[lines.line-1][0], line, lineLen);
	}
	else
	{
		if (lines.overlap == 1)
		{
			// Move message beetween lines
			for (uint8_t i=0; i<N_LINES-1; i++)
				memcpy(&lines.text[i][0], &lines.text[i+1][0], MAX_CHARS);

			// Set line to write - In this case is the last one
			lineToWrite = N_LINES - 1;
		}
		else
		{
			lines.line++;
			lineToWrite = lines.line-1;			

			if (lines.line > N_LINES-1)
			{
				lines.line = N_LINES;
				lines.overlap = 1;
			}
		}

		// Clear message before copy new one
		memset(&lines.text[lineToWrite][0], 0x00, MAX_CHARS);
		// Copy new message on the line defined
		memcpy(&lines.text[lineToWrite][0], line, lineLen);
	}
}

// Write line on the same line that was already written
void SetLine(char* line)
{
	uint8_t lineLen = strlen(line);

	// Clear message before copy new one
	memset(&lines.text[lines.line-1][0], 0x00, MAX_CHARS);
	// Copy new message on the line defined
	memcpy(&lines.text[lines.line-1][0], line, lineLen);
}

void DisplayLines(void)
{
	display.clearDisplay();
	display.display();

	for (uint8_t i = 0; i < lines.line; i++)
	{
		display.setCursor(X_INIT_POS, Y_INIT_POS + (i*N_PIXELS_LINE));
		display.println(&lines.text[i][0]);
	}

	display.display();
}

float CalculateDisplacement(void)
{
	float result = 0.0;

	// Test
	/*for (uint8_t idx = 0; idx < ECIT_BUFFER; idx++)
	{
		DebugSerial.printf("\r\n[%d]: %d | %d | %.1f | %d\r\n", idx, data[idx].dir, data[idx].deltaTimer, data[idx].speed, data[idx].sampleTime);
	}*/

	for (uint8_t idx = 0; idx < (ECIT_BUFFER-3); idx++)
	{
		if (data[idx].dir == PULL_DIR)
		{
			if ( (data[idx + 1].dir == PUSH_DIR)	\
				&& (data[idx + 2].dir == PUSH_DIR)	\
				&& (data[idx + 3].dir == PULL_DIR)	\
				&& (data[idx + 2].sampleTime > data[idx + 1].sampleTime)	\
				&& (data[idx + 1].speed != 0.0)		\
				&& (data[idx + 2].speed != 0.0) )
			{
				telemEcit[0].dir = CALCULATION_PASS;
				telemEcit[0].deltaSampleTime = data[idx + 2].sampleTime - data[idx + 1].sampleTime;
				telemEcit[0].sampleSpeed = (float)(fmax(data[idx + 1].speed, data[idx + 2].speed));
				result = telemEcit[0].sampleSpeed * (((float)telemEcit[0].deltaSampleTime) / 1000.0);
				break;			
			}
		}
	}

	return result;
}

