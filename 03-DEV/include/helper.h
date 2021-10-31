#ifndef HELPER_H_
#define HELPER_H_

/**************************************************************
  Debug to serial
  Also used by NeoGPS library
**************************************************************/
//#define DEBUG // /!\ DAWA will wait serial connection to start
#ifdef DEBUG
#define DEBUG_PORT SERIAL_PORT_USBVIRTUAL
#endif

/*****************************************************
 * ################################################# *
 * ############## Constants definition ############# *
 * ################################################# *
*****************************************************/

#define TFTDEFAULTFONTCOLOR 0xffffffUL               // TFT default font color
#define GEAR_CALIB_SIZE 7                            // How many gears we have
#define MAX_MLX_SIZE 6                               // Max MLX chips that could be declared (could be more but need to add object instances, just before setup())
#define FIRST_MLX_ADDRESS 0x01                       // First allocated MLX I2C address (then +1 on each discovered MLX sensor)
#define GEAR_OFFSET 200                              // Each gear has a corresponding value, this value define the interval (value - GEAR_OFFSET < measure < value + GEAR_OFFSET)
#define DEFAULT_ENDIGITALINPUTSBITS_VALUE 0b00001111 // Default enabled digital inputs
#define DEFAULT_ENANALOGINPUTSBITS_VALUE 0b00001111  // Default enabled analog inputs
#define DEFAULT_RPM_CORRECTION_RATIO 105             // RPM correction ratio (100 = no correction, 105 = x 1.05)
#define DEFAULT_RPM_FLYWHEEL_TEETH 22                // Flywheel teeth (Triumph Daytona = 22)
#define MAX_TRACK_DISTANCE 5                         // Autoselect nearest track (unit = km)
#define TFT_DEFAULT_BORDER_H_SIZE 5
#define TFT_DEFAULT_BORDER_V_SIZE 1
#define TFT_BUTTON_BORDER_SIZE 5
#define TFT_BUTTON_H_SIZE 80
#define TFT_BUTTON_H2_SIZE 160
#define TFT_BUTTON_H3_SIZE 20
#define TFT_BUTTON_H4_SIZE 60
#define TFT_BUTTON_V_SIZE 30
#define TFT_BUTTON_V3_SIZE 20
#define TFT_BUTTON_H_SPACE 10
#define TFT_BUTTON_V_SPACE 10
#define TFT_HEADER_H_SPACE 30
#define TFT_HEADER_V_SPACE 30
#define TFT_FONT_01_SIZE 12
#define TFT_FONT_01_V_SPACE 15
#define TFT_FONT_02_SIZE 13
#define TFT_FONT_02_V_SPACE 19
#define TFT_FONT_03_SIZE 14
#define TFT_FONT_03_V_SPACE 75

// I/O Pins
constexpr uint8_t inDigiBrakePin = 3; // D3, digital input
constexpr uint8_t inDigiOpt1Pin = 11; // D11, digital input
constexpr uint8_t sdCsPin = 5;        // D5, Chip Select for SDCARD on SPI bus
constexpr uint8_t mcp1Led1 = 8;       // Led 1 on MCP23017
constexpr uint8_t mcp1Led2 = 9;       // Led 2 on MCP23017
constexpr uint8_t mcp1Led3 = 10;      // Led 3 on MCP23017
constexpr uint8_t mcp1Led4 = 11;      // Led 4 on MCP23017
constexpr uint8_t mcp1Led5 = 12;      // Led 5 on MCP23017
constexpr uint8_t mcp1Led6 = 13;      // Led 6 on MCP23017
constexpr uint8_t mcp1Led7 = 14;      // Led 7 on MCP23017
constexpr uint8_t mcp1Led8 = 15;      // Led 8 on MCP23017
constexpr uint8_t powerState = A1;    // A1, power switch state (set 0 to power off)

// Others ...
constexpr char csvDelim = ';'; // CSV file delimiter

// GPS & Timing
constexpr float rescaleGPS = 10000000.0; // We use "long" for GPS coordinates to keep precision ("float" on Arduino have only 6 decimal digits of precision) ### https://gis.stackexchange.com/questions/8650/measuring-accuracy-of-latitude-and-longitude

// GPS Configuration - General
constexpr unsigned char ubxRate10Hz[] PROGMEM = {0x06, 0x08, 0x06, 0x00, 0x64, 0x00, 0x01, 0x00, 0x01, 0x00};
constexpr unsigned char ubxPrtConf[] PROGMEM = {0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xd0, 0x08, 0x00, 0x00, 0x00, 0xc2, 0x01, 0x00, 0x07, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00}; // 115200bps
//                                             |ID         |Lenght     |Port |res  |TX Ready   |mode                   |baudrate               |inPrMask   |outPrMask  |flags      |res       |

// GPS Configuration - Enable/disable specific NMEA sentences
const unsigned char ubxDisableGGA[] PROGMEM = {0x06, 0x01, 0x08, 0x00, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01};
const unsigned char ubxDisableGLL[] PROGMEM = {0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01};
const unsigned char ubxDisableGSA[] PROGMEM = {0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01};
const unsigned char ubxDisableGSV[] PROGMEM = {0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01};
const unsigned char ubxDisableRMC[] PROGMEM = {0x06, 0x01, 0x08, 0x00, 0xF0, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01};
const unsigned char ubxDisableVTG[] PROGMEM = {0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01};
const unsigned char ubxDisableZDA[] PROGMEM = {0x06, 0x01, 0x08, 0x00, 0xF0, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01};

// Set these values to the offset of your timezone from GMT
constexpr int32_t zone_hours = +1L;  // EST
constexpr int32_t zone_minutes = 0L; // usually zero
constexpr NeoGPS::clock_t zone_offset =
    zone_hours * NeoGPS::SECONDS_PER_HOUR +
    zone_minutes * NeoGPS::SECONDS_PER_MINUTE;

// Uncomment one DST changeover rule, or define your own:
//#define USA_DST
#define EU_DST
#if defined(USA_DST)
constexpr uint8_t springMonth = 3;
constexpr uint8_t springDate = 14; // latest 2nd Sunday
constexpr uint8_t springHour = 2;
constexpr uint8_t fallMonth = 11;
constexpr uint8_t fallDate = 7; // latest 1st Sunday
constexpr uint8_t fallHour = 2;
#define CALCULATE_DST

#elif defined(EU_DST)
constexpr uint8_t springMonth = 3;
constexpr uint8_t springDate = 31; // latest last Sunday
constexpr uint8_t springHour = 1;
constexpr uint8_t fallMonth = 10;
constexpr uint8_t fallDate = 31; // latest last Sunday
constexpr uint8_t fallHour = 1;
#define CALCULATE_DST
#endif

// Texts
constexpr static char LABEL_VERSION[] PROGMEM = "DAwA v7.0";
constexpr static char LABEL_KMH[] PROGMEM = "km/h";
constexpr static char LABEL_RPM[] PROGMEM = "rpm x1000";

constexpr static char MENU_SETUP[] PROGMEM = "SETUP";
constexpr static char MENU_SHUTDOWN[] PROGMEM = "OFF";
constexpr static char MENU_GENERAL[] PROGMEM = "GENERAL";
constexpr static char MENU_TEMP[] PROGMEM = "TEMP.";
constexpr static char MENU_OBD[] PROGMEM = "OBD";
constexpr static char MENU_ANALOG[] PROGMEM = "ANALOG";
constexpr static char MENU_DIGITAL[] PROGMEM = "DIGITAL";
constexpr static char MENU_TRACKS[] PROGMEM = "TRACKS";
constexpr static char MENU_START[] PROGMEM = "START";
constexpr static char MENU_STOP[] PROGMEM = "STOP";
constexpr static char MENU_FAKELAP[] PROGMEM = "FAKELAP";
constexpr static char MENU_GPS[] PROGMEM = "GPS";
constexpr static char MENU_SDCARD[] PROGMEM = "SDCARD";
constexpr static char MENU_EEPROM[] PROGMEM = "EEPROM";
constexpr static char MENU_RESET_EEPROM[] PROGMEM = "RESET";
constexpr static char MENU_DEFAULT_EEPROM[] PROGMEM = "DEFAULT";
constexpr static char MENU_BACK[] PROGMEM = "BACK";
constexpr static char MENU_AUTODETECT_MLX[] PROGMEM = "AUTO DETECT";

constexpr static char LABEL_DEBUG_IS_ENABLED[] PROGMEM = "DEBUG is enabled";
constexpr static char LABEL_DAWA_INIT[] PROGMEM = "D.A.W.A. Init ...";
constexpr static char LABEL_OK[] PROGMEM = "OK";
constexpr static char LABEL_OFF[] PROGMEM = "OFF";
constexpr static char LABEL_FAILED[] PROGMEM = "FAILED";
constexpr static char LABEL_BLE[] PROGMEM = "BLUETOOTH";
constexpr static char LABEL_OLED[] PROGMEM = "OLED";
constexpr static char LABEL_SD[] PROGMEM = "SD";
constexpr static char LABEL_OBD[] PROGMEM = "OBD";
constexpr static char LABEL_EEPROM[] PROGMEM = "EEPROM";
constexpr static char LABEL_ADC[] PROGMEM = "ADC";
constexpr static char LABEL_GPS[] PROGMEM = "GPS";
constexpr static char LABEL_READY[] PROGMEM = "READY !";
constexpr static char LABEL_DEG[] PROGMEM = "deg";
constexpr static char LABEL_G[] PROGMEM = "g";
constexpr static char LABEL_IN_CHANGED[] PROGMEM = "Input changed";
constexpr static char LABEL_ERR_WR_IN_ST[] PROGMEM = "Error writing inputs state";
constexpr static char LABEL_GEAR_CAL_OK[] PROGMEM = "Gear cal. done !";
constexpr static char LABEL_GEAR_CAL_FAILED[] PROGMEM = "Gear cal. failed !";
constexpr static char LABEL_SET_GEAR_TO_N[] PROGMEM = "Set GEAR to N";
constexpr static char LABEL_SET_GEAR_TO_X[] PROGMEM = "Set GEAR to ";
constexpr static char LABEL_THR_CAL[] PROGMEM = "Accelerate ... ";
constexpr static char LABEL_THR_CAL_OK[] PROGMEM = "Throttle cal. done !";
constexpr static char LABEL_THR_CAL_FAILED[] PROGMEM = "Throttle cal. failed !";
constexpr static char LABEL_ANAOPT1_CAL_MIN[] PROGMEM = "Set OPT1 to 0% ... ";
constexpr static char LABEL_ANAOPT1_CAL_MAX[] PROGMEM = "Set OPT1 to 100% ... ";
constexpr static char LABEL_ANAOPT1_CAL_OK[] PROGMEM = "OPT1 calib. done !";
constexpr static char LABEL_ANAOPT1_CAL_FAILED[] PROGMEM = "OPT1 calib. failed !";
constexpr static char LABEL_ANAOPT2_CAL_MIN[] PROGMEM = "Set OPT2 to 0% ... ";
constexpr static char LABEL_ANAOPT2_CAL_MAX[] PROGMEM = "Set OPT2 to 100% ... ";
constexpr static char LABEL_ANAOPT2_CAL_OK[] PROGMEM = "OPT2 calib. done !";
constexpr static char LABEL_ANAOPT2_CAL_FAILED[] PROGMEM = "OPT2 calib. failed !";
constexpr static char LABEL_NEW_VAL_SAVED[] PROGMEM = "New value saved";
constexpr static char LABEL_SAVE_FAILED[] PROGMEM = "save failed !";
constexpr static char LABEL_NEWVAL[] PROGMEM = "Measured val : ";
constexpr static char LABEL_NEWVAL_MIN[] PROGMEM = "New val (min) : ";
constexpr static char LABEL_NEWVAL_MAX[] PROGMEM = "New val (max) : ";
constexpr static char LABEL_TRK[] PROGMEM = "Track";
constexpr static char LABEL_TRK_L[] PROGMEM = "L";
constexpr static char LABEL_NO_TRACK[] PROGMEM = "Can't find any information about track";
constexpr static char LABEL_TRK_BEST[] PROGMEM = "Best";
constexpr static char LABEL_TRK_NO_LAP[] PROGMEM = "No valid lap";
constexpr static char LABEL_TRK_NO_RUN[] PROGMEM = "Can't find any run since the last ";
constexpr static char LABEL_TRK_HOUR[] PROGMEM = " hours";
constexpr static char LABEL_ABOUT1[] PROGMEM = "D.A.W.A.";
constexpr static char LABEL_ABOUT2[] PROGMEM = "Hw DAWA :     6.1";
constexpr static char LABEL_ABOUT3[] PROGMEM = "Hw DAWA-MPU : 6.3";
constexpr static char LABEL_ABOUT4[] PROGMEM = "Sw :          6.3";
constexpr static char LABEL_ABOUT5[] PROGMEM = "Author : E.PIGEON";
constexpr static char LABEL_ABOUT6[] PROGMEM = "dawa@panik-po.com";
constexpr static char LABEL_ABOUT7[] PROGMEM = "dawa.panik-po.com";
constexpr static char LABEL_RUNNING[] PROGMEM = "Running";
constexpr static char LABEL_THROTTLE[] PROGMEM = "Throttle";
constexpr static char LABEL_THROTTLE_MAX[] PROGMEM = "Throttle max";
constexpr static char LABEL_GEAR[] PROGMEM = "Gear";
constexpr static char LABEL_MENU_1[] PROGMEM = "MENU       SCR";
constexpr static char LABEL_MENU_2[] PROGMEM = "           BACK";
constexpr static char LABEL_MENU_3[] PROGMEM = "           STOP";
constexpr static char LABEL_MENU_4[] PROGMEM = " UP | DW | BACK | OK ";
constexpr static char LABEL_WORK_PROGRESS[] PROGMEM = "Work in progress !";
constexpr static char LABEL_LOG_HEADER_1[] PROGMEM = "TIME;DISTANCE;LAP;";
constexpr static char LABEL_LOG_HEADER_2[] PROGMEM = "KPH;HEADING;";
constexpr static char LABEL_LOG_HEADER_3[] PROGMEM = "IRTEMP";
constexpr static char LABEL_LOG_HEADER_4[] PROGMEM = "LATITUDE;LONGITUDE";
constexpr static char LABEL_LOG_NEWFILE[] PROGMEM = "Create new file";
constexpr static char LABEL_LOG_APPNDFILE[] PROGMEM = "Append file";
constexpr static char LABEL_LOG_FILEERROR[] PROGMEM = "R/W file error !";
constexpr static char LABEL_LOG_NOTRKFILE[] PROGMEM = "No track file !";
constexpr static char LABEL_LOG_KM[] PROGMEM = "km";
constexpr static char LABEL_AUTOSEL_TRK[] PROGMEM = "Auto-select track";
constexpr static char LABEL_NEW_SESSION[] PROGMEM = "[Start new session !]";
constexpr static char LABEL_END_SESSION[] PROGMEM = "[Session stopped !]";
constexpr static char LABEL_NO_GPS_SIGNAL[] PROGMEM = "No valid GPS signal !";
constexpr static char LABEL_RUN_AS_DATALOGGER[] PROGMEM = "No track file, run as datalogger only";
constexpr static char LABEL_GEAR_CALIBRATION[] PROGMEM = "Set gear to ";
constexpr static char LABEL_GEAR_CALIBRATION_OK[] PROGMEM = "Gears calibrated !";

const char *const digitalInputsLabel[] = {"RPM", "SQR1", "DIG1", "DIG2"};
const char *const analogInputsLabel[] = {"GEAR", "ANA2", "ANA3", "ANA4", "ANA5", "ANA6", "ANA7", "ANA8"};
const char gearName[] = {'N', '1', '2', '3', '4', '5', '6'};

/*****************************************************
 * ################################################# *
 * ############## Vars definition ################## *
 * ################################################# *
*****************************************************/

// Inputs
extern uint32_t inDigiSqrRpm;       // Digital square input (RPM)
extern uint32_t inDigiSqrOpt1;      // Digital square input (optional)
extern bool inDigiBrake;            // Digital boolean input (brake)
extern bool inDigiOpt1;             // Digital boolean input (optional)
extern uint16_t inAnaGear;          // Digital analog input (GEAR)
extern uint16_t inAnaThrottle;      // Digital analog input (THROTTLE)
extern uint16_t inAnaOpt1;          // Digital analog input (optional)
extern uint16_t inAnaOpt2;          // Digital analog input (optional)
extern uint8_t enDigitalInputsBits; // Store enabled digital inputs (use binary values, ex : 10100000 > DIGI_SQR_RPM and DIGI_BRAKE enabled)
// BIT 0 = DIGI_SQR_RPM
// BIT 1 = DIGI_SQR_OPT_1
// BIT 2 = DIGI_BRAKE
// BIT 3 = DIGI_OPT_1
// BIT 4 = N/A
// BIT 5 = N/A
// BIT 6 = N/A
// BIT 7 = N/A
extern uint8_t enAnalogInputsBits; // Store enabled digital inputs (use binary values, ex : 11000000 > inAnaOpt1 and inAnaOpt2 enabled)
// BIT 0 = inAnaOpt1
// BIT 1 = inAnaOpt2
// BIT 2 = inAnaOpt3
// BIT 3 = inAnaOpt4
// BIT 4 = inAnaOpt5
// BIT 5 = inAnaOpt6
// BIT 6 = inAnaOpt7
// BIT 7 = inAnaOpt8
extern uint8_t tmpComp, bitShift; // Used to compare values for enabled inputs checks

extern uint16_t inAnaGearCalib[7]; // Calibration values for GEAR input
extern uint8_t gearNCheck;

// SD Card
extern File logFile;     // One line every 100ms with all detailed data
extern File lapFile;     // One line per lap with laptime
extern File trackFile;   // One line per track with GPS coordinates of finish line
extern File historyFile; // History of all sessions
extern char filename[32];

// GPS & Timing
extern bool recordTrackData, addFinishLog, isRunning;
extern char trackName[16];
extern uint8_t lapCounter;
extern int16_t runMinutes, runSeconds;
extern uint16_t trackId, lapId;
extern int32_t timeCsec, timeSec, lastFlSec, lastFlCsec, lapSec, lapCsec, posCrossLat, posCrossLon, flineLat1, flineLon1, flineLat2, flineLon2; // Finish line GPS coordinates
extern uint32_t lastPinRead, lastOneSecSync, lastSdSync, fixCount, elapsedTime;
extern float coordsDistance, totalDistance, tToFl; // Time To Finish Line
extern uint8_t gpsFixStatus;

// Analog to Digital converter (ADC)
extern uint16_t anaValues[8];
extern char anaValuesChar[8]; // Store value as a single character (useful for ANA1/GEAR : N,1,2,3 ...)
extern uint32_t digValues[4];

// Infrared temp sensor
extern uint8_t mlxAddresses[MAX_MLX_SIZE]; // Store each MLX I2C addresses
extern double mlxValues[MAX_MLX_SIZE];     // Store MLX values

// RPM
extern uint8_t rpmFlywheelTeeth;
extern uint8_t rpmCorrectionRatio;

// Speed
//extern uint8_t getSpeedFromGps;

// Buttons & menu
extern bool fakeLap; // Used to simulate a new lap (first left button when laptimer running)
extern char msgLabel[255];
extern uint32_t msgDelay;
extern uint8_t msgType;

// TFT screen
extern uint32_t lastTftTouchSync;
extern uint8_t lastTftPrintSync;
extern uint32_t currentMs;
extern uint8_t tftScreenId; // Which screen is printed (0 = start page) / defined as "extern uint8_t" in tft.cpp

/*****************************************************
 * ################################################# *
 * ############## Instantiate objects ############## *
 * ################################################# *
*****************************************************/

// EEPROM
extern extEEPROM eep;

// I/O Expander MCP23017
extern Adafruit_MCP23017 MCP1;
extern Adafruit_MCP23017 MCP2;

// SD Card
extern SdFat sd;

// GPS
extern NMEAGPS gps;
extern gps_fix fix_data;
extern gps_fix fix_data_prev;

// GPS
extern HardwareSerial &GPS_PORT;

// OBD2 serial port
extern Uart OBD2_PORT;

// OBD2 ELMduino object
extern ELM327 myELM327;

/*****************************************************
 * ################################################# *
 * ############## Functions ######################## *
 * ################################################# *
*****************************************************/

void initError(uint8_t);
void eepromReload(void);
void eepromLoadDefaults(void);
void eepromSaveRunningValues(void);
void eepromReset(void);
int csvReadText(File *file, char *str, size_t size, char delim);
int csvReadInt32(File *file, int32_t *num, char delim);
int csvReadInt16(File *file, int16_t *num, char delim);
int csvReadUint32(File *file, uint32_t *num, char delim);
int csvReadUint16(File *file, uint16_t *num, char delim);
int csvReadDouble(File *file, double *num, char delim);
int csvReadFloat(File *file, float *num, char delim);
void dateTimeSd(uint16_t *date, uint16_t *time);
boolean segIntersect(int32_t pos_now_lat, int32_t pos_now_lon, int32_t pos_prev_lat, int32_t pos_prev_lon, int32_t trackLat1, int32_t trackLon1, int32_t trackLat2, int32_t trackLon2, int32_t &pos_cross_lat, int32_t &pos_cross_lon);
float gpsDistance(int32_t lat1, int32_t lon1, int32_t lat2, int32_t lon2);
void stopLaptimer(void);
boolean startLaptimer(void);
void showMessage(const char *label, uint32_t delay, uint8_t type);
void sendUBX(const unsigned char *progmemBytes, size_t len);
void timeAdd(float timeSecCsec, int32_t endSec, int32_t endCsec, int32_t &returnSec, int32_t &returnCsec);
void timeSubstract(int32_t s1, int32_t cs1, int32_t s2, int32_t cs2, int32_t &returnSec, int32_t &returnCsec);
void adjustTime(NeoGPS::time_t &dt);

uint8_t initADC(void);
uint8_t configureADC(uint8_t bits);
void showADCPortState(void);
uint16_t readAdcValue(uint8_t registerID);
void readAdcValues(uint16_t anaValues[]);
void formatAdcValues(uint16_t anaValues[]);
void gearCalibration(void);
void autodetectMlx(void);
void mcp2EnableOneOutput(uint8_t idOutput);

void SERCOM5_Handler(void);
template <class T>
int EEPROM_writeAnything(int ee, const T &value)
{
  const byte *p = (const byte *)(const void *)&value;
  unsigned int i;
  for (i = 0; i < sizeof(value); i++)
  {
    eep.write(ee++, *p++);
  }
  return i;
}
template <class T>
int EEPROM_readAnything(int ee, T &value)
{
  byte *p = (byte *)(void *)&value;
  unsigned int i;
  for (i = 0; i < sizeof(value); i++)
  {
    *p++ = eep.read(ee++);
  }
  return i;
}

#endif /* HELPER_H_ */