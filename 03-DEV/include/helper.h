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

#define GEAR_CALIB_SIZE 7                            // How many gears we have
#define MAX_MLX_SIZE 6                               // Max MLX chips that could be declared (could be more but need to add object instances, just before setup())
#define FIRST_MLX_ADDRESS 0x01                       // First allocated MLX I2C address (then +1 on each discovered MLX sensor)
#define GEAR_OFFSET 200                              // Each gear has a corresponding value, this value define the interval (value - GEAR_OFFSET < measure < value + GEAR_OFFSET)
#define DEFAULT_ENDIGITALINPUTSBITS_VALUE 0b00001111 // Default enabled digital inputs
#define DEFAULT_ENANALOGINPUTSBITS_VALUE 0b00001111  // Default enabled analog inputs
#define DEFAULT_RPM_CORRECTION_RATIO 105             // RPM correction ratio (100 = no correction, 105 = x 1.05)
#define DEFAULT_RPM_FLYWHEEL_TEETH 22                // Flywheel teeth (Triumph Daytona = 22)
#define DEFAULT_SELECTED_TRACK_ID -1                 // Default selected track ID (-1 = nullTrack)
#define DEFAULT_OBD_FEATURES 0                       // Default OBD features (disabled)
#define DEFAULT_TRACK_QTY 0                          // Default number of tracks in EEPROM
#define MAX_TRACKS 50                                // Maximum number of tracks that can be handle (loaded in EEPROM and RAM)
#define TRACK_PAGINATION 6                           // Number of tracks shown in one page
#define MAX_TRACK_DISTANCE 5                         // Autoselect nearest track (unit = km)
#define EEPROM_SIZE 8192                             // EEPROM size in kBytes (not kbits !)
#define EEPROM_GENERAL_ADDR 0                        // First address on EEPROM to save general data
#define EEPROM_GEAR_CALIBRATION_ADDR 10              // First address on EEPROM to save gear calibration data
#define EEPROM_MLX_ADDR 30                           // First address on EEPROM to save MLX data
#define EEPROM_FIRST_TRACK_ADDR 40                   // First address on EEPROM to save tracks data
#define MAX_LAPS 30                                  // Max laps in an array
#define LAP_PAGINATION 8                             // Number of laps shown in one page
#define MAX_HISTORY_WHEN_RUNNING 3                   // Number of history laptime printed when running
#define TFT_DEFAULT_FONT_COLOR 0xffffffUL            // TFT default font color
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
#define TFT_HEADER_V_SPACE_HEADER 60
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
constexpr static char MENU_LAPTIMES[] PROGMEM = "LAPS";
constexpr static char MENU_FAKELAP[] PROGMEM = "FAKELAP";
constexpr static char MENU_GPS[] PROGMEM = "GPS";
constexpr static char MENU_SDCARD[] PROGMEM = "SDCARD";
constexpr static char MENU_EEPROM[] PROGMEM = "EEPROM";
constexpr static char MENU_RESET_EEPROM[] PROGMEM = "RESET";
constexpr static char MENU_DEFAULT_EEPROM[] PROGMEM = "DEFAULT";
constexpr static char MENU_BACK[] PROGMEM = "BACK";
constexpr static char MENU_AUTODETECT_MLX[] PROGMEM = "AUTO DETECT";
constexpr static char MENU_IMPORT_TRACK[] PROGMEM = "IMPORT TRACKS";
constexpr static char MENU_PREVIOUS[] PROGMEM = "<<";
constexpr static char MENU_NEXT[] PROGMEM = ">>";
constexpr static char MENU_AUTOSELECT_TRACK[] PROGMEM = "AUTOSELECT";
constexpr static char MENU_BEST_LT_TRACK[] PROGMEM = "BEST TRK";
constexpr static char MENU_BEST_LT_ALL_TRK[] PROGMEM = "HoF";
constexpr static char MENU_BEST_LT_72H[] PROGMEM = "BEST 72H";
constexpr static char MENU_BEST_LT_LASTRUN[] PROGMEM = "LAST RUN";
constexpr static char LABEL_VERSION[] PROGMEM = "DAwA v7.0";
constexpr static char LABEL_KMH[] PROGMEM = "km/h";
constexpr static char LABEL_RPM[] PROGMEM = "rpm x1000";
constexpr static char LABEL_DEBUG_IS_ENABLED[] PROGMEM = "DEBUG is enabled";
constexpr static char LABEL_DAWA_INIT[] PROGMEM = "D.A.W.A. Init ...";
constexpr static char LABEL_OK[] PROGMEM = "OK";
constexpr static char LABEL_OFF[] PROGMEM = "OFF";
constexpr static char LABEL_FAILED[] PROGMEM = "FAILED";
constexpr static char LABEL_SD_OK[] PROGMEM = "SD : OK";
constexpr static char LABEL_SD_FAILED[] PROGMEM = "SD : FAILED";
constexpr static char LABEL_OBD_OK[] PROGMEM = "OBD : OK";
constexpr static char LABEL_OBD_FAILED[] PROGMEM = "OBD : FAILED";
constexpr static char LABEL_EEPROM_OK[] PROGMEM = "EEPROM : OK";
constexpr static char LABEL_EEPROM_FAILED[] PROGMEM = "EEPROM : FAILED";
constexpr static char LABEL_ADC_OK[] PROGMEM = "ADC : OK";
constexpr static char LABEL_ADC_FAILED[] PROGMEM = "ADC : FAILED";
constexpr static char LABEL_MCP1_OK[] PROGMEM = "MCP1 : OK";
constexpr static char LABEL_MCP1_FAILED[] PROGMEM = "MCP1 : FAILED";
constexpr static char LABEL_MCP2_OK[] PROGMEM = "MCP2 : OK";
constexpr static char LABEL_MCP2_FAILED[] PROGMEM = "MCP2 : FAILED";
constexpr static char LABEL_GPS_OK[] PROGMEM = "GPS : OK";
constexpr static char LABEL_READY[] PROGMEM = "READY !";
constexpr static char LABEL_TRK[] PROGMEM = "Track";
constexpr static char LABEL_TRK_UC[] PROGMEM = "TRK:";
constexpr static char LABEL_ABOUT1[] PROGMEM = "D.A.W.A.";
constexpr static char LABEL_ABOUT2[] PROGMEM = "Hw DAWA :     6.1";
constexpr static char LABEL_ABOUT3[] PROGMEM = "Hw DAWA-MPU : 6.3";
constexpr static char LABEL_ABOUT4[] PROGMEM = "Sw :          6.3";
constexpr static char LABEL_ABOUT5[] PROGMEM = "Author : E.PIGEON";
constexpr static char LABEL_ABOUT6[] PROGMEM = "dawa@panik-po.com";
constexpr static char LABEL_ABOUT7[] PROGMEM = "dawa.panik-po.com";
constexpr static char LABEL_LOG_HEADER_1[] PROGMEM = "TIME;DISTANCE;LAP;";
constexpr static char LABEL_LOG_HEADER_2[] PROGMEM = "KPH;HEADING;";
constexpr static char LABEL_LOG_HEADER_3[] PROGMEM = "IRTEMP";
constexpr static char LABEL_LOG_HEADER_4[] PROGMEM = "LATITUDE;LONGITUDE";
constexpr static char LABEL_LOG_NEWFILE[] PROGMEM = "Create new file";
constexpr static char LABEL_LOG_APPNDFILE[] PROGMEM = "Append file";
constexpr static char LABEL_LOG_FILEERROR[] PROGMEM = "R/W file error !";
constexpr static char LABEL_LOG_NOTRKFILE[] PROGMEM = "Can't find TRACKS.csv file on SDCARD";
constexpr static char LABEL_TRACKFILENAME[] PROGMEM = "TRACKS.csv";
constexpr static char LABEL_LOG_KM[] PROGMEM = "km";
constexpr static char LABEL_AUTOSEL_TRK[] PROGMEM = "Auto-select track";
constexpr static char LABEL_NO_GPS_SIGNAL[] PROGMEM = "No valid GPS signal !";
constexpr static char LABEL_RUN_AS_DATALOGGER[] PROGMEM = "No track or GPS fix, run as datalogger only";
constexpr static char LABEL_GEAR_CALIBRATION[] PROGMEM = "Set gear to ";
constexpr static char LABEL_GEAR_CALIBRATION_OK[] PROGMEM = "Gears calibrated !";
constexpr static char LABEL_MLX_AUTODETECT_OK[] PROGMEM = "New MLX addresses saved to EEPROM";
constexpr static char LABEL_MLX_AUTODETECT_NOK[] PROGMEM = "Error when saving new MLX addresses to EEPROM";
constexpr static char LABEL_MLX_AUTODETECT[] PROGMEM = "Detecting MLX ";
constexpr static char LABEL_IMPORT_TRACKS_OK[] PROGMEM = "Tracks imported to EEPROM";
constexpr static char LABEL_IMPORT_TRACKS_NOK[] PROGMEM = "Error when importing tracks to EEPROM";
constexpr static char LABEL_NO_TRACK[] PROGMEM = "No track available, please import them from SDCARD";
constexpr static char LABEL_EEPROM_FULL[] PROGMEM = "EEPROM is full, stop importing tracks";
constexpr static char LABEL_TOO_MANY_TRACKS[] PROGMEM = "You try to import to many tracks";
constexpr static char LABEL_READY_TO_GO[] PROGMEM = "Ready to GO !";
constexpr static char LABEL_UNKNOWN_TRACK[] PROGMEM = "UNK_TRK";
constexpr static char LABEL_ENABLE_OBD[] PROGMEM = "Please enable OBD features in options";
constexpr static char LABEL_SWITCH_2_921600[] PROGMEM = "Switching to 921600bps : ";
constexpr static char LABEL_NEW_RATE_921600[] PROGMEM = "New baud rate set 921600bps to default : ";
constexpr static char OBD_STSBR_921600[] PROGMEM = "STSBR 921600";
constexpr static char OBD_STI[] PROGMEM = "STI";
constexpr static char OBD_STN2100[] PROGMEM = "STN2100";
constexpr static char OBD_STWBR[] PROGMEM = "STWBR";

const char *const digitalInputsLabel[] = {"RPM", "SQR1", "DIG1", "DIG2"};
const char *const analogInputsLabel[] = {"GEAR", "ANA2", "ANA3", "ANA4", "ANA5", "ANA6", "ANA7", "ANA8"};
const char gearName[] = {'N', '1', '2', '3', '4', '5', '6'};

/*****************************************************
 * ################################################# *
 * ############## Vars definition ################## *
 * ################################################# *
*****************************************************/

// Inputs
extern uint16_t g_inAnaGearCalib[GEAR_CALIB_SIZE]; // Calibration values for GEAR input, each gear has a analogic value
extern uint8_t g_gearNCheck;                       // Counter to check if gear is set to neutral
extern bool g_enObd;                               // Enable OBD features
extern uint8_t g_enDigitalInputsBits;              // Store enabled digital inputs (use binary values, ex : 10100000 > DIGI_SQR_RPM and DIGI_BRAKE enabled)
// BIT 0 = DIGI_SQR_RPM
// BIT 1 = DIGI_SQR_OPT_1
// BIT 2 = DIGI_BRAKE
// BIT 3 = DIGI_OPT_1
// BIT 4 = N/A
// BIT 5 = N/A
// BIT 6 = N/A
// BIT 7 = N/A
extern uint8_t g_enAnalogInputsBits; // Store enabled digital inputs (use binary values, ex : 11000000 > inAnaOpt1 and inAnaOpt2 enabled)
// BIT 0 = inAnaOpt1
// BIT 1 = inAnaOpt2
// BIT 2 = inAnaOpt3
// BIT 3 = inAnaOpt4
// BIT 4 = inAnaOpt5
// BIT 5 = inAnaOpt6
// BIT 6 = inAnaOpt7
// BIT 7 = inAnaOpt8

// SD Card
extern File logFile;               // One line every 100ms with all detailed data
extern File lapFile;               // One line per lap with laptime
extern char g_workingFilename[64]; // Filename which is actually used

// GPS & Timing
extern int32_t g_lapSec, g_lapCsec; // Lap seconds and centiseconds
extern float g_totalDistance;       // Time To Finish Line
extern int8_t g_isRunning;          // Running status (-1 = error, 0 = not running, 1 = run as datalogger only, 2 = run as datalogger and laptimer)
extern time_t g_date;               // A date

// OBD
extern uint32_t g_rpm; // RPM from OBD
extern uint32_t g_engineCoolantT; // Engine coolant temperature from OBD

// Analog inputs (ADC)
extern uint16_t g_anaValues[8]; // All 8 measured analogic values
extern char g_anaValuesChar[8]; // Analogic value converted to a single character (useful for ANA1/GEAR : N,1,2,3 ...)

// Digital inputs
extern uint32_t g_digValues[4]; // Digital values

// Infrared temp sensor
extern uint8_t g_mlxAddresses[MAX_MLX_SIZE]; // Store each MLX I2C addresses
extern double g_mlxValues[MAX_MLX_SIZE];     // Store MLX values

// RPM
extern uint8_t g_rpmFlywheelTeeth;
extern uint8_t g_rpmCorrectionRatio;

// Buttons & menu
extern bool g_isFakeLap;     // Used to simulate a new lap (first left button when laptimer running)
extern char g_msgLabel[255]; // Info/error messages
extern uint32_t g_msgDelay;  // Info/error messages (print delay)
extern uint8_t g_msgType;    // Info/error messages (type : error, info, ok, ...)

// TFT screen
extern uint32_t g_lastTftTouchSync; // Refresh rate calculation
extern uint8_t g_lastTftPrintSync;  // Refresh rate calculation
extern uint8_t g_tftScreenId;       // Which screen is printed on TFT (0 = start page)

// Tracks
struct Track // Track structure (36 Bytes per track : 2 + 16 + 2 + 4 + 4 + 4 + 4 > TRACK_STRUCT_SIZE)
{
  int16_t trackId;        // Track ID
  char trackName[16];     // Track name
  int16_t trackDistance;  // Track distance from where you are now
  int32_t trackFlineLat1; // Finish line (point A latitude)
  int32_t trackFlineLon1; // Finish line (point A longitude)
  int32_t trackFlineLat2; // Finish line (point B latitude)
  int32_t trackFlineLon2; // Finish line (point B longitude)
};
extern Track g_tracksList[MAX_TRACKS]; // An array of MAX_TRACKS tracks
extern Track g_currentTrack;           // Current track (the nearest one)
extern Track g_nullTrack;              // A null track
extern int16_t g_currentTrackId;       // Current track ID
extern uint8_t g_trackQty;             // Number of defined tracks
extern uint8_t g_trackPage;            // ID of the printed page on tracks screen
extern char g_trackName[16];           // Track name

// Laps
struct Lap
{
  uint32_t lapStartTimestampS; // Timestamp in seconds when strating a new lap (used as a lap ID too)
  uint8_t lapStartTimestampCs; // Hundredth of a second part of the timestamp above /!\ this is an integer so a value of 4 is xxx.04 sec not xxx.4 sec
  uint32_t lapEndTimestampS;   // Timestamp in seconds when ending a new lap
  uint8_t lapEndTimestampCs;   // Hundredth of a second part of the timestamp above /!\ this is an integer so a value of 4 is xxx.04 sec not xxx.4 sec
  uint32_t lapTimeS;           // Total lap time (second part)
  uint8_t lapTimeCs;           // Total lap time (hundredth of a second part) /!\ this is an integer so a value of 4 is xxx.04 sec not xxx.4 sec
  float lapTime;               // Lap time in seconds
  uint8_t lapNumber;           // Lap number
  uint16_t lapMaxSpeed;        // Lap max speed
  int16_t lapTrackId;          // Track ID
  bool lapIsBest;              // Best lap
};
extern Lap g_lapsList[MAX_LAPS]; // An array of MAX_LAPS laps
extern Lap g_currentLap;         // Current lap (running)
extern Lap g_nullLap;            // A null lap
extern uint8_t g_lapQty;         // Number of laps in the array "g_lapsList"
extern uint8_t g_lapPage;        // ID of the printed page on laps screen

/*****************************************************
 * ################################################# *
 * ############## Instantiate objects ############## *
 * ################################################# *
*****************************************************/

// EEPROM
extern extEEPROM eep;

// I/O Expander MCP23017
extern Adafruit_MCP23X17 MCP1;
extern Adafruit_MCP23X17 MCP2;

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
void dateTimeSd(uint16_t *date, uint16_t *time);
boolean segIntersect(int32_t pos_now_lat, int32_t pos_now_lon, int32_t pos_prev_lat, int32_t pos_prev_lon, int32_t trackLat1, int32_t trackLon1, int32_t trackLat2, int32_t trackLon2, int32_t &pos_cross_lat, int32_t &pos_cross_lon);
float gpsDistance(int32_t lat1, int32_t lon1, int32_t lat2, int32_t lon2);
int8_t stopLaptimer(void);
int8_t startLaptimer(void);
void showMessage(const char *label, uint32_t delay, uint8_t type);
void sendUBX(const unsigned char *progmemBytes, size_t len);
void timeToFinishLineCalculation(float timeSecCsec, uint32_t endSec, uint8_t endCsec, uint32_t &returnSec, uint8_t &returnCsec);
void lapTimeCalculation(uint32_t s1, uint8_t cs1, uint32_t s2, uint8_t cs2, uint32_t &returnSec, uint8_t &returnCsec);
void adjustTime(NeoGPS::time_t &dt);
uint8_t initADC(uint8_t bits);
uint8_t configureADC(uint8_t bits);
void showADCPortState(void);
uint16_t readAdcValue(uint8_t registerID);
void readAdcValues(void);
void formatAdcValues(void);
void gearCalibration(void);
void autodetectMlx(void);
void mcp2EnableOneOutput(uint8_t idOutput);
void loadLapPage(uint8_t lapPage, bool lastRun);
uint8_t importTracksFromSd(void);
void loadTracksFromEeprom(void);
void loadSelectedTrack(int16_t trackId);
void getTrackName(int16_t trackId);
int16_t autoselectTrack(void);
void refreshTrackDistance(void);
void updateBestLap(void);

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