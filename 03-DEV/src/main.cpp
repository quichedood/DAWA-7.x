/**************************************************************
  DAWA 7.0 (Arduino M0 - Onboard SAMD21G)
  Bootloader have to be burned on new chip with for example Atmel ICE
  Motocycles laptimer/datalogger
  Edouard PIGEON - 2021
**************************************************************/

/**************************************************************
  TODO
  Un fichier unique pour synthèse chronos
  Lister l'historique des chronos (prédéfinir des affichages)
  LED/Shiftlight (+param)
  Alarmes (paramétrage/affichage)
  Passer trackID sur 8 bits (CSV read) - réduire empreinte EEPROM
  Nommer les inputs
  Optimisation code FT800
  Texte sous forme de constante (verifier tout le code)
  Voir EVE_cmd_dl
  OBD : turned off the adaptive timing "AT0" then reduced the time out "AT ST xx"
**************************************************************/

/*****************************************************
 * ################################################# *
 * ############## Includes ######################### *
 * ################################################# *
*****************************************************/

#include <Arduino.h>           // Arduino library
#include <SPI.h>               // SPI library
#include <Wire.h>              // I2C library
#include <extEEPROM.h>         // EEPROM library (http://github.com/PaoloP74/extEEPROM)
#include <Adafruit_MCP23X17.h> // I/O Expander MCP23017 (https://github.com/adafruit/Adafruit-MCP23017-Arduino-Library)
#include <TimeLib.h>           // Time library (https://github.com/PaulStoffregen/Time)
#include <SdFat.h>             // Greiman/SdFat library (https://github.com/greiman/SdFat)
#include <ELMduino.h>          // ELM327 OBD-II library (https://github.com/PowerBroker2/ELMduino)
#include <Adafruit_MLX90614.h> // MLX90614 Infrared temperature sensor (https://github.com/adafruit/Adafruit-MLX90614-Library)
#include <NMEAGPS.h>           // NeoGPS library (https://github.com/SlashDevin/NeoGPS)
#include <wiring_private.h>    // pinPeripheral() function, redefining SERCOM5 for serial port use
#include <helper.h>            // DAWA functions helper
#include <EVE_commands.h>      // EVE library
#include <tft.h>               // EVE library Rudolph Riedel FT800-FT813 (https://github.com/RudolphRiedel/FT800-FT813)

/**************************************************************
  EEPROM library

  EEPROM addresses used :
  1 RPM Correction Ratio (1x uint8_t)
  2 RPM Flywheel Teeth (1x uint8_t)
  44-56 Gear calibration data (7x int16_t)
  66 Enabled digital inputs (1x uint8_t)
  67 Enabled analog inputs (1x uint8_t)
  68-73 MLX I2C Address (6x uint8_t)
**************************************************************/

/*****************************************************
 * ################################################# *
 * ############## Vars definition ################## *
 * ################################################# *
*****************************************************/

// Inputs
uint16_t g_inAnaGearCalib[GEAR_CALIB_SIZE]; // Calibration values for GEAR input, each gear has a analogic value
uint8_t g_gearNCheck = 0;                   // Counter to check if gear is set to neutral
bool g_enObd;                               // Enable OBD features
uint8_t g_enDigitalInputsBits;              // Store enabled digital inputs (use binary values, ex : 10100000 > DIGI_SQR_RPM and DIGI_BRAKE enabled)
// BIT 0 = DIGI_SQR_RPM
// BIT 1 = DIGI_SQR_OPT_1
// BIT 2 = DIGI_BRAKE
// BIT 3 = DIGI_OPT_1
// BIT 4 = N/A
// BIT 5 = N/A
// BIT 6 = N/A
// BIT 7 = N/A
uint8_t g_enAnalogInputsBits; // Store enabled digital inputs (use binary values, ex : 11000000 > inAnaOpt1 and inAnaOpt2 enabled)
// BIT 0 = inAnaOpt1 (GEAR)
// BIT 1 = inAnaOpt2
// BIT 2 = inAnaOpt3
// BIT 3 = inAnaOpt4
// BIT 4 = inAnaOpt5
// BIT 5 = inAnaOpt6
// BIT 6 = inAnaOpt7
// BIT 7 = inAnaOpt8

// SD Card
File logFile;               // One line every 100ms with all detailed data
File lapFile;               // One line per lap with laptime
char g_workingFilename[64]; // Filename which is actually used

// GPS & Timing
int32_t g_lapSec, g_lapCsec; // Lap seconds and centiseconds
float g_totalDistance;       // Time To Finish Line
int8_t g_isRunning = 0;      // Running status (-1 = error, 0 = not running, 1 = run as datalogger only, 2 = run as datalogger and laptimer)
time_t g_date;               // A date

// OBD
uint32_t g_rpm;                 // RPM from OBD
uint32_t g_engineCoolantT;      // Engine coolant temperature from OBD
uint32_t g_obdTemp;             // Temp value from OBD
uint32_t g_lastObdSlowSync = 0; // OBD slow refresh rate calculation
uint32_t g_lastObdFastSync = 0; // OBD fast refresh rate calculation

// Analog inputs (ADC)
uint16_t g_anaValues[8]; // All 8 measured analogic values
char g_anaValuesChar[8]; // Analogic value converted to a single character (useful for ANA1/GEAR : N,1,2,3 ...)

// Digital inputs
uint32_t g_digValues[4]; // Digital values

// Infrared temperature sensor
uint8_t g_mlxAddresses[MAX_MLX_SIZE]; // Store each MLX I2C addresses
double g_mlxValues[MAX_MLX_SIZE];     // Store MLX values

// RPM
uint8_t g_rpmFlywheelTeeth;   // Number of flywheel teeth (configurable through setup menu)
uint8_t g_rpmCorrectionRatio; // RPM correction ratio (configurable through setup menu)

// Buttons & menu
bool g_isFakeLap = false; // Used to simulate a new lap (debug)
char g_msgLabel[255];     // Used to store on screen popup message
uint32_t g_msgDelay = 0;  // Number of ms the message is printed on screen
uint8_t g_msgType = 0;    // Display type (color : green, red or grey)

// TFT screen
uint32_t g_lastTftTouchSync = 0; // TFT refresh rate calculation
uint8_t g_lastTftPrintSync = 0;  // TFT refresh rate calculation
uint8_t g_tftScreenId = 10;      // Which screen is printed on TFT (0 = start page)

// Tracks
Track g_tracksList[MAX_TRACKS];                        // A array of TRACK_PAGINATION tracks
Track g_currentTrack;                                  // Current track (the nearest one)
Track g_nullTrack = {-1, "NO TRACK!", -1, 0, 0, 0, 0}; // A null track
int16_t g_currentTrackId;                              // Current track ID
uint8_t g_trackQty = 0;                                // Number of defined tracks
uint8_t g_trackPage = 0;                               // ID of the printed page on tracks screen
char g_trackName[16];                                  // Track name

// Laps
Lap g_lapsList[MAX_LAPS];                          // An array of MAX_LAPS laps
Lap g_currentLap;                                  // Current lap (running)
Lap g_nullLap = {0, 0, 0, 0, 0, 0, 0.00, 0, 0, 0}; // A null lap
uint8_t g_lapQty = 0;                              // Number of laps in the array "g_historyLap"
uint8_t g_lapPage = 0;                             // ID of the printed page on laps screen

// Debug data
#ifdef DEBUG
uint32_t g_dbgNow = 0;                     // used to calculate integration interval
uint32_t g_dbgSumCount = 0;                // used to control display output rate
float g_dbgDeltat = 0.0f, g_dbgSum = 0.0f; // integration interval for both filter schemes
uint32_t g_dbgLastUpdate = 0;              // used to calculate integration interval
#endif

/*****************************************************
 * ################################################# *
 * ############## Instantiate objects ############## *
 * ################################################# *
*****************************************************/

// EEPROM
extEEPROM eep(kbits_64, 1, 8); // I2C Address 0x50

// I/O Expander MCP23017
Adafruit_MCP23X17 MCP1;
Adafruit_MCP23X17 MCP2;

// SD Card
SdFat sd;

// MLX infrared temperature sensors
Adafruit_MLX90614 mlx[MAX_MLX_SIZE] = {Adafruit_MLX90614(FIRST_MLX_ADDRESS), Adafruit_MLX90614(FIRST_MLX_ADDRESS + 1), Adafruit_MLX90614(FIRST_MLX_ADDRESS + 2), Adafruit_MLX90614(FIRST_MLX_ADDRESS + 3), Adafruit_MLX90614(FIRST_MLX_ADDRESS + 4), Adafruit_MLX90614(FIRST_MLX_ADDRESS + 5)};

// GPS
NMEAGPS gps;
gps_fix fix_data;
gps_fix fix_data_prev;

// GPS
HardwareSerial &GPS_PORT = Serial5; // Use Serial5 port for GPS

// OBD2 serial port
Uart OBD2_PORT(&sercom5, 7, 6, SERCOM_RX_PAD_3, UART_TX_PAD_2); // Use serial port for OBD2 - Disable SERCOM5 (Serial + Sercom5) in [...]\.platformio\packages\framework-arduino-samd\variants\arduino_mzero\variant.cpp

// OBD2 ELMduino object
ELM327 myELM327;

/*****************************************************
 * ################################################# *
 * ############## Init./Setup ###################### *
 * ################################################# *
*****************************************************/

void setup()
{
  /**************************************************************
    I2C bus init
  **************************************************************/
  Wire.begin();
  Wire.setClock(400000);
  delay(500);

#ifdef DEBUG
  while (!DEBUG_PORT)
    ;
  DEBUG_PORT.println(LABEL_DEBUG_IS_ENABLED);
#endif

  /**************************************************************
    TFT Init
  **************************************************************/
  digitalWrite(EVE_CS, HIGH);
  pinMode(EVE_CS, OUTPUT);
  digitalWrite(EVE_PDN, HIGH);
  pinMode(EVE_PDN, OUTPUT);
  SPI.begin();                         // sets up the SPI to run in Mode 0 and 1 MHz
  SPI.setClockDivider(SPI_CLOCK_DIV2); // speed up SPI
  TFT_init();

  /**************************************************************
    Init I/O pins
  **************************************************************/
  pinMode(inDigiBrakePin, INPUT); // Digital boolean input (brake) (0v/12v)
  pinMode(inDigiOpt1Pin, INPUT);  // Digital boolean input (optional) (0v/12v)
  pinMode(sdCsPin, OUTPUT);       // Chip Select for SDCARD on SPI bus
  pinMode(powerState, OUTPUT);    // A1, power switch state (set 0 to power off)
  digitalWrite(powerState, HIGH);

  /**************************************************************
    I/O Expander 1 (Output = 8 leds)
  **************************************************************/
  if (!MCP1.begin_I2C())
  {
#ifdef DEBUG
    DEBUG_PORT.println(LABEL_MCP1_FAILED);
#endif
  }
  else
  {
#ifdef DEBUG
    DEBUG_PORT.println(LABEL_MCP1_OK);
#endif
    MCP1.pinMode(mcp1Led1, OUTPUT);
    MCP1.pinMode(mcp1Led2, OUTPUT);
    MCP1.pinMode(mcp1Led3, OUTPUT);
    MCP1.pinMode(mcp1Led4, OUTPUT);
    MCP1.pinMode(mcp1Led5, OUTPUT);
    MCP1.pinMode(mcp1Led6, OUTPUT);
    MCP1.pinMode(mcp1Led7, OUTPUT);
    MCP1.pinMode(mcp1Led8, OUTPUT);
    MCP1.digitalWrite(mcp1Led1, HIGH);
    MCP1.digitalWrite(mcp1Led2, HIGH);
    MCP1.digitalWrite(mcp1Led3, HIGH);
    MCP1.digitalWrite(mcp1Led4, HIGH);
    MCP1.digitalWrite(mcp1Led5, HIGH);
    MCP1.digitalWrite(mcp1Led6, HIGH);
    MCP1.digitalWrite(mcp1Led7, HIGH);
    MCP1.digitalWrite(mcp1Led8, HIGH);
  }

  /**************************************************************
    I/O Expander 2 (Output = 5v power for 6 devices like MLX sensors plugged on hub)
  **************************************************************/
  if (!MCP2.begin_I2C(0x24))
  {
#ifdef DEBUG
    DEBUG_PORT.println(LABEL_MCP2_FAILED);
#endif
  }
  else
  {
#ifdef DEBUG
    DEBUG_PORT.println(LABEL_MCP2_OK);
#endif
    MCP2.pinMode(0, OUTPUT);
    MCP2.pinMode(1, OUTPUT);
    MCP2.pinMode(2, OUTPUT);
    MCP2.pinMode(3, OUTPUT);
    MCP2.pinMode(4, OUTPUT);
    MCP2.pinMode(5, OUTPUT);
    MCP2.pinMode(6, OUTPUT);
    MCP2.pinMode(7, OUTPUT);
    MCP2.digitalWrite(0, HIGH);
    MCP2.digitalWrite(1, HIGH);
    MCP2.digitalWrite(2, HIGH);
    MCP2.digitalWrite(3, HIGH);
    MCP2.digitalWrite(4, HIGH);
    MCP2.digitalWrite(5, HIGH);
    MCP2.digitalWrite(6, HIGH);
    MCP2.digitalWrite(7, HIGH);
  }

  /**************************************************************
    OBD2 Init (Serial2)
  **************************************************************/
  // Trying to establish connection to OBD @921600bps (STN2100 default is 9600bps)
  OBD2_PORT.begin(921600);
  pinPeripheral(6, PIO_SERCOM);   // Pin D6 for RX
  pinPeripheral(7, PIO_SERCOM);   // Pin D7 for TX
  if (!myELM327.begin(OBD2_PORT)) // To DEBUG : if (!myELM327.begin(OBD2_PORT, true, 1000))
  {
    // @921600bps not working, probably the first time you run this program on new hardware
    // Trying @9600bps which is the default speed
    OBD2_PORT.begin(9600);
    pinPeripheral(6, PIO_SERCOM); // Pin D6 for RX
    pinPeripheral(7, PIO_SERCOM); // Pin D7 for TX
    if (myELM327.begin(OBD2_PORT))
    {
      // OBD is OK @9600bps, trying to switch to 921600bps
      myELM327.sendCommand_Blocking(OBD_STSBR_921600);
#ifdef DEBUG
      DEBUG_PORT.print(LABEL_SWITCH_2_921600);
      DEBUG_PORT.println(myELM327.payload);
#endif
      delay(75);
      OBD2_PORT.begin(921600);
      pinPeripheral(6, PIO_SERCOM); // Pin D6 for RX
      pinPeripheral(7, PIO_SERCOM); // Pin D7 for TX

      // Save new baud rate to default (NV memory) only if serial communication is successfull
      myELM327.sendCommand_Blocking(OBD_STI);
      if (strstr(myELM327.payload, OBD_STN2100) != NULL)
      {
        myELM327.sendCommand_Blocking(OBD_STWBR);
#ifdef DEBUG
        DEBUG_PORT.print(LABEL_NEW_RATE_921600);
        DEBUG_PORT.println(myELM327.payload);
#endif
      }
    }
  }

  // Check if OBD chip is responding correctly at specified baud rate
  myELM327.sendCommand_Blocking(OBD_STI);
  if (strstr(myELM327.payload, OBD_STN2100) != NULL)
  {
#ifdef DEBUG
    DEBUG_PORT.print(LABEL_OBD_OK);
    DEBUG_PORT.print(F(" ("));
    DEBUG_PORT.print(myELM327.payload);
    DEBUG_PORT.println(F(")"));
#endif
    MCP1.digitalWrite(mcp1Led1, LOW); // Power on led1, power off after successfull init
  }
  else
  {
#ifdef DEBUG
    DEBUG_PORT.print(LABEL_OBD_FAILED);
#endif
    initError(128);
  }

  /**************************************************************
    SD card Init (SPI)
  **************************************************************/
  if (!sd.begin(sdCsPin, SD_SCK_MHZ(12)))
  {
#ifdef DEBUG
    DEBUG_PORT.println(LABEL_SD_FAILED);
#endif
    initError(64);
  }
  else
  {
#ifdef DEBUG
    DEBUG_PORT.println(LABEL_SD_OK);
#endif
    MCP1.digitalWrite(mcp1Led2, LOW); // Power on led2, power off after successfull init
  }

  /**************************************************************
    EEPROM Init (I2C)
  **************************************************************/
  if (!eep.begin(eep.twiClock400kHz) == 0)
  {
#ifdef DEBUG
    DEBUG_PORT.println(LABEL_EEPROM_FAILED);
#endif
    initError(32);
  }
  else
  {
#ifdef DEBUG
    DEBUG_PORT.println(LABEL_EEPROM_OK);
#endif
    eepromReload();
    MCP1.digitalWrite(mcp1Led3, LOW); // Power on led3, power off after successfull init
  }

  /**************************************************************
    MLX Init (I2C)
  **************************************************************/
  // Read infrared temp sensors I2C Address from EEPROM (MAX_MLX_SIZE x 8 bits)
  EEPROM_readAnything(EEPROM_MLX_ADDR, g_mlxAddresses) == sizeof(g_mlxAddresses);
  for (uint8_t i = 0; i < MAX_MLX_SIZE; i++)
  {
    if (g_mlxAddresses[i] != 0x00)
    {
      mlx[i].begin();
    }
  }

  /**************************************************************
    ADC Init
  **************************************************************/
  if (!initADC(g_enAnalogInputsBits) == 0)
  {
#ifdef DEBUG
    DEBUG_PORT.println(LABEL_ADC_FAILED);
#endif
    initError(16);
  }
  else
  {
#ifdef DEBUG
    DEBUG_PORT.println(LABEL_ADC_OK);
#endif
    MCP1.digitalWrite(mcp1Led4, LOW); // Power on led4, power off after successfull init
  }

  /**************************************************************
    GPS Init (Serial5 - Default on Arduino m0)
  **************************************************************/
  GPS_PORT.begin(9600);                    // Start the UART @9600bps for the GPS device (default speed)
  sendUBX(ubxPrtConf, sizeof(ubxPrtConf)); // Set UART speed to 115200bps (Warning : @9600bps > ~5sec delay on GPS data)
  delay(100);
  GPS_PORT.end();
  GPS_PORT.begin(115200);                        // Start the UART @115200bps
  sendUBX(ubxRate10Hz, sizeof(ubxRate10Hz));     // Set refresh rate to 10Hz
  sendUBX(ubxDisableGLL, sizeof(ubxDisableGLL)); // Disable unused frames
  sendUBX(ubxDisableGSA, sizeof(ubxDisableGSA));
  sendUBX(ubxDisableGSV, sizeof(ubxDisableGSV));
  sendUBX(ubxDisableVTG, sizeof(ubxDisableVTG));
  sendUBX(ubxDisableZDA, sizeof(ubxDisableZDA));

#ifdef DEBUG
  DEBUG_PORT.println(LABEL_GPS_OK);
#endif
  MCP1.digitalWrite(mcp1Led5, LOW); // Power on led5, power off after successfull init

  /**************************************************************
    2 counters (TCC0 & TCC1)
    for RPM values and another optionnal square signal

    m0 pinout : https://github.com/arduino/ArduinoCore-samd/blob/master/variants/arduino_mzero/variant.cpp
    Big thanks to : https://forum.arduino.cc/index.php?topic=396804.45
  **************************************************************/
  REG_PM_APBCMASK |= PM_APBCMASK_EVSYS; // Switch on the event system peripheral
  PM->APBCMASK.reg |= PM_APBCMASK_TCC0; // Enable TCC0 Bus clock (Timer counter control clock)
  PM->APBCMASK.reg |= PM_APBCMASK_TCC1; // Enable TCC1 Bus clock (Timer counter control clock)

  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |     // Enable the generic clock...
                     GCLK_CLKCTRL_GEN_GCLK0 | // .... on GCLK0...
                     GCLK_CLKCTRL_ID_EIC;     // ... to feed the GCLK0 to EIC peripheral
  while (GCLK->STATUS.bit.SYNCBUSY)
    ; // Wait for synchronization

  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |       // Enable the generic clock...
                     GCLK_CLKCTRL_GEN_GCLK0 |   // ....on GCLK0...
                     GCLK_CLKCTRL_ID_TCC0_TCC1; // ... to feed the GCLK5 to TCC0 and TCC1 peripheral
  while (GCLK->STATUS.bit.SYNCBUSY)
    ; // Wait for synchronization

  PORT->Group[PORTA].PMUX[19 >> 1].reg |= PORT_PMUX_PMUXO_A; // Connect PA19 (pin 12 on m0) to peripheral A (EXTINT[3])
  PORT->Group[PORTA].PINCFG[19].reg |= PORT_PINCFG_PMUXEN;   // Enable pin peripheral multiplexation

  PORT->Group[PORTA].PMUX[18 >> 1].reg |= PORT_PMUX_PMUXO_A; // Connect PA18 (pin 10 on m0) to peripheral A (EXTINT[3])
  PORT->Group[PORTA].PINCFG[18].reg |= PORT_PINCFG_PMUXEN;   // Enable pin peripheral multiplexation

  REG_EIC_EVCTRL |= EIC_EVCTRL_EXTINTEO3;    // Enable event from pin on external interrupt 3 (EXTINT03)
  REG_EIC_CONFIG0 |= EIC_CONFIG_SENSE3_RISE; // Set event on rising edge of signal

  REG_EIC_EVCTRL |= EIC_EVCTRL_EXTINTEO2;    // Enable event from pin on external interrupt 2 (EXTINT02)
  REG_EIC_CONFIG0 |= EIC_CONFIG_SENSE2_RISE; // Set event on rising edge of signal

  REG_EIC_CTRL |= EIC_CTRL_ENABLE; // Enable EIC peripheral
  while (EIC->STATUS.bit.SYNCBUSY)
    ; // Wait for synchronization

  // EVSYS Configuration
  REG_EVSYS_USER = EVSYS_USER_CHANNEL(1) |                   // Attach the event user (receiver) to channel n=0 (n + 1)
                   EVSYS_USER_USER(EVSYS_ID_USER_TCC0_EV_0); // Set the event user (receiver) as timer TCC0, event 1

  REG_EVSYS_USER = EVSYS_USER_CHANNEL(2) |                   // Attach the event user (receiver) to channel n=0 (n + 1)
                   EVSYS_USER_USER(EVSYS_ID_USER_TCC1_EV_0); // Set the event user (receiver) as timer TCC0, event 1

  REG_EVSYS_CHANNEL = EVSYS_CHANNEL_EDGSEL_NO_EVT_OUTPUT |             // No event output edge detection
                      EVSYS_CHANNEL_PATH_ASYNCHRONOUS |                // Set event path as asynchronous
                      EVSYS_CHANNEL_EVGEN(EVSYS_ID_GEN_EIC_EXTINT_3) | // Set event generator (sender) as external interrupt 3
                      EVSYS_CHANNEL_CHANNEL(0);                        // Attach the generator (sender) to channel 0

  REG_EVSYS_CHANNEL = EVSYS_CHANNEL_EDGSEL_NO_EVT_OUTPUT |             // No event output edge detection
                      EVSYS_CHANNEL_PATH_ASYNCHRONOUS |                // Set event path as asynchronous
                      EVSYS_CHANNEL_EVGEN(EVSYS_ID_GEN_EIC_EXTINT_2) | // Set event generator (sender) as external interrupt 3
                      EVSYS_CHANNEL_CHANNEL(1);                        // Attach the generator (sender) to channel 0

  // TCC0 & TCC1 Configuration
  REG_TCC0_CTRLA &= ~TCC_CTRLA_ENABLE; // Disable TCC0 peripheral
  REG_TCC1_CTRLA &= ~TCC_CTRLA_ENABLE; // Disable TCC1 peripheral

  REG_TCC0_CTRLBCLR |= TCC_CTRLBCLR_DIR; // Clear DIR bit to count up
  while (TCC0->SYNCBUSY.bit.CTRLB)
    ;                                    // Wait for (write) synchronization
  REG_TCC1_CTRLBCLR |= TCC_CTRLBCLR_DIR; // Clear DIR bit to count up
  while (TCC1->SYNCBUSY.bit.CTRLB)
    ; // Wait for (write) synchronization

  REG_TCC0_EVCTRL |= TCC_EVCTRL_TCEI0 |       // Enable the TCC event 0 input
                     TCC_EVCTRL_EVACT0_COUNT; // Set up TCC timer/counter to count on event

  REG_TCC0_CTRLA |= TCC_CTRLA_ENABLE; // Enable TCC0
  while (TCC0->SYNCBUSY.bit.ENABLE)
    ; // Wait for synchronization

  REG_TCC1_EVCTRL |= TCC_EVCTRL_TCEI0 |       // Enable the TCC event 0 input
                     TCC_EVCTRL_EVACT0_COUNT; // Set up TCC timer/counter to count on event

  REG_TCC1_CTRLA |= TCC_CTRLA_ENABLE; // Enable TCC0
  while (TCC1->SYNCBUSY.bit.ENABLE)
    ; // Wait for synchronization

  /**************************************************************
    Power off leds as soon as init is done
  **************************************************************/
  MCP1.digitalWrite(mcp1Led1, HIGH);
  MCP1.digitalWrite(mcp1Led2, HIGH);
  MCP1.digitalWrite(mcp1Led3, HIGH);
  MCP1.digitalWrite(mcp1Led4, HIGH);
  MCP1.digitalWrite(mcp1Led5, HIGH);
  MCP1.digitalWrite(mcp1Led6, HIGH);
  MCP1.digitalWrite(mcp1Led7, HIGH);
  MCP1.digitalWrite(mcp1Led8, HIGH);
#ifdef DEBUG
  DEBUG_PORT.println(LABEL_READY);
#endif
}

/*****************************************************
 * ################################################# *
 * ############## Main loop ######################## *
 * ################################################# *
*****************************************************/
void loop()
{
  static uint8_t tmpComp, bitShift; // Used to compare values for enabled inputs checks
  static bool addFinishLog = false;
  static float tToFl;
  static uint32_t lastPinRead, lastOneSecSync, lastSdSync, fixCount, elapsedTime;
  static int32_t posCrossLat, posCrossLon;
  typedef enum
  {
    IDLE,
    COOLANT_TEMP,
    ENG_RPM
  } obd_pid_states;
  static obd_pid_states obd_state = IDLE;

#ifdef DEBUG
  // Here we calculate average update rate of main loop
  g_dbgNow = micros();
  g_dbgDeltat = ((g_dbgNow - g_dbgLastUpdate) / 1000000.0f);
  g_dbgLastUpdate = g_dbgNow;
  g_dbgSum += g_dbgDeltat;
  g_dbgSumCount++;
#endif

  /**************************************************************
    Get GPS frames through serial port (RX/TX)
    This is CRITICAL, data could be sent at any moment by the GPS so main loop should be executed in a minimum of time
    More information : https://github.com/SlashDevin/NeoGPS/blob/master/extras/doc/Troubleshooting.md#quiet-time-interval
  **************************************************************/
  if (gps.available(GPS_PORT))
  {
    /**************************************************************
      Read last available GPS data
    **************************************************************/
    fix_data_prev = fix_data; // Memorize previous values before next GPS fix
    fix_data = gps.read();    // Get GPS data
    adjustTime(fix_data.dateTime);
    fixCount++;

    /**************************************************************
      Read all 4 digital enabled INPUTS
    **************************************************************/
    elapsedTime = millis() - lastPinRead; // Used to have precise measures on RPM and SPEED
    lastPinRead = millis();               // Reset last pin read

    bitShift = 0b00000001;
    for (uint8_t i = 0; i < 4; i++)
    { // Read values of enabled inputs only
      tmpComp = bitShift & g_enDigitalInputsBits;
      if (tmpComp == bitShift)
      {
        switch (i)
        {
        // bit 0 : RPM;
        case 0:
          REG_TCC1_CTRLBSET = TCC_CTRLBSET_CMD_READSYNC; // Trigger a read synchronization on the COUNT register
          while (TCC1->SYNCBUSY.bit.CTRLB)
            ; // Wait for the CTRLB register write synchronization
          while (TCC1->SYNCBUSY.bit.COUNT)
            ;                              // Wait for the COUNT register read sychronization
          g_digValues[i] = REG_TCC1_COUNT; // Read TCNT1 register (timer1 counter)
          REG_TCC1_COUNT = 0x0000;         // Clear timer's COUNT value
          while (TCC1->SYNCBUSY.bit.COUNT)
            ;                                                                                              // Wait for synchronization
          g_digValues[i] = g_digValues[i] * g_rpmCorrectionRatio * 600 / elapsedTime / g_rpmFlywheelTeeth; // Ratio between pulse and rpm (22 > flywheel has 22 teeth ### 600 > with check every 100ms, RPM is by minute) ### g_rpmCorrectionRatio > *(100+corr) / elapsedTime) > if we read counter @101ms or 102ms values should be adjusted
          break;
        // bit 1 : SQR1;
        case 1:
          REG_TCC0_CTRLBSET = TCC_CTRLBSET_CMD_READSYNC; // Trigger a read synchronization on the COUNT register
          while (TCC0->SYNCBUSY.bit.CTRLB)
            ; // Wait for the CTRLB register write synchronization
          while (TCC0->SYNCBUSY.bit.COUNT)
            ;                              // Wait for the COUNT register read sychronization
          g_digValues[i] = REG_TCC0_COUNT; // Read TCNT0 register (timer0 counter)
          REG_TCC0_COUNT = 0x0000;         // Clear timer's COUNT value
          while (TCC0->SYNCBUSY.bit.COUNT)
            ;                                                  // Wait for synchronization
          g_digValues[i] = g_digValues[i] * 100 / elapsedTime; // Ratio is set to 1 so no maths ### (* 100 / elapsedTime) > if we read counter @99ms, 101ms or 102ms values should be adjusted
          break;
        // bit 2 : DIG1;
        case 2:
          g_digValues[i] = digitalRead(inDigiBrakePin); // Read "inDigiBrakePin" (pin is plugged on the "+" of the stop light)
          break;
        // bit 3 : DIG2;
        case 3:
          g_digValues[i] = digitalRead(inDigiOpt1Pin); // Read "inDigiOpt1Pin"
          break;
        }
      }
      bitShift = bitShift << 1;
    }

    /**************************************************************
      Read analog enabled INPUTS
    **************************************************************/
    readAdcValues(); // It takes time to read all values (one voltage conversion = 12.2ms !)
    formatAdcValues();

    /**************************************************************
      Sync files on SDcard every 300 fixes (300x100ms = 30sec) to avoid dataloss on power failure
    **************************************************************/
    if (g_isRunning >= 1 && (fixCount - lastSdSync >= 300))
    {
      lastSdSync = fixCount;
      SdFile::dateTimeCallback(dateTimeSd);
      logFile.sync();
      if (g_isRunning == 2)
      {
        lapFile.sync();
      }
    }

    /**************************************************************
      1 second loop (every 10 fixes > 10x100ms = 1sec), could be used for :
      - Get values with a low refresh rate like ambiant temperature
      - Printing debug information
      - ...
    **************************************************************/
    if (fixCount - lastOneSecSync >= 10)
    {
      lastOneSecSync = fixCount;
#ifdef DEBUG
      DEBUG_PORT.print("rate = ");
      DEBUG_PORT.print((float)g_dbgSumCount / g_dbgSum, 0);
      DEBUG_PORT.println(" Hz");
      g_dbgSumCount = 0;
      g_dbgSum = 0;
#endif

      /**************************************************************
        Read and store MLX temperature in array
      **************************************************************/
      for (uint8_t i = 0; i < MAX_MLX_SIZE; i++)
      {
        if (g_mlxAddresses[i] != 0x00)
        {
          g_mlxValues[i] = mlx[i].readObjectTempC();
        }
      }
    }

    /**************************************************************
      If laptimer is running :
      - Calculate distances, laptime with GPS data
      - Log data on SDcard

      If laptimer is NOT running :
      - N/A
    **************************************************************/
    if (g_isRunning >= 1)
    {
      if (g_isRunning == 2) // Laptimer and datalogger
      {
        // Calculate total distance (for SeriousRacing compatibility)
        g_totalDistance += gpsDistance(fix_data_prev.latitudeL(), fix_data_prev.longitudeL(), fix_data.latitudeL(), fix_data.longitudeL());

        // Check if we pass the finishline (2x2 coordinates for finish line points + 2x2 coordinates for last position + position now)
        if (segIntersect(fix_data.latitudeL(), fix_data.longitudeL(), fix_data_prev.latitudeL(), fix_data_prev.longitudeL(), g_currentTrack.trackFlineLat1, g_currentTrack.trackFlineLon1, g_currentTrack.trackFlineLat2, g_currentTrack.trackFlineLon2, posCrossLat, posCrossLon) || g_isFakeLap == true)
        {
          // Debug button to simulate finish line crossing
          if (g_isFakeLap == true)
          {
            g_isFakeLap = false;
            tToFl = random(0, 9) / 100.00; // Random value to simulate Time To Finish Line (TimeToFinishLine is always < 0.10 sec as there is a GPS fix every 0.10 sec !)
          }
          else
          {
            // Calculate Time To Finish Line (from last know position by GPS just before crossing the finish line / format : sss.ms) ### tToFl = (distance between previous position and finish line (Ex : 0.00112km) / distance between previous position and actual position (Ex : 0.00254km)) * (time of actual fix - time of previous fix)
            tToFl = (gpsDistance(fix_data_prev.latitudeL(), fix_data_prev.longitudeL(), posCrossLat, posCrossLon) / gpsDistance(fix_data.latitudeL(), fix_data.longitudeL(), fix_data_prev.latitudeL(), fix_data_prev.longitudeL())) * ((fix_data.dateTime.hours * 3600 + fix_data.dateTime.minutes * 60 + fix_data.dateTime.seconds + fix_data.dateTime_cs / 100.00) - (fix_data_prev.dateTime.hours * 3600 + fix_data_prev.dateTime.minutes * 60 + fix_data_prev.dateTime.seconds + fix_data_prev.dateTime_cs / 100.00));
          }
          // Add "Time to finish line (tToFl)" to the last known Epoch Time (fix_data_prev)
          timeToFinishLineCalculation(tToFl, fix_data_prev.dateTime, fix_data_prev.dateTime_cs, g_currentLap.lapEndTimestampS, g_currentLap.lapEndTimestampCs);

          // Get first laptime at the end of the lap 1 (g_lapQty = 1 / We start from the paddocks)
          if (g_lapQty > 0)
          {
            // Calculate total laptime (substract previous finish laptime to actual laptime)
            lapTimeCalculation(g_currentLap.lapEndTimestampS, g_currentLap.lapEndTimestampCs, g_currentLap.lapStartTimestampS, g_currentLap.lapStartTimestampCs, g_currentLap.lapTimeS, g_currentLap.lapTimeCs);

            // Complete the "lapTime" value (float value in sss.cs ... easier to compare laptimes)
            g_currentLap.lapTime = g_currentLap.lapTimeS + (g_currentLap.lapTimeCs / 100.00);
          }

          // Copy current lap (ended) to history laps array
          g_lapsList[g_lapQty] = g_currentLap;

          // Update best lap
          updateBestLap();

          // Start a new lap with end values of the last lap
          g_currentLap.lapStartTimestampS = g_lapsList[g_lapQty].lapEndTimestampS;
          g_currentLap.lapStartTimestampCs = g_lapsList[g_lapQty].lapEndTimestampCs;
          g_currentLap.lapEndTimestampS = fix_data.dateTime;
          g_currentLap.lapEndTimestampCs = fix_data.dateTime_cs;
          g_currentLap.lapTimeS = 0;
          g_currentLap.lapTimeCs = 0;
          g_currentLap.lapTime = 0.00;
          g_currentLap.lapNumber = g_lapsList[g_lapQty].lapNumber + 1;
          g_currentLap.lapMaxSpeed = 0;
          g_currentLap.lapTrackId = g_lapsList[g_lapQty].lapTrackId;
          g_currentLap.lapIsBest = false;

          // This is the end of a lap, write special line in the log file
          addFinishLog = true;
        }
        else
        {
          // Continuously update lap time (end time + total time) for diplaying realtime values on TFT (and only for this !)
          if (g_lapQty > 0)
          {
            g_currentLap.lapEndTimestampS = fix_data.dateTime;
            g_currentLap.lapEndTimestampCs = fix_data.dateTime_cs;
            lapTimeCalculation(g_currentLap.lapEndTimestampS, g_currentLap.lapEndTimestampCs, g_currentLap.lapStartTimestampS, g_currentLap.lapStartTimestampCs, g_currentLap.lapTimeS, g_currentLap.lapTimeCs);

            // Update max speed if needed
            if (fix_data.speed_kph() > g_currentLap.lapMaxSpeed)
            {
              g_currentLap.lapMaxSpeed = fix_data.speed_kph();
            }
          }

          // This is not the end of a lap, no need to write special line in the log file
          addFinishLog = false;
        }
      }

      /**************************************************************
        Write all data to file on SD card (GPS, inAnaThrottle, gear, rpm, temperature sensors)
      **************************************************************/
      // Time, distance and lap (always printed)
      if (addFinishLog == true)
      {
        if (g_lapQty > 0)
        {
          // Add lap time to the laptime file
          lapFile.print(g_lapsList[g_lapQty].lapStartTimestampS);
          lapFile.print(F(";"));
          lapFile.print(g_workingFilename);
          lapFile.print(F(";"));
          lapFile.print(g_lapsList[g_lapQty].lapTrackId);
          lapFile.print(F(";"));
          lapFile.print(uint8_t(g_lapsList[g_lapQty].lapTimeS / 60)); // Store laptime mm:sss:ms (human readable)
          lapFile.print(F(":"));
          if (g_lapsList[g_lapQty].lapTimeS % 60 < 10)
            lapFile.print(F("0")); // Leading zeros (remember "g_lapsList[g_lapQty].lapTimeS" is an integer !!)
          lapFile.print(g_lapsList[g_lapQty].lapTimeS % 60);
          lapFile.print(F(":"));
          if (g_lapsList[g_lapQty].lapTimeCs < 10)
            lapFile.print(F("0")); // Leading zeros (remember "g_lapsList[g_lapQty].lapTimeCs" is an integer !!)
          lapFile.print(g_lapsList[g_lapQty].lapTimeCs);
          lapFile.print(F(";")); // Store laptime sss.ms (enable float comparaison for best lap or other calculations)
          lapFile.print(g_lapsList[g_lapQty].lapTimeS);
          lapFile.print(F("."));
          if (g_lapsList[g_lapQty].lapTimeCs < 10)
            lapFile.print(F("0")); // Leading zeros (remember "g_lapsList[g_lapQty].lapTimeCs" is an integer !!)
          lapFile.println(g_lapsList[g_lapQty].lapTimeCs);
        }

        // Start building the special line in the log file (end of a lap) ...
        logFile.print(g_lapsList[g_lapQty].lapEndTimestampS);
        logFile.print(F("."));
        if (g_lapsList[g_lapQty].lapEndTimestampCs < 10)
          logFile.print(F("0")); // Leading zeros (remember "timeCsec" is an integer !!)
        logFile.print(g_lapsList[g_lapQty].lapEndTimestampCs);

        // Automatically stop laptimer if MAX_LAPS is reached
        if (g_lapQty >= MAX_LAPS - 1)
        {
          g_isRunning = stopLaptimer();
          g_tftScreenId = 150;
        }
        else
        {
          // Lap number is incremented
          g_lapQty++;
        }
      }
      else
      {
        // Start building a normal line in the log file (NOT the end of a lap) ...
        logFile.print(fix_data.dateTime);
        logFile.print(F("."));
        if (fix_data.dateTime_cs < 10)
          logFile.print(F("0")); // Leading zeros (remember "fix_data.dateTime_cs" is an integer !!)
        logFile.print(fix_data.dateTime_cs);
      }

      // ... common data in the log file (end of lap or not)
      logFile.print(F(";"));
      logFile.print(g_totalDistance, 3);
      logFile.print(F(";"));
      logFile.print(g_lapQty);
      logFile.print(F(";"));

      // KPH, heading (always printed)
      logFile.print(fix_data.speed_kph(), 0);
      logFile.print(F(";"));
      logFile.print(fix_data.heading(), 1);
      logFile.print(F(";"));

      // Digital inputs (printed if enabled)
      bitShift = 0b00000001;
      for (uint8_t i = 0; i < 4; i++)
      {
        tmpComp = bitShift & g_enDigitalInputsBits;
        if (tmpComp == bitShift)
        {
          logFile.print(g_digValues[i]);
          logFile.print(F(";"));
        }
        bitShift = bitShift << 1;
      }

      // Analog inputs (printed if enabled)
      bitShift = 0b00000001;
      for (uint8_t i = 0; i < 8; i++)
      {
        tmpComp = bitShift & g_enAnalogInputsBits;
        if (tmpComp == bitShift)
        {
          if (i == 0) // ANA1/GEAR
          {
            logFile.print(g_anaValuesChar[i]);
          }
          else
          {
            logFile.print(g_anaValues[i]);
          }
          logFile.print(F(";"));
        }
        bitShift = bitShift << 1;
      }

      // Infrared temperature (printed if enabled)
      for (uint8_t i = 0; i < MAX_MLX_SIZE; i++)
      {
        if (g_mlxAddresses[i] != 0x00)
        {
          logFile.print(g_mlxValues[i]);
          logFile.print(F(";"));
        }
      }

      // Latitude & longitude (always printed)
      if (addFinishLog == true)
      {
        logFile.print((posCrossLat / rescaleGPS), 9);
        logFile.print(F(";"));
        logFile.println((posCrossLon / rescaleGPS), 9);
      }
      else
      {
        logFile.print(fix_data.latitude(), 9);
        logFile.print(F(";"));
        logFile.println(fix_data.longitude(), 9);
      }

      /**************************************************************
        Do something else when laptimer IS running
      **************************************************************/
      // Anything that is added here doesn't have to be time consumming as we'll loose some GPS frames and screw everything up !
    }
    else
    { // g_isRunning == false
      /**************************************************************
        Do something else when laptimer IS NOT running
      **************************************************************/
      // Do what you want :)
      // You could check the refresh rate of the main loop when enabling debug mode (now it's approximately : 38000Hz)
    }
  }

  /**************************************************************
    Refresh TFT screen
  **************************************************************/
  if ((millis() - g_lastTftTouchSync) >= 5) // execute the code only every 5 ms
  {
    g_lastTftTouchSync = millis();
    TFT_touch();
    g_lastTftPrintSync++;
    if (g_lastTftPrintSync >= 4) // refresh the display every 20ms
    {
      g_lastTftPrintSync = 0;
      TFT_display();
    }
  }

  /**************************************************************
    Read OBD
  **************************************************************/
  /*g_currentMs = millis();
  if (g_enObd == true)
  {
    if ((g_currentMs - g_lastObdFastSync) >= 15) // execute the code only every 15 ms
    {
      g_lastObdFastSync = g_currentMs;
      switch (obd_state)
      {
      case ENG_RPM:
        g_obdTemp = (uint32_t)myELM327.rpm();
        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
          g_rpm = g_obdTemp;
          obd_state = COOLANT_TEMP;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
          //myELM327.printError();
          obd_state = COOLANT_TEMP;
        }
        break;
      case COOLANT_TEMP:
        g_obdTemp = (uint32_t)myELM327.engineCoolantTemp();
        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
          g_engineCoolantT = g_obdTemp;
          obd_state = ENG_RPM;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
          //myELM327.printError();
          obd_state = ENG_RPM;
        }
        break;
      }
    }
  }*/

  // g_currentMs = millis();
  if (g_enObd == true)
  {
    switch (obd_state)
    {
    case IDLE:
      if (millis() > g_lastObdSlowSync + 500)
      {
        g_lastObdSlowSync = millis();
        obd_state = COOLANT_TEMP;
#ifdef DEBUG
        DEBUG_PORT.println("check COOLANT");
#endif
      }
      else if (millis() > g_lastObdFastSync + 100)
      {
        g_lastObdFastSync = millis();
        obd_state = ENG_RPM;
#ifdef DEBUG
        DEBUG_PORT.println("check RPM");
#endif
      }
      break;
    case ENG_RPM:
      g_obdTemp = (uint32_t)myELM327.rpm();
      if (myELM327.nb_rx_state == ELM_SUCCESS)
      {
        g_rpm = g_obdTemp;
        obd_state = IDLE; // Or any other fast OBD sync
      }
      else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
      {
        //myELM327.printError();
        obd_state = IDLE; // Or any other fast OBD sync
      }
      break;
    case COOLANT_TEMP:
      g_obdTemp = (uint32_t)myELM327.engineCoolantTemp();
      if (myELM327.nb_rx_state == ELM_SUCCESS)
      {
        g_engineCoolantT = g_obdTemp;
#ifdef DEBUG
        DEBUG_PORT.println(g_engineCoolantT);
#endif
        obd_state = IDLE; // Or any other slow OBD sync
      }
      else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
      {
        //myELM327.printError();
        obd_state = IDLE; // Or any other slow OBD sync
      }
      break;
    }
  }
}