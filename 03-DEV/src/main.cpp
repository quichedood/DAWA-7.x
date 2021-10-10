/**************************************************************
  DAWA 7.0 (Arduino M0 - Onboard SAMD21G)
  Bootloader have to be burned on new chip with for example Atmel ICE
  Triumph motorbikes laptimer/datalogger
  Edouard PIGEON - 2020
**************************************************************/

/**************************************************************
  TODO
  OK - Tester vitesse
  OK - Tester RPM
  OK - Tester MLX
  OK - Tester valeurs sur carte SD
  OK - Régler variables vitesse
  Pouvoir choisir Vgps ou Vcapteur
  Calibration gear
  Supprimer min/max analog inputs ?
  Affichage running
  Lister les circuits sur la carte SD
  Lister l'historique des chronos
  Ajouter le nom du circuit aux noms de fichiers de log
  Graphisme RPM
  Nommer les inputs
  Teste sous forme de constante (verifier tout le code)

**************************************************************/

/**************************************************************
  #################################################
  ############## Includes definition ##############
  #################################################
**************************************************************/

#include <Arduino.h>           // Arduino library
#include <SPI.h>               // SPI library
#include <Wire.h>              // I2C library
#include <extEEPROM.h>         // EEPROM library (http://github.com/PaoloP74/extEEPROM)
#include <Adafruit_MCP23017.h> // I/O Expander MCP23017 (https://github.com/adafruit/Adafruit-MCP23017-Arduino-Library)
#include <Time.h>              // Time library
#include <TimeLib.h>           // Time library (https://github.com/PaulStoffregen/Time)
                               // In "TimeLib.h" > rename var "DAYS_PER_WEEK" to "DAYS_PER_WEEK_ALT" as the same var is already used in NeoGPS library
#include <SdFat.h>             // Greiman/SdFat library (https://github.com/greiman/SdFat)
#include <ELMduino.h>          // ELM327 OBD-II library (https://github.com/PowerBroker2/ELMduino)
#include <Adafruit_MLX90614.h> // MLX90614 Infrared temperature sensor (https://github.com/jfitter/MLX90614)
#include <dtostrf.h>           // Modified dtostrf function defnition (http://forum.arduino.cc/index.php?topic=368720.0)
#include <NMEAGPS.h>           // NeoGPS library (https://github.com/SlashDevin/NeoGPS)
                               // In "NeoTime.h" > static const uint16_t s_epoch_year = POSIX_EPOCH_YEAR; static const uint8_t  s_epoch_weekday = POSIX_EPOCH_WEEKDAY;
                               // In NMEAGPS_cfg.h > uncomment #define NMEAGPS_PARSE_PROPRIETARY and #define NMEAGPS_DERIVED_TYPES
#include <wiring_private.h>    // pinPeripheral() function, redefining SERCOM5 for serial port use
#include <helper.h>            // DAWA functions helper
#include <EVE_commands.h>      // EVE library
#include <tft.h>               // EVE library Rudolph Riedel FT800-FT813 (https://github.com/RudolphRiedel/FT800-FT813)

/**************************************************************
  EEPROM library

  extEEPROM.cpp >
  Comment l.105/106 :
  //communication->begin();
  //communication->setClock(twiFreq);

  Wire is already initialized before @400kHz

  EEPROM addresses used :
  28 RPM Correction Ratio
  29 RPM Flywheel Teeth
  30 Enabled inputs bits (1x uint8_t)
  31-32 Throttle max value (1x int16_t)
  33-34 AnaOpt1 max value (1x int16_t)
  35-36 AnaOpt2 max value (1x int16_t)
  37-38 AnaOpt1 min value (1x int16_t)
  39-40 AnaOpt2 min value (1x int16_t)
  41-54 Gear calibration data (7x int16_t)
  60-65 MLX I2C Address (6x uint8_t)
**************************************************************/

/**************************************************************
  #############################################
  ############## Vars definition ##############
  #############################################
**************************************************************/

// Inputs
uint32_t inDigiSqrRpm;       // Digital square input (RPM)
uint32_t inDigiSqrOpt1;      // Digital square input (optional)
bool inDigiBrake;            // Digital boolean input (brake)
bool inDigiOpt1;             // Digital boolean input (optional)
uint16_t inAnaGear;          // Digital analog input (GEAR)
uint16_t inAnaThrottle;      // Digital analog input (THROTTLE)
uint16_t inAnaOpt1;          // Digital analog input (optional)
uint16_t inAnaOpt2;          // Digital analog input (optional)
uint8_t enDigitalInputsBits; // Store enabled digital inputs (use binary values, ex : 10100000 > DIGI_SQR_RPM and DIGI_BRAKE enabled)
// BIT 0 = DIGI_SQR_RPM
// BIT 1 = DIGI_SQR_OPT_1
// BIT 2 = DIGI_BRAKE
// BIT 3 = DIGI_OPT_1
// BIT 4 = N/A
// BIT 5 = N/A
// BIT 6 = N/A
// BIT 7 = N/A
uint8_t enAnalogInputsBits; // Store enabled digital inputs (use binary values, ex : 11000000 > inAnaOpt1 and inAnaOpt2 enabled)
// BIT 0 = inAnaOpt1 (GEAR)
// BIT 1 = inAnaOpt2
// BIT 2 = inAnaOpt3
// BIT 3 = inAnaOpt4
// BIT 4 = inAnaOpt5
// BIT 5 = inAnaOpt6
// BIT 6 = inAnaOpt7
// BIT 7 = inAnaOpt8

uint8_t tmpComp, bitShift; // Used to compare values for enabled inputs checks

uint16_t inAnaGearCalib[GEAR_CALIB_SIZE]; // Calibration values for GEAR input, each gear has a analogic value
uint8_t gearNCheck = 0;

// SD Card
File logFile;     // One line every 100ms with all detailed data
File lapFile;     // One line per lap with laptime
File trackFile;   // One line per track with GPS coordinates of finish line
File historyFile; // History of all sessions
char filename[32];

// GPS & Timing
bool recordTrackData, addFinishLog, isRunning = false;
char trackName[16];
uint8_t lapCounter = 0;
int16_t runMinutes, runSeconds;
uint16_t trackId, lapId;
int32_t timeCsec, timeSec, lastFlSec, lastFlCsec, lapSec, lapCsec, posCrossLat, posCrossLon, flineLat1, flineLon1, flineLat2, flineLon2; // Finish line GPS coordinates
uint32_t lastPinRead = 0, lastOneSecSync = 0, lastSdSync = 0, fixCount = 0, elapsedTime = 0;
float coordsDistance, totalDistance, tToFl; // Time To Finish Line
uint8_t gpsFixStatus;

// Infrared temp sensor
uint8_t mlxAddresses[MAX_MLX_SIZE]; // Store each MLX I2C addresses
double mlxValues[MAX_MLX_SIZE];     // Store MLX values

// Analog to Digital converter (ADC)
uint16_t anaValues[8];
char anaValuesChar[8]; // Store value as a single character (useful for ANA1/GEAR : N,1,2,3 ...)
uint16_t anaMinValues[8];
uint16_t anaMaxValues[8];
uint32_t digValues[4];

// RPM
uint8_t rpmFlywheelTeeth;
uint8_t rpmCorrectionRatio;

// Buttons & menu
bool fakeLap = false; // Used to simulate a new lap (debug)
char msgLabel[255];
uint8_t msgDelay = 0, msgType = 0;

// TFT screen
uint32_t lastTftTouchSync = 0;
uint8_t lastTftPrintSync = 0;
uint32_t currentMs;
uint8_t tftScreenId = 10; // Default start screen

#ifdef DEBUG
uint32_t Now = 0;                 // used to calculate integration interval
uint32_t count = 0, sumCount = 0; // used to control display output rate
float deltat = 0.0f, sum = 0.0f;  // integration interval for both filter schemes
uint32_t rpm = 0;
uint32_t lastUpdate = 0, firstUpdate = 0; // used to calculate integration interval
#endif

/**************************************************************
  #################################################
  ############## Instantiate objects ##############
  #################################################
**************************************************************/

// EEPROM
extEEPROM eep(kbits_2, 1, 8); // I2C Address 0x50

// I/O Expander MCP23017
Adafruit_MCP23017 MCP1;
Adafruit_MCP23017 MCP2;

// SD Card
SdFat sd;

// MLX infrared temperature sensors
Adafruit_MLX90614 mlx[MAX_MLX_SIZE] = {Adafruit_MLX90614(FIRST_MLX_ADDRESS), Adafruit_MLX90614(FIRST_MLX_ADDRESS + 1), Adafruit_MLX90614(FIRST_MLX_ADDRESS + 2), Adafruit_MLX90614(FIRST_MLX_ADDRESS + 3), Adafruit_MLX90614(FIRST_MLX_ADDRESS + 4), Adafruit_MLX90614(FIRST_MLX_ADDRESS + 5)};

// GPS
NMEAGPS gps;
gps_fix fix_data;
gps_fix fix_data_prev;

// GPS
HardwareSerial &GPS_PORT = Serial5; // Use serial port for GPS

// OBD2 serial port
Uart OBD2_PORT(&sercom5, 7, 6, SERCOM_RX_PAD_3, UART_TX_PAD_2); // Use serial port for OBD2 - Disable SERCOM5 (Serial + Sercom5) in %localappdata%\Arduino15\packages\arduino\hardware\samd\1.8.9\variants\arduino_mzero\variant.cpp OR %username%\.platformio\packages\framework-arduino-samd\variants\arduino_mzero

// OBD2 ELMduino object
ELM327 myELM327;

/**************************************************************
  #################################################
  ############## Initialisation ###################
  #################################################
**************************************************************/

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
    I/O Expander 1 (Output = 4 leds, Input = 4 buttons)
  **************************************************************/
  MCP1.begin();
  MCP1.pinMode(mcp1Led1, OUTPUT);
  MCP1.pinMode(mcp1Led2, OUTPUT);
  MCP1.pinMode(mcp1Led3, OUTPUT);
  MCP1.pinMode(mcp1Led4, OUTPUT);
  MCP1.pinMode(mcp1Led5, OUTPUT);
  MCP1.pinMode(mcp1Led6, OUTPUT);
  MCP1.pinMode(mcp1Led7, OUTPUT);
  MCP1.pinMode(mcp1Led8, OUTPUT);
  MCP1.digitalWrite(mcp1Led1, LOW); // Power on led1, power off after successfull init
  MCP1.digitalWrite(mcp1Led2, LOW); // Power on led2, power off after successfull init
  MCP1.digitalWrite(mcp1Led3, LOW); // Power on led3, power off after successfull init
  MCP1.digitalWrite(mcp1Led4, LOW); // Power on led4, power off after successfull init
  MCP1.digitalWrite(mcp1Led5, LOW); // Power on led5, power off after successfull init
  MCP1.digitalWrite(mcp1Led6, LOW); // Power on led6, power off after successfull init
  MCP1.digitalWrite(mcp1Led7, LOW); // Power on led7, power off after successfull init
  MCP1.digitalWrite(mcp1Led8, LOW); // Power on led8, power off after successfull init

  /**************************************************************
    I/O Expander 2 (Output = 5v power for 6 devices like MLX sensors plugged on hub)
  **************************************************************/
  MCP2.begin(4);
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

  /**************************************************************
    OBD2 Init (Serial2)
  **************************************************************/
  OBD2_PORT.begin(9600);
  pinPeripheral(6, PIO_SERCOM); // Pin D6 for RX
  pinPeripheral(7, PIO_SERCOM); // Pin D7 for TX
  if (!myELM327.begin(OBD2_PORT))
  {
#ifdef DEBUG
    DEBUG_PORT.print(LABEL_OBD);
    DEBUG_PORT.print(F(" : "));
    DEBUG_PORT.println(LABEL_FAILED);
#endif
    initError(1);
  }
  else
  {
#ifdef DEBUG
    DEBUG_PORT.print(LABEL_OBD);
    DEBUG_PORT.print(F(" : "));
    DEBUG_PORT.println(LABEL_OK);
#endif
  }

  /**************************************************************
    SD card Init (SPI)
  **************************************************************/
  if (!sd.begin(sdCsPin, SD_SCK_MHZ(50)))
  {
#ifdef DEBUG
    DEBUG_PORT.print(LABEL_SD);
    DEBUG_PORT.print(F(" : "));
    DEBUG_PORT.println(LABEL_FAILED);
#endif
    initError(2);
  }
  else
  {
#ifdef DEBUG
    DEBUG_PORT.print(LABEL_SD);
    DEBUG_PORT.print(F(" : "));
    DEBUG_PORT.println(LABEL_OK);
#endif
  }

  /**************************************************************
    EEPROM Init (I2C)
  **************************************************************/
  if (!eep.begin(eep.twiClock400kHz) == 0)
  {
#ifdef DEBUG
    DEBUG_PORT.println(LABEL_EEPROM);
    DEBUG_PORT.print(F(" : "));
    DEBUG_PORT.print(LABEL_FAILED);
#endif
    initError(3);
  }
  else
  {
    eepromReload();
#ifdef DEBUG
    DEBUG_PORT.print(LABEL_EEPROM);
    DEBUG_PORT.print(F(" : "));
    DEBUG_PORT.println(LABEL_OK);
#endif
  }

  /**************************************************************
    MLX Init (I2C)
  **************************************************************/
  // Read infrared temp sensors I2C Address from EEPROM (MAX_MLX_SIZE x 8 bits)
  EEPROM_readAnything(50, mlxAddresses) == sizeof(mlxAddresses);
  for (uint8_t i = 0; i < MAX_MLX_SIZE; i++)
  {
    if (mlxAddresses[i] != 0x00)
    {
      mlx[i].begin();
    }
  }

  /**************************************************************
    ADC Init
  **************************************************************/
  if (!initADC() == 0)
  {
#ifdef DEBUG
    DEBUG_PORT.print(LABEL_ADC);
    DEBUG_PORT.print(F(" : "));
    DEBUG_PORT.println(LABEL_FAILED);
#endif
    initError(4);
  }
  else
  {
#ifdef DEBUG
    DEBUG_PORT.print(LABEL_ADC);
    DEBUG_PORT.print(F(" : "));
    DEBUG_PORT.println(LABEL_OK);
#endif
  }

  /**************************************************************
    GPS Init (Serial5 - Default on Arduino m0)
  **************************************************************/
  GPS_PORT.begin(9600);                    // Start the UART @9600bps for the GPS device (default speed)
  sendUBX(ubxPrtConf, sizeof(ubxPrtConf)); // Set UART speed to 115200bps (Warning : @9600bps > ~5sec delay on GPS data)
  delay(100);
  GPS_PORT.end();
  GPS_PORT.begin(115200);                      // Start the UART @115200bps
  sendUBX(ubxRate10Hz, sizeof(ubxRate10Hz));   // Set refresh rate to 10Hz
  sendUBX(ubxTimepulse, sizeof(ubxTimepulse)); // Set timepulse output ON
  sendUBX(ubxEnableRMC, sizeof(ubxEnableRMC)); // Enable RMC trames, disable all others
  sendUBX(ubxDisableGLL, sizeof(ubxDisableGLL));
  sendUBX(ubxDisableGSA, sizeof(ubxDisableGSA));
  sendUBX(ubxDisableGSV, sizeof(ubxDisableGSV));
  sendUBX(ubxDisableVTG, sizeof(ubxDisableVTG));
  sendUBX(ubxDisableZDA, sizeof(ubxDisableZDA));
#ifdef DEBUG
  DEBUG_PORT.print(LABEL_GPS);
  DEBUG_PORT.print(F(" : "));
  DEBUG_PORT.println(LABEL_OK);
#endif

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

/**************************************************************
  #################################################
  ############## Main loop ###################
  #################################################
**************************************************************/

void loop()
{
#ifdef DEBUG
  // Here we calculate average update rate of main loop
  Now = micros();
  deltat = ((Now - lastUpdate) / 1000000.0f);
  lastUpdate = Now;
  sum += deltat;
  sumCount++;
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

    bitShift = B00000001;
    for (uint8_t i = 0; i < 4; i++)
    { // Read values of enabled inputs only
      tmpComp = bitShift & enDigitalInputsBits;
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
            ;                            // Wait for the COUNT register read sychronization
          digValues[i] = REG_TCC1_COUNT; // Read TCNT1 register (timer1 counter)
          REG_TCC1_COUNT = 0x0000;       // Clear timer's COUNT value
          while (TCC1->SYNCBUSY.bit.COUNT)
            ;                                                                                      // Wait for synchronization
          digValues[i] = digValues[i] * rpmCorrectionRatio * 600 / elapsedTime / rpmFlywheelTeeth; // Ratio between pulse and rpm (22 > flywheel has 22 teeth ### 600 > with check every 100ms, RPM is by minute) ### rpmCorrectionRatio > *(100+corr) / elapsedTime) > if we read counter @101ms or 102ms values should be adjusted
          break;
        // bit 1 : SQR1/SPEED;
        case 1:
          REG_TCC0_CTRLBSET = TCC_CTRLBSET_CMD_READSYNC; // Trigger a read synchronization on the COUNT register
          while (TCC0->SYNCBUSY.bit.CTRLB)
            ; // Wait for the CTRLB register write synchronization
          while (TCC0->SYNCBUSY.bit.COUNT)
            ;                            // Wait for the COUNT register read sychronization
          digValues[i] = REG_TCC0_COUNT; // Read TCNT0 register (timer0 counter)
          REG_TCC0_COUNT = 0x0000;       // Clear timer's COUNT value
          while (TCC0->SYNCBUSY.bit.COUNT)
            ;                                              // Wait for synchronization
          digValues[i] = digValues[i] * 100 / elapsedTime; // Ratio is set to 1 so no maths ### (* 100 / elapsedTime) > if we read counter @99ms, 101ms or 102ms values should be adjusted
          break;
        // bit 2 : DIG1;
        case 2:
          digValues[i] = digitalRead(inDigiBrakePin); // Read "inDigiBrakePin" (pin is plugged on the "+" of the stop light)
          break;
        // bit 3 : DIG2;
        case 3:
          digValues[i] = digitalRead(inDigiOpt1Pin); // Read "inDigiOpt1Pin"
          break;
        }
      }
      bitShift = bitShift << 1;
    }

    /**************************************************************
      Read analog enabled INPUTS
    **************************************************************/
    readAdcValues(anaValues); // Takes time to read all values (one voltage conversion = 12.2ms !)
    formatAdcValues(anaValues);

    /**************************************************************
      Sync files on SDcard every 300 fixes (300x100ms = 30sec) to avoid dataloss on power failure
    **************************************************************/
    if (isRunning && (fixCount - lastSdSync >= 300))
    {
      lastSdSync = fixCount;
      SdFile::dateTimeCallback(dateTimeSd);
      logFile.sync();
      lapFile.sync();
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
      DEBUG_PORT.print((float)sumCount / sum, 2);
      DEBUG_PORT.println(" Hz");
      sumCount = 0;
      sum = 0;

      /*float tempRPM = myELM327.rpm();
      if (myELM327.status == ELM_SUCCESS)
      {
        rpm = (uint32_t)tempRPM;
        DEBUG_PORT.print("RPM: "); DEBUG_PORT.println(rpm);
      } else {
        DEBUG_PORT.print("RPM: NS "); DEBUG_PORT.println(myELM327.status);
      }*/
#endif

      /**************************************************************
        If there is a message printed on screen, remove it after specified delay
      **************************************************************/
      if (msgLabel != "")
      {
        if (msgDelay == 0)
        {
          strcpy(msgLabel, "");
        }
        else
        {
          msgDelay--;
        }
      }

      /**************************************************************
        Read and store MLX temperature in array
      **************************************************************/
      for (uint8_t i = 0; i < MAX_MLX_SIZE; i++)
      {
        if (mlxAddresses[i] != 0x00)
        {
          mlxValues[i] = mlx[i].readObjectTempC();
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
    if (isRunning)
    {
      // Calculate total distance (for SeriousRacing)
      coordsDistance = gpsDistance(fix_data_prev.latitudeL(), fix_data_prev.longitudeL(), fix_data.latitudeL(), fix_data.longitudeL());
      totalDistance += coordsDistance;

      if (recordTrackData == true)
      {
        // Check if we pass the finishline (2x2 coordinates for finish line points + 2x2 coordinates for last position + position now)
        if (segIntersect(fix_data.latitudeL(), fix_data.longitudeL(), fix_data_prev.latitudeL(), fix_data_prev.longitudeL(), flineLat1, flineLon1, flineLat2, flineLon2, posCrossLat, posCrossLon) || fakeLap == true)
        {
          if (fakeLap == true)
          {
            fakeLap = false;
            tToFl = 0; //random(1, 100) / 100.00; // Random value to simulate Time To Finish Line
          }
          else
          {
            // Calculate Time To Finish Line (from last know position by GPS just before crossing the finish line / format : sss.ms) ### tToFl = (distance between previous position and finish line (Ex : 0.00112km) / distance between previous position and actual position (Ex : 0.00254km)) * (time of actual fix - time of previous fix)
            tToFl = (gpsDistance(fix_data_prev.latitudeL(), fix_data_prev.longitudeL(), posCrossLat, posCrossLon) / gpsDistance(fix_data.latitudeL(), fix_data.longitudeL(), fix_data_prev.latitudeL(), fix_data_prev.longitudeL())) * ((fix_data.dateTime.hours * 3600 + fix_data.dateTime.minutes * 60 + fix_data.dateTime.seconds + fix_data.dateTime_cs / 100.00) - (fix_data_prev.dateTime.hours * 3600 + fix_data_prev.dateTime.minutes * 60 + fix_data_prev.dateTime.seconds + fix_data_prev.dateTime_cs / 100.00));
          }
          // Add "Time to finish line (tToFl)" to the last known Epoch Time (fix_data_prev)
          timeAdd(tToFl, fix_data_prev.dateTime, fix_data_prev.dateTime_cs, timeSec, timeCsec);

          // Calculate total laptime (substract previous finish laptime to actual laptime)
          if (lapCounter > 0)
          { // Get first laptime at the end of the lap 1 (lapCounter = 1 / We start from the paddocks)
            timeSubstract(timeSec, timeCsec, lastFlSec, lastFlCsec, lapSec, lapCsec);

            runMinutes = lapSec / 60;
            runSeconds = lapSec % 60;

            lapFile.print(lapCounter);
            lapFile.print(F(";"));
            lapFile.print(runMinutes); // Store laptime mm:sss:ms (human readable)
            lapFile.print(F(":"));
            if (runSeconds < 10)
              lapFile.print(F("0")); // Leading zeros (remember "runSeconds" is an integer !!)
            lapFile.print(runSeconds);
            lapFile.print(F(":"));
            if (lapCsec < 10)
              lapFile.print(F("0")); // Leading zeros (remember "lapCsec" is an integer !!)
            lapFile.print(lapCsec);
            lapFile.print(F(";")); // Store laptime sss.ms (enable float comparaison for best lap or other calculations)
            lapFile.print(lapSec);
            lapFile.print(F("."));
            if (lapCsec < 10)
              lapFile.print(F("0")); // Leading zeros (remember "lapCsec" is an integer !!)
            lapFile.println(lapCsec);
          }

          // Store timestamp (sec+ms) at the finish line to calculate next lap time
          lastFlSec = timeSec;
          lastFlCsec = timeCsec;

          // Write the finish log line + inc lapCounter
          addFinishLog = true;
          lapCounter++;
        }
        else
        {
          // Continuously update lapSec and lapCsec for diplaying realtime values on TFT (and only for this !)
          if (lapCounter > 0)
          {
            timeSubstract(fix_data.dateTime, fix_data.dateTime_cs, lastFlSec, lastFlCsec, lapSec, lapCsec);
          }
          addFinishLog = false;
        }
      }

      /**************************************************************
        Write all data to file on SD card (GPS, inAnaThrottle, gear, rpm, temperature sensors)
      **************************************************************/
      // Time, distance and lap (always printed)
      if (addFinishLog == true)
      {
        logFile.print(timeSec);
        logFile.print(F("."));
        if (timeCsec < 10)
          logFile.print(F("0")); // Leading zeros (remember "timeCsec" is an integer !!)
        logFile.print(timeCsec);
      }
      else
      {
        logFile.print(fix_data.dateTime);
        logFile.print(F("."));
        if (fix_data.dateTime_cs < 10)
          logFile.print(F("0")); // Leading zeros (remember "fix_data.dateTime_cs" is an integer !!)
        logFile.print(fix_data.dateTime_cs);
      }
      logFile.print(F(";"));
      logFile.print(totalDistance, 3);
      logFile.print(F(";"));
      logFile.print(lapCounter);
      logFile.print(F(";"));

      // KPH, heading (always printed)
      logFile.print(fix_data.speed_kph(), 0);
      logFile.print(F(";"));
      logFile.print(fix_data.heading(), 1);
      logFile.print(F(";"));

      // Digital inputs (printed if enabled)
      bitShift = B00000001;
      for (uint8_t i = 0; i < 4; i++)
      {
        tmpComp = bitShift & enDigitalInputsBits;
        if (tmpComp == bitShift)
        {
          logFile.print(digValues[i]);
          logFile.print(F(";"));
        }
        bitShift = bitShift << 1;
      }

      // Analog inputs (printed if enabled)
      bitShift = B00000001;
      for (uint8_t i = 0; i < 8; i++)
      {
        tmpComp = bitShift & enAnalogInputsBits;
        if (tmpComp == bitShift)
        {
          logFile.print(anaValues[i]);
          logFile.print(F(";"));
        }
        bitShift = bitShift << 1;
      }

      // Infrared temperature (printed if enabled)
      for (uint8_t i = 0; i < MAX_MLX_SIZE; i++)
      {
        if (mlxAddresses[i] != 0x00)
        {
          logFile.print(mlxValues[i]);
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
    { // isRunning == false
      /**************************************************************
        Do something else when laptimer IS NOT running
      **************************************************************/
      // Do what you want :)
    }
  }

  /**************************************************************
    Refresh TFT screen
  **************************************************************/
  currentMs = millis();
  if ((currentMs - lastTftTouchSync) >= 5)
  { /* execute the code only every 5 milli-seconds */
    lastTftTouchSync = currentMs;
    TFT_touch();
    lastTftPrintSync++;
    if (lastTftPrintSync >= 4)
    { /* refresh the display every 20ms */
      lastTftPrintSync = 0;
      TFT_display();
    }
  }
}