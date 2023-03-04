/*****************************************************
 * ################################################# *
 * ############## Includes ######################### *
 * ################################################# *
*****************************************************/

#include <Arduino.h>           // Arduino library
#include <Adafruit_MCP23X17.h> // I/O Expander MCP23017 (https://github.com/adafruit/Adafruit-MCP23017-Arduino-Library)
#include <SdFat.h>             // Greiman/SdFat library (https://github.com/greiman/SdFat)
#include <ELMduino.h>          // ELM327 OBD-II library (https://github.com/PowerBroker2/ELMduino)
#include <Adafruit_MLX90614.h> // MLX90614 Infrared temperature sensor (https://github.com/adafruit/Adafruit-MLX90614-Library)
#include <NMEAGPS.h>           // NeoGPS library (https://github.com/SlashDevin/NeoGPS) - In "NeoTime.h" > static const uint16_t s_epoch_year = POSIX_EPOCH_YEAR; static const uint8_t  s_epoch_weekday = POSIX_EPOCH_WEEKDAY;
#include <extEEPROM.h>         // EEPROM library (http://github.com/PaoloP74/extEEPROM)
#include <helper.h>            // DAWA functions helper
#include <tft.h>               // EVE library Rudolph Riedel FT800-FT813 (https://github.com/RudolphRiedel/FT800-FT813)

/*****************************************************
 * ################################################# *
 * ############## Functions ######################## *
 * ################################################# *
*****************************************************/

/**
 * Initialization error diagnostic
 * 
 * Use the 8 leds to show errors during DAWA initialization
 * This is an infinite loop
 *
 * @param errCode a code error number
 * @return
 */
void initError(uint8_t errCode = 0)
{
  // Infinite loop, blinking leds with 8 bits coding
  while (1)
  {
    if (bitRead(errCode, 7) == true)
    {
      MCP1.digitalWrite(mcp1Led1, MCP1.digitalRead(mcp1Led1) ^ 1);
    }
    if (bitRead(errCode, 6) == true)
    {
      MCP1.digitalWrite(mcp1Led2, MCP1.digitalRead(mcp1Led2) ^ 1);
    }
    if (bitRead(errCode, 5) == true)
    {
      MCP1.digitalWrite(mcp1Led3, MCP1.digitalRead(mcp1Led3) ^ 1);
    }
    if (bitRead(errCode, 4) == true)
    {
      MCP1.digitalWrite(mcp1Led4, MCP1.digitalRead(mcp1Led4) ^ 1);
    }
    if (bitRead(errCode, 3) == true)
    {
      MCP1.digitalWrite(mcp1Led5, MCP1.digitalRead(mcp1Led5) ^ 1);
    }
    if (bitRead(errCode, 2) == true)
    {
      MCP1.digitalWrite(mcp1Led6, MCP1.digitalRead(mcp1Led6) ^ 1);
    }
    if (bitRead(errCode, 1) == true)
    {
      MCP1.digitalWrite(mcp1Led7, MCP1.digitalRead(mcp1Led7) ^ 1);
    }
    if (bitRead(errCode, 0) == true)
    {
      MCP1.digitalWrite(mcp1Led8, MCP1.digitalRead(mcp1Led8) ^ 1);
    }
    delay(200);
  }
}

/**
 * Reset EEPROM
 * 
 * Set all bits to 0
 *
 * @param
 * @return
 */
void eepromReset(void)
{
  for (uint8_t i = 0; i < 128; i++)
  {
    eep.write(i, 0);
  }
}

/**
 * Write default values to EEPROM
 * 
 * Write some default values to EEPROM (option to quickly setup DAWA)
 *
 * @param
 * @return
 */
void eepromLoadDefaults(void)
{
  EEPROM_writeAnything(EEPROM_GENERAL_ADDR, (uint8_t)DEFAULT_RPM_CORRECTION_RATIO) == sizeof(g_rpmCorrectionRatio);           // Write default RPM correction ratio
  EEPROM_writeAnything(EEPROM_GENERAL_ADDR + 1, (uint8_t)DEFAULT_RPM_FLYWHEEL_TEETH) == sizeof(g_rpmFlywheelTeeth);           // Write default RPM flywheel teeth number
  EEPROM_writeAnything(EEPROM_GENERAL_ADDR + 2, (uint8_t)DEFAULT_TRACK_QTY) == sizeof(g_trackQty);                            // Write the number of tracks in EEPROM
  EEPROM_writeAnything(EEPROM_GENERAL_ADDR + 3, (uint8_t)DEFAULT_ENDIGITALINPUTSBITS_VALUE) == sizeof(g_enDigitalInputsBits); // Write enabled digital inputs (8 bits)
  EEPROM_writeAnything(EEPROM_GENERAL_ADDR + 4, (uint8_t)DEFAULT_ENANALOGINPUTSBITS_VALUE) == sizeof(g_enAnalogInputsBits);   // Write enabled analog inputs (8 bits)
  EEPROM_writeAnything(EEPROM_GENERAL_ADDR + 5, (int16_t)DEFAULT_SELECTED_TRACK_ID) == sizeof(g_currentTrackId);              // Write selected track ID (16 bits)
  EEPROM_writeAnything(EEPROM_GENERAL_ADDR + 7, (bool)DEFAULT_OBD_FEATURES) == sizeof(g_enObd);                               // Write default OBD features state
}

/**
 * Write running values to EEPROM
 * 
 * Write new values to EEPROM when quitting menu : setup/general
 *
 * @param
 * @return
 */
void eepromSaveRunningValues(void)
{
  // Write RPM correction ratio, RPM flywheel teeth number and OBD state
  EEPROM_writeAnything(EEPROM_GENERAL_ADDR, g_rpmCorrectionRatio) == sizeof(g_rpmCorrectionRatio);
  EEPROM_writeAnything(EEPROM_GENERAL_ADDR + 1, g_rpmFlywheelTeeth) == sizeof(g_rpmFlywheelTeeth);
  EEPROM_writeAnything(EEPROM_GENERAL_ADDR + 7, g_enObd) == sizeof(g_enObd);
}

/**
 * Reload EEPROM settings to global variables
 * 
 * Read some parameters stored in EEPROM and write them to global variables in RAM
 * Load track list from EEPROM to RAM
 * Load last selected track
 *
 * @param
 * @return
 */
void eepromReload(void)
{
  EEPROM_readAnything(EEPROM_GENERAL_ADDR, g_rpmCorrectionRatio) == sizeof(g_rpmCorrectionRatio);       // Read default RPM correction ratio
  EEPROM_readAnything(EEPROM_GENERAL_ADDR + 1, g_rpmFlywheelTeeth) == sizeof(g_rpmFlywheelTeeth);       // Read default RPM flywheel teeth number
  EEPROM_readAnything(EEPROM_GENERAL_ADDR + 2, g_trackQty) == sizeof(g_trackQty);                       // Read the number of tracks in EEPROM
  EEPROM_readAnything(EEPROM_GENERAL_ADDR + 3, g_enDigitalInputsBits) == sizeof(g_enDigitalInputsBits); // Read enabled digital inputs (8 bits)
  EEPROM_readAnything(EEPROM_GENERAL_ADDR + 4, g_enAnalogInputsBits) == sizeof(g_enAnalogInputsBits);   // Read enabled analog inputs (8 bits)
  EEPROM_readAnything(EEPROM_GENERAL_ADDR + 5, g_currentTrackId) == sizeof(g_currentTrackId);           // Read selected track ID (16 bits)
  EEPROM_readAnything(EEPROM_GENERAL_ADDR + 7, g_enObd) == sizeof(g_enObd);                             // Read OBD features state

  // Read GEAR (ANA1) calibration data (GEAR_CALIB_SIZE x 16 bits)
  for (uint8_t i = 0; i < GEAR_CALIB_SIZE; i++)
  {
    EEPROM_readAnything(EEPROM_GEAR_CALIBRATION_ADDR + (2 * i), g_inAnaGearCalib[i]) == sizeof(g_inAnaGearCalib[i]);
  }

  // Load track list from EEPROM to RAM
  loadTracksFromEeprom();

  // Load selected track
  loadSelectedTrack(g_currentTrackId);
}

/**
 * Read CSV file
 * https://github.com/greiman/SdFat/blob/master/examples/ReadCsv/ReadCsv.ino
 *
 * @param file File to read
 * @param str Character array for the field
 * @param size Size of str array
 * @param delim csv delimiter
 * @return return '\n' or zero(EOF) for success, negative value for failure.
 */
int csvReadText(File *file, char *str, size_t size, char delim)
{
  char ch;
  int rtn;
  size_t n = 0;
  while (true)
  {
    // check for EOF
    if (!file->available())
    {
      rtn = 0;
      break;
    }
    if (file->read(&ch, 1) != 1)
    {
      // read error
      rtn = -1;
      break;
    }
    // Delete CR.
    if (ch == '\r')
    {
      continue;
    }
    if (ch == delim || ch == '\n')
    {
      rtn = ch;
      break;
    }
    if ((n + 1) >= size)
    {
      // string too long
      rtn = -2;
      n--;
      break;
    }
    str[n++] = ch;
  }
  str[n] = '\0';
  return rtn;
}

/**
 * Read CSV file
 * https://github.com/greiman/SdFat/blob/master/examples/ReadCsv/ReadCsv.ino
 *
 * @param file File to read
 * @param num Address of number to read
 * @param delim csv delimiter
 * @return integer
 */
int csvReadInt32(File *file, int32_t *num, char delim)
{
  char buf[20];
  char *ptr;
  int rtn = csvReadText(file, buf, sizeof(buf), delim);
  if (rtn < 0)
    return rtn;
  *num = strtol(buf, &ptr, 10);
  if (buf == ptr)
    return -3;
  while (isspace(*ptr))
    ptr++;
  return *ptr == 0 ? rtn : -4;
}

/**
 * Read CSV file
 * https://github.com/greiman/SdFat/blob/master/examples/ReadCsv/ReadCsv.ino
 *
 * @param file File to read
 * @param num Address of number to read
 * @param delim csv delimiter
 * @return integer
 */
int csvReadInt16(File *file, int16_t *num, char delim)
{
  int32_t tmp;
  int rtn = csvReadInt32(file, &tmp, delim);
  if (rtn < 0)
    return rtn;
  if (tmp < INT_MIN || tmp > INT_MAX)
    return -5;
  *num = tmp;
  return rtn;
}

/**
 * Read CSV file
 * https://github.com/greiman/SdFat/blob/master/examples/ReadCsv/ReadCsv.ino
 *
 * @param file File to read
 * @param num Address of number to read
 * @param delim csv delimiter
 * @return integer
 */
int csvReadUint32(File *file, uint32_t *num, char delim)
{
  char buf[20];
  char *ptr;
  int rtn = csvReadText(file, buf, sizeof(buf), delim);
  if (rtn < 0)
    return rtn;
  *num = strtoul(buf, &ptr, 10);
  if (buf == ptr)
    return -3;
  while (isspace(*ptr))
    ptr++;
  return *ptr == 0 ? rtn : -4;
}

/**
 * Read CSV file
 * https://github.com/greiman/SdFat/blob/master/examples/ReadCsv/ReadCsv.ino
 *
 * @param file File to read
 * @param num Address of number to read
 * @param delim csv delimiter
 * @return integer
 */
int csvReadUint16(File *file, uint16_t *num, char delim)
{
  uint32_t tmp;
  int rtn = csvReadUint32(file, &tmp, delim);
  if (rtn < 0)
    return rtn;
  if (tmp > UINT_MAX)
    return -5;
  *num = tmp;
  return rtn;
}

/**
 * Get datetime for files creation/modification on SD Card
 *
 * @param date date now
 * @param time time now
 * @return
 */
void dateTimeSd(uint16_t *date, uint16_t *time)
{
  *date = FAT_DATE(fix_data.dateTime.year + 2000, fix_data.dateTime.month, fix_data.dateTime.date); // return date using FAT_DATE macro to format fields
  *time = FAT_TIME(fix_data.dateTime.hours, fix_data.dateTime.minutes, fix_data.dateTime.seconds);  // return time using FAT_TIME macro to format fields
}

/**
 * Calculate 2 line segments intersection
 *
 * Calculate intersection of 2 line segments (the motocycle trace and the finish line)
 *
 * @param pos_now_lat latitude now
 * @param pos_now_lon longitude now
 * @param pos_prev_lat previous latitude (previous GPS fix)
 * @param pos_prev_lon previous longitude (previous GPS fix)
 * @param trackLat1 finish line latitude 1
 * @param trackLon1 finish line longitude 1
 * @param trackLat2 finish line latitude 2
 * @param trackLon2 finish line longitude 2
 * @param pos_cross_lat crossing point latitude
 * @param pos_cross_lon crossing point longitude
 * @return is these lines crossing ? (Y/N)
 */
boolean segIntersect(int32_t pos_now_lat, int32_t pos_now_lon, int32_t pos_prev_lat, int32_t pos_prev_lon, int32_t trackLat1, int32_t trackLon1, int32_t trackLat2, int32_t trackLon2, int32_t &pos_cross_lat, int32_t &pos_cross_lon)
{
  bool denomPositive;
  float denom, s_numer, t_numer, t;
  int32_t track_pos_x, track_pos_y, pos_x, pos_y, trackLon, trackLat;

  trackLon = trackLon2 - trackLon1;
  trackLat = trackLat2 - trackLat1;
  pos_x = pos_now_lon - pos_prev_lon;
  pos_y = pos_now_lat - pos_prev_lat;
  denom = trackLon * pos_y - pos_x * trackLat;
  if (denom == 0)
  {
    return 0; // Collinear
  }

  if (denom > 0)
  {
    denomPositive = true;
  }
  else
  {
    denomPositive = false;
  }

  track_pos_x = trackLon1 - pos_prev_lon;
  track_pos_y = trackLat1 - pos_prev_lat;

  s_numer = trackLon * track_pos_y - trackLat * track_pos_x;
  if ((s_numer < 0) == denomPositive)
  {
    return 0; // No collision
  }

  t_numer = pos_x * track_pos_y - pos_y * track_pos_x;
  if ((t_numer < 0) == denomPositive)
  {
    return 0; // No collision
  }

  if (((s_numer > denom) == denomPositive) || ((t_numer > denom) == denomPositive))
  {
    return 0; // No collision
  }

  // Collision detected
  t = t_numer / denom;
  pos_cross_lat = trackLat1 + (t * trackLat);
  pos_cross_lon = trackLon1 + (t * trackLon);

  return 1;
}

/**
 * GPS distance calculation
 *
 * Calculate distance between 2 GPS coords
 * Based on haversine formula
 *
 * @param lat1 latitude 1
 * @param lon1 longitude 1
 * @param lat2 latitude 1
 * @param lon2 longitude 1
 * @return distance in meters
 */
float gpsDistance(int32_t lat1, int32_t lon1, int32_t lat2, int32_t lon2)
{
  float dlam, dphi, p = 0.017453292519943295; // (Pi / 180)
  dphi = p * (lat1 + lat2) * 0.5e-7;          //average latitude in radians
  float cphi = cos(dphi);
  dphi = p * (lat2 - lat1) * 1.0e-7; //differences in degrees (to radians)
  dlam = p * (lon2 - lon1) * 1.0e-7;
  dlam *= cphi; //correct for latitude
  return 6371000.0 * sqrt(dphi * dphi + dlam * dlam);
}

/**
 * Stop data acquisition/laptimer
 *
 * Stop data acquisition
 * Properly close files on SDCARD
 * Remove the first lap which is not an entire lap as you start from paddocks
 *
 * @param
 * @return Running status (0 = not running)
 */
int8_t stopLaptimer(void)
{
  SdFile::dateTimeCallback(dateTimeSd);
  logFile.close(); // Close file on SDcard
  lapFile.close(); // Close file on SDcard

  // Remove lap 0 (the one you start from paddock)
  for (uint8_t i = 0; i < g_lapQty; i++)
  {
    if (i < g_lapQty - 1)
    {
      g_lapsList[i] = g_lapsList[i + 1];
    }
  }
  g_lapQty--;
  return 0;
}

/**
 * Start data acquisition/laptimer
 *
 * Start data acquisition (and laptimer if GPS signal is ok and a track is selected)
 * Some CSV files are created on SDCARD for logging
 * Some vars are initialized
 *
 * @param
 * @return Running status (-1 = error, 0 = not running, 1 = run as datalogger only, 2 = run as datalogger and laptimer)
 */
int8_t startLaptimer(void)
{
  char filename[64]; // Filename
  uint8_t bitShift, tmpComp;
  SdFile::dateTimeCallback(dateTimeSd);

  // Create new datafile : log file (create new)
  sprintf(filename, "%02u%02u%02u-%02u%02u%02u-%s.csv", fix_data.dateTime.year, fix_data.dateTime.month, fix_data.dateTime.date, fix_data.dateTime.hours, fix_data.dateTime.minutes, fix_data.dateTime.seconds, g_currentTrack.trackName);
  strcpy(g_workingFilename, filename);
  if (logFile.open(filename, O_CREAT | O_WRITE | O_EXCL))
  {
    // Time, distance and lap (always printed)
    logFile.print(LABEL_LOG_HEADER_1);

    // KPH, heading (always printed)
    logFile.print(LABEL_LOG_HEADER_2);

    // Digital inputs (printed if enabled)
    bitShift = 0b00000001;
    for (uint8_t i = 0; i < 4; i++)
    {
      tmpComp = bitShift & g_enDigitalInputsBits;
      if (tmpComp == bitShift)
      {
        logFile.print(digitalInputsLabel[i]);
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
        logFile.print(analogInputsLabel[i]);
        logFile.print(F(";"));
      }
      bitShift = bitShift << 1;
    }

    // Infrared temperature (printed if enabled)
    for (uint8_t i = 0; i < MAX_MLX_SIZE; i++)
    {
      if (g_mlxAddresses[i] != 0x00)
      {
        logFile.print(LABEL_LOG_HEADER_3);
        logFile.print(i);
        logFile.print(F(";"));
      }
    }

    // Latitude & longitude (always printed)
    logFile.println(LABEL_LOG_HEADER_4);
  }
  else
  {
    showMessage(LABEL_LOG_FILEERROR, 2000, 2);
    return -1;
  }

  // If GPS ready and track selected we do data acquisition AND laptimer
  if (fix_data.valid.location && g_currentTrack.trackId >= 0)
  {
    // Create/append laptimes file
    sprintf(filename, "LAPTIMES.csv");
    if (lapFile.open(filename, O_CREAT | O_APPEND | O_WRITE))
    {
      // headers ?
    }
    else
    {
      showMessage(LABEL_LOG_FILEERROR, 2000, 2);
      return -1;
    }
    g_lapQty = 0;
    g_lapSec = 0;
    g_lapCsec = 0;
    g_totalDistance = 0;

    // Init current lap (starting from paddocks)
    g_currentLap.lapStartTimestampS = fix_data.dateTime;
    g_currentLap.lapStartTimestampCs = fix_data.dateTime_cs; // This first value (lap 0 only) is not accurate because GPS is running @ 10hz so the precision is on 1/10 (and not on 1/100). This is not important because we start from paddocks by pressing the start button manually.
    g_currentLap.lapEndTimestampS = fix_data.dateTime;
    g_currentLap.lapEndTimestampCs = fix_data.dateTime_cs;
    g_currentLap.lapTimeS = 0;
    g_currentLap.lapTimeCs = 0;
    g_currentLap.lapTime = 0.00;
    g_currentLap.lapNumber = 0;
    g_currentLap.lapMaxSpeed = 0;
    g_currentLap.lapTrackId = g_currentTrack.trackId;
    g_currentLap.lapIsBest = false;
    return 2;
  }
  else // Run as data acquisition only (no laptimer because no valid GPS signal and/or track selected)
  {
    g_lapQty = 0;
    g_lapSec = 0;
    g_lapCsec = 0;
    g_totalDistance = 0;
    showMessage(LABEL_RUN_AS_DATALOGGER, 2000, 1);
    return 1;
  }
}

/**
 * Print message on TFT screen
 *
 * @param label a text message
 * @param delay the time during which the message should be printed on screen (ms)
 * @param type the type of message (1 = OK/green, 2 = ERROR/red)
 * @return
 */
void showMessage(const char *label, uint32_t delay, uint8_t type)
{
  strcpy(g_msgLabel, label);
  g_msgDelay = millis() + delay;
  g_msgType = type;
}

/**
 * Send UBX commands to UBLOX GPS
 *
 * @param progmemBytes data
 * @param len length of progmemBytes
 * @return
 */
void sendUBX(const unsigned char *progmemBytes, size_t len)
{
  GPS_PORT.write(0xB5); // SYNC1
  GPS_PORT.write(0x62); // SYNC2

  uint8_t a = 0, b = 0;
  while (len-- > 0)
  {
    uint8_t c = pgm_read_byte(progmemBytes++);
    a += c;
    b += a;
    GPS_PORT.write(c);
  }
  GPS_PORT.write(a); // CHECKSUM A
  GPS_PORT.write(b); // CHECKSUM B
}

/**
 * Time to finish line calculation
 *
 * Add a time [arg0] (Ex : 5.14) to another which is composed of seconds [arg1] and centiseconds [arg2]. Return seconds [arg3] and centiseconds [arg4]
 *
 * @param timeSecCsec time 1
 * @param endSec seconds from time 2
 * @param endCsec hundredth of seconds from time 2
 * @param returnSec total seconds time 1 + time 2
 * @param returnCsec total hundredth of seconds time 1 + time 2
 * @return
 */
void timeToFinishLineCalculation(float timeSecCsec, uint32_t endSec, uint8_t endCsec, uint32_t &returnSec, uint8_t &returnCsec)
{
  returnSec = endSec + (int32_t)(timeSecCsec + (endCsec / 100.00));
  returnCsec = ((timeSecCsec + (endCsec / 100.00)) - (int32_t)(timeSecCsec + (endCsec / 100.00))) * 100;
}

/**
 * Lap time calculation
 *
 * Substract a time [arg0][arg1] to another [arg2][arg3]. Return seconds [arg4] and centiseconds [arg5]
 *
 * @param s1 seconds from time 1
 * @param cs1 hundredth of seconds from time 1
 * @param s2 seconds from time 2
 * @param cs2 hundredth of seconds from time 2
 * @param returnSec total seconds time 1 - time 2
 * @param returnCsec total hundredth of seconds time 1 - time 2
 * @return
 */
void lapTimeCalculation(uint32_t s1, uint8_t cs1, uint32_t s2, uint8_t cs2, uint32_t &returnSec, uint8_t &returnCsec)
{
  if (cs1 < cs2)
  {
    returnSec = (s1 - s2) - 1;
    returnCsec = 100 + cs1 - cs2;
  }
  else
  {
    returnSec = s1 - s2;
    returnCsec = cs1 - cs2;
  }
}

/**
 * GPS time adjust
 *
 * NeoGPS function used to adjust time
 *
 * @param dt datetime from a GPS fix
 * @return
 */
void adjustTime(NeoGPS::time_t &dt)
{
  NeoGPS::clock_t seconds = dt; // convert date/time structure to seconds
#ifdef CALCULATE_DST
  //  Calculate DST changeover times once per reset and year!
  static NeoGPS::time_t changeover;
  static NeoGPS::clock_t springForward, fallBack;
  if ((springForward == 0) || (changeover.year != dt.year))
  {
    //  Calculate the spring changeover time (seconds)
    changeover.year = dt.year;
    changeover.month = springMonth;
    changeover.date = springDate;
    changeover.hours = springHour;
    changeover.minutes = 0;
    changeover.seconds = 0;
    changeover.set_day();
    // Step back to a Sunday, if day != SUNDAY
    changeover.date -= (changeover.day - NeoGPS::time_t::SUNDAY);
    springForward = (NeoGPS::clock_t)changeover;

    //  Calculate the fall changeover time (seconds)
    changeover.month = fallMonth;
    changeover.date = fallDate;
    changeover.hours = fallHour - 1; // to account for the "apparent" DST +1
    changeover.set_day();
    // Step back to a Sunday, if day != SUNDAY
    changeover.date -= (changeover.day - NeoGPS::time_t::SUNDAY);
    fallBack = (NeoGPS::clock_t)changeover;
  }
#endif
  //  First, offset from UTC to the local timezone
  seconds += zone_offset;
#ifdef CALCULATE_DST
  //  Then add an hour if DST is in effect
  if ((springForward <= seconds) && (seconds < fallBack))
    seconds += NeoGPS::SECONDS_PER_HOUR;
#endif
  dt = seconds; // convert seconds back to a date/time structure
}

/**
 * Initialize ADC chip
 *
 * Configuration mode, conversion rate and which channels are enabled
 *
 * @param bits Which channels are enabled
 * @return
 */
uint8_t initADC(uint8_t bits)
{
  Wire.beginTransmission(0x1D);
  Wire.write(uint8_t(0x0B)); // Advanced Configuration Register
  Wire.write(uint8_t(0x03)); // 00000011 (Vref ext & mode = 1)
  Wire.endTransmission();

  Wire.beginTransmission(0x1D);
  Wire.write(uint8_t(0x07)); // Conversion Rate Register
  Wire.write(uint8_t(0x01)); // 00000001 (Conversion mode = continuous)
  Wire.endTransmission();

  Wire.beginTransmission(0x1D);
  Wire.write(uint8_t(0x08));  // Channel Disable Register
  Wire.write(uint8_t(~bits)); // Enable or disable channels
  Wire.endTransmission();

  Wire.beginTransmission(0x1D);
  Wire.write(uint8_t(0x00)); // Configuration Register
  Wire.write(uint8_t(0x01)); // 00000001 (Start conversion)
  return Wire.endTransmission();
}

/**
 * Configure ADC chip input bits
 *
 * Send a byte to reconfigure ADC enabled channels
 *
 * @param bits Which channels are enabled
 * @return
 */
uint8_t configureADC(uint8_t bits)
{
  Wire.beginTransmission(0x1D);
  Wire.write(uint8_t(0x00)); // Configuration Register
  Wire.write(uint8_t(0x00)); // 00000000 (Shutdown mode)
  Wire.endTransmission();

  Wire.beginTransmission(0x1D);
  Wire.write(uint8_t(0x08));  // Channel Disable Register
  Wire.write(uint8_t(~bits)); // Enable or disable channels
  Wire.endTransmission();

  Wire.beginTransmission(0x1D);
  Wire.write(uint8_t(0x00)); // Configuration Register
  Wire.write(uint8_t(0x01)); // 00000001 (Start conversion)
  return Wire.endTransmission();
}

/**
 * Read single ADC value
 *
 * Read single value from the ADC
 *
 * @param registerID Register ID corresponding to the input you want to read
 * @return ADC raw value
 */
uint16_t readAdcValue(uint8_t registerID)
{
  Wire.beginTransmission(0x1D); // Talk to ADC128D818 - A0 and A1 to GND
  Wire.write(registerID);       // Busy Status Register (Bit 1 = Not Ready > Waiting for the power-up sequence to end)
  Wire.endTransmission();
  Wire.requestFrom(0x1D, 2); // Read Register 0x20
  if (Wire.available() == 2)
  {
    uint8_t high_byte = Wire.read();
    uint8_t low_byte = Wire.read();
    return (((((uint16_t)high_byte) << 8) | ((uint16_t)low_byte)) & 0xFFF0) >> 4; // We read a 12 bits value (2x 8 bits I2C reads)
  }
  return -1;
}

/**
 * Read all 8 ADC values
 *
 * This function read and store in a array raw values of all 8 raw ADC values
 *
 * @param
 * @return
 */
void readAdcValues(void)
{
  uint8_t bitShift = 0b00000001, tmpComp;
  uint8_t registerID = 0x20; // First register ID is 0x20
  for (uint8_t i = 0; i < 8; i++)
  {
    tmpComp = bitShift & g_enAnalogInputsBits;
    if (tmpComp == bitShift)
    {
      g_anaValues[i] = readAdcValue(registerID);
    }
    else
    {
      g_anaValues[i] = 0;
    }
    registerID++;
    bitShift = bitShift << 1;
  }
}

/**
 * Format all 8 ADC values
 *
 * This function store in a array formated values of all 8 raw ADC values
 * The analog 1 is specific as this is the gear value : a char is stored in another array > neutral is "N"
 * All the others a simple mapped value (0 to 100)
 *
 * @param
 * @return
 */
void formatAdcValues(void)
{
  uint8_t bitShift = 0b00000001, tmpComp;
  for (uint8_t i = 0; i < 8; i++)
  {
    tmpComp = bitShift & g_enAnalogInputsBits;
    if (tmpComp == bitShift)
    {
      switch (i)
      {
      // bit 0 : Analog 1 (gear)
      case 0:
        for (uint8_t j = 0; j < GEAR_CALIB_SIZE; j++)
        {
          if (g_anaValues[i] > g_inAnaGearCalib[j] - GEAR_OFFSET && g_anaValues[i] <= g_inAnaGearCalib[j] + GEAR_OFFSET)
          {
            if (j == 0)
            {
              g_gearNCheck++;
            }
            if ((j == 0 && g_gearNCheck > 3) || j > 0) // We test 3 times to prevent displaying "N" between 2 gears
            {
              g_anaValuesChar[i] = gearName[j];
              g_gearNCheck = 0;
            }
          }
        }
        // If condition is never true, last gear value is printed
        break;
      // bit 1 to 7 : Analog 2 to 8 (all of the others except ANA1/GEAR)
      default:
        //g_anaValues[i] = map(g_anaValues[i], 0, 4096, 0, 100); // Map values from 0 to 100 (default is 0 to 4096)
        break;
      }
    }
    bitShift = bitShift << 1;
  }
}

/**
 * Calibrate gears
 *
 * For each gear we read and save the corresponding analogic value
 * This value is stored on EEPROM
 *
 * @param
 * @return
 */
void gearCalibration(void)
{
  char bufferMsg[13];
  char bufferGear[2];
  for (uint8_t i = 0; i < GEAR_CALIB_SIZE; i++)
  {
    sprintf(bufferGear, "%c", gearName[i]);
    strcpy(bufferMsg, LABEL_GEAR_CALIBRATION);
    strcat(bufferMsg, bufferGear);
    showMessage(bufferMsg, 3000, 0);
    TFT_display(); // Refresh TFT to print message
    delay(3000);
    g_inAnaGearCalib[i] = readAdcValue(0x20); // 0x20 is first register (ANA1/GEAR)
    EEPROM_writeAnything(EEPROM_GEAR_CALIBRATION_ADDR + (2 * i), g_inAnaGearCalib[i]) == sizeof(g_inAnaGearCalib[i]);
  }
  showMessage(LABEL_GEAR_CALIBRATION_OK, 2000, 1);
}

/**
 * Add new MLX chips (infrared temperature sensor)
 *
 * MLX chips address can be defined if there's only one available on I2C bus
 * I'm using in I/O expander (MCP23017) to power on or power off MLX chips
 * I power on the first MLX, set a new I2C address, then power it off, power on the second one, set a new I2C address, etc
 * MLX I2C address are then stored on EEPROM
 * DAWA has to be restarted
 *
 * @param
 * @return
 */
void autodetectMlx(void)
{
  char bufferMsg[15];
  char bufferMlx[2];
  uint8_t mlxAddress, mlxId = 0; // 8 to 16 bit
  for (uint8_t i = 0; i < MAX_MLX_SIZE; i++)
  {
    g_mlxAddresses[i] = 0x00; // Array which will contains all discovered MLX addresses. This array will be wrote on EEPROM at the end
  }
  for (uint8_t i = 0; i < MAX_MLX_SIZE; i++)
  {
    sprintf(bufferMlx, "%d", i + 1);
    strcpy(bufferMsg, LABEL_MLX_AUTODETECT);
    strcat(bufferMsg, bufferMlx);
    showMessage(bufferMsg, 1000, 0);
    TFT_display();                                     // Refresh TFT to print message
    mcp2EnableOneOutput(i);                            // Enable only one MCP output, all the others are disabled
    Adafruit_MLX90614 mynewmlx = Adafruit_MLX90614(0); // 0 = broadcast, only one MLX chip should be plugged/powered on on I2C bus
    mynewmlx.begin();
    mlxAddress = mynewmlx.readAddr(); // Read MLX I2C address (default = 0x5A)
#ifdef DEBUG
    DEBUG_PORT.print("Autodetect MLX on MCP port ");
    DEBUG_PORT.print(i);
    DEBUG_PORT.print(" > addr=");
    DEBUG_PORT.print(mlxAddress, HEX);
    DEBUG_PORT.print(", temp=");
    DEBUG_PORT.println(mynewmlx.readObjectTempC());
#endif
    if (mlxAddress != 0xFF)
    { // If we read something ...
      g_mlxAddresses[mlxId] = FIRST_MLX_ADDRESS + mlxId;
      mynewmlx.writeAddr(g_mlxAddresses[mlxId]); // Set custom I2C address
#ifdef DEBUG
      DEBUG_PORT.print("Change MLX addr to : ");
      DEBUG_PORT.print(mynewmlx.readAddr());
      DEBUG_PORT.print(" (should be : ");
      DEBUG_PORT.print(g_mlxAddresses[mlxId]);
      DEBUG_PORT.println(")");
#endif
      mlxId++;
    }
  }
  if (EEPROM_writeAnything(EEPROM_MLX_ADDR, g_mlxAddresses) == sizeof(g_mlxAddresses)) // Save to EEPROM MLX addresses
  {
    showMessage(LABEL_MLX_AUTODETECT_OK, 2000, 1);
  }
  else
  {
    showMessage(LABEL_MLX_AUTODETECT_NOK, 2000, 2);
  }
  mcp2EnableOneOutput(8); // Re-enable all outputs on MCP2
}

/**
 * Control MCP2 outputs
 *
 * If specified ID is between 0 and 7 the concerned output is enabled
 * If specified ID = 8 then enable the 8 outputs
 *
 * @param output id to enable
 * @return
 */
void mcp2EnableOneOutput(uint8_t idOutput)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    if (i == idOutput || idOutput == 8)
    {
      MCP2.digitalWrite(i, HIGH);
    }
    else
    {
      MCP2.digitalWrite(i, LOW);
    }
    delay(50);
  }
}

/**
 * Import tracks from SDCARD to EEPROM
 *
 * Read the file LABEL_TRACKFILENAME on the SDCARD
 * Put the data in a "Track" structure then write it on EEPROM
 *
 * @param
 * @return number of tracks imported
 */
uint8_t importTracksFromSd(void)
{
  Track myTrack;
  uint8_t trackInc;
  File trackFile;
  trackFile = sd.open(LABEL_TRACKFILENAME, FILE_READ);
  if (trackFile)
  {
    trackInc = 0;
    while (trackFile.available())
    {
      // Read data
      csvReadInt16(&trackFile, &myTrack.trackId, csvDelim);
      csvReadText(&trackFile, myTrack.trackName, sizeof(myTrack.trackName), csvDelim); // One line per track : "1;CAROLE;489799930;25224350;489800230;25226330" (<trackID>;<trackname>;<startline_a_lat>;<startline_a_lon>;<startline_b_lat>;<startline_b_lon>)
      csvReadInt32(&trackFile, &myTrack.trackFlineLat1, csvDelim);                     // Points A & B should be at the left and at the right of the finishline (a few meters)
      csvReadInt32(&trackFile, &myTrack.trackFlineLon1, csvDelim);
      csvReadInt32(&trackFile, &myTrack.trackFlineLat2, csvDelim);
      csvReadInt32(&trackFile, &myTrack.trackFlineLon2, csvDelim);
      myTrack.trackDistance = 0; // Distance is not stored in EEPROM this is useless

      // Check if there is less than MAX_TRACKS to import AND if there is enough space on EEPROM
      if (((trackInc + 1) <= MAX_TRACKS) && (((trackInc + 1) * sizeof(myTrack)) < EEPROM_SIZE - EEPROM_FIRST_TRACK_ADDR))
      {
        // Write data to EEPROM
        EEPROM_writeAnything(EEPROM_FIRST_TRACK_ADDR + (trackInc * sizeof(myTrack)), myTrack);
        trackInc++;
      }
      else
      {
        if ((trackInc + 1) <= MAX_TRACKS)
        {
          showMessage(LABEL_TOO_MANY_TRACKS, 2000, 2);
          TFT_display(); // Refresh TFT to print message
          delay(2000);
          break; // Too many tracks, stop importing
        }
        else
        {
          showMessage(LABEL_EEPROM_FULL, 2000, 2);
          TFT_display(); // Refresh TFT to print message
          delay(2000);
          break; // EEPROM is full, stop importing
        }
      }
    }
    trackFile.close();
    showMessage(LABEL_IMPORT_TRACKS_OK, 2000, 1);
    return trackInc;
  }
  else
  {
    showMessage(LABEL_LOG_NOTRKFILE, 2000, 2);
    return 0;
  }
}

/**
 * Load entire track list from EEPROM to RAM
 *
 * Load entire track list from EEPROM into an array of tracks (g_tracksList)
 * Distance from your position is dynamically calculated here if GPS fix is valid
 *
 * @param
 * @return
 */
void loadTracksFromEeprom(void)
{
  uint8_t i;
  for (i = 0; i < g_trackQty; i++)
  {
    EEPROM_readAnything(EEPROM_FIRST_TRACK_ADDR + (i * sizeof(g_tracksList[i])), g_tracksList[i]);
    if (fix_data.valid.location)
    {
      refreshTrackDistance();
    }
  }
}

/**
 * Load selected Track from track list based on track ID
 *
 * Load specific track data based on his ID
 * The track is loaded in a global var (g_currentTrack)
 * if track ID is not found the g_nullTrack is loaded
 *
 * @param trackId the track ID specified in the csv file imported from sdcard
 * @return
 */
void loadSelectedTrack(int16_t trackId)
{
  uint8_t i;
  g_currentTrack = g_nullTrack;
  if (trackId >= 0)
  {
    for (i = 0; i < g_trackQty; i++)
    {
      if (g_tracksList[i].trackId == trackId)
      {
        g_currentTrack = g_tracksList[i];
      }
    }
  }
}

/**
 * Get track name corresponding to track ID
 *
 * Track name is copied in global var "g_trackName"
 * if track ID is not found a label error is copied in the same global var
 *
 * @param trackId the track ID specified in the csv file imported from sdcard
 * @return
 */
void getTrackName(int16_t trackId)
{
  uint8_t i;
  strcpy(g_trackName, LABEL_UNKNOWN_TRACK);
  if (trackId >= 0)
  {
    for (i = 0; i < g_trackQty; i++)
    {
      if (g_tracksList[i].trackId == trackId)
      {
        strcpy(g_trackName, g_tracksList[i].trackName);
      }
    }
  }
}

/**
 * Return ID of the nearest track from where you are now
 *
 * Get the ID of the nearest track from your position
 * Read all tracks data from an array of tracks (g_tracksList) and check if the distance is <= MAX_TRACK_DISTANCE
 * The last track which meets the condition is returned
 *
 * @param
 * @return track ID (-1 if not found)
 */
int16_t autoselectTrack(void)
{
  uint8_t i;
  int16_t nearestTrackId = -1;
  if (fix_data.valid.location)
  {
    for (i = 0; i < g_trackQty; i++)
    {
      refreshTrackDistance();
      if (g_tracksList[i].trackDistance <= MAX_TRACK_DISTANCE)
      {
        nearestTrackId = g_tracksList[i].trackId;
      }
    }
  }
  return nearestTrackId;
}

/**
 * Refresh the distance to all the tracks from where you are now
 *
 * Update the distance to the tracks from where you are now
 * Distances are calculated with Haversine formula and GPS coordinates of your position and track finish line
 *
 * @param
 * @return
 */
void refreshTrackDistance(void)
{
  uint8_t i;
  if (fix_data.valid.location)
  {
    for (i = 0; i < g_trackQty; i++)
    {
      g_tracksList[i].trackDistance = gpsDistance(fix_data.latitudeL(), fix_data.longitudeL(), g_tracksList[i].trackFlineLat1, g_tracksList[i].trackFlineLon1) / 1000;
    }
  }
}

/**
 * Update best lap
 *
 * Parse all laps in "g_lapsList" and get the best laptime
 * Parse again all laps in "g_lapsList" and set for each one "lapIsBest" value (true or false)
 *
 * @param
 * @return
 */
void updateBestLap(void)
{
  uint8_t i;
  float bestLap = 999.99;
  for (i = 1; i <= g_lapQty; i++)
  {
    if (g_lapsList[i].lapTime < bestLap)
    {
      bestLap = g_lapsList[i].lapTime;
    }
  }
  for (i = 1; i <= g_lapQty; i++)
  {
    if (g_lapsList[i].lapTime == bestLap)
    {
      g_lapsList[i].lapIsBest = true;
    }
    else
    {
      g_lapsList[i].lapIsBest = false;
    }
  }
}

/**
 * SERCOM5_Handler
 *
 * Secondary hardware serial for bluetooth
 *
 * @param
 * @return
 */
void SERCOM5_Handler(void)
{
  OBD2_PORT.IrqHandler();
}