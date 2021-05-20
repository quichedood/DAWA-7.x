#include <Arduino.h>           // Arduino library
#include <Adafruit_MCP23017.h> // I/O Expander MCP23017 (https://github.com/adafruit/Adafruit-MCP23017-Arduino-Library)
#include <SdFat.h>             // Greiman/SdFat library (https://github.com/greiman/SdFat)
#include <ELMduino.h>          // ELM327 OBD-II library (https://github.com/PowerBroker2/ELMduino)
#include <MLX90614.h>          //MLX90614 Infrared temperature sensor (https://github.com/jfitter/MLX90614)
#include <NMEAGPS.h>           // NeoGPS library (https://github.com/SlashDevin/NeoGPS) - In "NeoTime.h" > static const uint16_t s_epoch_year = POSIX_EPOCH_YEAR; static const uint8_t  s_epoch_weekday = POSIX_EPOCH_WEEKDAY;
#include <extEEPROM.h>         // EEPROM library (http://github.com/PaoloP74/extEEPROM)
#include <helper.h>            // DAWA functions helper

/**************************************************************
  #################################################
  ############## Functions##### ###################
  #################################################
**************************************************************/

/**************************************************************
  #initError > Error code on 8 bits (1 > 255)
**************************************************************/
void initError(uint8_t errCode = 0)
{
  delay(2000); // so we can look the error on the display
  // Poweroff all 8 leds
  MCP1.digitalWrite(mcp1Led1, HIGH);
  MCP1.digitalWrite(mcp1Led2, HIGH);
  MCP1.digitalWrite(mcp1Led3, HIGH);
  MCP1.digitalWrite(mcp1Led4, HIGH);
  MCP1.digitalWrite(mcp1Led5, HIGH);
  MCP1.digitalWrite(mcp1Led6, HIGH);
  MCP1.digitalWrite(mcp1Led7, HIGH);
  MCP1.digitalWrite(mcp1Led8, HIGH);

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

void eepromReload(void)
{
  // Read RPM correction ratio
  EEPROM_readAnything(28, rpmCorrectionRatio) == sizeof(rpmCorrectionRatio);

  // Read RPM flywheel teeth
  EEPROM_readAnything(29, rpmFlywheelTeeth) == sizeof(rpmFlywheelTeeth);

  // Read max values for analogic inputs (throttle, anaopt1, anaopt2) > 3x16 bits
  EEPROM_readAnything(31, inAnaThrottleMax) == sizeof(inAnaThrottleMax);
  EEPROM_readAnything(33, inAnaOpt1Max) == sizeof(inAnaOpt1Max);
  EEPROM_readAnything(35, inAnaOpt2Max) == sizeof(inAnaOpt2Max);
  EEPROM_readAnything(37, inAnaOpt1Min) == sizeof(inAnaOpt1Min);
  EEPROM_readAnything(39, inAnaOpt2Min) == sizeof(inAnaOpt2Min);
  //TODO

  // Read GEAR calibration data > 16 bits
  EEPROM_readAnything(41, inAnaGearCalib) == sizeof(inAnaGearCalib);

  // Read saved MLX I2C Address (infrared temp sensors) > 6x8 bits
  EEPROM_readAnything(60, mlxAddresses) == sizeof(mlxAddresses);
  for (uint8_t i = 0; i < MAX_MLX_SIZE; i++)
  {
    if (mlxAddresses[i] != 0x00)
    {
      mlx[i].begin();
    }
  }

  // Read enabled digital inputs > 8 bits
  EEPROM_readAnything(70, enDigitalInputsBits) == sizeof(enDigitalInputsBits);

  // Read enabled analog inputs > 8 bits
  EEPROM_readAnything(71, enAnalogInputsBits) == sizeof(enAnalogInputsBits);
}

/**************************************************************
  Read CSV file
  https://github.com/greiman/SdFat/blob/master/examples/ReadCsv/ReadCsv.ino

  Read a CSV file one field at a time.
  file - File to read.
  str - Character array for the field.
  size - Size of str array.
  delim - csv delimiter.
  return - negative value for failure.
        delimiter, '\n' or zero(EOF) for success.
**************************************************************/
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

int csvReadDouble(File *file, double *num, char delim)
{
  char buf[20];
  char *ptr;
  int rtn = csvReadText(file, buf, sizeof(buf), delim);
  if (rtn < 0)
    return rtn;
  *num = strtod(buf, &ptr);
  if (buf == ptr)
    return -3;
  while (isspace(*ptr))
    ptr++;
  return *ptr == 0 ? rtn : -4;
}

int csvReadFloat(File *file, float *num, char delim)
{
  double tmp;
  int rtn = csvReadDouble(file, &tmp, delim);
  if (rtn < 0)
    return rtn;
  // could test for too large.
  *num = tmp;
  return rtn;
}

/**************************************************************
  Get datetime for files creation/modification on SD Card
**************************************************************/
void dateTimeSd(uint16_t *date, uint16_t *time)
{
  *date = FAT_DATE(fix_data.dateTime.year + 2000, fix_data.dateTime.month, fix_data.dateTime.date); // return date using FAT_DATE macro to format fields
  *time = FAT_TIME(fix_data.dateTime.hours, fix_data.dateTime.minutes, fix_data.dateTime.seconds);  // return time using FAT_TIME macro to format fields
}

/**************************************************************
  #segIntersect > Calculate 2 line segments intersection
**************************************************************/
bool segIntersect(int32_t pos_now_lat, int32_t pos_now_lon, int32_t pos_prev_lat, int32_t pos_prev_lon, int32_t trackLat1, int32_t trackLon1, int32_t trackLat2, int32_t trackLon2, int32_t &pos_cross_lat, int32_t &pos_cross_lon)
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

/**************************************************************
  #gpsDistance > Calculate distance between 2 GPS coords (unit = meters) - haversine formula
**************************************************************/
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

/**************************************************************
  #stopLaptimer > Stop laptimer
**************************************************************/
void stopLaptimer(void)
{
  SdFile::dateTimeCallback(dateTimeSd);
  isRunning = false; // ... we stop recording
  logFile.close();   // Close file on SDcard
  lapFile.close();   // Close file on SDcard
  //OLED_PORT.clear();
}

/**************************************************************
  #startLaptimer > Start laptimer (init multiple var, auto select nearest track, create files on SD card)
**************************************************************/
void startLaptimer(void)
{
  SdFile::dateTimeCallback(dateTimeSd);
  if (fix_data.valid.location)
  { // We need GPS fix before starting
    //OLED_PORT.clear();
    //OLED_PORT.print(LABEL_AUTOSEL_TRK); OLED_PORT.println(F(" ..."));

    /**************************************************************
      Select nearest track from the file "TRACKS.csv" on sdcard
    **************************************************************/
    recordTrackData = false;
    trackFile = sd.open("TRACKS.csv", FILE_READ);
    if (trackFile)
    {
      while (trackFile.available())
      {
        csvReadUint16(&trackFile, &trackId, csvDelim);
        csvReadText(&trackFile, trackName, sizeof(trackName), csvDelim); // One line per track : "1;CAROLE;489799930;25224350;489800230;25226330" (<trackname>;<startline_a_lat>;<startline_a_lon>;<startline_b_lat>;<startline_b_lon>)
        csvReadInt32(&trackFile, &flineLat1, csvDelim);                  // Points A & B should be at the left and at the right of the finishline (a few meters)
        csvReadInt32(&trackFile, &flineLon1, csvDelim);
        csvReadInt32(&trackFile, &flineLat2, csvDelim);
        csvReadInt32(&trackFile, &flineLon2, csvDelim);

        // Calculate distance between 2 GPS coordinates
        coordsDistance = gpsDistance(fix_data.latitudeL(), fix_data.longitudeL(), flineLat1, flineLon1) / 1000;

        // If you are on a known track, then we select it
        if (coordsDistance <= maxTrackDistance)
        {
          recordTrackData = true;
          //OLED_PORT.print(trackName); OLED_PORT.print(F(" (")); OLED_PORT.print(coordsDistance, 1); OLED_PORT.print(LABEL_LOG_KM); OLED_PORT.println(F(")"));
          break; // Break here so last read values are the good ones !
        }
      }
      trackFile.close();
    }
    if (recordTrackData == false)
    {
      //OLED_PORT.println(LABEL_LOG_NOTRKFILE);
    }

    /**************************************************************
      Create new datafile : history file (append)
    **************************************************************/
    if (recordTrackData == true)
    { // No track = no history !
      sprintf(filename, "HISTORY.csv");
      if (historyFile.open(filename, O_CREAT | O_APPEND | O_WRITE))
      {
        historyFile.print(fix_data.dateTime);
        historyFile.print(F(";"));
        historyFile.print(fix_data.dateTime.year);
        if (fix_data.dateTime.month < 10)
          historyFile.print(F("0")); // Leading zeros
        historyFile.print(fix_data.dateTime.month);
        if (fix_data.dateTime.date < 10)
          historyFile.print(F("0")); // Leading zeros
        historyFile.print(fix_data.dateTime.date);
        historyFile.print(F("-"));
        if (fix_data.dateTime.hours < 10)
          historyFile.print(F("0")); // Leading zeros
        historyFile.print(fix_data.dateTime.hours);
        if (fix_data.dateTime.minutes < 10)
          historyFile.print(F("0")); // Leading zeros
        historyFile.print(fix_data.dateTime.minutes);
        if (fix_data.dateTime.seconds < 10)
          historyFile.print(F("0")); // Leading zeros
        historyFile.print(fix_data.dateTime.seconds);
        historyFile.print(F(";"));
        historyFile.println(trackId);
        historyFile.close(); // Close file on SDcard
      }
      else
      {
        initError(0);
      }
    }

    /**************************************************************
      Create new datafile : log file (create new)
    **************************************************************/
    sprintf(filename, "%02u%02u%02u-%02u%02u%02u.csv", fix_data.dateTime.year, fix_data.dateTime.month, fix_data.dateTime.date, fix_data.dateTime.hours, fix_data.dateTime.minutes, fix_data.dateTime.seconds);
    if (logFile.open(filename, O_CREAT | O_WRITE | O_EXCL))
    {
      // Time, distance and lap (always printed)
      logFile.print(LABEL_LOG_HEADER_1);

      // Digital inputs (printed if enabled)
      bitShift = B00000001;
      for (uint8_t i = 0; i < 4; i++)
      {
        tmpComp = bitShift & enDigitalInputsBits;
        if (tmpComp == bitShift)
        {
          logFile.print(digitalInputsLabel[i]);
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
          logFile.print(analogInputsLabel[i]);
          logFile.print(F(";"));
        }
        bitShift = bitShift << 1;
      }

      // KPH (GPS), Orientation, ambiant temperature (always printed)
      logFile.print(LABEL_LOG_HEADER_2);

      // Infrared temperature (printed if enabled)
      for (uint8_t i = 0; i < MAX_MLX_SIZE; i++)
      {
        if (mlxAddresses[i] != 0x00)
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
      initError(0);
    }

    /**************************************************************
      Create new datafile : laptime file (create new)
    **************************************************************/
    sprintf(filename, "%02u%02u%02u-%02u%02u%02u-LAPTIMES.csv", fix_data.dateTime.year, fix_data.dateTime.month, fix_data.dateTime.date, fix_data.dateTime.hours, fix_data.dateTime.minutes, fix_data.dateTime.seconds);
    if (lapFile.open(filename, O_CREAT | O_WRITE | O_EXCL))
    {
      //
    }
    else
    {
      initError(0);
    }

    /**************************************************************
      Init some vars
    **************************************************************/
    isRunning = true; // ... we start recording
    lapCounter = 0;   // Lap 0 (we start from paddocks)
    totalDistance = 0;
    addFinishLog = false;
  }
}

/**************************************************************
  #sendUBX > Send UBX commands to UBLOX GPS
**************************************************************/
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

/**************************************************************
  #timeAdd > Add a time [arg0] (Ex : 5.14) to another which is composed of seconds [arg1] and milliseconds [arg2]. Return seconds [arg3] and milliseconds [arg4]
**************************************************************/
void timeAdd(float timeSecCsec, int32_t endSec, int32_t endCsec, int32_t &returnSec, int32_t &returnCsec)
{
  returnSec = endSec + (int32_t)(timeSecCsec + (endCsec / 100.00));
  returnCsec = ((timeSecCsec + (endCsec / 100.00)) - (int32_t)(timeSecCsec + (endCsec / 100.00))) * 100;
}

/**************************************************************
  #timeSubstract > Substract a time [arg0][arg1] to another [arg2][arg3]. Return seconds [arg4] and milliseconds [arg6]
**************************************************************/
void timeSubstract(int32_t s1, int32_t cs1, int32_t s2, int32_t cs2, int32_t &returnSec, int32_t &returnCsec)
{
  returnCsec = cs1 - cs2;
  if (returnCsec < 0)
  {
    returnSec = (s1 - s2) - 1;
    returnCsec += 100;
  }
  else
  {
    returnSec = s1 - s2;
  }
}

/**************************************************************
  GPS time adjust
**************************************************************/
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

/**************************************************************
  #initADC > Initialize ADC (I2C)
**************************************************************/
uint8_t initADC()
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
  Wire.write(uint8_t(0x08));            // Channel Disable Register
  Wire.write(uint8_t(~enAnalogInputsBits)); // Enable or disable channels
  Wire.endTransmission();

  Wire.beginTransmission(0x1D);
  Wire.write(uint8_t(0x00)); // Configuration Register
  Wire.write(uint8_t(0x01)); // 00000001 (Start conversion)
  return Wire.endTransmission();
}

/**************************************************************
  #configureADC > configure ADC input bits (I2C)
**************************************************************/
uint8_t configureADC(uint8_t bits)
{
  Wire.beginTransmission(0x1D);
  Wire.write(uint8_t(0x00)); // Configuration Register
  Wire.write(uint8_t(0x00)); // 00000000 (Shutdown mode)
  Wire.endTransmission();

  Wire.beginTransmission(0x1D);
  Wire.write(uint8_t(0x08)); // Channel Disable Register
  Wire.write(uint8_t(~bits)); // Enable or disable channels
  Wire.endTransmission();

  Wire.beginTransmission(0x1D);
  Wire.write(uint8_t(0x00)); // Configuration Register
  Wire.write(uint8_t(0x01)); // 00000001 (Start conversion)
  return Wire.endTransmission();
}

/**************************************************************
  #readAdcValue > Read one ADC value
**************************************************************/
int16_t readAdcValue(uint8_t registerID)
{
  Wire.beginTransmission(0x1D); // Talk to ADC128D818 - A0 and A1 to GND
  Wire.write(registerID);       // Busy Status Register (Bit 1 = Not Ready > Waiting for the power-up sequence to end)
  Wire.endTransmission();
  Wire.requestFrom(0x1D, 2); // Read Register 0x20
  if (Wire.available() == 2)
  {
    return (Wire.read() << 4 | Wire.read()); // We read a 12 bits value (2x 8 bits I2C reads)
  }
  return -1;
}

/**************************************************************
  #readAdcValues > Read all ADC values (8)
  Return ADC value or -1 if disable by user
**************************************************************/
void readAdcValues(uint16_t anaValues[])
{
  uint8_t bitShift = B00000001, tmpComp;
  uint8_t registerID = 0x20; // First register ID is 0x20
  for (uint8_t i = 0; i < 8; i++)
  {
    tmpComp = bitShift & enAnalogInputsBits;
    if (tmpComp == bitShift)
    {
      anaValues[i] = readAdcValue(registerID);
    }
    else
    {
      anaValues[i] = 0;
    }
    registerID++;
    bitShift = bitShift << 1;
  }
}

/**************************************************************
  SERCOM5_Handler : Secondary hardware serial for bluetooth
**************************************************************/
void SERCOM5_Handler(void)
{
  OBD2_PORT.IrqHandler();
}