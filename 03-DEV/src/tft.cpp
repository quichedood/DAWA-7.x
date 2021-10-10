/*
@file    tft.c
@brief   TFT handling functions for EVE_Test project
@version 1.13
@date    2020-09-05
@author  Rudolph Riedel

@section History

1.0
- initial release

1.1
- added a simple .png image that is drawn beneath the clock just as example of how it could be done

1.2
- replaced the clock with a scrolling line-strip
- moved the .png image over to the font-test section to make room
- added a scrolling bar display
- made scrolling depending on the state of the on/off button
- reduced the precision of VERTEX2F to 1 Pixel with VERTEXT_FORMAT() (FT81x only) to avoid some pointless "*16" multiplications

1.3
- adapted to release 3 of Ft8xx library with EVE_ and ft_ prefixes changed to EVE_
- removed "while (EVE_busy());" lines after EVE_cmd_execute() since it does that by itself now
- removed "EVE_cmd_execute();" line after EVE_cmd_loadimage(MEM_PIC1, EVE_OPT_NODL, pngpic, pngpic_size); as EVE_cmd_loadimage() executes itself now

1.4
- added num_profile_a and num_profile_b for simple profiling
- utilised EVE_cmd_start()
- some general cleanup and house-keeping to make it fit for release

1.5
- simplified the example layout a lot and made it to scale with all display-sizes

1.6
- a bit of house-keeping after trying a couple of things

1.7
- added a display for the amount of bytes generated in the display-list by the command co-pro

1.8
- moved the color settings from EVE_config.h to here

1.9
- changed FT8_ prefixes to EVE_

1.10
- some cleanup

1.11
- updated to be more similar to what I am currently using in projects

1.12
- minor cleanup, switched from EVE_get_touch_tag(1); to EVE_memRead8(REG_TOUCH_TAG);
- added some more sets of calibration data from my displays

1.13
- adapted to V5
- new logo

 */

/*
tftScreenId reference (BUTTON NAME/TAG_ID)
10 : default/start screen (SHUTDOWN/10, SETUP/11, START/12)
20 : Screen when running laptimer (STOP/20, FAKELAP/21)
50 : Setup screen (BACK/50, DIGITAL/51, ANALOG/52, OBD/53, TEMP./54, LED/55, SDCARD/56, GPS/57, TRACKS/58, EEPROM/59)
60 : DIGITAL setup (BACK/60)
70 : ANALOG setup (BACK/70)
80 : OBD setup (BACK/80)
90 : TEMP setup (BACK/90, AUTODETECT/91)
100 : LEDs setup
110 : SDCARD setup (BACK/110)
120 : GPS setup (BACK/120)
130 : TRACKS setup (BACK/130)
140 : EEPROM setup (BACK/140, RESET EEPROM/141, LOAD DEFAULT/142

*/

#include <EVE.h>
#include <Arduino.h>           // Arduino library
#include <Adafruit_MCP23017.h> // I/O Expander MCP23017 (https://github.com/adafruit/Adafruit-MCP23017-Arduino-Library)
#include <SdFat.h>             // Greiman/SdFat library (https://github.com/greiman/SdFat)
#include <ELMduino.h>          // ELM327 OBD-II library (https://github.com/PowerBroker2/ELMduino)
#include <Adafruit_MLX90614.h> // MLX90614 Infrared temperature sensor (https://github.com/jfitter/MLX90614)
#include <NMEAGPS.h>           // NeoGPS library (https://github.com/SlashDevin/NeoGPS) - In "NeoTime.h" > static const uint16_t s_epoch_year = POSIX_EPOCH_YEAR; static const uint8_t  s_epoch_weekday = POSIX_EPOCH_WEEKDAY;
#include <extEEPROM.h>         // EEPROM library (http://github.com/PaoloP74/extEEPROM)
#include <helper.h>            // DAWA functions helper

/* some pre-definded colors */
#define RED 0xff0000UL
#define ORANGE 0xffa500UL
#define GREEN 0x00ff00UL
#define BLUE 0x0000ffUL
#define BLUE_1 0x5dade2L
#define YELLOW 0xffff00UL
#define PINK 0xff00ffUL
#define PURPLE 0x800080UL
#define WHITE 0xffffffUL
#define BLACK 0x000000UL

/* memory-map defines */
#define MEM_FONT_01 0
#define MEM_FONT_02 4352
#define MEM_FONT_03 8640
#define MEM_FONT_04 8832
#define MEM_PICT_01 9024 //4352 //5696 //192 //4288 /* start of 100x100 pixel test image, ARGB565, needs 20000 bytes of memory */

#define MEM_DL_STATIC (EVE_RAM_G_SIZE - 4096) /* 0xff000 - start-address of the static part of the display-list, upper 4k of gfx-mem */

uint32_t num_dl_static; /* amount of bytes in the static part of our display-list */
uint8_t tft_active = 0;
uint16_t num_profile_a, num_profile_b;
uint32_t PST_Cmd_n_SpeedMechCmd;
uint32_t tracker_val;

bool analog1Enable = 0;

#define LAYOUT_Y1 66

void touch_calibrate(void)
{
  /* send pre-recorded touch calibration values, depending on the display the code is compiled for */
#if defined(EVE_RiTFT43)
  EVE_memWrite32(REG_TOUCH_TRANSFORM_A, 33172);
  EVE_memWrite32(REG_TOUCH_TRANSFORM_B, 123);
  EVE_memWrite32(REG_TOUCH_TRANSFORM_C, 4293527846);
  EVE_memWrite32(REG_TOUCH_TRANSFORM_D, 4294967163);
  EVE_memWrite32(REG_TOUCH_TRANSFORM_E, 4294947501);
  EVE_memWrite32(REG_TOUCH_TRANSFORM_F, 18950047);
#endif

  /* activate this if you are using a module for the first time or if you need to re-calibrate it */
  /* write down the numbers on the screen and either place them in one of the pre-defined blocks above or make a new block */
#if 0
  /* calibrate touch and displays values to screen */
  EVE_cmd_dl(CMD_DLSTART);
  EVE_cmd_dl(DL_CLEAR_RGB | BLACK);
  EVE_cmd_dl(DL_CLEAR | CLR_COL | CLR_STN | CLR_TAG);
  EVE_cmd_text((EVE_HSIZE / 2), 50, 26, EVE_OPT_CENTER, "Please tap on the dot.");
  EVE_cmd_calibrate();
  EVE_cmd_dl(DL_DISPLAY);
  EVE_cmd_dl(CMD_SWAP);
  EVE_cmd_execute();

  uint32_t touch_a, touch_b, touch_c, touch_d, touch_e, touch_f;

  touch_a = EVE_memRead32(REG_TOUCH_TRANSFORM_A);
  touch_b = EVE_memRead32(REG_TOUCH_TRANSFORM_B);
  touch_c = EVE_memRead32(REG_TOUCH_TRANSFORM_C);
  touch_d = EVE_memRead32(REG_TOUCH_TRANSFORM_D);
  touch_e = EVE_memRead32(REG_TOUCH_TRANSFORM_E);
  touch_f = EVE_memRead32(REG_TOUCH_TRANSFORM_F);

  EVE_cmd_dl(CMD_DLSTART);
  EVE_cmd_dl(DL_CLEAR_RGB | BLACK);
  EVE_cmd_dl(DL_CLEAR | CLR_COL | CLR_STN | CLR_TAG);
  EVE_cmd_dl(TAG(0));

  EVE_cmd_text(5, 15, 26, 0, "TOUCH_TRANSFORM_A:");
  EVE_cmd_text(5, 30, 26, 0, "TOUCH_TRANSFORM_B:");
  EVE_cmd_text(5, 45, 26, 0, "TOUCH_TRANSFORM_C:");
  EVE_cmd_text(5, 60, 26, 0, "TOUCH_TRANSFORM_D:");
  EVE_cmd_text(5, 75, 26, 0, "TOUCH_TRANSFORM_E:");
  EVE_cmd_text(5, 90, 26, 0, "TOUCH_TRANSFORM_F:");

  EVE_cmd_setbase(16L);
  EVE_cmd_number(310, 15, 26, EVE_OPT_RIGHTX | 8, touch_a);
  EVE_cmd_number(310, 30, 26, EVE_OPT_RIGHTX | 8, touch_b);
  EVE_cmd_number(310, 45, 26, EVE_OPT_RIGHTX | 8, touch_c);
  EVE_cmd_number(310, 60, 26, EVE_OPT_RIGHTX | 8, touch_d);
  EVE_cmd_number(310, 75, 26, EVE_OPT_RIGHTX | 8, touch_e);
  EVE_cmd_number(310, 90, 26, EVE_OPT_RIGHTX | 8, touch_f);

  EVE_cmd_dl(DL_DISPLAY); /* instruct the graphics processor to show the list */
  EVE_cmd_dl(CMD_SWAP); /* make this list active */
  EVE_cmd_execute();

  while (1);
#endif
}

void initStaticBackground()
{
  EVE_cmd_dl(CMD_DLSTART);      /* Start the display list */
  EVE_cmd_dl(TAG(0));           /* do not use the following objects for touch-detection */
  EVE_cmd_bgcolor(0x00c0c0c0);  /* light grey */
  EVE_cmd_dl(VERTEX_FORMAT(0)); /* reduce precision for VERTEX2F to 1 pixel instead of 1/16 pixel default */

  /* display the logo */
  EVE_cmd_dl(DL_COLOR_RGB | TFTDEFAULTFONTCOLOR);
  EVE_cmd_dl(DL_BEGIN | EVE_BITMAPS);
  EVE_cmd_setbitmap(MEM_PICT_01, EVE_COMPRESSED_RGBA_ASTC_4x4_KHR, 480, 272);
  EVE_cmd_dl(VERTEX2F(0, 0));
  EVE_cmd_setfont2(12, MEM_FONT_01, 32);
  EVE_cmd_setfont2(13, MEM_FONT_02, 32);
  EVE_cmd_setfont2(14, MEM_FONT_03, 32);
  EVE_cmd_setfont2(15, MEM_FONT_04, 32);
  EVE_cmd_dl(DL_END);

  while (EVE_busy())
    ;
  num_dl_static = EVE_memRead16(REG_CMD_DL);

  EVE_cmd_memcpy(MEM_DL_STATIC, EVE_RAM_DL, num_dl_static);
  while (EVE_busy())
    ;
}

void TFT_init(void)
{
  if (EVE_init() != 0)
  {
    tft_active = 1;
    EVE_memWrite8(REG_PWM_DUTY, 0x80); /* setup backlight, range is from 0 = off to 0x80 = max */
    touch_calibrate();

    EVE_init_flash();
    /*
    008.bin
    unified.blob                                       : 0      : 4096
    monoMMM_5_12_ASTC.glyph                            : 4096   : 129536
    monoMMM_5_12_ASTC.xfont                            : 133632 : 4352
    monoMMM_5_16_ASTC.glyph                            : 137984 : 129536
    monoMMM_5_16_ASTC.xfont                            : 267520 : 4288
    monoMMM_5_72_ASTC.glyph                            : 271808 : 57344
    monoMMM_5_72_ASTC.xfont                            : 329152 : 192
    wallpaper_480x272_COMPRESSED_RGBA_ASTC_4x4_KHR.raw : 329344 : 130560

    unified.blob                                    : 0      : 4096
    monoMMM_5_12_ASTC.glyph                         : 4096   : 129536
    monoMMM_5_12_ASTC.xfont                         : 133632 : 4352
    monoMMM_5_16_ASTC.glyph                         : 137984 : 129536
    monoMMM_5_16_ASTC.xfont                         : 267520 : 4288
    monoMMM_5_72_ASTC.glyph                         : 271808 : 57344
    monoMMM_5_72_ASTC.xfont                         : 329152 : 192
    carbon_480x272_COMPRESSED_RGBA_ASTC_4x4_KHR.raw : 329344 : 130560
    */
    EVE_cmd_flashread(MEM_FONT_01, 133632, 4352); /* copy from FLASH (read 4288 bits starting offset 84608) to G-RAM (MEM_FONT_01/.xfont) */
    EVE_cmd_flashread(MEM_FONT_02, 267520, 4288);
    EVE_cmd_flashread(MEM_FONT_03, 329152, 192);
    EVE_cmd_flashread(MEM_PICT_01, 329344, 130560);

    initStaticBackground();
  }
}

uint16_t toggle_state = 0;

void TFT_display_setup_buttons(void)
{
  EVE_cmd_dl(DL_COLOR_RGB | WHITE);
  EVE_cmd_fgcolor(0x00c0c0c0); /* some grey */
  EVE_cmd_dl(TAG(50));
  EVE_cmd_button_var(TFT_BUTTON_BORDER_SIZE, EVE_VSIZE - TFT_BUTTON_BORDER_SIZE - TFT_BUTTON_V_SIZE, TFT_BUTTON_H_SIZE, TFT_BUTTON_V_SIZE, 13, EVE_OPT_FORMAT, MENU_BACK, 0);
  EVE_cmd_dl(TAG(51));
  EVE_cmd_button_var(EVE_HSIZE / 4 - TFT_BUTTON_H_SIZE / 2, TFT_BUTTON_V_SIZE * 2, TFT_BUTTON_H_SIZE, TFT_BUTTON_V_SIZE, 13, EVE_OPT_FORMAT, MENU_DIGITAL, 0);
  EVE_cmd_dl(TAG(52));
  EVE_cmd_button_var(EVE_HSIZE / 4 - TFT_BUTTON_H_SIZE / 2, TFT_BUTTON_V_SIZE * 4, TFT_BUTTON_H_SIZE, TFT_BUTTON_V_SIZE, 13, EVE_OPT_FORMAT, MENU_ANALOG, 0);
  EVE_cmd_dl(TAG(53));
  EVE_cmd_button_var(EVE_HSIZE / 4 - TFT_BUTTON_H_SIZE / 2, TFT_BUTTON_V_SIZE * 6, TFT_BUTTON_H_SIZE, TFT_BUTTON_V_SIZE, 13, EVE_OPT_FORMAT, MENU_OBD, 0);
  EVE_cmd_dl(TAG(54));
  EVE_cmd_button_var(EVE_HSIZE * 2 / 4 - TFT_BUTTON_H_SIZE / 2, TFT_BUTTON_V_SIZE * 2, TFT_BUTTON_H_SIZE, TFT_BUTTON_V_SIZE, 13, EVE_OPT_FORMAT, MENU_TEMP, 0);
  EVE_cmd_dl(TAG(55));
  EVE_cmd_button_var(EVE_HSIZE * 2 / 4 - TFT_BUTTON_H_SIZE / 2, TFT_BUTTON_V_SIZE * 4, TFT_BUTTON_H_SIZE, TFT_BUTTON_V_SIZE, 13, EVE_OPT_FORMAT, MENU_GENERAL, 0);
  EVE_cmd_dl(TAG(56));
  EVE_cmd_button_var(EVE_HSIZE * 2 / 4 - TFT_BUTTON_H_SIZE / 2, TFT_BUTTON_V_SIZE * 6, TFT_BUTTON_H_SIZE, TFT_BUTTON_V_SIZE, 13, EVE_OPT_FORMAT, MENU_SDCARD, 0);
  EVE_cmd_dl(TAG(57));
  EVE_cmd_button_var(EVE_HSIZE * 3 / 4 - TFT_BUTTON_H_SIZE / 2, TFT_BUTTON_V_SIZE * 2, TFT_BUTTON_H_SIZE, TFT_BUTTON_V_SIZE, 13, EVE_OPT_FORMAT, MENU_GPS, 0);
  EVE_cmd_dl(TAG(58));
  EVE_cmd_button_var(EVE_HSIZE * 3 / 4 - TFT_BUTTON_H_SIZE / 2, TFT_BUTTON_V_SIZE * 4, TFT_BUTTON_H_SIZE, TFT_BUTTON_V_SIZE, 13, EVE_OPT_FORMAT, MENU_TRACKS, 0);
  EVE_cmd_dl(TAG(59));
  EVE_cmd_button_var(EVE_HSIZE * 3 / 4 - TFT_BUTTON_H_SIZE / 2, TFT_BUTTON_V_SIZE * 6, TFT_BUTTON_H_SIZE, TFT_BUTTON_V_SIZE, 13, EVE_OPT_FORMAT, MENU_EEPROM, 0);
  EVE_cmd_dl(TAG(0));
}

void TFT_display_header(void)
{
  // Display version
  EVE_cmd_text_var(TFT_DEFAULT_BORDER_H_SIZE, TFT_DEFAULT_BORDER_V_SIZE, 12, EVE_OPT_FORMAT, LABEL_VERSION, 0);

  // Display date/time
  EVE_cmd_text_var(EVE_HSIZE - TFT_DEFAULT_BORDER_H_SIZE, TFT_DEFAULT_BORDER_V_SIZE, 12, EVE_OPT_FORMAT | EVE_OPT_RIGHTX, "%02u/%02u/%02u %02u:%02u:%02u", 6, fix_data.dateTime.date, fix_data.dateTime.month, fix_data.dateTime.year, fix_data.dateTime.hours, fix_data.dateTime.minutes, fix_data.dateTime.seconds);

  // Display GPS signal quality
  if (fix_data.status < 3)
  {
    EVE_color_rgb(RED);
  }
  else
  {
    EVE_color_rgb(GREEN);
  }
  EVE_cmd_text_var(EVE_HSIZE - (TFT_DEFAULT_BORDER_H_SIZE + 150), TFT_DEFAULT_BORDER_V_SIZE, 12, EVE_OPT_FORMAT | EVE_OPT_RIGHTX, "QUAL:%u/4", 1, fix_data.status);
  EVE_color_rgb(TFTDEFAULTFONTCOLOR);

  // Display number of GPS acquired sattelites
  if (fix_data.satellites < 6)
  {
    EVE_color_rgb(RED);
  }
  else
  {
    EVE_color_rgb(GREEN);
  }
  EVE_cmd_text_var(EVE_HSIZE - (TFT_DEFAULT_BORDER_H_SIZE + 225), TFT_DEFAULT_BORDER_V_SIZE, 12, EVE_OPT_FORMAT | EVE_OPT_RIGHTX, "SAT:%u", 1, fix_data.satellites);
  EVE_color_rgb(TFTDEFAULTFONTCOLOR);

  // Display error/information
  if (msgLabel != "")
  {
    switch (msgType)
    {
    case 1:
      EVE_color_rgb(GREEN);
      break;
    case 2:
      EVE_color_rgb(RED);
      break;
    default:
      EVE_color_rgb(TFTDEFAULTFONTCOLOR);
    }
    EVE_cmd_text(EVE_HSIZE / 2, EVE_VSIZE / 2, TFT_FONT_01_SIZE, EVE_OPT_FORMAT | EVE_OPT_CENTERX | EVE_OPT_CENTERY, msgLabel);
    EVE_color_rgb(TFTDEFAULTFONTCOLOR);
  }
}

/* check for touch events and setup vars for TFT_display() */
void TFT_touch(void)
{
  uint8_t tag;
  static uint8_t track = 0;
  static uint8_t toggle_lock = 0;

  if (EVE_busy())
  { // is EVE still processing the last display list?
    return;
  }

  tag = EVE_memRead8(REG_TOUCH_TAG); // read the value for the first touch point
  uint32_t touchtest = EVE_memRead32(REG_TOUCH_RAW_XY);

  switch (tag)
  {
  case 0:
    toggle_lock = 0;

    // sliders
    if (touchtest == 0xffffffff) /* display is not touched anymore */
    {
      if (track != 0)
      {
        EVE_cmd_track(0, 0, 0, 0, 0); // stop tracking
        track = 0;
      }
    }
    break;

  case 10: // OFF
    if (toggle_lock == 0)
    {
      digitalWrite(powerState, LOW);
    }
    break;
  case 11: // SETUP
    if (toggle_lock == 0)
    {
      tftScreenId = 50;
      toggle_lock = 1;
      /*if (toggle_state == 0)
      {
        toggle_state = EVE_OPT_FLAT;
      }
      else
      {
        toggle_state = 0;
      }*/
    }
    break;
  case 12: // START
    if (toggle_lock == 0)
    {
      tftScreenId = 20;
      toggle_lock = 1;
      if (startLaptimer() == 1)
      {
        tftScreenId = 10;
      }
    }
    break;
  case 20: // STOP
    if (toggle_lock == 0)
    {
      tftScreenId = 10;
      toggle_lock = 1;
      stopLaptimer();
    }
    break;
  case 21: // FAKELAP
    if (toggle_lock == 0)
    {
      toggle_lock = 1;
      fakeLap = true;
    }
    break;
  case 50: // Setup screen > Back to start screen
    if (toggle_lock == 0)
    {
      tftScreenId = 10;
      toggle_lock = 1;
    }
    break;
  case 51: // Setup screen > DIGITAL setup
    if (toggle_lock == 0)
    {
      tftScreenId = 60;
      toggle_lock = 1;
    }
    break;
  case 52: // Setup screen > ANALOG setup
    if (toggle_lock == 0)
    {
      tftScreenId = 70;
      toggle_lock = 1;
    }
    break;
  case 53: // Setup screen > OBD setup
    if (toggle_lock == 0)
    {
      tftScreenId = 80;
      toggle_lock = 1;
    }
    break;
  case 54: // Setup screen > TEMP sensors setup
    if (toggle_lock == 0)
    {
      tftScreenId = 90;
      toggle_lock = 1;
    }
    break;
  case 55: // Setup screen > GENERAL setup
    if (toggle_lock == 0)
    {
      tftScreenId = 100;
      toggle_lock = 1;
    }
    break;
  case 56: // Setup screen > SDCARD setup
    if (toggle_lock == 0)
    {
      tftScreenId = 110;
      toggle_lock = 1;
    }
    break;
  case 57: // Setup screen > GPS setup
    if (toggle_lock == 0)
    {
      tftScreenId = 120;
      toggle_lock = 1;
    }
    break;
  case 58: // Setup screen > TRACKS setup
    if (toggle_lock == 0)
    {
      tftScreenId = 130;
      toggle_lock = 1;
    }
    break;
  case 59: // Setup screen > EEPROM setup
    if (toggle_lock == 0)
    {
      tftScreenId = 140;
      toggle_lock = 1;
    }
    break;
  case 61: // DIGITAL setup > Set ON/OFF digital input 1
    if (toggle_lock == 0)
    {
      toggle_lock = 1;
      if (bitRead(enDigitalInputsBits, 0) == true)
      {
        bitClear(enDigitalInputsBits, 0);
      }
      else
      {
        bitSet(enDigitalInputsBits, 0);
      }
    }
    break;
  case 62: // DIGITAL setup > Set ON/OFF digital input 2
    if (toggle_lock == 0)
    {
      toggle_lock = 1;
      if (bitRead(enDigitalInputsBits, 1) == true)
      {
        bitClear(enDigitalInputsBits, 1);
      }
      else
      {
        bitSet(enDigitalInputsBits, 1);
      }
    }
    break;
  case 63: // DIGITAL setup > Set ON/OFF digital input 3
    if (toggle_lock == 0)
    {
      toggle_lock = 1;
      if (bitRead(enDigitalInputsBits, 2) == true)
      {
        bitClear(enDigitalInputsBits, 2);
      }
      else
      {
        bitSet(enDigitalInputsBits, 2);
      }
    }
    break;
  case 64: // DIGITAL setup > Set ON/OFF digital input 4
    if (toggle_lock == 0)
    {
      toggle_lock = 1;
      if (bitRead(enDigitalInputsBits, 3) == true)
      {
        bitClear(enDigitalInputsBits, 3);
      }
      else
      {
        bitSet(enDigitalInputsBits, 3);
      }
    }
    break;
  case 65: // DIGITAL setup > Back to setup screen and save digital setup input states to EEPROM
    if (toggle_lock == 0)
    {
      EEPROM_writeAnything(70, enDigitalInputsBits) == sizeof(enDigitalInputsBits);
      tftScreenId = 50;
      toggle_lock = 1;
    }
    break;
  case 71: // ANALOG setup > Set ON/OFF analog input 1
    if (toggle_lock == 0)
    {
      toggle_lock = 1;
      if (bitRead(enAnalogInputsBits, 0) == true)
      {
        bitClear(enAnalogInputsBits, 0);
      }
      else
      {
        bitSet(enAnalogInputsBits, 0);
      }
      configureADC(enAnalogInputsBits);
    }
    break;
  case 72: // ANALOG setup > Set ON/OFF analog input 2
    if (toggle_lock == 0)
    {
      toggle_lock = 1;
      if (bitRead(enAnalogInputsBits, 1) == true)
      {
        bitClear(enAnalogInputsBits, 1);
      }
      else
      {
        bitSet(enAnalogInputsBits, 1);
      }
      configureADC(enAnalogInputsBits);
    }
    break;
  case 73: // ANALOG setup > Set ON/OFF analog input 3
    if (toggle_lock == 0)
    {
      toggle_lock = 1;
      if (bitRead(enAnalogInputsBits, 2) == true)
      {
        bitClear(enAnalogInputsBits, 2);
      }
      else
      {
        bitSet(enAnalogInputsBits, 2);
      }
      configureADC(enAnalogInputsBits);
    }
    break;
  case 74: // ANALOG setup > Set ON/OFF analog input 4
    if (toggle_lock == 0)
    {
      toggle_lock = 1;
      if (bitRead(enAnalogInputsBits, 3) == true)
      {
        bitClear(enAnalogInputsBits, 3);
      }
      else
      {
        bitSet(enAnalogInputsBits, 3);
      }
      configureADC(enAnalogInputsBits);
    }
    break;
  case 75: // ANALOG setup > Set ON/OFF analog input 5
    if (toggle_lock == 0)
    {
      toggle_lock = 1;
      if (bitRead(enAnalogInputsBits, 4) == true)
      {
        bitClear(enAnalogInputsBits, 4);
      }
      else
      {
        bitSet(enAnalogInputsBits, 4);
      }
      configureADC(enAnalogInputsBits);
    }
    break;
  case 76: // ANALOG setup > Set ON/OFF analog input 6
    if (toggle_lock == 0)
    {
      toggle_lock = 1;
      if (bitRead(enAnalogInputsBits, 5) == true)
      {
        bitClear(enAnalogInputsBits, 5);
      }
      else
      {
        bitSet(enAnalogInputsBits, 5);
      }
      configureADC(enAnalogInputsBits);
    }
    break;
  case 77: // ANALOG setup > Set ON/OFF analog input 7
    if (toggle_lock == 0)
    {
      toggle_lock = 1;
      if (bitRead(enAnalogInputsBits, 6) == true)
      {
        bitClear(enAnalogInputsBits, 6);
      }
      else
      {
        bitSet(enAnalogInputsBits, 6);
      }
      configureADC(enAnalogInputsBits);
    }
    break;
  case 78: // ANALOG setup > Set ON/OFF analog input 8
    if (toggle_lock == 0)
    {
      toggle_lock = 1;
      if (bitRead(enAnalogInputsBits, 7) == true)
      {
        bitClear(enAnalogInputsBits, 7);
      }
      else
      {
        bitSet(enAnalogInputsBits, 7);
      }
      configureADC(enAnalogInputsBits);
    }
    break;
  case 79: // ANALOG setup > Back to setup screen and save analog setup input states to EEPROM
    if (toggle_lock == 0)
    {
      EEPROM_writeAnything(71, enAnalogInputsBits) == sizeof(enAnalogInputsBits);
      tftScreenId = 50;
      toggle_lock = 1;
    }
    break;
  case 80: // OBD > Back to setup screen
    if (toggle_lock == 0)
    {
      tftScreenId = 50;
      toggle_lock = 1;
    }
    break;
  case 90: // TEMP. > Back to setup screen
    if (toggle_lock == 0)
    {
      tftScreenId = 50;
      toggle_lock = 1;
    }
    break;
  case 91: // TEMP. > Autodetect MLX
    if (toggle_lock == 0)
    {
      autodetectMlx();
      tftScreenId = 90;
      toggle_lock = 1;
    }
    break;
  case 100: // GENERAL > Back to setup screen (and save to EEPROM)
    if (toggle_lock == 0)
    {
      eepromSaveRunningValues();
      tftScreenId = 50;
      toggle_lock = 1;
    }
    break;
  case 101: // GENERAL > + rpmCorrectionRatio
    if (toggle_lock == 0)
    {
      rpmCorrectionRatio++;
      toggle_lock = 1;
    }
    break;
  case 102: // GENERAL > - rpmCorrectionRatio
    if (toggle_lock == 0)
    {
      rpmCorrectionRatio--;
      toggle_lock = 1;
    }
    break;
  case 103: // GENERAL > + rpmFlywheelTeeth
    if (toggle_lock == 0)
    {
      rpmFlywheelTeeth++;
      toggle_lock = 1;
    }
    break;
  case 104: // GENERAL > - rpmFlywheelTeeth
    if (toggle_lock == 0)
    {
      rpmFlywheelTeeth--;
      toggle_lock = 1;
    }
    break;
  /*case 100: // LEDs (TEST)
    if (track == 0)
    {
      EVE_cmd_track(80, 80, 200, 40, 100);
      track = 10;
    }
    else
    {
      tracker_val = EVE_memRead32(REG_TRACKER);
      if ((tracker_val & 0xff) == 100)
      {
        tracker_val = tracker_val / 536870;
        PST_Cmd_n_SpeedMechCmd = 8000 - tracker_val;
      }
    }
    break;*/
  case 110: // SDCARD > Back to setup screen
    if (toggle_lock == 0)
    {
      tftScreenId = 50;
      toggle_lock = 1;
    }
    break;
  case 130: // TRACKS > Back to setup screen
    if (toggle_lock == 0)
    {
      tftScreenId = 50;
      toggle_lock = 1;
    }
    break;
  case 140: // Back to setup screen
    if (toggle_lock == 0)
    {
      tftScreenId = 50;
      toggle_lock = 1;
    }
    break;
  case 141: // Reset EEPROM
    if (toggle_lock == 0)
    {
      eepromReset();
      eepromReload();
      tftScreenId = 140;
      toggle_lock = 1;
    }
    break;
  case 142: // Load default EEPROM values
    if (toggle_lock == 0)
    {
      eepromLoadDefaults();
      eepromReload();
      tftScreenId = 140;
      toggle_lock = 1;
    }
    break;

    //case ww:
    /*if (track == 0) //whatever this does, do nothing when tracking is in progress
    {
      //....
    }
    break;*/
  }
}

/*
  dynamic portion of display-handling, meant to be called every 20ms or more
*/
void TFT_display(void)
{
  if (tft_active != 0)
  {
    switch (tftScreenId)
    {
    case 10:                                              // default/start screen
      EVE_cmd_dl(CMD_DLSTART);                            /* start the display list */
      EVE_cmd_dl(DL_CLEAR_RGB | BLACK);                   /* set the default clear color to white */
      EVE_cmd_dl(DL_CLEAR | CLR_COL | CLR_STN | CLR_TAG); /* clear the screen - this and the previous prevent artifacts between lists, Attributes are the color, stencil and tag buffers */
      EVE_cmd_dl(TAG(0));

      // insert static part of display-list from copy in gfx-mem
      EVE_cmd_append(MEM_DL_STATIC, num_dl_static);

      // Display button
      EVE_cmd_dl(DL_COLOR_RGB | TFTDEFAULTFONTCOLOR);
      EVE_cmd_fgcolor(0x00c0c0c0); /* some grey */
      EVE_cmd_dl(TAG(10));
      EVE_cmd_button_var(TFT_BUTTON_BORDER_SIZE, EVE_VSIZE - TFT_BUTTON_BORDER_SIZE - TFT_BUTTON_V_SIZE, TFT_BUTTON_H_SIZE, TFT_BUTTON_V_SIZE, 13, EVE_OPT_FORMAT, MENU_SHUTDOWN, 0);
      EVE_cmd_dl(TAG(11));
      EVE_cmd_button_var(TFT_BUTTON_BORDER_SIZE + (TFT_BUTTON_H_SPACE + TFT_BUTTON_H_SIZE) * 1, EVE_VSIZE - TFT_BUTTON_BORDER_SIZE - TFT_BUTTON_V_SIZE, TFT_BUTTON_H_SIZE, TFT_BUTTON_V_SIZE, 13, EVE_OPT_FORMAT, MENU_SETUP, 0);
      EVE_cmd_dl(TAG(12));
      EVE_cmd_button_var(TFT_BUTTON_BORDER_SIZE + (TFT_BUTTON_H_SPACE + TFT_BUTTON_H_SIZE) * 3, EVE_VSIZE - TFT_BUTTON_BORDER_SIZE - TFT_BUTTON_V_SIZE, TFT_BUTTON_H_SIZE, TFT_BUTTON_V_SIZE, 13, EVE_OPT_FORMAT, MENU_START, 0);
      EVE_cmd_dl(TAG(0));

      TFT_display_header();

      // Display ...
      EVE_cmd_text_var(TFT_DEFAULT_BORDER_H_SIZE + 10, TFT_FONT_01_V_SPACE + 10, TFT_FONT_03_SIZE, EVE_OPT_FORMAT, "%c", 1, anaValuesChar[0]);    // GEAR
      EVE_cmd_text_var(EVE_HSIZE - TFT_DEFAULT_BORDER_V_SIZE, 52, TFT_FONT_02_SIZE, EVE_OPT_FORMAT | EVE_OPT_RIGHTX, "%u", 1, (int)digValues[0]); // RPM
      EVE_cmd_text_var(EVE_HSIZE - TFT_DEFAULT_BORDER_V_SIZE, 75, TFT_FONT_01_SIZE, EVE_OPT_RIGHTX, LABEL_RPM, 0);
      EVE_cmd_text_var(EVE_HSIZE - TFT_DEFAULT_BORDER_H_SIZE, EVE_VSIZE - 100, TFT_FONT_03_SIZE, EVE_OPT_FORMAT | EVE_OPT_RIGHTX, "%u", 1, (int)digValues[1]); // SPEED / digValues[1] or fix_data.speed_kph()
      EVE_cmd_text_var(EVE_HSIZE - TFT_DEFAULT_BORDER_V_SIZE, EVE_VSIZE - 39, TFT_FONT_01_SIZE, EVE_OPT_RIGHTX, LABEL_KMH, 0);

      // Infrared temp
      for (uint8_t i = 0; i < MAX_MLX_SIZE; i++)
      {
        if (mlxAddresses[i] != 0x00)
        {
          if ((int)round(mlxValues[i]) < 200)
          {
            EVE_cmd_text_var(TFT_DEFAULT_BORDER_H_SIZE + (i * 60), 210, TFT_FONT_01_SIZE, EVE_OPT_FORMAT, "T%u:%u°", 2, mlxAddresses[i], (int)round(mlxValues[i]));
          }
          else
          {
            EVE_cmd_text_var(TFT_DEFAULT_BORDER_H_SIZE + (i * 60), 210, TFT_FONT_01_SIZE, EVE_OPT_FORMAT, "T%u:HOT!", 1, mlxAddresses[i]);
          }
        }
      }

      // RPM
      //digValues[0] = 13300; // TEST
      EVE_cmd_dl(COLOR_RGB(67, 109, 121));
      for (uint8_t i = 0; i <= 16; i++)
      {
        EVE_cmd_text_var((i * 25) + 70, 25, TFT_FONT_01_SIZE, EVE_OPT_FORMAT | EVE_OPT_CENTERX, "%u", 1, i);
        EVE_cmd_dl(LINE_WIDTH(1 * 16));
        EVE_cmd_dl(BEGIN(EVE_LINES));
        EVE_cmd_dl(VERTEX2F((i * 25) + 70, 42));
        EVE_cmd_dl(VERTEX2F((i * 25) + 70, 48));
      }
      EVE_cmd_dl(SCISSOR_XY(70, 48));                      // Coords top-left
      EVE_cmd_dl(SCISSOR_SIZE((digValues[0] / 40), 25));   // width, height
      EVE_cmd_gradient(75, 0, 0x436d79, 470, 0, 0x2effff); // Don't bother with "Y", Xstart and Xend

      /*
      EVE_cmd_dl(COLOR_RGB(0, 255, 0));
      uint16_t rpmBarPos;
      for (uint8_t i = 0; i <= 36; i++)
      {
        if (i < 35) // Last 2 bar have same position
        {
          rpmBarPos = (int)(log(i + 2) * 20); // +2 to have a nice log curve
        }
        if (digValues[0] >= i * 500)
        {
          if (i < 25)
          {
            EVE_cmd_dl(COLOR_RGB(0, 255, 0));
            //EVE_cmd_dl(COLOR_A(128));
          }
          else
          {
            EVE_cmd_dl(COLOR_RGB(255, 0, 0));
          }
          EVE_cmd_dl(BEGIN(EVE_RECTS));
          EVE_cmd_dl(VERTEX2F((i * 9) + 100, 150 - rpmBarPos));
          EVE_cmd_dl(VERTEX2F((i * 9) + 105, 100 - rpmBarPos));
        }
        else
        {
          EVE_cmd_dl(COLOR_RGB(150, 150, 150));
          EVE_cmd_dl(BEGIN(EVE_RECTS));
          EVE_cmd_dl(VERTEX2F((i * 9) + 100, 150 - rpmBarPos));
          EVE_cmd_dl(VERTEX2F((i * 9) + 105, 100 - rpmBarPos));
        }
        if ((i * 500) % 1000 == 0)
        {
          EVE_cmd_text_var((i * 9) + 100, 150 - rpmBarPos + 10, TFT_FONT_01_SIZE, EVE_OPT_FORMAT | EVE_OPT_CENTERX, "%u", 1, (i / 2));
        }
      }*/

      EVE_cmd_dl(DL_DISPLAY); /* instruct the graphics processor to show the list */
      EVE_cmd_dl(CMD_SWAP);   /* make this list active */
      break;
    case 20:                                              // Screen when running laptimer (show "STOP" button)
      EVE_cmd_dl(CMD_DLSTART);                            /* start the display list */
      EVE_cmd_dl(DL_CLEAR_RGB | BLACK);                   /* set the default clear color to white */
      EVE_cmd_dl(DL_CLEAR | CLR_COL | CLR_STN | CLR_TAG); /* clear the screen - this and the previous prevent artifacts between lists, Attributes are the color, stencil and tag buffers */
      EVE_cmd_dl(TAG(0));

      // insert static part of display-list from copy in gfx-mem
      EVE_cmd_append(MEM_DL_STATIC, num_dl_static);

      // Display button
      EVE_cmd_dl(DL_COLOR_RGB | TFTDEFAULTFONTCOLOR);
      EVE_cmd_fgcolor(0x00c0c0c0); /* some grey */
      EVE_cmd_dl(TAG(20));
      EVE_cmd_button_var(TFT_BUTTON_BORDER_SIZE + (TFT_BUTTON_H_SPACE + TFT_BUTTON_H_SIZE) * 2, EVE_VSIZE - TFT_BUTTON_BORDER_SIZE - TFT_BUTTON_V_SIZE, TFT_BUTTON_H_SIZE, TFT_BUTTON_V_SIZE, 13, EVE_OPT_FORMAT, MENU_STOP, 0);
      EVE_cmd_dl(TAG(21));
      EVE_cmd_button_var(TFT_BUTTON_BORDER_SIZE + (TFT_BUTTON_H_SPACE + TFT_BUTTON_H_SIZE) * 3, EVE_VSIZE - TFT_BUTTON_BORDER_SIZE - TFT_BUTTON_V_SIZE, TFT_BUTTON_H_SIZE, TFT_BUTTON_V_SIZE, 13, EVE_OPT_FORMAT, MENU_FAKELAP, 0);
      EVE_cmd_dl(TAG(0)); /* no touch */

      TFT_display_header();

      // Display ...
      EVE_cmd_text_var(TFT_DEFAULT_BORDER_H_SIZE + 10, TFT_FONT_01_V_SPACE + 10, TFT_FONT_03_SIZE, EVE_OPT_FORMAT, "%c", 1, anaValuesChar[0]); // GEAR
      if (recordTrackData == true)
      {
        EVE_cmd_text_var(EVE_HSIZE / 6, EVE_VSIZE / 3, TFT_FONT_03_SIZE, EVE_OPT_FORMAT, "%02u:%02u:%u", 3, lapSec / 60, lapSec % 60, lapCsec / 10);
      }
      EVE_cmd_text_var(EVE_HSIZE - TFT_DEFAULT_BORDER_V_SIZE, 52, TFT_FONT_02_SIZE, EVE_OPT_FORMAT | EVE_OPT_RIGHTX, "%u", 1, (int)digValues[0]); // RPM
      EVE_cmd_text_var(EVE_HSIZE - TFT_DEFAULT_BORDER_V_SIZE, 75, TFT_FONT_01_SIZE, EVE_OPT_RIGHTX, LABEL_RPM, 0);
      EVE_cmd_text_var(EVE_HSIZE - TFT_DEFAULT_BORDER_H_SIZE, EVE_VSIZE - 100, TFT_FONT_03_SIZE, EVE_OPT_FORMAT | EVE_OPT_RIGHTX, "%u", 1, (int)digValues[1]); // SPEED / digValues[1] or fix_data.speed_kph()
      EVE_cmd_text_var(EVE_HSIZE - TFT_DEFAULT_BORDER_V_SIZE, EVE_VSIZE - 39, TFT_FONT_01_SIZE, EVE_OPT_RIGHTX, LABEL_KMH, 0);

      // Infrared temp
      for (uint8_t i = 0; i < MAX_MLX_SIZE; i++)
      {
        if (mlxAddresses[i] != 0x00)
        {
          if ((int)round(mlxValues[i]) < 200)
          {
            EVE_cmd_text_var(TFT_DEFAULT_BORDER_H_SIZE + (i * 60), 210, TFT_FONT_01_SIZE, EVE_OPT_FORMAT, "T%u:%u°", 2, mlxAddresses[i], (int)round(mlxValues[i]));
          }
          else
          {
            EVE_cmd_text_var(TFT_DEFAULT_BORDER_H_SIZE + (i * 60), 210, TFT_FONT_01_SIZE, EVE_OPT_FORMAT, "T%u:HOT!", 1, mlxAddresses[i]);
          }
        }
      }

      // RPM
      EVE_cmd_dl(COLOR_RGB(67, 109, 121));
      for (uint8_t i = 0; i <= 16; i++)
      {
        EVE_cmd_text_var((i * 25) + 70, 25, TFT_FONT_01_SIZE, EVE_OPT_FORMAT | EVE_OPT_CENTERX, "%u", 1, i);
        EVE_cmd_dl(BEGIN(EVE_LINES));
        EVE_cmd_dl(VERTEX2F((i * 25) + 70, 42));
        EVE_cmd_dl(VERTEX2F((i * 25) + 70, 48));
      }
      EVE_cmd_dl(SCISSOR_XY(70, 48));                      // Coords top-left
      EVE_cmd_dl(SCISSOR_SIZE((digValues[0] / 40), 25));   // width, height
      EVE_cmd_gradient(75, 0, 0x436d79, 470, 0, 0x2effff); // Don't bother with "Y", Xstart and Xend

      EVE_cmd_dl(DL_DISPLAY); /* instruct the graphics processor to show the list */
      EVE_cmd_dl(CMD_SWAP);   /* make this list active */
      break;
    case 50:                                              // Setup screen (show all subcategories for configuration)
      EVE_cmd_dl(CMD_DLSTART);                            /* start the display list */
      EVE_cmd_dl(DL_CLEAR_RGB | BLACK);                   /* set the default clear color to white */
      EVE_cmd_dl(DL_CLEAR | CLR_COL | CLR_STN | CLR_TAG); /* clear the screen - this and the previous prevent artifacts between lists, Attributes are the color, stencil and tag buffers */
      EVE_cmd_dl(TAG(0));

      // insert static part of display-list from copy in gfx-mem
      EVE_cmd_append(MEM_DL_STATIC, num_dl_static);

      // Display date/time
      TFT_display_header();

      // Display setup menu buttons
      TFT_display_setup_buttons();

      EVE_cmd_dl(DL_DISPLAY); /* instruct the graphics processor to show the list */
      EVE_cmd_dl(CMD_SWAP);   /* make this list active */
      break;
    case 60:                                              // DIGITAL setup
      EVE_cmd_dl(CMD_DLSTART);                            /* start the display list */
      EVE_cmd_dl(DL_CLEAR_RGB | BLACK);                   /* set the default clear color to white */
      EVE_cmd_dl(DL_CLEAR | CLR_COL | CLR_STN | CLR_TAG); /* clear the screen - this and the previous prevent artifacts between lists, Attributes are the color, stencil and tag buffers */
      EVE_cmd_dl(TAG(0));

      // insert static part of display-list from copy in gfx-mem
      EVE_cmd_append(MEM_DL_STATIC, num_dl_static);

      // Display date/time
      TFT_display_header();

      // Parse 4 digital inputs (label + on/off button)
      for (uint8_t i = 0; i < 4; i++)
      {
        EVE_cmd_text(EVE_HSIZE / 4, TFT_HEADER_V_SPACE + (i * (TFT_FONT_02_V_SPACE + 8)), TFT_FONT_02_SIZE, EVE_OPT_FORMAT, digitalInputsLabel[i]);
        EVE_cmd_dl(TAG(61 + i));
        EVE_cmd_toggle(EVE_HSIZE * 2 / 4, TFT_HEADER_V_SPACE + (i * (TFT_FONT_02_V_SPACE + 8)), 40, TFT_FONT_02_SIZE, 0, (bitRead(enDigitalInputsBits, i) == true) ? 65535 : 0, "OFF\xffON");
        EVE_cmd_dl(TAG(0));
        EVE_cmd_text_var(EVE_HSIZE * 3 / 4, TFT_HEADER_V_SPACE + (i * (TFT_FONT_02_V_SPACE + 8)), TFT_FONT_02_SIZE, EVE_OPT_FORMAT, "(%d)", 1, digValues[i]);
      }

      // Display setup menu buttons
      EVE_cmd_dl(DL_COLOR_RGB | WHITE);
      EVE_cmd_fgcolor(0x00c0c0c0); /* some grey */
      EVE_cmd_dl(TAG(65));
      EVE_cmd_button_var(TFT_BUTTON_BORDER_SIZE, EVE_VSIZE - TFT_BUTTON_BORDER_SIZE - TFT_BUTTON_V_SIZE, TFT_BUTTON_H_SIZE, TFT_BUTTON_V_SIZE, TFT_FONT_02_SIZE, EVE_OPT_FORMAT, MENU_BACK, 0);
      EVE_cmd_dl(TAG(0));

      EVE_cmd_dl(DL_DISPLAY); /* instruct the graphics processor to show the list */
      EVE_cmd_dl(CMD_SWAP);   /* make this list active */
      break;
    case 70:                                              // ANALOG setup
      EVE_cmd_dl(CMD_DLSTART);                            /* start the display list */
      EVE_cmd_dl(DL_CLEAR_RGB | BLACK);                   /* set the default clear color to white */
      EVE_cmd_dl(DL_CLEAR | CLR_COL | CLR_STN | CLR_TAG); /* clear the screen - this and the previous prevent artifacts between lists, Attributes are the color, stencil and tag buffers */
      EVE_cmd_dl(TAG(0));

      // insert static part of display-list from copy in gfx-mem
      EVE_cmd_append(MEM_DL_STATIC, num_dl_static);

      // Display date/time
      TFT_display_header();

      // Parse 8 analog inputs (label + on/off button)
      for (uint8_t i = 0; i < 8; i++)
      {
        EVE_cmd_text(EVE_HSIZE / 4, TFT_HEADER_V_SPACE + (i * (TFT_FONT_02_V_SPACE + 8)), TFT_FONT_02_SIZE, EVE_OPT_FORMAT, analogInputsLabel[i]);
        EVE_cmd_dl(TAG(71 + i));
        EVE_cmd_toggle(EVE_HSIZE * 2 / 4, TFT_HEADER_V_SPACE + (i * (TFT_FONT_02_V_SPACE + 8)), 40, TFT_FONT_02_SIZE, 0, (bitRead(enAnalogInputsBits, i) == true) ? 65535 : 0, "OFF\xffON");
        EVE_cmd_dl(TAG(0));
        EVE_cmd_text_var(EVE_HSIZE * 3 / 4, TFT_HEADER_V_SPACE + (i * (TFT_FONT_02_V_SPACE + 8)), TFT_FONT_02_SIZE, EVE_OPT_FORMAT, "(%d)", 1, anaValues[i]);
      }

      // Display setup menu buttons
      EVE_cmd_dl(DL_COLOR_RGB | WHITE);
      EVE_cmd_fgcolor(0x00c0c0c0); /* some grey */
      EVE_cmd_dl(TAG(79));
      EVE_cmd_button_var(TFT_BUTTON_BORDER_SIZE, EVE_VSIZE - TFT_BUTTON_BORDER_SIZE - TFT_BUTTON_V_SIZE, TFT_BUTTON_H_SIZE, TFT_BUTTON_V_SIZE, TFT_FONT_02_SIZE, EVE_OPT_FORMAT, MENU_BACK, 0);
      EVE_cmd_dl(TAG(0));

      EVE_cmd_dl(DL_DISPLAY); /* instruct the graphics processor to show the list */
      EVE_cmd_dl(CMD_SWAP);   /* make this list active */
      break;
    case 80:                                              // OBD setup
      EVE_cmd_dl(CMD_DLSTART);                            /* start the display list */
      EVE_cmd_dl(DL_CLEAR_RGB | BLACK);                   /* set the default clear color to white */
      EVE_cmd_dl(DL_CLEAR | CLR_COL | CLR_STN | CLR_TAG); /* clear the screen - this and the previous prevent artifacts between lists, Attributes are the color, stencil and tag buffers */
      EVE_cmd_dl(TAG(0));

      // insert static part of display-list from copy in gfx-mem
      EVE_cmd_append(MEM_DL_STATIC, num_dl_static);

      // Display date/time
      TFT_display_header();

      // Display ...

      // TODO ...

      // Display setup menu buttons
      EVE_cmd_dl(DL_COLOR_RGB | WHITE);
      EVE_cmd_fgcolor(0x00c0c0c0); /* some grey */
      EVE_cmd_dl(TAG(80));
      EVE_cmd_button_var(TFT_BUTTON_BORDER_SIZE, EVE_VSIZE - TFT_BUTTON_BORDER_SIZE - TFT_BUTTON_V_SIZE, TFT_BUTTON_H_SIZE, TFT_BUTTON_V_SIZE, TFT_FONT_02_SIZE, EVE_OPT_FORMAT, MENU_BACK, 0);
      EVE_cmd_dl(TAG(0));

      EVE_cmd_dl(DL_DISPLAY); /* instruct the graphics processor to show the list */
      EVE_cmd_dl(CMD_SWAP);   /* make this list active */
      break;
    case 90:                                              // TEMP sensors setup
      EVE_cmd_dl(CMD_DLSTART);                            /* start the display list */
      EVE_cmd_dl(DL_CLEAR_RGB | BLACK);                   /* set the default clear color to white */
      EVE_cmd_dl(DL_CLEAR | CLR_COL | CLR_STN | CLR_TAG); /* clear the screen - this and the previous prevent artifacts between lists, Attributes are the color, stencil and tag buffers */
      EVE_cmd_dl(TAG(0));

      // insert static part of display-list from copy in gfx-mem
      EVE_cmd_append(MEM_DL_STATIC, num_dl_static);

      // Display date/time
      TFT_display_header();

      // Display ...
      // Parse the MAX_MLX_SIZE MLX sensors
      for (uint8_t i = 0; i < MAX_MLX_SIZE; i++)
      {
        if (mlxAddresses[i] != 0x00)
        {
          if ((int)round(mlxValues[i]) < 200)
          {
            EVE_cmd_text_var(TFT_DEFAULT_BORDER_H_SIZE + 50, TFT_HEADER_V_SPACE + (i * (TFT_FONT_02_V_SPACE + 8)), TFT_FONT_02_SIZE, EVE_OPT_FORMAT, "Temp. sensor %u : %u°", 2, mlxAddresses[i], (int)round(mlxValues[i]));
          }
          else
          {
            EVE_cmd_text_var(TFT_DEFAULT_BORDER_H_SIZE + 50, TFT_HEADER_V_SPACE + (i * (TFT_FONT_02_V_SPACE + 8)), TFT_FONT_02_SIZE, EVE_OPT_FORMAT, "Temp. sensor %u : HOT!", 1, mlxAddresses[i]);
          }
        }
      }

      // Display setup menu buttons
      EVE_cmd_dl(DL_COLOR_RGB | WHITE);
      EVE_cmd_fgcolor(0x00c0c0c0); /* some grey */
      EVE_cmd_dl(TAG(90));
      EVE_cmd_button_var(TFT_BUTTON_BORDER_SIZE, EVE_VSIZE - TFT_BUTTON_BORDER_SIZE - TFT_BUTTON_V_SIZE, TFT_BUTTON_H_SIZE, TFT_BUTTON_V_SIZE, TFT_FONT_02_SIZE, EVE_OPT_FORMAT, MENU_BACK, 0);
      EVE_cmd_dl(TAG(91));
      EVE_cmd_button_var(TFT_BUTTON_BORDER_SIZE + (TFT_BUTTON_H_SIZE + TFT_BUTTON_H_SPACE), EVE_VSIZE - TFT_BUTTON_BORDER_SIZE - TFT_BUTTON_V_SIZE, TFT_BUTTON_H2_SIZE, TFT_BUTTON_V_SIZE, TFT_FONT_02_SIZE, EVE_OPT_FORMAT, MENU_AUTODETECT_MLX, 0);
      EVE_cmd_dl(TAG(0));

      EVE_cmd_dl(DL_DISPLAY); /* instruct the graphics processor to show the list */
      EVE_cmd_dl(CMD_SWAP);   /* make this list active */
      break;
    case 100:                                             // GENERAL setup
      EVE_cmd_dl(CMD_DLSTART);                            /* start the display list */
      EVE_cmd_dl(DL_CLEAR_RGB | BLACK);                   /* set the default clear color to white */
      EVE_cmd_dl(DL_CLEAR | CLR_COL | CLR_STN | CLR_TAG); /* clear the screen - this and the previous prevent artifacts between lists, Attributes are the color, stencil and tag buffers */
      EVE_cmd_dl(TAG(0));

      // insert static part of display-list from copy in gfx-mem
      EVE_cmd_append(MEM_DL_STATIC, num_dl_static);

      // Display date/time
      TFT_display_header();

      // Display ...

      // Display setup menu buttons
      EVE_cmd_dl(DL_COLOR_RGB | WHITE);
      EVE_cmd_fgcolor(0x00c0c0c0); /* some grey */
      EVE_cmd_dl(TAG(100));
      EVE_cmd_button_var(TFT_BUTTON_BORDER_SIZE, EVE_VSIZE - TFT_BUTTON_BORDER_SIZE - TFT_BUTTON_V_SIZE, TFT_BUTTON_H_SIZE, TFT_BUTTON_V_SIZE, TFT_FONT_02_SIZE, EVE_OPT_FORMAT, MENU_BACK, 0);
      EVE_cmd_dl(TAG(0));

      EVE_cmd_text_var(EVE_HSIZE / 12, TFT_HEADER_V_SPACE + (0 * (TFT_FONT_02_V_SPACE + 8)), TFT_FONT_02_SIZE, EVE_OPT_FORMAT, "RPM Correction ratio : %u", 1, rpmCorrectionRatio);
      EVE_cmd_text_var(EVE_HSIZE / 12, TFT_HEADER_V_SPACE + (1 * (TFT_FONT_02_V_SPACE + 8)), TFT_FONT_02_SIZE, EVE_OPT_FORMAT, "RPM Flywheel teeth : %u", 1, rpmFlywheelTeeth);

      EVE_cmd_dl(TAG(102));
      EVE_cmd_button_var(EVE_HSIZE / 12 * 10, TFT_HEADER_V_SPACE + (0 * (TFT_FONT_02_V_SPACE + 8)), TFT_BUTTON_H3_SIZE, TFT_BUTTON_V3_SIZE, TFT_FONT_02_SIZE, EVE_OPT_FORMAT, "-", 0);
      EVE_cmd_dl(TAG(101));
      EVE_cmd_button_var(EVE_HSIZE / 12 * 11, TFT_HEADER_V_SPACE + (0 * (TFT_FONT_02_V_SPACE + 8)), TFT_BUTTON_H3_SIZE, TFT_BUTTON_V3_SIZE, TFT_FONT_02_SIZE, EVE_OPT_FORMAT, "+", 0);

      EVE_cmd_dl(TAG(104));
      EVE_cmd_button_var(EVE_HSIZE / 12 * 10, TFT_HEADER_V_SPACE + (1 * (TFT_FONT_02_V_SPACE + 8)), TFT_BUTTON_H3_SIZE, TFT_BUTTON_V3_SIZE, TFT_FONT_02_SIZE, EVE_OPT_FORMAT, "-", 0);
      EVE_cmd_dl(TAG(103));
      EVE_cmd_button_var(EVE_HSIZE / 12 * 11, TFT_HEADER_V_SPACE + (1 * (TFT_FONT_02_V_SPACE + 8)), TFT_BUTTON_H3_SIZE, TFT_BUTTON_V3_SIZE, TFT_FONT_02_SIZE, EVE_OPT_FORMAT, "+", 0);

      EVE_cmd_dl(DL_DISPLAY); /* instruct the graphics processor to show the list */
      EVE_cmd_dl(CMD_SWAP);   /* make this list active */
      break;
    /*case 100:                                             // LEDS setup
      EVE_cmd_dl(CMD_DLSTART);                            
      EVE_cmd_dl(DL_CLEAR_RGB | BLACK);                   
      EVE_cmd_dl(DL_CLEAR | CLR_COL | CLR_STN | CLR_TAG); 
      EVE_cmd_dl(TAG(0));

      //EVE_color_rgb(YELLOW);
      //EVE_cmd_dl(LINE_WIDTH(1 * 16));
      //EVE_cmd_dl(DL_BEGIN | EVE_RECTS);
      //EVE_cmd_dl(TAG(40));
      //EVE_cmd_dl(VERTEX2F(1280, 1280)); // 80,80
      //EVE_cmd_dl(VERTEX2F(4480, 1920)); // 280,120
      //EVE_cmd_dl(TAG(0));
      //EVE_cmd_dl(DL_END);

      EVE_cmd_dl(TAG(100));
      EVE_cmd_slider(80, 80, 200, 10, 0, 8000 - PST_Cmd_n_SpeedMechCmd, 8000); 
      EVE_cmd_dl(TAG(0));

      EVE_cmd_text_var(TFT_DEFAULT_BORDER_H_SIZE, TFT_DEFAULT_BORDER_V_SIZE + (TFT_FONT_01_V_SPACE)*1, TFT_FONT_01_SIZE, EVE_OPT_FORMAT, "slider : %u", 1, tracker_val);

      // Display setup menu buttons
      EVE_cmd_dl(DL_COLOR_RGB | WHITE);
      EVE_cmd_fgcolor(0x00c0c0c0);
      EVE_cmd_dl(TAG(50));
      EVE_cmd_button_var(TFT_BUTTON_BORDER_SIZE, EVE_VSIZE - TFT_BUTTON_BORDER_SIZE - TFT_BUTTON_V_SIZE, TFT_BUTTON_H_SIZE, TFT_BUTTON_V_SIZE, 13, EVE_OPT_FORMAT, MENU_BACK, 0);
      EVE_cmd_dl(TAG(0));

      EVE_cmd_dl(DL_DISPLAY);
      EVE_cmd_dl(CMD_SWAP); 
      break;*/
    case 110:                                             // SDCARD setup
      EVE_cmd_dl(CMD_DLSTART);                            /* start the display list */
      EVE_cmd_dl(DL_CLEAR_RGB | BLACK);                   /* set the default clear color to white */
      EVE_cmd_dl(DL_CLEAR | CLR_COL | CLR_STN | CLR_TAG); /* clear the screen - this and the previous prevent artifacts between lists, Attributes are the color, stencil and tag buffers */
      EVE_cmd_dl(TAG(0));

      // insert static part of display-list from copy in gfx-mem
      EVE_cmd_append(MEM_DL_STATIC, num_dl_static);

      // Display date/time
      TFT_display_header();

      // Display ...

      // TODO ...

      // Display setup menu buttons
      EVE_cmd_dl(DL_COLOR_RGB | WHITE);
      EVE_cmd_fgcolor(0x00c0c0c0); /* some grey */
      EVE_cmd_dl(TAG(110));
      EVE_cmd_button_var(TFT_BUTTON_BORDER_SIZE, EVE_VSIZE - TFT_BUTTON_BORDER_SIZE - TFT_BUTTON_V_SIZE, TFT_BUTTON_H_SIZE, TFT_BUTTON_V_SIZE, TFT_FONT_02_SIZE, EVE_OPT_FORMAT, MENU_BACK, 0);
      EVE_cmd_dl(TAG(0));

      EVE_cmd_dl(DL_DISPLAY); /* instruct the graphics processor to show the list */
      EVE_cmd_dl(CMD_SWAP);   /* make this list active */
      break;
    case 120:                                             // GPS setup
      EVE_cmd_dl(CMD_DLSTART);                            /* start the display list */
      EVE_cmd_dl(DL_CLEAR_RGB | BLACK);                   /* set the default clear color to white */
      EVE_cmd_dl(DL_CLEAR | CLR_COL | CLR_STN | CLR_TAG); /* clear the screen - this and the previous prevent artifacts between lists, Attributes are the color, stencil and tag buffers */
      EVE_cmd_dl(TAG(0));

      // insert static part of display-list from copy in gfx-mem
      EVE_cmd_append(MEM_DL_STATIC, num_dl_static);

      // Display date/time
      TFT_display_header();

      EVE_cmd_text_var(EVE_HSIZE / 4, TFT_HEADER_V_SPACE, TFT_FONT_02_SIZE, EVE_OPT_FORMAT, "Latitude : %u.%u°", 2, fix_data.latitudeL() / 10000000, fix_data.latitudeL() % 10000000);
      EVE_cmd_text_var(EVE_HSIZE / 4, TFT_HEADER_V_SPACE + TFT_FONT_02_V_SPACE * 1, TFT_FONT_02_SIZE, EVE_OPT_FORMAT, "Longitude : %u.%u°", 2, fix_data.longitudeL() / 10000000, fix_data.longitudeL() % 10000000);
      EVE_cmd_text_var(EVE_HSIZE / 4, TFT_HEADER_V_SPACE + TFT_FONT_02_V_SPACE * 2, TFT_FONT_02_SIZE, EVE_OPT_FORMAT, "Altitude : %um", 1, fix_data.alt.whole);
      EVE_cmd_text_var(EVE_HSIZE / 4, TFT_HEADER_V_SPACE + TFT_FONT_02_V_SPACE * 3, TFT_FONT_02_SIZE, EVE_OPT_FORMAT, "Speed : %ukm/h", 1, (int)fix_data.speed_kph());
      EVE_cmd_text_var(EVE_HSIZE / 4, TFT_HEADER_V_SPACE + TFT_FONT_02_V_SPACE * 4, TFT_FONT_02_SIZE, EVE_OPT_FORMAT, "Satellites : %u", 1, fix_data.satellites);
      EVE_cmd_text_var(EVE_HSIZE / 4, TFT_HEADER_V_SPACE + TFT_FONT_02_V_SPACE * 5, TFT_FONT_02_SIZE, EVE_OPT_FORMAT, "Heading : %u°", 1, fix_data.heading());
      EVE_cmd_text_var(EVE_HSIZE / 4, TFT_HEADER_V_SPACE + TFT_FONT_02_V_SPACE * 6, TFT_FONT_02_SIZE, EVE_OPT_FORMAT, "Quality : %u/4", 1, fix_data.status);

      // Display setup menu buttons
      EVE_cmd_dl(DL_COLOR_RGB | WHITE);
      EVE_cmd_fgcolor(0x00c0c0c0); /* some grey */
      EVE_cmd_dl(TAG(140));
      EVE_cmd_button_var(TFT_BUTTON_BORDER_SIZE, EVE_VSIZE - TFT_BUTTON_BORDER_SIZE - TFT_BUTTON_V_SIZE, TFT_BUTTON_H_SIZE, TFT_BUTTON_V_SIZE, TFT_FONT_02_SIZE, EVE_OPT_FORMAT, MENU_BACK, 0);
      EVE_cmd_dl(TAG(0));

      EVE_cmd_dl(DL_DISPLAY); /* instruct the graphics processor to show the list */
      EVE_cmd_dl(CMD_SWAP);   /* make this list active */
      break;
    case 130:                                             // TRACKS setup
      EVE_cmd_dl(CMD_DLSTART);                            /* start the display list */
      EVE_cmd_dl(DL_CLEAR_RGB | BLACK);                   /* set the default clear color to white */
      EVE_cmd_dl(DL_CLEAR | CLR_COL | CLR_STN | CLR_TAG); /* clear the screen - this and the previous prevent artifacts between lists, Attributes are the color, stencil and tag buffers */
      EVE_cmd_dl(TAG(0));

      // insert static part of display-list from copy in gfx-mem
      EVE_cmd_append(MEM_DL_STATIC, num_dl_static);

      // Display date/time
      TFT_display_header();

      // Display ...

      // TODO ...

      // Display setup menu buttons
      EVE_cmd_dl(DL_COLOR_RGB | WHITE);
      EVE_cmd_fgcolor(0x00c0c0c0); /* some grey */
      EVE_cmd_dl(TAG(130));
      EVE_cmd_button_var(TFT_BUTTON_BORDER_SIZE, EVE_VSIZE - TFT_BUTTON_BORDER_SIZE - TFT_BUTTON_V_SIZE, TFT_BUTTON_H_SIZE, TFT_BUTTON_V_SIZE, TFT_FONT_02_SIZE, EVE_OPT_FORMAT, MENU_BACK, 0);
      EVE_cmd_dl(TAG(0));

      EVE_cmd_dl(DL_DISPLAY); /* instruct the graphics processor to show the list */
      EVE_cmd_dl(CMD_SWAP);   /* make this list active */
      break;
    case 140:                                             // EEPROM setup
      EVE_cmd_dl(CMD_DLSTART);                            /* start the display list */
      EVE_cmd_dl(DL_CLEAR_RGB | BLACK);                   /* set the default clear color to white */
      EVE_cmd_dl(DL_CLEAR | CLR_COL | CLR_STN | CLR_TAG); /* clear the screen - this and the previous prevent artifacts between lists, Attributes are the color, stencil and tag buffers */
      EVE_cmd_dl(TAG(0));

      // insert static part of display-list from copy in gfx-mem
      EVE_cmd_append(MEM_DL_STATIC, num_dl_static);

      // Display date/time
      TFT_display_header();

      // Display ...
      EVE_cmd_text_var(TFT_DEFAULT_BORDER_H_SIZE, TFT_DEFAULT_BORDER_V_SIZE + (TFT_FONT_01_V_SPACE)*2, TFT_FONT_01_SIZE, EVE_OPT_FORMAT, "rpmCorrectionRatio : %u", 1, rpmCorrectionRatio);
      EVE_cmd_text_var(TFT_DEFAULT_BORDER_H_SIZE, TFT_DEFAULT_BORDER_V_SIZE + (TFT_FONT_01_V_SPACE)*3, TFT_FONT_01_SIZE, EVE_OPT_FORMAT, "rpmFlywheelTeeth : %u", 1, rpmFlywheelTeeth);
      EVE_cmd_text_var(TFT_DEFAULT_BORDER_H_SIZE, TFT_DEFAULT_BORDER_V_SIZE + (TFT_FONT_01_V_SPACE)*4, TFT_FONT_01_SIZE, EVE_OPT_FORMAT, "enDigitalInputsBits : %u", 1, enDigitalInputsBits);
      EVE_cmd_text_var(TFT_DEFAULT_BORDER_H_SIZE, TFT_DEFAULT_BORDER_V_SIZE + (TFT_FONT_01_V_SPACE)*5, TFT_FONT_01_SIZE, EVE_OPT_FORMAT, "enAnalogInputsBits : %u", 1, enAnalogInputsBits);

      // Parse 8 analog inputs (min/max values)
      for (uint8_t i = 0; i < 8; i++)
      {
        EVE_cmd_text(TFT_DEFAULT_BORDER_H_SIZE, TFT_DEFAULT_BORDER_V_SIZE + (TFT_FONT_01_V_SPACE) * (7 + i), TFT_FONT_01_SIZE, EVE_OPT_FORMAT, analogInputsLabel[i]);
        EVE_cmd_text_var(TFT_DEFAULT_BORDER_H_SIZE + 50, TFT_DEFAULT_BORDER_V_SIZE + (TFT_FONT_01_V_SPACE) * (7 + i), TFT_FONT_01_SIZE, EVE_OPT_FORMAT, "Min/Max : %u/%u", 2, anaMinValues[i], anaMaxValues[i]);
      }
      EVE_cmd_text_var(220, TFT_DEFAULT_BORDER_V_SIZE + (TFT_FONT_01_V_SPACE)*2, TFT_FONT_01_SIZE, EVE_OPT_FORMAT, "inAnaGearCalib", 0);
      for (uint8_t i = 0; i < GEAR_CALIB_SIZE; i++)
      {
        EVE_cmd_text_var(220, TFT_DEFAULT_BORDER_V_SIZE + (TFT_FONT_01_V_SPACE) * (3 + i), TFT_FONT_01_SIZE, EVE_OPT_FORMAT, "%u", 1, inAnaGearCalib[i]);
      }
      EVE_cmd_text_var(360, TFT_DEFAULT_BORDER_V_SIZE + (TFT_FONT_01_V_SPACE)*2, TFT_FONT_01_SIZE, EVE_OPT_FORMAT, "mlxAddresses", 0);
      for (uint8_t i = 0; i < MAX_MLX_SIZE; i++)
      {
        EVE_cmd_text_var(360, TFT_DEFAULT_BORDER_V_SIZE + (TFT_FONT_01_V_SPACE) * (3 + i), TFT_FONT_01_SIZE, EVE_OPT_FORMAT, "%u", 1, mlxAddresses[i]);
      }

      // Display setup menu buttons
      EVE_cmd_dl(DL_COLOR_RGB | WHITE);
      EVE_cmd_fgcolor(0x00c0c0c0); /* some grey */
      EVE_cmd_dl(TAG(140));
      EVE_cmd_button_var(TFT_BUTTON_BORDER_SIZE, EVE_VSIZE - TFT_BUTTON_BORDER_SIZE - TFT_BUTTON_V_SIZE, TFT_BUTTON_H_SIZE, TFT_BUTTON_V_SIZE, TFT_FONT_02_SIZE, EVE_OPT_FORMAT, MENU_BACK, 0);
      EVE_cmd_dl(TAG(141));
      EVE_cmd_button_var(TFT_BUTTON_BORDER_SIZE + (TFT_BUTTON_H_SIZE + TFT_BUTTON_H_SPACE), EVE_VSIZE - TFT_BUTTON_BORDER_SIZE - TFT_BUTTON_V_SIZE, TFT_BUTTON_H_SIZE, TFT_BUTTON_V_SIZE, TFT_FONT_02_SIZE, EVE_OPT_FORMAT, MENU_RESET_EEPROM, 0);
      EVE_cmd_dl(TAG(142));
      EVE_cmd_button_var(TFT_BUTTON_BORDER_SIZE + (TFT_BUTTON_H_SIZE + TFT_BUTTON_H_SPACE) * 2, EVE_VSIZE - TFT_BUTTON_BORDER_SIZE - TFT_BUTTON_V_SIZE, TFT_BUTTON_H_SIZE, TFT_BUTTON_V_SIZE, TFT_FONT_02_SIZE, EVE_OPT_FORMAT, MENU_DEFAULT_EEPROM, 0);
      EVE_cmd_dl(TAG(0));

      EVE_cmd_dl(DL_DISPLAY); /* instruct the graphics processor to show the list */
      EVE_cmd_dl(CMD_SWAP);   /* make this list active */
      break;
    default:                                              // If no tagged button (safe mode)
      EVE_cmd_dl(CMD_DLSTART);                            /* start the display list */
      EVE_cmd_dl(DL_CLEAR_RGB | BLACK);                   /* set the default clear color to white */
      EVE_cmd_dl(DL_CLEAR | CLR_COL | CLR_STN | CLR_TAG); /* clear the screen - this and the previous prevent artifacts between lists, Attributes are the color, stencil and tag buffers */
      EVE_cmd_dl(TAG(0));

      // Display setup menu buttons
      EVE_cmd_dl(DL_COLOR_RGB | WHITE);
      EVE_cmd_fgcolor(0x00c0c0c0); /* some grey */
      EVE_cmd_dl(TAG(50));
      EVE_cmd_button_var(TFT_BUTTON_BORDER_SIZE, EVE_VSIZE - TFT_BUTTON_BORDER_SIZE - TFT_BUTTON_V_SIZE, TFT_BUTTON_H_SIZE, TFT_BUTTON_V_SIZE, 13, EVE_OPT_FORMAT, MENU_BACK, 0);
      EVE_cmd_dl(TAG(0));

      EVE_cmd_dl(DL_DISPLAY); /* instruct the graphics processor to show the list */
      EVE_cmd_dl(CMD_SWAP);   /* make this list active */
      break;
    }
  }
}

/*EVE_start_cmd_burst();
EVE_cmd_dl_burst(CMD_DLSTART);                
EVE_cmd_dl_burst(DL_CLEAR_RGB | WHITE);         
EVE_cmd_dl_burst(DL_CLEAR | CLR_COL | CLR_STN | CLR_TAG); 
EVE_cmd_dl_burst(TAG(0));
EVE_cmd_append_burst(MEM_DL_STATIC, num_dl_static);
EVE_color_rgb_burst(WHITE);
EVE_cmd_fgcolor_burst(0x00c0c0c0); 
EVE_cmd_dl_burst(TAG(10));      
EVE_cmd_button_burst(20, 20, 80, 30, 28, toggle_state, "Touch!");
EVE_cmd_dl_burst(TAG(0)); 
EVE_cmd_dl_burst(DL_COLOR_RGB | BLACK);
EVE_cmd_button_burst(20, 120, 80, 30, 28, 0, "Touch!");
EVE_cmd_setfont2_burst(12, MEM_FONT_01, 32);
EVE_cmd_text_burst(10, 160, 12, 0, "LOOP - The quick brown fox");
EVE_cmd_text_burst(10, 80, 12, 0, "Hello UTF-8: €µäöü"); //€µ°");
EVE_color_rgb_burst(BLACK);
EVE_cmd_dl_burst(DL_DISPLAY);
EVE_cmd_dl_burst(CMD_SWAP);  
EVE_end_cmd_burst();*/