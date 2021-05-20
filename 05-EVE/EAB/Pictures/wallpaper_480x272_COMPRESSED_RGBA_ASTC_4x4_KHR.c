/*
This file is automatically generated
wallpaper_480x272_COMPRESSED_RGBA_ASTC_4x4_KHR
C Source
*/

#include "App_Common.h"

void Load_Image(Gpu_Hal_Context_t *phost)
{
	Gpu_Hal_WaitCmdfifo_empty(phost);

	Gpu_CoCmd_Dlstart(phost);
	App_WrCoCmd_Buffer(phost, CLEAR(1, 1, 1));
	App_WrCoCmd_Buffer(phost, COLOR_RGB(255, 255, 255));

	uint16_t iw = 480;
	uint16_t ih = 272;
	uint16_t format = COMPRESSED_RGBA_ASTC_4x4_KHR;

#ifdef USE_BT81X_FLASH
#define BITMAP_ADDRESS_ON_FLASH <address> // address of bitmap file from Flash Map after generating Flash Image
#define BITMAP_SIZE_ON_FLASH    <size>    // size of bitmap file from Flash Map after generating Flash Image
	/* Switch Flash to FULL Mode */
	Gpu_CoCmd_FlashHelper_SwitchFullMode(phost);
	Gpu_CoCmd_SetBitmap(phost, (0x800000 | BITMAP_ADDRESS_ON_FLASH / 32), format, iw, ih);
#else
	//load bitmap file into graphics RAM
	//RAM_G is starting address in graphics RAM, for example 00 0000h
	Gpu_Hal_LoadImageToMemory(phost, "path\\to\\wallpaper_480x272_COMPRESSED_RGBA_ASTC_4x4_KHR.raw", RAM_G, LOAD);
	Gpu_CoCmd_SetBitmap(phost, RAM_G, format, iw, ih);
#endif
	//Start drawing bitmap
	App_WrCoCmd_Buffer(phost, BEGIN(BITMAPS));
	App_WrCoCmd_Buffer(phost, VERTEX2II(0, 0, 0, 0));
	App_WrCoCmd_Buffer(phost, END());
	App_WrCoCmd_Buffer(phost, RESTORE_CONTEXT());
	App_WrCoCmd_Buffer(phost, DISPLAY());
	Gpu_CoCmd_Swap(phost);
	App_Flush_Co_Buffer(phost);
	Gpu_Hal_WaitCmdfifo_empty(phost);
}

/* end of file */
