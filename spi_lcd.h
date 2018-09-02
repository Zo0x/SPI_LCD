#ifndef SPI_LCD_H
#define SPI_LCD_H
//
// SPI_LCD using the SPI interface
// Copyright (c) 2017 Larry Bank
// email: bitbank@pobox.com
// Project started 4/25/2017
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//

#define DEBUG

//
// Treat the LCD as a 240x320 portrait-mode image
// or a 320x240 landscape mode image
// This affects the coordinate system and rotates the
// drawing direction of fonts and tiles
//
#define LCD_ORIENTATION_NATIVE 1
#define LCD_ORIENTATION_ROTATED 2

#define LCD_ILI9341 1
#define LCD_HX8357 2
#define LCD_ST7735 3
#define LCD_SSD1351 4
#define LCD_ILI9342 5
#define LCD_ST7789 6

// touch panel types
#define TOUCH_XPT2046 1

// The ILITEK LCD display controllers communicate through the SPI interface
// and two GPIO pins to control the RESET, and D/C (data/command)
// control lines.

// Use one of the following 4 methods for talking to the SPI/GPIO
//#define USE_PIGPIO
//#define USE_BCM2835
//#define USE_WIRINGPI
#define USE_GENERIC

// For generic SPI access (kernel drivers), select the board pinout (only one)
//#define USE_NANOPI2
//#define USE_NANOPIK2
//#define USE_NANOPIDUO
//#define USE_NANOPINEO
//define USE_NANOPIM1
//#define USE_RPI
//#define USE_ORANGEPIZERO
//#define USE_ORANGEPIONE
//#define USE_BANANAPIM2ZERO
#define USE_BANANAPIM2MAGIC
//#define USE_NANOPINEOCORE
//#define USE_ORANGEPIZEROPLUS2

#include <unistd.h>
#include <exception>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <cstdint>
#include <fcntl.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <time.h>
#ifdef USE_WIRINGPI
#include <wiringPi.h>
#include <wiringPiSPI.h>
#endif // USE_WIRINGPI
#include <linux/types.h>
//#include <linux/spi/spidev.h>
#ifdef USE_BCM2835
#include <bcm2835.h>
#endif
#include "font.h"
#ifdef USE_PIGPIO
#include <pigpio.h>
#endif // USE_PIGPIO
#ifdef __NEON__
#include <arm_neon.h>
#endif //__NEON__

#ifdef USE_GENERIC
#include <linux/spi/spidev.h>
#define GPIO_OUT 0
#define GPIO_IN 1
#endif // USE_GENERIC

class SpiLcd
        {
                public:
#ifdef USE_GENERIC
                struct spi_ioc_transfer xfer;
                int iPinHandles[256]; // keep file handles open for GPIO access
#endif // USE_GENERIC

                typedef enum
                {
                    MODE_DATA = 0,
                            MODE_COMMAND
                } DC_MODE;

                unsigned char ucRXBuf[4096]; //, ucRXBuf2[4096];
                int file_spi = -1; // SPI system handle
                int file_touch = -1; // SPI handle for touch controller
                int iTouchChannel, iTouchType;
                int iDCPin, iResetPin, iLEDPin; // pin numbers for the GPIO control lines
                int iMinX, iMaxX, iMinY, iMaxY; // touch calibration values
                int iScrollOffset; // current scroll amount
                int iOrientation = LCD_ORIENTATION_NATIVE; // default to 'natural' orientation
                int iLCDType;
                int iWidth, iHeight;
                int iCurrentWidth, iCurrentHeight; // reflects virtual size due to orientation

                // For Raspberry Pi boards, we can use the generic GPIO/SPI access too
#ifdef USE_RPI
                int iGenericPins[41] = {-1,-1,-1,2,-1,3,-1,4,14,-1,
			15,17,18,27,-1,22,23,-1,24,10,
			-1,9,25,11,8,-1,7,0,1,5,
			-1,6,12,13,-1,19,16,26,20,-1,
			21};
#endif // USE_RPI

#ifdef USE_NANOPINEOCORE
                int iGenericPins[41] = {-1,-1,-1,12,-1,11,-1,203,198,-1,
                        199,0,6,2,-1,3,200,-1,201,64,
                        -1,65,1,66,67,-1,-1,-1,-1,-1,
                        -1,-1,-1,-1,-1,-1,-1,-1,167,-1,
                        140,-1,141,-1,-1,-1,15,-1,16,-1,
                        14,-1,13,-1,-1,363,-1,17,-1,18,
                        -1,19,164,20,162,21,-1,-1,-1};
#endif // USE_NANOPINEOCORE

#ifdef USE_ORANGEPIZEROPLUS2
                int iGenericPins[41] = {-1,-1,-1,12,-1,11,-1,6,0,-1,1,352,107,353,-1,3,
	19,-1,18,-1,-1,-1,2,14,13,-1,110,-1,5,4,-1,-1,-1,-1,-1,-1,-1,-1,-1,
	-1,-1};
#endif // USE_ORANGEPIZEROPLUS2

#ifdef USE_BANANAPIM2ZERO
                int iGenericPins[41] = {-1,-1,-1,12,-1,11,-1,6,13,-1,14,1,110,0,-1,3,15,-1,68,64,-1,65,2,66,67,-1,71,19,18,7,-1,8,354,9,-1,10,356,17,21,-1,20};
#endif // BPI-M2-Zero

#ifdef USE_BANANAPIM2MAGIC
                int iGenericPins[41] = {-1,-1,-1,-1,-1,-1,-1,225,32,-1,33,230,203,231,-1,117,34,-1,35,64,-1,65,116,66,67,-1,121,115,120,114,-1,119,118,123,-1,202,122,363,205,-114,204};
#endif // BPI-M2-Magic

#ifdef USE_ORANGEPIONE
                int iGenericPins[41] = {-1,-1,-1,12,-1,11,-1,6,13,-1,14,1,110,0,-1,3,68,-1,71,64,-1,65,2,66,67,-1,21,19,18,7,-1,8,200,9,-1,10,201,20,198,-1,199};
#endif // ORANGEPIONE

#ifdef USE_ORANGEPIZERO
                int iGenericPins[41] = {-1,-1,-1,12,-1,11,-1,6,198,-1,199,1,7,0,-1,3,19,-1,18,15,-1,16,2,14,13,-1,10,-1,5,4,-1,
-1,-1,-1,-1,-1,-1,-1,-1};
#endif // ORANGEPIZERO

#ifdef USE_NANOPIDUO
                int iGenericPins[41] = {-1,5,-1,4,-1,-1,-1,11,
			-1,12,363,13,203,14,-1,16,
			-1,15,-1,199,-1,198,-1,-1,
			-1,-1,-1,-1,-1,-1,-1,-1,
			-1,355,-1,-1,-1,-1,-1,-1,-1};
#endif // USE_NANOPIDUO

#ifdef USE_NANOPI2
                int iGenericPins[41] = {-1,-1,-1,99,-1,98,-1,32+28,96+21,-1,96+17,
                       32+29,32+26,32+30,-1,32+31,64+14,-1,32+27,64+31,-1,96+0,96+1,64+29,64+30,-1,64+13,103,102,
                       64+8,-1,64+9,64+28,64+10,-1,64+12,64+7,64+11,162,-1,163};

#endif // USE_NANOPI2

#ifdef USE_NANOPIM1
                int iGenericPins[41] = {-1,-1,-1,12,-1,11,-1,203,198,-1,199,0,6,2,-1,3,200,-1,201,64,-1,65,1,66,67,-1,17,19,18,20,-1,21,7,8,-1,16,13,9,15,-1,14};
#endif // USE_NANOPIM1

#ifdef USE_NANOPIK2
                int iGenericPins[41] = {-1,-1,-1,205,-1,206,-1,211,102,-1,225,212,227,213,-1,214,226,-1,215,216,-1,218,217,220,219,-1,221,207,208,222,-1,127,223,155,-1,252,-1,-1,-1,-1,-1};
#endif // USE_NANOPIK2

#ifdef USE_NANOPINEO
                // NanoPi NEO
// define 40 pins since the 12 pin header has 2 GPIOs available and so does
// the 4-pin TTY header
int iGenericPins[41] = {-1,-1,-1,12,-1,11,-1,203,198,-1,
                        199,0,6,2,-1,3,200,-1,201,64,
                        -1,65,1,66,67,-1,-1,-1,-1,-1,
                        363,17,-1,-1,-1,-1,-1,-1,-1,4,
                        5};
#endif // USE_NANOPINEO
#ifdef USE_PIGPIO
                int iPIGPins[41] = {-1,-1,-1,2,-1,3,-1,4,14,-1,15,
                       17,18,27,-1,22,23,-1,24,10,-1,9,25,11,8,-1,7,0,1,
                       5,-1,6,12,13,-1,19,16,26,20,-1,21};
#endif // USE_PIGPIO

#ifdef USE_WIRINGPI
                int iWPPins[41] = {-1,-1,-1,8,-1,9,-1,7,15,-1,16,0,1,
        2,-1,3,4,-1,5,12,-1,13,6,14,10,-1,11,30,31,21,-1,22,26,23,-1,24,27,25,28,-1,29};
#endif // USE_WIRINGPI

#ifdef USE_BCM2835
                int iBCM2835Pins[41] = {-1,-1,-1,RPI_V2_GPIO_P1_03,-1,RPI_V2_GPIO_P1_05,-1,
	RPI_V2_GPIO_P1_07, RPI_V2_GPIO_P1_08,-1, RPI_V2_GPIO_P1_10, RPI_V2_GPIO_P1_11,
        RPI_V2_GPIO_P1_12, RPI_V2_GPIO_P1_13, -1, RPI_V2_GPIO_P1_15,RPI_V2_GPIO_P1_16,
        -1, RPI_V2_GPIO_P1_18, RPI_V2_GPIO_P1_19, -1, RPI_V2_GPIO_P1_21, RPI_V2_GPIO_P1_22,
        RPI_V2_GPIO_P1_23, RPI_V2_GPIO_P1_24, -1, RPI_V2_GPIO_P1_26, -1, -1,
        RPI_V2_GPIO_P1_29, -1, RPI_V2_GPIO_P1_31, RPI_V2_GPIO_P1_32, RPI_V2_GPIO_P1_33,
        -1, RPI_V2_GPIO_P1_35, RPI_V2_GPIO_P1_36, RPI_V2_GPIO_P1_37, RPI_V2_GPIO_P1_38,
        -1, RPI_V2_GPIO_P1_40};
#endif // BCM2835

                // List of command/parameters to initialize the SSD1351 OLED display
                unsigned char ucOLEDInitList[58] = {
                    2, 0xfd, 0x12, // unlock the controller
                            2, 0xfd, 0xb1, // unlock the command
                            1, 0xae,	// display off
                            2, 0xb3, 0xf1,  // clock divider
                            2, 0xca, 0x7f,	// mux ratio
                            2, 0xa0, 0x74,	// set remap
                            3, 0x15, 0x00, 0x7f,	// set column
                            3, 0x75, 0x00, 0x7f,	// set row
                            2, 0xb5, 0x00,	// set GPIO state
                            2, 0xab, 0x01,	// function select (internal diode drop)
                            2, 0xb1, 0x32,	// precharge
                            2, 0xbe, 0x05,	// vcomh
                            1, 0xa6,	// set normal display mode
                            4, 0xc1, 0xc8, 0x80, 0xc8, // contrast ABC
                            2, 0xc7, 0x0f,	// contrast master
                            4, 0xb4, 0xa0,0xb5,0x55,	// set VSL
                            2, 0xb6, 0x01,	// precharge 2
                            1, 0xaf,	// display ON
                            0};
// List of command/parameters to initialize the ili9341 display
                unsigned char uc240InitList[108] = {
                    4, 0xEF, 0x03, 0x80, 0x02,
                            4, 0xCF, 0x00, 0XC1, 0X30,
                            5, 0xED, 0x64, 0x03, 0X12, 0X81,
                            4, 0xE8, 0x85, 0x00, 0x78,
                            6, 0xCB, 0x39, 0x2C, 0x00, 0x34, 0x02,
                            2, 0xF7, 0x20,
                            3, 0xEA, 0x00, 0x00,
                            2, 0xc0, 0x23, // Power control
                            2, 0xc1, 0x10, // Power control
                            3, 0xc5, 0x3e, 0x28, // VCM control
                            2, 0xc7, 0x86, // VCM control2
                            2, 0x36, 0x48, // Memory Access Control
                            2, 0x3a, 0x55,
                            3, 0xb1, 0x00, 0x18,
                            4, 0xb6, 0x08, 0x82, 0x27, // Display Function Control
                            2, 0xF2, 0x00, // Gamma Function Disable
                            2, 0x26, 0x01, // Gamma curve selected
                            16, 0xe0, 0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08,
                            0x4E, 0xF1, 0x37, 0x07, 0x10, 0x03, 0x0E, 0x09, 0x00, // Set Gamma
                            16, 0xe1, 0x00, 0x0E, 0x14, 0x03, 0x11, 0x07,
                            0x31, 0xC1, 0x48, 0x08, 0x0F, 0x0C, 0x31, 0x36, 0x0F, // Set Gamma
                            3, 0xb1, 0x00, 0x10, // FrameRate Control 119Hz
                            0
                };
// List of command/parameters to initialize the ili9342 display
                unsigned char uc320InitList[22] = {
                    2, 0xc0, 0x23, // Power control
                            2, 0xc1, 0x10, // Power control
                            3, 0xc5, 0x3e, 0x28, // VCM control
                            2, 0xc7, 0x86, // VCM control2
                            2, 0x36, 0x08, // Memory Access Control (flip x/y/bgr/rgb)
                            2, 0x3a, 0x55,
                            1, 0x21,	// inverted display off
//        2, 0x26, 0x01, // Gamma curve selected
//        16, 0xe0, 0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08,
//                0x4E, 0xF1, 0x37, 0x07, 0x10, 0x03, 0x0E, 0x09, 0x00, // Set Gamma
//        16, 0xe1, 0x00, 0x0E, 0x14, 0x03, 0x11, 0x07,
//                0x31, 0xC1, 0x48, 0x08, 0x0F, 0x0C, 0x31, 0x36, 0x0F, // Set Gamma
//        3, 0xb1, 0x00, 0x10, // FrameRate Control 119Hz
                            0
                };
// List of command/parameters to initialize the ST7789 LCD
                unsigned char uc240x240InitList[81] = {
                    1, 0x13, // partial mode off
                            1, 0x21, // display inversion off
                            2, 0x36,0x08,	// memory access 0xc0 for 180 degree flipped
                            2, 0x3a,0x55,	// pixel format; 5=RGB565
                            3, 0x37,0x00,0x00, //
                            6, 0xb2,0x0c,0x0c,0x00,0x33,0x33, // Porch control
                            2, 0xb7,0x35,	// gate control
                            2, 0xbb,0x1a,	// VCOM
                            2, 0xc0,0x2c,	// LCM
                            2, 0xc2,0x01,	// VDV & VRH command enable
                            2, 0xc3,0x0b,	// VRH set
                            2, 0xc4,0x20,	// VDV set
                            2, 0xc6,0x0f,	// FR control 2
                            3, 0xd0, 0xa4, 0xa1, 	// Power control 1
                            15, 0xe0, 0x00,0x19,0x1e,0x0a,0x09,0x15,0x3d,0x44,0x51,0x12,0x03,
                            0x00,0x3f,0x3f, 	// gamma 1
                            15, 0xe1, 0x00,0x18,0x1e,0x0a,0x09,0x25,0x3f,0x43,0x52,0x33,0x03,
                            0x00,0x3f,0x3f,		// gamma 2
                            1, 0x29,	// display on
                            0
                };

// List of command/parameters to initialize the st7735 display
                unsigned char uc128InitList[43] = {
//	4, 0xb1, 0x01, 0x2c, 0x2d,	// frame rate control
//	4, 0xb2, 0x01, 0x2c, 0x2d,	// frame rate control (idle mode)
//	7, 0xb3, 0x01, 0x2c, 0x2d, 0x01, 0x2c, 0x2d, // frctrl - partial mode
//	2, 0xb4, 0x07,	// non-inverted
//	4, 0xc0, 0x82, 0x02, 0x84,	// power control
//	2, 0xc1, 0xc5, 	// pwr ctrl2
//	2, 0xc2, 0x0a, 0x00, // pwr ctrl3
//	3, 0xc3, 0x8a, 0x2a, // pwr ctrl4
//	3, 0xc4, 0x8a, 0xee, // pwr ctrl5
//	2, 0xc5, 0x0e,		// pwr ctrl
//	1, 0x20,	// display inversion off
                    2, 0x3a, 0x55,	// pixel format RGB565
                            2, 0x36, 0xc0, // MADCTL
                            17, 0xe0, 0x09, 0x16, 0x09,0x20,
                            0x21,0x1b,0x13,0x19,
                            0x17,0x15,0x1e,0x2b,
                            0x04,0x05,0x02,0x0e, // gamma sequence
                            17, 0xe1, 0x0b,0x14,0x08,0x1e,
                            0x22,0x1d,0x18,0x1e,
                            0x1b,0x1a,0x24,0x2b,
                            0x06,0x06,0x02,0x0f,
                            0
                };
// List of command/parameters to initialize the hx8357 display
                unsigned char uc480InitList[67] = {
                    2, 0x3a, 0x55,
                            2, 0xc2, 0x44,
                            5, 0xc5, 0x00, 0x00, 0x00, 0x00,
                            16, 0xe0, 0x0f, 0x1f, 0x1c, 0x0c, 0x0f, 0x08, 0x48, 0x98, 0x37,
                            0x0a,0x13, 0x04, 0x11, 0x0d, 0x00,
                            16, 0xe1, 0x0f, 0x32, 0x2e, 0x0b, 0x0d, 0x05, 0x47, 0x75, 0x37,
                            0x06, 0x10, 0x03, 0x24, 0x20, 0x00,
                            16, 0xe2, 0x0f, 0x32, 0x2e, 0x0b, 0x0d, 0x05, 0x47, 0x75, 0x37,
                            0x06, 0x10, 0x03, 0x24, 0x20, 0x00,
                            2, 0x36, 0x48,
                            0
                };

                int myspiReadWrite(unsigned char *pTxBuf, unsigned char *pRxBuf, int iLen);
                void spilcdWriteCommand(unsigned char);
                void spilcdWriteData8(unsigned char c);
                void spilcdWriteData16(unsigned short us);
                void spilcdSetPosition(int x, int y, int w, int h);
                void spilcdWriteDataBlock(unsigned char *pData, int iLen);
                void myPinWrite(int iPin, int iValue);

                // Sets the D/C pin to data or command mode
                void spilcdSetMode(int iMode);

                //
                // Choose the gamma curve between 2 choices (0/1)
                // ILI9341 only
                //
                int spilcdSetGamma(int iMode);

                // Initialize the library
                int spilcdInit(int iLCDType, int bFlipped, int iSPIChannel, int iSPIFreq, int iDCPin, int iResetPin, int iLEDPin);

                //
                // Initialize the touch controller
                //
                int spilcdInitTouch(int iType, int iChannel, int iSPIFreq);

                //
                // Set touch calibration values
                // These are the minimum and maximum x/y values returned from the sensor
                // These values are used to normalize the position returned from the sensor
                //
                void spilcdTouchCalibration(int iminx, int imaxx, int iminy, int imaxy);

                //
                // Shut down the touch interface
                //
                void spilcdShutdownTouch(void);

                //
                // Read the current touch values
                // values are normalized to 0-1023 range for x and y
                // returns: -1=not initialized, 0=nothing touching, 1=good values
                //
                int spilcdReadTouchPos(int *pX, int *pY);

                // Turns off the display and frees the resources
                void spilcdShutdown(void);

                // Fills the display with the byte pattern
                int spilcdFill(unsigned short usData);

                //
                // Draw a rectangle and optionally fill it
                //
                void spilcdRectangle(int x, int y, int w, int h, unsigned short usColor, int bFill);

                //
                // Reset the scroll position to 0
                //
                void spilcdScrollReset(void);

                // Configure a GPIO pin for input
                // Returns 0 if successful, -1 if unavailable
                int spilcdConfigurePin(int iPin);

                // Read from a GPIO pin
                int spilcdReadPin(int iPin);

                //
                // Scroll the screen N lines vertically (positive or negative)
                // This is a delta which affects the current hardware scroll offset
                // If iFillcolor != -1, the newly exposed lines will be filled with that color
                //
                void spilcdScroll(int iLines, int iFillColor);

                //
                // Draw a NxN tile scaled 150% in both directions
                int spilcdDrawTile150(int x, int y, int iTileWidth, int iTileHeight, unsigned char *pTile, int iPitch);

                // Draw a NxN tile
                int spilcdDrawTile(int x, int y, int iTileWidth, int iTileHeight, unsigned char *pTile, int iPitch);

                // Draw a 16x16 tile with variable cols/rows removed
                int spilcdDrawMaskedTile(int x, int y, unsigned char *pTile, int iPitch, int iColMask, int iRowMask);

                // Draw a NxN tile scaled to 2x width, 1.5x height with pixel averaging
                int spilcdDrawScaledTile(int x, int y, int cx, int cy, unsigned char *pTile, int iPitch);

                int spilcdDraw53Tile(int x, int y, int cx, int cy, unsigned char *pTile, int iPitch);

                // Draw a 16x16 tile as 16x13 (with priority to non-black pixels)
                int spilcdDrawRetroTile(int x, int y, unsigned char *pTile, int iPitch);

                // Draw a 16x16 tile scaled to 16x14 with pixel averaging
                int spilcdDrawSmallTile(int x, int y, unsigned char *pTile, int iPitch);

                // Write a text string to the display at x (column 0-83) and y (row 0-5)
                // bLarge = 0 - 8x8 font, bLarge = 1 - 16x24 font
                int spilcdWriteString(int x, int y, char *szText, unsigned short usFGColor, unsigned short usBGColor, int bLarge);

                // Write a text string of 8x8 characters
                // quickly to the LCD with a single data block write.
                // This is necessary because there is a lot of latency between
                // writes when using the spidev kernel driver
                int spilcdWriteStringFast(int x, int y, char *szText, unsigned short usFGColor, unsigned short usBGColor);

                // Sets a pixel to the given color
                // Coordinate system is pixels, not text rows (0-239, 0-319)
                int spilcdSetPixel(int x, int y, unsigned short usPixel);

                // Set the software orientation
                int spilcdSetOrientation(int iOrientation);
                void myspiWrite(unsigned char *pBuf, int iLen);
                void GenericAddGPIO(int iPin, int iDirection, int bPullup);
                void GenericRemoveGPIO(int iPin);
                void Demo(int width, int height);
                int MilliTime();
        };

#endif // SPI_LCD_H
