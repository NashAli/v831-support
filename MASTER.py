#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
    ETMS.py
 	Created:      01:57 AM 09/24/2021
  	Last Updated: 12:01 PM 09/24/2021
  	MIT License

  	Copyright (c) 2021 Zulfikar Naushad(Nash) Ali

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and the associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
  
  NOTES:
	Uses the MAIX-II V831 ONLY! (has not been tested with any other product) 
  	This program is the new ETMS library for the generic 3.95 SPI i/f ILI9488
  	tft full colour display controller, MAX30102 Oximeter, CJMCU 8128 Sensor Array. See docs.

  Commercial use of this library requires you to buy a license that
  will allow commercial use. This includes using the library,
  modified or not, as a tool to sell products.

  The license applies to all part of the library including the 
  examples and tools supplied with the library.
 
  This is the main user general python3 library.
  SUPPORTED PRODUCTS*:
  ILI9844 480x320 TFT 3.5 inch LCD:
  FT-812 EVE:
    MaixIIDock(V831)      ->          LCD           EVE
        GND               ->          GND(1)    
        5V/3.3V           ->          VCC(2)
        PH0(SPI1.0:CLK)   ->          CLK(3)
        PH1(SPI1.0:MOSI)  ->          MOSI(4)
        PH7               ->          RES(5)
        PH8               ->          DC(6)
        PH14  N/C         ->          BLK(7)  
        PH2               ->          MISO(8)
        
MCP23017 Port Expander: i2c address:0x20

        
MAX30102 Oximeter:
        GND
        5V/3
        SDA (PH12)
        SCK (PH11)
        
CJMCU-8128 Environmental Sensors: BMP280 - 0x76, CCS811 - 0x5A, HDC1080 - 0x40, ONBOARD MSA301 - 0x26
        GND
        5V/3.3V
        SDA (PH12)
        SCK (PH11)

MLX90640 IR Sensor:
        GND
        5V/3.3V
        SDA (PH12)
        SCK (PH11)
        
"""
from maix import i2c
import time, numbers, spidev, gpiod, traceback, asyncio
from pyzbar import pyzbar
from PIL import Image,ImageDraw,ImageFont
from types import MethodType


#LCD TFT
#IO
#
#
#
PH_BASE = 224 # "PH"
LED_CHIP = "gpiochip1"
RST = PH_BASE + 7 #PH7 new
DC = PH_BASE + 8 # PH8

#RST = 237# PH13
LED = PH_BASE + 14 # PH14
#SPI : CLK0 MOSI0
bus = 1    #:1
device = 0   #:0
spi_speed = 32000000

# tft defaults

MADCTL_MY = 0x80  #< Bottom to top
MADCTL_MX = 0x40  #< Right to left
MADCTL_MV = 0x20  #< Reverse Mode
MADCTL_ML = 0x10  #< LCD refresh Bottom to top
MADCTL_RGB = 0x00 #< Red-Green-Blue pixel order
MADCTL_BGR = 0x08 #< Blue-Green-Red pixel order
MADCTL_MH = 0x04  #< LCD refresh right to left
ILI9488_TFTWIDTH    = 480
ILI9488_TFTHEIGHT   = 320
_height = ILI9488_TFTHEIGHT
_width = ILI9488_TFTWIDTH
PORTRAIT = 0
LANDSCAPE = 1

# tft ILI9488 control registers

ILI9488_NOP         = 0x00
ILI9488_SWRESET     = 0x01
ILI9488_READID      = 0x04
ILI9488_SLEEP       = 0x10
ILI9488_WAKE        = 0x11
ILI9488_NORMALMODE  = 0x13
ILI9488_INVOFF      = 0x20
ILI9488_INVON       = 0x21
ILI9488_ALLPIXOFF   = 0x22
ILI9488_ALLPIXON    = 0x23
ILI9488_GAMMASET    = 0x26
ILI9488_DISPOFF     = 0x28
ILI9488_DISPON      = 0x29
ILI9488_CASET       = 0x2A
ILI9488_PASET       = 0x2B
ILI9488_RAMWR       = 0x2C
ILI9488_RAMRD       = 0x2E
ILI9488_VSCRLDEF    = 0x33
ILI9488_MADCTRL     = 0x36
ILI9488_SCRLDISP    = 0x37
ILI9488_IDLE_OFF    = 0x38
ILI9488_IDLE_ON     = 0x39
ILI9488_PIXFMT      = 0x3A
ILI9488_MEMRD_CONT  = 0x3E
ILI9488_IMC         = 0xB0
ILI9488_FRMCTRL     = 0xB1
ILI9488_DIC         = 0xB4
ILI9488_BPCTRL      = 0xB5
ILI9488_DFUNCTRL    = 0xB6
ILI9488_ENTRYMODE   = 0xB7
ILI9488_CEC1        = 0xB9
ILI9488_CEC2        = 0xBA
ILI9488_PWRCTRL1    = 0xC0
ILI9488_PWRCTRL2    = 0xC1
ILI9488_PWRCTRL3    = 0xC2
ILI9488_VMCTRL1     = 0xC5
ILI9488_CABCCTRL1   = 0xC6
ILI9488_CABCCTRL9   = 0xCF
ILI9488_VMCTRL2     = 0xC7
ILI9488_NVMEMWR     = 0xD0
ILI9488_NVMEMKEY    = 0xD1
ILI9488_NVMEMSTRD   = 0xD2
ILI9488_READID4     = 0xD3
ILI9488_READIDVER   = 0xD8
ILI9488_READID1     = 0xDA
ILI9488_READID2     = 0xDB
ILI9488_READID3     = 0xDC
ILI9488_GMCTRLP1    = 0xE0
ILI9488_GMCTRLN1    = 0xE1
ILI9488_DGMCTRL1    = 0xE2
ILI9488_DGMCTRL2    = 0xE3
ILI9488_SETIFUNCT   = 0xE9
ILI9488_ADJCTRL2    = 0xF2
ILI9488_ADJCTRL3    = 0xF7
ILI9488_ADJCTRL4    = 0xF8
ILI9488_ADJCTRL5    = 0xF9
ILI9488_SPIRCSET    = 0xFB
ILI9488_ADJCTRL6    = 0xFC
ILI9488_ADJCTRL7    = 0xFF


#Colours
#RGB 24-bits color table definition (RGB888).
BLACK          =0x000000
WHITE          =0xFFFFFF
BLUE           =0x0000FF
GREEN          =0x00FF00
RED            =0xFF0000
NAVY           =0x000080
DARKBLUE       =0x00008B
DARKGREEN      =0x006400
DARKCYAN       =0x008B8B
CYAN           =0x00FFFF
TURQUOISE      =0x40E0D0
INDIGO         =0x4B0082
DARKRED        =0x800000
OLIVE          =0x808000
GRAY           =0x808080
SKYBLUE        =0x87CEEB
BLUEVIOLET     =0x8A2BE2
LIGHTGREEN     =0x90EE90
DARKVIOLET     =0x9400D3
YELLOWGREEN    =0x9ACD32
BROWN          =0xA52A2A
DARKGRAY       =0xA9A9A9
SIENNA         =0xA0522D
LIGHTBLUE      =0xADD8E6
GREENYELLOW    =0xADFF2F
SILVER         =0xC0C0C0
LIGHTGREY      =0xD3D3D3
LIGHTCYAN      =0xE0FFFF
VIOLET         =0xEE82EE
AZUR           =0xF0FFFF
BEIGE          =0xF5F5DC
MAGENTA        =0xFF00FF
TOMATO         =0xFF6347
GOLD           =0xFFD700
ORANGE         =0xFFA500
SNOW           =0xFFFAFA
YELLOW         =0xFFFF00



#define RGB_24_TO_RGB565(RGB) \		(((RGB >>19)<<11) | (((RGB & 0x00FC00) >>5)) | (RGB & 0x00001F))
#define RGB_24_TO_18BIT(RGB) \		((RGB & 0xFC0000) | (RGB & 0x00FC00) | (RGB & 0x0000FC))
#define RGB_16_TO_18BIT(RGB) \		(((((RGB >>11)*63)/31) << 18) | (((RGB >> 5) & 0x00003F) << 10) | (((RGB & 0x00001F)*63)/31) << 2)
#define BGR_TO_RGB_18BIT(RGB) \		(RGB & 0xFF0000) | ((RGB & 0x00FF00) >> 8 ) | ( (RGB & 0x0000FC) >> 16 ))
#define BGR_16_TO_18BITRGB(RGB)   BGR_TO_RGB_18BIT(RGB_16_TO_18BIT(RGB))

#define ILI9488_COLOR(r, g, b)\ ((((uint16_t)b) >> 3) |\ ((((uint16_t)g) << 3) & 0x07E0) |\	((((uint16_t)r) << 8) & 0xf800))
#Misc
Buffer = None

#spi
spi = spidev.SpiDev()
#spi.open(bus,device)    #spi /dev/spidev<bus>.<device>
#spi.max_speed_hz=spi_speed# spi

#GPIO pins
cpin = gpiod.chip(LED_CHIP)
dc = cpin.get_line(DC)
rst= cpin.get_line(RST)
led= cpin.get_line(LED)

dcconfig = gpiod.line_request()
dcconfig.consumer = "Blink"
dcconfig.request_type = gpiod.line_request.DIRECTION_OUTPUT

rstconfig = gpiod.line_request()
rstconfig.consumer = "Blink"
rstconfig.request_type = gpiod.line_request.DIRECTION_OUTPUT

ledconfig = gpiod.line_request()
ledconfig.consumer = "Blink"
ledconfig.request_type = gpiod.line_request.DIRECTION_OUTPUT

dc.request(dcconfig)
rst.request(rstconfig)
led.request(ledconfig)

#   ILI9488 TFT Display 480x320 16.7M colours support library   *******************************************************
class ILI9488():

    def __init__(self):
        """
        3.5 inch 480x320 LCD
        :return:None
        """
        global dc,rst,led
        led.set_value(1)
        self.resetlcd()
        self.init9488()

    def send2lcd(self, data, isData=True, chunk_size=4096):
        """
        LCD
        :param data: data
        :param isData: True sets the COMM/DATA pin PH8
        :param chunk_size: 4096
        :return:None
        """
        global dc,spi,bus,device,spi_speed
        try:
            dc.set_value(isData) #DC/RS
            spi.open(bus,device)    #spi   /dev/spidev<bus>.<device>
            spi.max_speed_hz=spi_speed #spi
            if isinstance(data, numbers.Number):
                data = [data & 0xFF]
            for start in range(0, len(data), chunk_size):
                end = min(start+chunk_size, len(data))
                spi.writebytes(data[start:end])
            spi.close()  #spi
        except:
            traceback.print_exc()
            return None

    def command(self, data):
        """
        :param data: 
        :return: None
        """
        self.send2lcd(data, False)

    def data(self, data):
        """
        :param data: 
        :return: None
        """
        self.send2lcd(data, True)

    def resetlcd(self):
        """
        LCD
        :return:None
        """
        global dc,rst,led
        try:
            if rst is not None:
                rst.set_value(1)
                time.sleep(0.005)
                rst.set_value(0)
                time.sleep(0.02)
                rst.set_value(1)
                time.sleep(0.150)
            else:
                self.command(ILI9488_SWRESET)
                time.sleep(1)
        except:
            traceback.print_exc()
            return None
            
    
    def init9488(self):
        """
        ILI9488 Initialization sequence
        :return: None
        """    
        try:
            self.command(ILI9488_WAKE)
            time.sleep(0.200)
            self.command(ILI9488_MADCTRL)
            self.data(MADCTL_MY|MADCTL_MX|MADCTL_MV|MADCTL_ML|MADCTL_BGR)
            self.command(ILI9488_CABCCTRL9)
            self.data(0x04)
            self.command(ILI9488_PIXFMT)
            self.data(0x77)
            time.sleep(0.1)
            self.command(ILI9488_NORMALMODE)
            time.sleep(0.100)
            self.command(ILI9488_GMCTRLP1)
            self.data([0x00, 0x04, 0x0E, 0x08, 0x17, 0x0A, 0x40, 0x79, 0x4D, 0x07, 0x0E, 0x0A, 0x1A, 0x1D, 0x0F])
            self.command(ILI9488_GMCTRLN1)
            self.data([0x00, 0x1B, 0x1F, 0x02, 0x10, 0x05, 0x32, 0x34, 0x43, 0x02, 0x0A, 0x09, 0x33, 0x37, 0x0F])
            self.command(ILI9488_PWRCTRL1)
            self.data([0x17, 0x15])
            self.command(ILI9488_PWRCTRL2)
            self.data(0x41)
            self.command(ILI9488_VMCTRL1)
            self.data([0x00, 0x12, 0x80])
            self.command(ILI9488_IMC)
            self.data(0x80)
            self.command(ILI9488_FRMCTRL)
            self.data(0xA0)
            self.command(ILI9488_DIC)
            self.data(0x02)
            self.command(ILI9488_DFUNCTRL)
            self.data([0x02, 0x02])
            self.command(ILI9488_SETIFUNCT)
            self.data(0x00)
            self.command(ILI9488_ENTRYMODE)
            self.data(0xC6)
            self.command(ILI9488_ADJCTRL3)
            self.data([0xA9, 0x51, 0x2C, 0x82])
            self.command(ILI9488_DISPON)
            time.sleep(0.125)
        except:
            traceback.print_exc()
            return None
            
    
    def setFrame(self, x0=0, y0=0, x1=None, y1=None):
        """
        Sets the Data Window
        :param x0:
        :param y0:
        :param x1:
        :param y1:
        :return:None
        """   
        try:
            if x1 is None:
                x1 = ILI9488_TFTWIDTH-1
            if y1 is None:
                y1 = ILI9488_TFTHEIGHT-1
            self.command(ILI9488_CASET)
            self.data(x0)
            self.data(x0 >> 8)
            self.data(x1)
            self.data(x1 >> 8)
            
            self.command(ILI9488_PASET)
            self.data(y0)
            self.data(y0 >> 8)
            self.data(y1)
            self.data(y1 >> 8)
            self.command(ILI9488_RAMWR)
        except:
            traceback.print_exc()
            return None
    

    def drawPixel(self, xpos, ypos, colour):
        """
        Draws a pixel at the position indicated by xpos,ypos and is of colour = colour
        :param xpos:
        :param ypos:
        :param colour:
        :return:None
        """
        try:
            r = (colour & 0xF800) >> 11
            g = (colour & 0x07E0) >> 5
            b = colour & 0x001F
            r = (r * 255) / 31
            g = (g * 255) / 63
            b = (b * 255) / 31
            setFrame(xpos,ypos,xpos+1,ypos+1)
            self.data([r, g, b])
        except:
            traceback.print_exc()
            return None
            
    def fillRect(self, x, y, h, w, colour):
        try:
            # rudimentary clipping (drawChar w/big text requires this)
            if((x >= _width) & (y >= _height)):
                return
            if((x + w - 1) >= _width):
                w = _width - x
            if((y + h - 1) >= _height):
                h = _height - y
            setFrame(x, y, x+w-1, y+h-1)
        except:
            traceback.print_exc()
            return None
            
    def setScrollArea(self, topFixedArea, bottomFixedArea):
        try:
            self.command(ILI9488_VSCRLDEF)
            self.data(topFixedArea >> 8)
            self.data(topFixedArea)
            self.data((_height - topFixedArea - bottomFixedArea) >> 8)
            self.data(_height - topFixedArea - bottomFixedArea)
            self.data(bottomFixedArea >> 8)
            self.data(bottomFixedArea)
        except:
            traceback.print_exc()
            return None

    def scroll(self, pixels):
        try:
            self.command(ILI9488_SCRLDISP)
            self.data(pixels >> 8)
            self.data(pixels)
        except:
            traceback.print_exc()
            return None
            
    def imageToData(self, image):
        """
        16bit 565 colour
        :param image:
        :return: none
        """
        try:
            #ETMS_ILI9488 decode
            pixels = image.convert('RGB').load()
            width, height = image.size
            for y in range(height):
                for x in range(width):
                    r,g,b = pixels[(x,y)]
                    yield r
                    yield g
                    yield b
        except:
            traceback.print_exc()
            return None

    def show(self, image=None,width=480,height=320):
        """
        :param image: None
        :param width:480
        :param height:320
        :return: True/False
        """
        try:
            if image is None:
                return False
            w, h = image.size
            if w > width or h > height:
                image = image.copy().resize((width, height))
            if image.size[0] == 320:    #if larger than 320 pixels then，rotate 90 degrees
                image = image.rotate(90)
            self.setFrame()
            pixelbytes = list(self.imageToData(image))# 24bit 888 RGB
            self.data(pixelbytes)
            return True
        except:
            traceback.print_exc()
            return False

    def invert(self, onoff=False):
        """
        :param onoff:
        :return:True/False
        """
        try:
            if onoff:
                self.command(ILI9488_INVON)
            else:
                self.command(ILI9488_INVOFF)
            return True
        except:
            traceback.print_exc()
            return False

    def backlight(self, onoff=True):
        """
        :param onoff:
        :return:True/False
        """
        global led
        try:
            if led is not None:
                led.set_value(onoff)
                return True
            return False
        except:
            traceback.print_exc()
            return False
            
    def display(self, image=None):
        """Write the display buffer or provided image to the hardware.  If no
        image parameter is provided the display buffer will be written to the
        hardware.  If an image is provided, it should be RGB format and the
        same dimensions as the display hardware.
        """
        image = Image.open(image)
        # By default write the internal buffer to the display.
        if image.size[0] == 320:
            image = image.rotate(90)
        # Set address bounds to entire display.
        self.setFrame()
        # Convert image to array of 16bit 565 RGB data bytes.
        pixelbytes = list(self.imageToData(image))
        # Write data to hardware.
        self.data(pixelbytes)
        
    def draw(self):
        """Return a PIL ImageDraw instance for drawing on the image buffer."""
        d = ImageDraw.Draw(Buffer)
        # Add custom methods to the draw object:
        d.textrotated = MethodType(_textrotated, d, ImageDraw.Draw)
        d.pasteimage = MethodType(_pasteimage, d, ImageDraw.Draw)
        d.textwrapped = MethodType(_textwrapped, d, ImageDraw.Draw)
        return d
        
    def loadWallpaper(self, filename):
        # The image should be 480x320 only (full wallpaper!). Errors otherwise.
        # We need to cope with whatever orientations file image and TFT canvas are.
        image = Image.open(filename)
        Buffer.paste(image)
        if image.size[0] > Buffer.size[0]:
            Buffer.paste(image.rotate(90))
        elif image.size[0] < Buffer.size[0]:
            Buffer.paste(image.rotate(-90))
        else:
            Buffer.paste(image)
        #   example:    loadWallpaper(filename)
        
    def backupBuffer(self):
        self.buffer2.paste(Buffer)

    def restoreBuffer(self):
        Buffer.paste(self.buffer2)
    
    def textRotated(self, position, text, angle, font, fill="white"):
        # Define a function to create rotated text.
        # Source of this rotation coding: Adafruit ILI9341 python project
        # "Unfortunately PIL doesn't have good
        # native support for rotated fonts, but this function can be used to make a
        # text image and rotate it so it's easy to paste in the buffer."
        width, height = self.textsize(text, font=font)
        # Create a new image with transparent background to store the text.
        textimage = Image.new('RGBA', (width, height), (0,0,0,0))
        # Render the text.
        textdraw = ImageDraw.Draw(textimage)
        textdraw.text((0,0), text, font=font, fill=fill)
        # Rotate the text image.
        rotated = textimage.rotate(angle, expand=1)
        # Paste the text into the TFT canvas image, using text itself as a mask for transparency.
        Buffer.paste(rotated, position, rotated)  # into the global Buffer
        #   example:  draw.textrotated(position, text, angle, font, fill)

    def pasteImage(self, filename, position):
        Buffer.paste(Image.open(filename), position)
        # example: draw.pasteimage('bl.jpg', (30,80))        
            
    def textDirect(self, pos, text, font, fill="white"):
        width, height = self.draw().textsize(text, font=font)
        # Create a new image with transparent background to store the text.
        textimage = Image.new('RGBA', (width, height), (255,255,255,0))
        # Render the text.
        textdraw = ImageDraw.Draw(textimage)
        textdraw.text((0,0), text, font=font, fill=fill)
        self.set_frame(pos[0], pos[1], pos[0]+width-1, pos[1]+height-1)
        # Convert image to array of 16bit 565 RGB data bytes.
        pixelbytes = list(self.image_to_data(textimage))
        # Write data to hardware.
        self.data(pixelbytes)      
#   End of library ILI9488  *******************************************************************************************


#   MCP23017 I2C Library  *********************************************************************************************
# IOCON.BANK = 0 on reset.
i2c_dev = '/dev/i2c-2'
PortExpander = 0x20
ALLOUTS = 0x00

# Define registers values from datasheet
IODIRA = 0x00  # IO direction A - 1= input 0 = output
IODIRB = 0x01  # IO direction B - 1= input 0 = output    
IPOLA = 0x02  # Input polarity A
IPOLB = 0x03  # Input polarity B
GPINTENA = 0x04  # Interrupt-onchange A
GPINTENB = 0x05  # Interrupt-onchange B
DEFVALA = 0x06  # Default value for port A
DEFVALB = 0x07  # Default value for port B
INTCONA = 0x08  # Interrupt control register for port A
INTCONB = 0x09  # Interrupt control register for port B
IOCON = 0x0A  # Configuration register
GPPUA = 0x0C  # Pull-up resistors for port A
GPPUB = 0x0D  # Pull-up resistors for port B
INTFA = 0x0E  # Interrupt condition for port A
INTFB = 0x0F  # Interrupt condition for port B
INTCAPA = 0x10  # Interrupt capture for port A
INTCAPB = 0x11  # Interrupt capture for port B
GPIOA = 0x12  # Data port A
GPIOB = 0x13  # Data port B
OLATA = 0x14  # Output latches A
OLATB = 0x15  # Output latches B


PINA7 = 0x80
PINA6 = 0x40
PINA5 = 0x20
PINA4 = 0x10
PINA3 = 0x08
PINA2 = 0x04
PINA1 = 0x02
PINA0 = 0x01

class I2C_Port_Expander():

    def __init__(self):
            """
            i2c port expander
            :return:None
            """
            self.initMCP23017()
            return None

    def initMCP23017(self):
            """
            initializes the MCP23017 for use.
            """
            
            try:
                ports = i2c.I2CDevice(i2c_dev, PortExpander)
                ports.write(IODIRA, ALLOUTS)
                
            except:
                traceback.print_exc()
                return False

    def writePort(self, data, *, start=0, end=None):
            """
            """
            try:
            ports = i2c.I2CDevice(i2c_dev, PortExpander)
            ports.write(buf[0], bytes(buf[start+1:end]))
            except:
            
    def readPort(self):
            """
            """
            buf = 0
            try:
                self.i2c.readfrom_into(self.device_address, buf[start:end])
                return data
            
            except:
                traceback.print_exc()
                return False
            
            

#   end of MCP23017 I2C Library  **************************************************************************************

#   MSA301 Library - basic  *******************************************************************************************
MSA301_I2CADDR_DEFAULT = 0x26 # I2C address - tested - OK.
MSA301_REG_PARTID = 0x01    # Register that contains the part ID - (0x13)
MSA301_REG_OUT_X_L = 0x02   # Register address for X axis lower byte
MSA301_REG_OUT_X_H = 0x03   # Register address for X axis higher byte
MSA301_REG_OUT_Y_L = 0x04   # Register address for Y axis lower byte
MSA301_REG_OUT_Y_H = 0x05   # Register address for Y axis higher byte
MSA301_REG_OUT_Z_L = 0x06   # Register address for Z axis lower byte
MSA301_REG_OUT_Z_H = 0x07   # Register address for Z axis higher byte
MSA301_REG_MOTIONINT = 0x09 # Register address for motion interrupt
MSA301_REG_DATAINT = 0x0A   # Register address for data interrupt
MSA301_REG_CLICKSTATUS = 0x0B #< Register address for click/doubleclick status
MSA301_REG_RESRANGE = 0x0F  # Register address for resolution range
MSA301_REG_ODR = 0x10       # Register address for data rate setting
MSA301_REG_POWERMODE = 0x11 # Register address for power mode setting
MSA301_REG_INTSET1 = 0x17   # Register address for interrupt setting #1
MSA301_REG_INTMAP0 = 0x19   # Register address for interrupt map #0
MSA301_REG_INTMAP1 = 0x1A   # Register address for interrupt map #1
MSA301_REG_TAPDUR = 0x2A    # Register address for tap duration
MSA301_REG_TAPTH = 0x2B     # Register address for tap threshold

MSA301_RANGE_2_G = 0b00     # +/- 2g (default value)
MSA301_RANGE_4_G = 0b01     # +/- 4g
MSA301_RANGE_8_G = 0b10     # +/- 8g
MSA301_RANGE_16_G = 0b11    # +/- 16g
MSA301_AXIS_X = 0x0         # X axis bit
MSA301_AXIS_Y = 0x1         # Y axis bit
MSA301_AXIS_Z = 0x2         # Z axis bit

MSA301_NORMALMODE = 0b00    # Normal (high speed) mode
MSA301_LOWPOWERMODE = 0b01  # Low power (slow speed) mode
MSA301_SUSPENDMODE = 0b010  # Suspend (sleep) mode


#The accelerometer read resolution
MSA301_RESOLUTION_14 = 0b00   # 14-bit resolution
MSA301_RESOLUTION_12 = 0b01   # 12-bit resolution
MSA301_RESOLUTION_10 = 0b10   # 10-bit resolution
MSA301_RESOLUTION_8 = 0b11    #  8-bit resolution

class   MSA301():

    def __init__(self):
        """
        3-axis accelerometer
        :return:None
        """
        self.initMSA301()
        return None
    def initMSA301(self):
        """
        """
        i2c_dev = '/dev/i2c-2'
        try:
            accelerometer = i2c.I2CDevice(i2c_dev, MSA_ADDR)
            
        except:
            traceback.print_exc()
            return False    
        
        
    def getGsensor(self): # acquires acceleration data
        """
        """
        xa, ya, za = 0
        return xa, ya, za
#   End Of Library MSA301   *********************************************************************************

#   Environmental/air quality i2c library   *****************************************************************
#   Contains support for the following... CCS811, HDC1080, BMP280 -> CJMCU-8128
#   CCS811 Timings
CCS811_WAIT_AFTER_RESET_US     = 2000 # The CCS811 needs a wait after reset
CCS811_WAIT_AFTER_APPSTART_US  = 1000 # The CCS811 needs a wait after app start
CCS811_WAIT_AFTER_WAKE_US      =   50 # The CCS811 needs a wait after WAKE signal
CCS811_WAIT_AFTER_APPERASE_MS  =  500 # The CCS811 needs a wait after app erase (300ms from spec not enough)
CCS811_WAIT_AFTER_APPVERIFY_MS =   70 # The CCS811 needs a wait after app verify
CCS811_WAIT_AFTER_APPDATA_MS   =   50 # The CCS811 needs a wait after writing app data

CCS811_ADDR                     =   0x5A
CCS811_MODE_IDLE                =   0
CCS811_MODE_1SEC                =   1
CCS811_MODE_10SEC               =   2
CCS811_MODE_60SEC               =   3
# error defines
CCS811_ERRSTAT_ERROR            =   0x0001 # There is an error, the ERROR_ID register (0xE0) contains the error source
CCS811_ERRSTAT_I2CFAIL          =   0x0002 # Bit flag added by software: I2C transaction error
CCS811_ERRSTAT_DATA_READY       =   0x0008 # A new data sample is ready in ALG_RESULT_DATA
CCS811_ERRSTAT_APP_VALID        =   0x0010 # Valid application firmware loaded
CCS811_ERRSTAT_FW_MODE          =   0x0080 # Firmware is in application mode (not boot mode)
CCS811_ERRSTAT_WRITE_REG_INVALID=   0x0100 # The CCS811 received an I²C write request addressed to this station but with invalid register address ID
CCS811_ERRSTAT_READ_REG_INVALID =   0x0200 # The CCS811 received an I²C read request to a mailbox ID that is invalid
CCS811_ERRSTAT_MEASMODE_INVALID =   0x0400 # The CCS811 received an I²C request to write an unsupported mode to MEAS_MODE
CCS811_ERRSTAT_MAX_RESISTANCE   =   0x0800 # The sensor resistance measurement has reached or exceeded the maximum range
CCS811_ERRSTAT_HEATER_FAULT     =   0x1000 # The heater current in the CCS811 is not in range
CCS811_ERRSTAT_HEATER_SUPPLY    =   0x2000 # The heater voltage is not being applied correctly
# These flags should not be set. They flag errors.
CCS811_ERRSTAT_HWERRORS = ( CCS811_ERRSTAT_ERROR | CCS811_ERRSTAT_WRITE_REG_INVALID | CCS811_ERRSTAT_READ_REG_INVALID | CCS811_ERRSTAT_MEASMODE_INVALID | CCS811_ERRSTAT_MAX_RESISTANCE | CCS811_ERRSTAT_HEATER_FAULT | CCS811_ERRSTAT_HEATER_SUPPLY )
CCS811_ERRSTAT_ERRORS = ( CCS811_ERRSTAT_I2CFAIL | CCS811_ERRSTAT_HWERRORS )
# These flags should normally be set - after a measurement. They flag data available (and valid app running).
CCS811_ERRSTAT_OK = ( CCS811_ERRSTAT_DATA_READY | CCS811_ERRSTAT_APP_VALID | CCS811_ERRSTAT_FW_MODE )
# These flags could be set after a measurement. They flag data is not yet available (and valid app running).
CCS811_ERRSTAT_OK_NODATA = ( CCS811_ERRSTAT_APP_VALID | CCS811_ERRSTAT_FW_MODE )

#   CCS811 registers, all 1 byte except when stated otherwise
CCS811_STATUS           =   0x00
CCS811_MEAS_MODE        =   0x01
CCS811_ALG_RESULT_DATA  =   0x02 # up to 8 bytes
CCS811_RAW_DATA         =   0x03 # 2 bytes
CCS811_ENV_DATA         =   0x05 # 4 bytes
CCS811_THRESHOLDS       =   0x10 # 5 bytes
CCS811_BASELINE         =   0x11 # 2 bytes
CCS811_HW_ID            =   0x20
CCS811_HW_VERSION       =   0x21
CCS811_FW_BOOT_VERSION  =   0x23 # 2 bytes
CCS811_FW_APP_VERSION   =   0x24 # 2 bytes
CCS811_ERROR_ID         =   0xE0
CCS811_APP_ERASE        =   0xF1 # 4 bytes
CCS811_APP_DATA         =   0xF2 # 9 bytes
CCS811_APP_VERIFY       =   0xF3 # 0 bytes
CCS811_APP_START        =   0xF4 # 0 bytes
CCS811_SW_RESET         =   0xFF # 4 bytes

#   ClosedCube_HDC1080 defines

HDC1080_RESOLUTION_8BIT  =   0
HDC1080_RESOLUTION_11BIT =   1
HDC1080_RESOLUTION_14BIT =   2
#   ClosedCube_HDC1080 registers
HDC1080_ADDR            =   0x40    #   tested - OK
HDC1080_TEMPERATURE		=   0x00
HDC1080_HUMIDITY		=   0x01
HDC1080_CONFIGURATION	=   0x02
HDC1080_MANUFACTURER_ID =   0xFE
HDC1080_DEVICE_ID		=   0xFF
HDC1080_SERIAL_ID_FIRST	=   0xFB
HDC1080_SERIAL_ID_MID	=   0xFC
HDC1080_SERIAL_ID_LAST	=   0xFD

#   BMP280 registers
BMP280_ADDR                 =   0x76    #   tested - OK
BMP280_CHIPID               =   0x58
BMP280_REGISTER_DIG_T1      =   0x88
BMP280_REGISTER_DIG_T2      =   0x8A
BMP280_REGISTER_DIG_T3      =   0x8C
BMP280_REGISTER_DIG_P1      =   0x8E
BMP280_REGISTER_DIG_P2      =   0x90
BMP280_REGISTER_DIG_P3      =   0x92
BMP280_REGISTER_DIG_P4      =   0x94
BMP280_REGISTER_DIG_P5      =   0x96
BMP280_REGISTER_DIG_P6      =   0x98
BMP280_REGISTER_DIG_P7      =   0x9A
BMP280_REGISTER_DIG_P8      =   0x9C
BMP280_REGISTER_DIG_P9      =   0x9E
BMP280_REGISTER_CHIPID      =   0xD0
BMP280_REGISTER_VERSION     =   0xD1
BMP280_REGISTER_SOFTRESET   =   0xE0
BMP280_REGISTER_CAL26       =   0xE1, # R calibration = 0xE1-0xF0
BMP280_REGISTER_STATUS      =   0xF3
BMP280_REGISTER_CONTROL     =   0xF4
BMP280_REGISTER_CONFIG      =   0xF5
BMP280_REGISTER_PRESSUREDATA=   0xF7
BMP280_REGISTER_TEMPDATA    =   0xFA
#   Sampling rate
BMP280_SAMPLING_NONE        =   0x00  #   no over-sampling.
BMP280_SAMPLING_X1          =   0x01  #   1x over-sampling.
BMP280_SAMPLING_X2          =   0x02  #   2x over-sampling.
BMP280_SAMPLING_X4          =   0x03  #   4x over-sampling.
BMP280_SAMPLING_X8          =   0x04  #   8x over-sampling.
BMP280_SAMPLING_X16         =   0x05  #   16x over-sampling.
#   Operating mode for the sensor.
BMP280_MODE_SLEEP           =   0x00  #   Sleep mode.
BMP280_MODE_FORCED          =   0x01  #   Forced mode.
BMP280_MODE_NORMAL          =   0x03  #   Normal mode.
BMP280_MODE_SOFT_RESET_CODE =   0xB6  #   Software reset.
#   Filtering level for sensor data.
BMP280_FILTER_OFF           =   0x00  #   No filtering.
BMP280_FILTER_X2            =   0x01  #   2x filtering.
BMP280_FILTER_X4            =   0x02  #   4x filtering.
BMP280_FILTER_X8            =   0x03  #   8x filtering.
BMP280_FILTER_X16           =   0x04  #   16x filtering.
#   Standby duration in ms.
BMP280_STANDBY_MS_1         =   0x00  #   1 ms standby. 
BMP280_STANDBY_MS_63        =   0x01  #   62.5 ms standby.
BMP280_STANDBY_MS_125       =   0x02  #   125 ms standby.
BMP280_STANDBY_MS_250       =   0x03  #   250 ms standby.
BMP280_STANDBY_MS_500       =   0x04  #   500 ms standby.
BMP280_STANDBY_MS_1000      =   0x05  #   1000 ms standby.
BMP280_STANDBY_MS_2000      =   0x06  #   2000 ms standby.
BMP280_STANDBY_MS_4000      =   0x07  #   4000 ms standby.

class AirQuality():
    def __init__(self):
        """
        """
        
    def wakeUp(self, mode=1):
        """
        """
        
    def wakeDown(self, mode=0):
        """
        """
    def getCO2(self):
        """
        """
    def getTVOC(self):
        """
        """
    def getAirTemp(self):
        """
        """
    def getHumidity(self):
        """
        """
    def getBarometric(self):
        """
        """
    def getFloatAltitudeMeters(self, seaLevelPressure=1013.25):
        """
        """
    def waterBoilPoint(self, pressure):
        """
        """
    def setBMPSampling(self, rate):
        """
        """
import urllib.request
import gzip
import json
def get_weather_data() :
    city_name = input('Please enter the name of the city you want to query:')
    url1 = 'http://wthrcdn.etouch.cn/weather_mini?city='+urllib.parse.quote(city_name)  #URL link with suffix
    weather_data = urllib.request.urlopen(url1).read()  #retrieve data
    weather_data = gzip.decompress(weather_data).decode('utf-8') #Adjust coding form
    weather_dict = json.loads(weather_data)
    return weather_dict

def show_weather(weather_data):
    weather_dict = weather_data 
    if weather_dict.get('desc') == 'invilad-citykey':
        print('The city name you entered is wrong, or the weather center does not include your city')
    elif weather_dict.get('desc') =='OK':
        forecast = weather_dict.get('data').get('forecast')
        print('city:',weather_dict.get('data').get('city'))
        print('temperature:',weather_dict.get('data').get('wendu')+'℃ ')
        print('cold:',weather_dict.get('data').get('ganmao'))
        print('wind direction:',forecast[0].get('fengxiang'))
        print('high temperature:',forecast[0].get('high'))
        print('Low temperature:',forecast[0].get('low'))
        print('the weather:',forecast[0].get('type'))
        print('date:',forecast[0].get('date'))   
#usage: show_weather(get_weather_data())

#   End of library AQ   *************************************************************************************

#   MLX90640 IR Library
#   defines
MLX90640_I2CADDR_DEFAULT    =   0x33
MLX90640_0_5_HZ             =   0x00
MLX90640_1_HZ               =   0x01
MLX90640_2_HZ               =   0x02
MLX90640_4_HZ               =   0x03
MLX90640_8_HZ               =   0x04
MLX90640_16_HZ              =   0x05
MLX90640_32_HZ              =   0x06
MLX90640_64_HZ              =   0x07

MLX90640_ADC_16BIT          =   0x00
MLX90640_ADC_17BIT          =   0x01
MLX90640_ADC_18BIT          =   0x02
MLX90640_ADC_19BIT          =   0x03

MLX90640_INTERLEAVED        =   0x00
MLX90640_CHESS              =   0x01

#   MLX90640 IR Library
class MLX90640():
    def setMode(self, mode):
        """
        """
    def setResolution(self, resol):
        """
        """
    def setRefreshRate(self, refrate):
        """
        """
    def getAmbTemp(self):
        """
        """
    def getVdd(self):
        """
        """
    def getMode(self):
        """
        """
    def getResolution(self):
        """
        """
    def getRefreshRate(self):
        """
        """
    def getFrameData(self):
        """
        """
    
#   End of Library MLX90640 *********************************************************************************

#   MAX30102 Oximeter Library
MAX30102_ADDR               =   0x00
#Configuration Registers
MAX30105_FIFOCONFIG         =   0x08
MAX30105_MODECONFIG         =   0x09
MAX30105_PARTICLECONFIG     = 	0x0A    # Note, sometimes listed as "SPO2" config in datasheet (pg. 11)
MAX30105_LED1_PULSEAMP      = 	0x0C
MAX30105_LED2_PULSEAMP      = 	0x0D
MAX30105_LED3_PULSEAMP      = 	0x0E
MAX30105_LED_PROX_AMP       =   0x10
MAX30105_MULTILEDCONFIG1    =   0x11
MAX30105_MULTILEDCONFIG2    =   0x12
   
#   MAX30102 Oximeter Library
class MAX30102():
    def __init__(self):
        """
        """
        
    def start(self):
        """
        """
        
    def stop(self):
        """
        """
#   End of library MAX30102 *********************************************************************************

#   some common network routines 
class NetStuff():
    def getHostIp(self):
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(('8.8.8.8', 80))
            ip = "IP: " + s.getsockname()[0]
        except Exception as e:
            ip = "IP does not exist"
        finally:
            s.close()
        return ip
    
    def getWifiContent():
        """
        wifi
        :return: None
        """
        camera.config(size=(224, 224))#224*224
        while 1:
            image=camera.capture()
            #print("image=",image)
            result=decode_qr_code_image(image)#WLAN:WIFI:S:wellgaga2;T:WPA;P:wisdom1234;H:false;;
            image=image.resize((224, 224))
            if result==None:#None
                display.show(image)
                continue
            if result.startswith("WIFI:"):
                r_list=result[5:].split(";")
                ssid=None#ssid
                passwd=None#password
                T=None
                for r in r_list:
                    r_l=r.split(":")#
                    if len(r_l)==2 and r_l[0]=="S":
                        ssid=r_l[1]#ssid
                    elif len(r_l)==2 and r_l[0]=="P":
                        passwd=r_l[1]#passworld
                    elif len(r_l)==2 and r_l[0]=="T":
                        T=r_l[1]#T
                        #print(T)
            if ssid!=None and passwd!=None:
                font = ImageFont.truetype("/home/res/baars.ttf", 18, encoding="unic")
                draw = ImageDraw.Draw(image)
                mywifi="S:%s,P:%s"%(ssid,passwd)
                draw.text((10, 180),mywifi, "#000000", font)
                draw.text((10, 200), "T:%s"%T, "#000000", font)
        display.show(image)
#   End of library NetStuff ********************************************************************************

#   Button manager stuff.
class M2dockKeys():
    
    def __init__(self) -> None:
        from evdev import InputDevice
        self.dev = InputDevice('/dev/input/event0')
        asyncio.ensure_future(self.on_keys(self.dev))

    def key_S1_press(self):
        print('press key S1')
        
    def key_S2_press(self):
        print('press key S2')

    async def on_keys(self, device):
        async for event in device.async_read_loop():
            # print(event)
            await asyncio.sleep(0.05)
            if event.value == 1 and event.code == 0x02:
                self.key_S1_press()
            elif event.value == 1 and event.code == 0x03:
                self.key_S2_press()
            else:
                pass
#   End of library M2DockKeys   ****************************************************************************

#   Quick Response Code Stuff
class QRCode():
    def decodeQRCodeImage(img):
        """
        :param img:
        :return:/None
        """
        if isinstance(img,Image.Image)==False and isinstance(img,numpy.ndarray)==False:
            return None
        info=pyzbar.decode(img, symbols=[pyzbar.ZBarSymbol.QRCODE])
        #print("info=",info)#info= [Decoded(data=b'{"name": "wifi123", "password": "123465", "ip": "192.168.1.10", "port": "9999", "mask": "255.255.255.0", "gate": "192.168.1.1", "dns": "114.114.114.0"}', type='QRCODE', rect=Rect(left=39, top=39, width=490, height=490), polygon=[Point(x=39, y=39), Point(x=42, y=529), Point(x=529, y=529), Point(x=529, y=42)])]
        if len(info)>0:
            return info[0].data.decode("utf-8")
        else:
            return None
#   End of library QRCode   ********************************************************************************