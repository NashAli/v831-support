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
  ILI9844 480x320 TFT 3.5 inch LCD:(adapted from the ETMS_ST7796S C++ Library)
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
        
MAX30102 Oximeter:
        GND
        5V/3
        SDA (PH12)
        SCK (PH11)
        
CJMCU-8128 Environmental Sensors: BMP280 - 0x76, CCS811 - 0x5A, HDC1080 - 0x40, 
        GND
        5V/3.3V
        SDA (PH12)
        SCK (PH11)

MLX90640 IR Sensor:
        GND
        5V/3.3V
        SDA (PH12)
        SCK (PH11)
        
ONBOARD MSA301:
        address - 0x26
        
"""
from maix import i2c
import time, numbers, spidev, gpiod, traceback, asyncio
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
spi_speed = 40000000    #set to 32Mhz - 40Mhz MAX?

# dictionary usage example
RESOLUTION = {
    "24BIT": 0b000,
    "18BIT": 0b001,
    "16BIT": 0b010,
    "12BIT": 0b011,
    "8BIT":  0b100,
}

# tft defaults

MADCTL_MY = 0x80  #< Bottom to top
MADCTL_MX = 0x40  #< Right to left
MADCTL_MV = 0x20  #< Reverse Mode
MADCTL_ML = 0x10  #< LCD refresh Bottom to top
MADCTL_RGB = 0x00 #< Red-Green-Blue pixel order
MADCTL_BGR = 0x08 #< Blue-Green-Red pixel order
MADCTL_MH = 0x04  #< LCD refresh right to left
ILI9488_WIDTH    = 480
ILI9488_HEIGHT   = 320
_height = ILI9488_HEIGHT
_width = ILI9488_WIDTH
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
ILI9488_ADJCTRL1    = 0xD7
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


#Misc
Buffer = None

#spi
spi = spidev.SpiDev()
#spi.open(bus,device)    #spi /dev/spidev<bus>.<device>
#spi.max_speed_hz=spi_speed# 32Mhz

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

#   ILI9488 TFT Display 480x320 262K colours support library   *********************************************
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
            spi.max_speed_hz=32000000 #spi 32Mhz
            if isinstance(data, numbers.Number):
                data = [data & 0xFF]
            for start in range(0, len(data), chunk_size):
                end = min(start+chunk_size, len(data))
                spi.writebytes(data[start:end])
            spi.close()  #spi
        except:
            traceback.print_exc()
            return None
            
    def recLcd(self):
        """
        """

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
            self.command(ILI9488_SWRESET)
            time.sleep(0.125)
            
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
            
            self.command(ILI9488_MADCTRL)
            self.data(MADCTL_MY|MADCTL_MX|MADCTL_MV|MADCTL_ML|MADCTL_BGR)
            
            #self.command(ILI9488_CABCCTRL9)
            #self.data(0x04)                    #14.11Khz
            self.command(ILI9488_PIXFMT)
            self.data(0x66)                 #RGB666
            time.sleep(0.1)
            self.command(ILI9488_NORMALMODE)
            time.sleep(0.100)
            
            
            self.command(ILI9488_IMC)
            self.data(0x00)                     #using the MISO
            self.command(ILI9488_FRMCTRL)
            self.data(0xA0)                     #60Hz Framerate
            self.data(0x11)
            self.command(ILI9488_DIC)
            self.data(0x02)                     #2-dot
            self.command(ILI9488_DFUNCTRL)      #Display function controll RGB/MCU Interface Control
            self.data([0x02, 0x02, 0x3B])             # MCU - Source,Gate scan direction
            self.command(ILI9488_SETIFUNCT)
            self.data(0x00) #disable the 24bit data bus.
            self.command(ILI9488_ENTRYMODE)
            self.data(0xC6)
            self.command(ILI9488_ADJCTRL3)
            self.data([0xA9, 0x51, 0x2C, 0x82]) # D7 stream, loose
            self.command(ILI9488_WAKE)
            time.sleep(0.200)
            self.command(ILI9488_DISPON)
            time.sleep(0.125)
        except:
            traceback.print_exc()
            return None

    def clear(self):
        self.setAddrWindow()
        for p in range(_width * _height):
            self.data([0,0,0])

    def setAddrWindow(self, x0=0, y0=0, x1=None, y1=None):
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
                x1 = _width-1
            if y1 is None:
                y1 = _height-1
            self.command(ILI9488_CASET)
            self.data(x0 >> 8)
            self.data(x0 & 0xFF)
            self.data(x1 >> 8)
            self.data(x1 & 0xFF)
            
            self.command(ILI9488_PASET)
            self.data(y0 >> 8)
            self.data(y0 & 0xFF)
            self.data(y1 >> 8)
            self.data(y1 & 0xFF)
            self.command(ILI9488_RAMWR)
        except:
            traceback.print_exc()
            return None

    def drawPixel(self, colour):
        """
        Draws a pixel at the position indicated by xpos,ypos and is of colour = colour (RGB888)
        :param xpos:1 - 480
        :param ypos:1 - 320
        :param colour:RGB888
        :return:None
        """
        try:
            r = (colour & 0xFF0000)>>16
            g = (colour & 0x00FF00)>>8
            b = (colour & 0x0000FF)
            #self.setAddrWindow(xpos, ypos, xpos+1, ypos+1)
            self.data([r, g, b])
        except:
            traceback.print_exc()
            return None
            
    def drawLine(self, xs, ys, xd, yd, colour):
        """
        Adapted from ETMS_ST7796S.cpp
        Bresenham's algorithm - thx wikpedia
        """
        steep = abs(yd - ys) > abs(xd - xs)
        if (steep):
            xs, ys = ys, xs
            xd, yd = yd, xd
        if (xs > xd):
            xs, xd = xd, xs
            ys, yd = yd, ys
        dx = xd - xs
        dy = abs(yd - ys)
        err = dx / 2
        if (ys < yd):
            ystep = 1
        else:
            ystep = -1
        for i in range(dx):
            if (steep):
                self.setAddrWindow(ys,i,ys,i)
                self.drawPixel(colour)
            else:
                self.setAddrWindow(i,ys,i,ys)
                self.drawPixel(colour)
            err -= dy
            if (err < 0):
                ys += ystep
                err += dx
	
            
    def drawHLine(self, xs, ys, width, colour):
        """
        Draws a Horizontal Line.
        
        """
        if((xs >= _width) & (ys >= _height)):
            return
        if((xs+width-1) >= _width):
            width = _width-x
        self.setAddrWindow(xs, ys, xs+width-1, ys)
        for x in range(width):
            self.drawPixel(colour)
        
    def drawVLine(self, xs, ys, height, colour):
        """
        """
        self.setAddrWindow(xs, ys, xs, ys+height-1)
        for i in range(height):
            self.drawPixel(colour)

    def drawRect(self, xs, ys, height, width, colour):
        """
        """
        self.drawHLine( xs, ys, width, colour)
        self.drawHLine( xs, ys+height, width, colour)
        self.drawVLine( xs, ys, height, colour)
        self.drawVLine( xs+width, ys, height, colour)
    
    def fillRect(self, xs, ys, h, w, colour):
        try:
            # rudimentary clipping - if rect is outside the screen boundaries reject.
            if((xs >= _width) & (ys >= _height)):
                return
            if((xs + w - 1) >= _width):
                w = _width - xs
            if((ys + h - 1) >= _height):
                h = _height - y
            self.setAddrWindow(xs, ys, xs+w-1, ys+h-1)
            #loop thru x & Y while writing colour here.
            for xpos in range(xs+w-1):
                for ypos in range(ys+h-1):
                    self.drawPixel(colour)
        except:
            traceback.print_exc()
            return None

    def drawTriangle(self, xA, yA, xB, yB, xC, yC, colour):
        self.drawLine(xA, yA, xB, yB, colour)
        self.drawLine(xB, yB, xC, yC, colour)
        self.drawLine(xC, yC, xA, yA, colour)
        
    def drawCircle(self, xo, yo, rad, colour):
        """
        """
            
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
        24bit 888 colour
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
            self.setAddrWindow()
            pixelbytes = list(self.imageToData(image))# read as 24bit RGB888 but will be displayed as RGB666
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
        self.setAddrWindow()
        # Convert image to array of 24bit 888 RGB data bytes.
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
        self.setAddrWindow(pos[0], pos[1], pos[0]+width-1, pos[1]+height-1)
        # Convert image to array of 16bit 565 RGB data bytes.
        pixelbytes = list(self.image_to_data(textimage))
        # Write data to hardware.
        self.data(pixelbytes)      
#   End of library ILI9488  *********************************************************************************
