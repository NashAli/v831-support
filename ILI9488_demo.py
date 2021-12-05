#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
    ILI9488_demo.py
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
  	This program tests the new ETMS library for the generic 3.95 parallel i/f ILI9488
  	tft colour display controller in a 16-bit HW config. See docs.

  Commercial use of this library requires you to buy a license that
  will allow commercial use. This includes using the library,
  modified or not, as a tool to sell products.

  The license applies to all part of the library including the 
  examples and tools supplied with the library.
 
  This will include the Header File so that the Source File has access
  to the function definitions in the ILI9488 library.



MaixIIDock(V831) ILI9844 480x320 TFT 3.5 inch LCD
    MaixIIDock(V831)      ->          LCD
        GND               ->          GND(1)
        5V/3.3V           ->          VCC(2)
        PH0(SPI1.0:CLK)   ->          CLK(3)
        PH1(SPI1.0:MOSI)  ->          MOSI(4)
        PH13              ->          RES(5)
        PH8               ->          DC(6)
        H14  N/C          ->          BLK(7)  
        PH2 N/C           ->          MISO(8)
"""

from ETMS import ILI9488

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




TFT = ILI9488()
TFT.clear()
#TFT.fillRect(0,0,480,320,BLACK)
TFT.drawRect(0,0,320,480,MAGENTA)
TFT.fillRect(50,50,50,50,RED)
TFT.fillRect(100,100,50,50,GREEN)
TFT.fillRect(150,150,50,50,BLUE)
TFT.drawRect(100,100,100,100,YELLOW)
TFT.drawTriangle(200,50,175,75,225,90,CYAN)
