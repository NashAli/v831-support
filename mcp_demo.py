#!/usr/bin/env python
# -*- coding: utf-8 -*-
import warnings, time
warnings.filterwarnings("ignore", category=DeprecationWarning) 
from maix import i2c
# IOCON.BANK = 0 on reset.
PortExpander = 0x20
ALLOUTS = 0x00
ALLOFF = 0x00
mcp = i2c.I2CDevice('/dev/i2c-2', PortExpander)

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

init_data1=[IODIRA,ALLOUTS]
init_data2=[IODIRB,ALLOUTS]
port_data1=[OLATA,PINA7]
port_data2=[OLATA,ALLOFF]
port_data3=[OLATA,0xF0]
print('hello, MCP23017 initializing')
X=mcp.read(GPIOA,1)
BX = int.from_bytes(X, byteorder='big')
print('Port:A',hex(BX))
X=mcp.read(GPIOB,1)
BX = int.from_bytes(X, byteorder='big')
print('Port:B',hex(BX))
start=0
end1=len(init_data1)
end2=len(init_data2)
end3=len(port_data1)
end4=len(port_data2)
end5=len(port_data3)
mcp.write(init_data1[0], bytes(init_data1[start+1:end1]))    #configure portA outputs
mcp.write(init_data2[0], bytes(init_data2[start+1:end2]))    #configure portB outputs
print('setting PA7 ON')
mcp.write(port_data1[0], bytes(port_data1[start+1:end3]))    #set port bit PA7 high
X=mcp.read(OLATA,1)
BX = int.from_bytes(X, byteorder='big')
print('OUTLATCH: ', hex(BX))
time.sleep(2)
print('setting ALL OFF')
mcp.write(port_data2[0], bytes(port_data2[start+1:end4]))    #set port bit PA7 low
X=mcp.read(OLATA,1)
BX = int.from_bytes(X, byteorder='big')
print('OUTLATCH: ', hex(BX))
X=mcp.read(GPIOA,1)
BX = int.from_bytes(X, byteorder='big')
print('Port:A',hex(BX))

time.sleep(2)


print('setting PA5-7 ON')
mcp.write(port_data3[0], bytes(port_data3[start+1:end5]))    #set port bits PA5-7 high
time.sleep(2)
print('setting PA5-7 OFF')
mcp.write(port_data2[0], bytes(port_data2[start+1:end4]))    #set port bits PA5-7 low
print('done')
