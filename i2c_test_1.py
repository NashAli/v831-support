#!/usr/bin/env python
# -*- coding: utf-8 -*-

from maix import i2c

i2cdev = i2c.I2CDevice('/dev/i2c-2', 0x20)
print(i2cdev)
print(i2cdev.read(0x1, 1))
print('start test loop')

for address in range(128):
    try:
        i2cdev = i2c.I2CDevice('/dev/i2c-2', address)
        da = i2cdev.read(0x01,1)
        print(hex(address) + '-OK')
    except:
        print(hex(address) + '-fail')
    else:
        pass
        