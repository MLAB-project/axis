#!/usr/bin/env python
# -*- coding: utf-8 -*-

import axis
from pymlab import config
import time

import logging 
logging.basicConfig(level=logging.INFO) 

cfg = config.Config(
    i2c = {
        "device": 'hid',
        "port": 1,
    },

    bus = [
        { 
        "name":"spi", 
        "type":"i2cspi"
        },
    ],
)


cfg.initialize()
spi = cfg.get_device("spi")
spi.SPI_config(spi.I2CSPI_MSB_FIRST| spi.I2CSPI_MODE_CLK_IDLE_HIGH_DATA_EDGE_TRAILING| spi.I2CSPI_CLK_461kHz)


motor = axis.axis(SPI = spi, SPI_CS = spi.I2CSPI_SS0, StepsPerUnit=200)
motor.setConfig(F_PWM_INT = None, F_PWM_DEC = None, POW_SR = None, OC_SD = None, RESERVED = None, EN_VSCOMP = None, SW_MODE = None, EXT_CLK = None, OSC_SEL = None)
motor.Setup(MAX_SPEED = 200, KVAL_ACC=0.8, KVAL_RUN=0.3, ACC = 100, DEC = 100, FS_SPD=3000)

motor.setStepsPerUnit(SPU = 200)

#motor.setConfig(EXT_CLK = 0b1, OSC_SEL = 0b110) # Ext clock source: 24 MHz(Crystal/resonator driver disabled)

motor.Float()
print motor.getStatus()


print("Direction one")
motor.Move(units = 1, direction = False, wait = True)
motor.Move(units = 1, direction = True, wait = True)
motor.Move(units = 1, direction = False, wait = True)
motor.Move(units = 1, direction = True, wait = True)
motor.Move(units = 1, direction = False, wait = True)
motor.Move(units = 1, direction = True, wait = True)
print("Done")
time.sleep(10)

#motor.Run(speed = 1)
#time.sleep(4)
#motor.Float()
#time.sleep(5)

while True:

    print motor.getStatus()
    motor.Float()
    print motor.getStatus()
    motor.MoveWait(20)
    print motor.getStatus()
    time.sleep(10)
    motor.Float()
    motor.MoveWait(-20)
    time.sleep(10)

motor.GoToDir(20000)
motor.wait()
motor.GoTo(0)