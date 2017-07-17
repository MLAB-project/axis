#!/usr/bin/env python
# -*- coding: utf-8 -*-

import axis
from pymlab import config
import time
import json

import logging 
logging.basicConfig(level=logging.INFO) 

cfg = config.Config(
    i2c = {
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
#motor.Reset(KVAL_RUN = 0x20, KVAL_ACC = 0x20, KVAL_DEC = 0x20, FS_SPD = 0x0, stall_th = 203)
#motor.setConfig(F_PWM_INT = None, F_PWM_DEC = None, POW_SR = None, OC_SD = None, RESERVED = None, EN_VSCOMP = None, SW_MODE = None, EXT_CLK = None, OSC_SEL = None)
#motor.Setup(MAX_SPEED = 2000, KVAL_ACC=0.8, KVAL_RUN=0.3, ACC = 100, DEC = 100, FS_SPD=3000)

#motor.setConfig(EXT_CLK = 0b1, OSC_SEL = 0b110) # Ext clock source: 24 MHz(Crystal/resonator driver disabled)

#motor.Float()
print motor.getStatus()


#motor.Run(speed = 1)
#time.sleep(4)
#motor.Float()
#time.sleep(5)

while True:
    print json.dumps(motor.getStatus(), indent=4, sort_keys=True)
    time.sleep(0.10)
