#!/usr/bin/env python
# -*- coding: utf-8 -*-

import axis
from pymlab import config

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


motor = axis.axis(SPI = spi, SPI_CS = spi.I2CSPI_SS0)
motor.Reset(KVAL_RUN = 0xa0, KVAL_ACC = 0xaF, KVAL_DEC = 0xaF, FS_SPD = 0xFFFFFF)
motor.MaxSpeed(0x3FF)

#motor.setConfig(EXT_CLK = 0b1, OSC_SEL = 0b110) # Ext clock source: 24 MHz(Crystal/resonator driver disabled)

motor.Float()
print motor.getStatus()


motor.MoveWait(10000)
motor.MoveWait(-10000)
print "aaa"

motor.GoToDir(20000)
motor.wait()
motor.GoTo(0)