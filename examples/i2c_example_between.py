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
        "led": False
    },
    bus = [
        {
        "name":"spi",
        "type":"i2cspi"
        },
        {
        "name": "usbi2c",
        "type": "USBI2C_gpio"
        }
    ])


cfg.initialize()
spi = cfg.get_device("spi")
usbi2c = cfg.get_device("usbi2c")
spi.SPI_config(spi.I2CSPI_MSB_FIRST| spi.I2CSPI_MODE_CLK_IDLE_HIGH_DATA_EDGE_TRAILING| spi.I2CSPI_CLK_461kHz)


motor = axis.axis_between(SPI = spi, SPI_CS = spi.I2CSPI_SS0, StepsPerUnit=1)
motor.setConfig(F_PWM_INT = None, F_PWM_DEC = None, POW_SR = None, OC_SD = None, RESERVED = None, EN_VSCOMP = None, SW_MODE = None, EXT_CLK = None, OSC_SEL = None)
motor.Setup(MAX_SPEED = 200, KVAL_ACC=0.3, KVAL_RUN=0.3, ACC = 100, DEC = 100, FS_SPD=3000)

motor.setMaxSpeed(speed = 500)
motor.setMinSpeed(speed = 400, LSPD_OPT = True)

#motor.setConfig(EXT_CLK = 0b1, OSC_SEL = 0b110) # Ext clock source: 24 MHz(Crystal/resonator driver disabled)

motor.Float()
print(motor.getStatus())


#TODO: Tohle pak pojmenovat jako ''DirToHome''
def_dir = False
gpio_pins = [0,1]
data = {
    'dirToHome': def_dir,
    'GPIO_pins': gpio_pins
    }

print("-------------")
print("MIN rychlost je: ",motor.getMinSpeed())
print("MAX rychlost je: ",motor.getMaxSpeed())


#motor.search_range()

usbi2c.setup(0, usbi2c.OUT, usbi2c.PUSH_PULL)
usbi2c.setup(1, usbi2c.OUT, usbi2c.PUSH_PULL)
usbi2c.output(0, 0)
usbi2c.output(1, 0)

(a,b) = motor.validate_switch(usbi2c, gpio_pins)
help(motor)

if motor.getStatus()['SW_F']:
    return("Je zmacknute tlacitku, ukoncuji")


motor.Run(def_dir, 100)

status = motor.getStatus()
while not status['SW_F']:
    status = motor.getStatus()
    print(status['POSITION'])
    time.sleep(1)

(a,b) = motor.validate_switch(usbi2c, gpio_pins)
print("Koncaky", a, b)

motor.ReleaseSW(not def_dir)
motor.Wait()
print("Konca uvolnen")
motor.ResetPos()

data['start_position'] =  motor.getStatus()['POSITION']

motor.Run(def_dir, -100)
status = motor.getStatus()
while not status['SW_F']:
    status = motor.getStatus()
    print(status['POSITION'])
    time.sleep(1)

(a,b) = motor.validate_switch(usbi2c, [0,1])
print("Koncaky", a, b)
motor.ReleaseSW(def_dir)
motor.Wait()
data['lenght'] =  data['start_position'] - motor.getStatus()['POSITION']

print(data)

motor.GoToDir(int(data['lenght']/2), def_dir, def_dir)



motor.Float()
