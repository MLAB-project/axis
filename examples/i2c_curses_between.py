#!/usr/bin/env python
# -*- coding: utf-8 -*-

import axis
from pymlab import config
import time
import curses

import logging 
logging.basicConfig(level=logging.INFO) 




class debug(object):



    def init_i2c(self):
        cfg = config.Config(
            i2c = {
                "device": 'hid',
                "led": True,
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


        self.motor = axis.axis_between(SPI = spi, SPI_CS = spi.I2CSPI_SS0, StepsPerUnit=1)
        self.motor.setConfig(F_PWM_INT = None, F_PWM_DEC = None, POW_SR = None, OC_SD = None, RESERVED = None, EN_VSCOMP = None, SW_MODE = None, EXT_CLK = None, OSC_SEL = None)
        self.motor.Setup(MAX_SPEED = 200, KVAL_ACC=0.3, KVAL_RUN=0.3, ACC = 100, DEC = 100, FS_SPD=3000)

        self.motor.setMaxSpeed(speed = 900)
        self.motor.setMinSpeed(speed = 200*16, LSPD_OPT = True)




    def update_status(self, msg):
        win = self.statuswin
        win.clear()
        win.box()
        for i, m in enumerate(msg):
            try:
                row = int((i*10+1) / 150)*2+1
                col = int((i*10+1) % 150)
                win.addstr(row, col, "  "+str(m))
                win.addstr(1+row, col, " > "+str(msg[m]))
            except Exception as e:
                pass

    def __init__(self):
        self.init_i2c()

        direc = True

        screen = curses.initscr()
        screen.immedok(True)
        screen.nodelay(True)
        screen.clear()
        #curses.noecho()
        curses.echo()


        begin_x = 20
        begin_y = 7
        height = 5
        width = 40

        win = curses.newwin(height, width, begin_y, begin_x)
        self.statuswin = curses.newwin(7, 165, 1, 1)
        self.statuswin.immedok(True)
        self.statuswin.box()
        self.statuswin.refresh()

        #screen.addstr(0, 0, "Current mode: Typing mode", curses.A_REVERSE)
        #screen.refresh()
        #tb = curses.textpad.Textbox(win)
        #text = tb.edit()
        #curses.addstr(4,1,text.encode('utf_8'))

        self.motor.Run(speed=200)

        while True:
            c = screen.getch()
            if c == ord('p'):
                PrintDocument()
            elif c == ord('q'):
                self.motor.Float()
                break

            elif c == ord('s'):
                self.motor.Float()
            elif c == ord('r'):
                direc = not direc
                self.motor.Run(speed = 200, direction = direc)
            elif c == ord('<'):
                self.motor.Move(10)
            elif c == ord(';'):
                self.motor.Move(-10)
            elif c == curses.KEY_HOME:
                x = y = 0
            self.update_status(self.motor.getStatus())

        curses.endwin()

if __name__ == '__main__':
    debug()