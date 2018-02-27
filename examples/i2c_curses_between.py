#!/usr/bin/env python
# -*- coding: utf-8 -*-

import axis
from pymlab import config
import time
import curses

import logging 
logging.basicConfig(level=logging.INFO) 




class debug(object):
    def __init__(self):
        

        stdscr = curses.initscr()
        stdscr.clear()
        curses.noecho()
        curses.echo()


        begin_x = 20
        begin_y = 7
        height = 5
        width = 40
        win = curses.newwin(height, width, begin_y, begin_x)

        stdscr.addstr(0, 0, "Current mode: Typing mode", curses.A_REVERSE)
        stdscr.refresh()
        #tb = curses.textpad.Textbox(win)
        #text = tb.edit()
        #curses.addstr(4,1,text.encode('utf_8'))


        while True:
            c = stdscr.getch()
            if c == ord('p'):
                PrintDocument()
            elif c == ord('q'):
                break  # Exit the while loop
            elif c == curses.KEY_HOME:
                x = y = 0
        curses.endwin()

if __name__ == '__main__':
    debug()