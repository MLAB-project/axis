#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
import sys
import datetime
import math

class axis():
    STEP_MODE_FULL = [0b000, 1]
    STEP_MODE_HALF = [0b001, 2]
    STEP_MODE_1_2 = [0b001, 2]
    STEP_MODE_1_4 = [0b010, 4]
    STEP_MODE_1_8 = [0b011, 8]
    STEP_MODE_1_16 = [0b100, 16]
    STEP_MODE_1_32 = [0b101, 32]
    STEP_MODE_1_64 = [0b110, 64]
    STEP_MODE_1_128 = [0b111, 128]

    def __init__(self, SPI, SPI_CS, Direction  = True, StepsPerUnit = 1, protocol = 'i2c', arom_spi_name = 'spi'):
        ' One axis of robot '
        self.CS = SPI_CS
        self.Dir = Direction
        self.spi = SPI
        self.protocol = protocol
        self.arom_spi_name = arom_spi_name
        self.SPU = StepsPerUnit
        self.sw_range = None
        self.last_status = {}
        self.last_pos = 0
        self.last_pos_date = 0
        self.statusCallback = None

        self.virtualPosition = 0
        self.virtualPositionLast = 0
        self.virtualDeltaLast = None
        self.virtualRound = 0

        self.virtualGoToLastTarget = 0

        self.L6470_ABS_POS      =0x01
        self.L6470_EL_POS       =0x02
        self.L6470_MARK         =0x03
        self.L6470_SPEED        =0x04
        self.L6470_ACC          =0x05
        self.L6470_DEC          =0x06
        self.L6470_MAX_SPEED    =0x07
        self.L6470_MIN_SPEED    =0x08
        self.L6470_FS_SPD       =0x15
        self.L6470_KVAL_HOLD    =0x09
        self.L6470_KVAL_RUN     =0x0A
        self.L6470_KVAL_ACC     =0x0B
        self.L6470_KVAL_DEC     =0x0C
        self.L6470_INT_SPEED    =0x0D
        self.L6470_ST_SLP       =0x0E
        self.L6470_FN_SLP_ACC   =0x0F
        self.L6470_FN_SLP_DEC   =0x10
        self.L6470_K_THERM      =0x11
        self.L6470_ADC_OUT      =0x12
        self.L6470_OCD_TH       =0x13
        self.L6470_STALL_TH     =0x14
        self.L6470_STEP_MODE    =0x16
        self.L6470_ALARM_EN     =0x17
        self.L6470_CONFIG       =0x18
        self.L6470_STATUS       =0x19

        self.config_F_PWM_INT = 0b001
        self.config_F_PWM_DEC = 0b110
        self.config_POW_SR    = 0b00
        self.config_OC_SD     = 0b0
        self.config_RESERVED  = 0b0
        self.config_EN_VSCOMP = 0b0
        self.config_SW_MODE   = 0b0
        self.config_EXT_CLK   = 0b0
        self.config_OSC_SEL   = 0b000

        self.Setup(ResetDevice=True)



    def writeByte(self, address):
        if self.protocol == 'i2c':
            return self.spi.SPI_write_byte(self.CS, address)
        elif self.protocol == 'spi':
            return "Err. SPI neni podporovano"
        elif self.protocol == 'arom':
            response = self.spi(device=self.arom_spi_name, method="SPI_write_byte", parameters=str((self.CS,address)))
            return eval(response.value) # self.spi.I2CSPI_MSB_FIRST| self.spi.I2CSPI_MODE_CLK_IDLE_HIGH_DATA_EDGE_TRAILING| self.spi.I2CSPI_CLK_461kHz

    def readByte(self):
        if self.protocol == 'i2c':
            return self.spi.SPI_read_byte()
        elif self.protocol == 'spi':
            return "Err. SPI neni podporovano"
        elif self.protocol == 'arom':
            return eval(self.spi(device=self.arom_spi_name, method="SPI_read_byte").value)

    def writeData(self):
        if self.protocol == 'i2c':
            return "Err. SPI neni podporovano"
        elif self.protocol == 'spi':
            return "Err. SPI neni podporovano"
        elif self.protocol == 'arom':
            return eval(self.spi(device=self.arom_spi_name, method="SPI_write_data", parameters=str((self.CS,address))).value) # self.spi.I2CSPI_MSB_FIRST| self.spi.I2CSPI_MODE_CLK_IDLE_HIGH_DATA_EDGE_TRAILING| self.spi.I2CSPI_CLK_461kHz


    def Setup(self, ResetDevice = True, ACC = None, DEC = None, STALL_TH = None, OCD_TH = None,
        KVAL_HOLD = None, KVAL_RUN = None, KVAL_ACC = None, KVAL_DEC = None, FS_SPD = None, STEP_MODE = None,
        StepsPerUnit = None, MIN_SPEED = None, MAX_SPEED = None, CONFIG = None):

        '''
        Reset Axis and set default parameters for H-bridge

        Keyword arguments:

        name -- desc (default: def)

        ACC -- Acceleration (default: 0x008A)
        DEC -- Deceleration (default: 0x0081)
        stall_th -- Stall treshold value in mA (default: 2030)
        ocd_th -- Over current treshold in mA (default: 3380)
        KVAL_HOLD -- Holding KVAL (default: 0x29)
        KVAL_RUN -- Constant speed KVAL (default: 0x29)
        KVAL_ACC -- Acceleration startingKVAL (default: 0x29)
        KVAL_DEC -- Deceleration startingKVAL (default: 0x29)
        FS_SPD -- Fullstep speed (default: 0x027)
        '''
        if ResetDevice:
            self.writeByte( 0b11000000)

        self.SPU = 1

        if STALL_TH:
            if STALL_TH > 4000:
                STALL_TH = 4000

            self.writeByte( self.L6470_STALL_TH)      # Stall Treshold setup
            self.writeByte( int(STALL_TH/31.25)-1)      # 0x70 = 3.5A

        if OCD_TH:
            if OCD_TH > 6000:
                OCD_TH = 6000

            self.writeByte( self.L6470_OCD_TH)      # Over Current Treshold setup
            self.writeByte( int(OCD_TH/375)-1)      # 0x0A = 4A

        if FS_SPD:
            if FS_SPD < 0: FS_SPD = 0
            if FS_SPD > 15610: FS_SPD=15610
            FS_SPD_value = int(((FS_SPD+0.5) * 250e-9)/(2**-18))
            self.writeByte( self.L6470_FS_SPD)      # Full Step speed, 0x03FF - maximal - always microstepping
            self.writeByte( (FS_SPD_value >> 8) & 0xFF)
            self.writeByte( (FS_SPD_value >> 0) & 0xFF)

        # KAVL hodnoty v rozsahu 0-1
        # KVAL values in range 0-1

        if KVAL_HOLD:
            if KVAL_HOLD < 0: KVAL_HOLD = 0
            elif KVAL_HOLD > 0.9961: KVAL_HOLD = 0.9961
            self.writeByte( self.L6470_KVAL_HOLD)
            self.writeByte( KVAL_HOLD)

        if KVAL_RUN:
            if KVAL_RUN < 0: KVAL_RUN = 0
            elif KVAL_RUN > 0.9961: KVAL_RUN = 0.9961
            self.writeByte( self.L6470_KVAL_RUN)
            self.writeByte( int(255.0*KVAL_RUN))

        if KVAL_ACC:
            if KVAL_ACC < 0: KVAL_ACC = 0
            elif KVAL_ACC > 0.9961: KVAL_ACC = 0.9961
            self.writeByte( self.L6470_KVAL_ACC)
            self.writeByte( int(256.0*KVAL_ACC))

        if KVAL_DEC:
            if KVAL_DEC < 0: KVAL_DEC = 0
            elif KVAL_DEC > 0.9961: KVAL_DEC = 0.9961
            self.writeByte( self.L6470_KVAL_DEC)
            self.writeByte( int(256.0*KVAL_DEC))

        if ACC:
            if ACC < 14.55: ACC = 14.55
            elif ACC > 59590: ACC = 59590
            ACC_value = int(ACC * (250e-9)**2/(2**-40))
            if ACC_value == 0xFFF: ACC_value = 0xFFE
            self.writeByte( self.L6470_ACC)      # ACC (0x008a)
            self.writeByte( (ACC_value >> 8) & 0xFF)
            self.writeByte( (ACC_value >> 0) & 0xFF)

        if DEC:
            if DEC < 14.55: DEC = 14.55
            elif DEC > 59590: DEC = 59590
            DEC_value = int((DEC * (250e-9)**2)/(2**-40))
            self.writeByte( self.L6470_DEC)      # DEC (0x008a)
            self.writeByte( (DEC_value >> 8) & 0xFF)
            self.writeByte( (DEC_value >> 0) & 0xFF)

        if MAX_SPEED:
            if MAX_SPEED < 15.25: MAX_SPEED = 15.25
            elif MAX_SPEED > 15610: MAX_SPEED = 15610
            speed_value = int((MAX_SPEED * 250e-9)/(2**-18))
            data = [(speed_value >> i & 0xff) for i in (8,0)]
            self.writeByte( self.L6470_MAX_SPEED)       # Max Speed setup
            self.writeByte( data[0])
            self.writeByte( data[1])

        if MIN_SPEED:
            lspd = 0
            if LSPD_OPT: lspd = 0b10000000

            if (MIN_SPEED < 0): MIN_SPEED = 0
            elif (MIN_SPEED > 976.3): MIN_SPEED = 976.3

            speed_value = int((MIN_SPEED * 250e-9)/(2**-24))

            data = [(speed_value >> i & 0xff) for i in (8,0)]

            self.writeByte( self.L6470_MIN_SPEED)       # Min Speed setup
            self.writeByte( data[0] | lspd)
            self.writeByte( data[1])

        if CONFIG:
            print("config:", CONFIG)

        if not STEP_MODE: STEP_MODE = self.STEP_MODE_1_16
        print("strep mode:", STEP_MODE)
        self.set_stepmode(STEP_MODE)    


    def set_stepmode(self, mode):
            self.writeByte( self.L6470_STEP_MODE)      # Microstepping
            self.writeByte( mode[0])
            self.microstepping = mode[1]

    def Reset(self, init = True, ACC = 0x00a, DEC = 0x00a, stall_th = 2030, ocd_th = 3380, KVAL_HOLD = 0x29, KVAL_RUN = 0x29, KVAL_ACC = 0x29, KVAL_DEC = 0x29, FS_SPD = 0x027):
        '''
        Reset Axis and set default parameters for H-bridge

        Keyword arguments:

        name -- desc (default: def)

        ACC -- Acceleration (default: 0x008A)
        DEC -- Deceleration (default: 0x0081)
        stall_th -- Stall treshold value in mA (default: 2030)
        ocd_th -- Over current treshold in mA (default: 3380)
        KVAL_HOLD -- Holding KVAL (default: 0x29)
        KVAL_RUN -- Constant speed KVAL (default: 0x29)
        KVAL_ACC -- Acceleration startingKVAL (default: 0x29)
        KVAL_DEC -- Deceleration startingKVAL (default: 0x29)
        FS_SPD -- Fullstep speed (default: 0x027)
        '''

        if init:
            self.writeByte( 0xC0)      # reset OC

        if stall_th > 4000:
            stall_th = 4000

        self.writeByte( self.L6470_STALL_TH)      # Stall Treshold setup
        self.writeByte( int(stall_th/31.25)-1)      # 0x70 = 3.5A

        if ocd_th > 6000:
            ocd_th = 6000

        self.writeByte( self.L6470_OCD_TH)      # Over Current Treshold setup
        self.writeByte( int(ocd_th/375)-1)      # 0x0A = 4A

        self.writeByte( self.L6470_FS_SPD)      # Full Step speed, 0x03FF - maximal - always microstepping
        #self.writeByte( (FS_SPD >>16) & 0xFF)
        self.writeByte( (FS_SPD >> 8) & 0xFF)
        self.writeByte( (FS_SPD >> 0) & 0xFF)


        self.writeByte( self.L6470_ACC)      # ACC (0x008a)
        self.writeByte( (ACC >> 8) & 0xFF)
        self.writeByte( (ACC >> 0) & 0xFF)
        self.writeByte( self.L6470_DEC)      # DEC (0x008a)
        self.writeByte( (DEC >> 8) & 0xFF)
        self.writeByte( (DEC >> 0) & 0xFF)

        self.writeByte( self.L6470_KVAL_HOLD)
        self.writeByte( KVAL_HOLD)
        self.writeByte( self.L6470_KVAL_RUN)
        self.writeByte( KVAL_RUN)
        self.writeByte( self.L6470_KVAL_ACC)
        self.writeByte( KVAL_ACC)
        self.writeByte( self.L6470_KVAL_DEC)
        self.writeByte( KVAL_DEC)

        self.MinSpeed(speed = 0x00, LSPD_OPT = True)

        self.setConfig(F_PWM_INT = 0b001, F_PWM_DEC = 0b110, POW_SR = 0b00, OC_SD = 0b0, RESERVED = 0b0, EN_VSCOMP =  0b1, SW_MODE = 0b0, EXT_CLK = 0b0, OSC_SEL = 0b000)


        self.writeByte( self.L6470_STEP_MODE)      # Microstepping
        self.writeByte( 0x04)      # 0x04 - 1/16
        self.microstepping = 16


    def clearStatus(self):
        self.writeByte( self.L6470_ABS_POS)
        self.writeByte( 0x00)
        self.writeByte( 0x00)
        self.writeByte( 0x00)

    def setStepsPerUnit(self, SPU):
        if SPU > 0:
            self.SPU = SPU

    def setSPU(self, SPU):
        self.setStepsPerUnit(SPU)


    def setConfig(self, F_PWM_INT = None, F_PWM_DEC = None, POW_SR = None, OC_SD = None, RESERVED = None, EN_VSCOMP = None, SW_MODE = None, EXT_CLK = None, OSC_SEL = None):

        F_PWM_INT = F_PWM_INT if F_PWM_INT else self.config_F_PWM_INT
        F_PWM_DEC = F_PWM_DEC if F_PWM_DEC else self.config_F_PWM_DEC
        POW_SR = POW_SR if POW_SR else self.config_POW_SR
        OC_SD = OC_SD if OC_SD else self.config_OC_SD
        RESERVED = RESERVED if RESERVED else self.config_RESERVED
        EN_VSCOMP = EN_VSCOMP if EN_VSCOMP else self.config_EN_VSCOMP
        SW_MODE = SW_MODE if SW_MODE else self.config_SW_MODE
        EXT_CLK = EXT_CLK if EXT_CLK else self.config_EXT_CLK
        OSC_SEL = OSC_SEL if OSC_SEL else self.config_OSC_SEL

        config = F_PWM_INT << 13 | F_PWM_DEC << 10 | POW_SR << 8 | OC_SD << 7 | RESERVED << 6 | EN_VSCOMP << 5 | SW_MODE << 4 | EXT_CLK << 3 | OSC_SEL << 0

        self.config_F_PWM_INT = F_PWM_INT
        self.config_F_PWM_DEC = F_PWM_DEC
        self.config_POW_SR = POW_SR
        self.config_OC_SD = OC_SD
        self.config_RESERVED = RESERVED
        self.config_EN_VSCOMP = EN_VSCOMP
        self.config_SW_MODE = SW_MODE
        self.config_EXT_CLK = EXT_CLK
        self.config_OSC_SEL = OSC_SEL

        print(bin(config))

        data = [(config >> i & 0xff) for i in (8,0)]

        self.writeByte( self.L6470_CONFIG)
        self.writeByte( data[0])
        self.writeByte( data[1])

        return config


    def MaxSpeed(self, speed):
        self.setMaxSpeed(speed)

    def setMaxSpeed(self, speed):
        ' Setup of maximum speed  - 15.25 to 1560 steps/s'
        if speed < 15.25: speed = 15.25
        elif speed > 15610: speed = 15610

        speed_value = int((speed * 250e-9)/(2**-18))

        data = [(speed_value >> i & 0xff) for i in (8,0)]

        self.writeByte( self.L6470_MAX_SPEED)       # Max Speed setup
        self.writeByte( data[0])
        self.writeByte( data[1])
        return speed


    def MinSpeed(self, speed, LSPD_OPT = True):
        self.setMinSpeed(speed, LSPD_OPT)

    def setMinSpeed(self, speed, LSPD_OPT = True):
        ' Setup of minimum speed  - 0 to 976 steps/s'

        if (speed < 0): speed = 0
        elif (speed > 976.3): speed = 976.3

        speed_value = int((speed * 250e-9)/(2**-24)) | int(LSPD_OPT)<<12

        data = [(speed_value >> i & 0xff) for i in (8,0)]


        self.writeByte( self.L6470_MIN_SPEED)       # Max Speed setup
        self.writeByte( data[0])
        self.writeByte( data[1])
        return speed

    def getMinSpeed(self):
        spd = self.getParam(self.L6470_MIN_SPEED)>>8
        spd = (spd & 0b111111111111) * (2.0**-24.0) / 250.0e-9
        return spd

    def getMaxSpeed(self):
        spd = self.getParam(self.L6470_MAX_SPEED)>>8
        spd = spd * (2**-18) / (250e-9)
        return spd




    '''
          APPLICATION COMMANDS
        ========================
    '''


    def GoTo(self, abspos, wait=False, float = True):
        data = [(abspos*self.SPU >> i & 0xff) for i in (16,8,0)]

        self.writeByte( 0b01100000)
        self.writeByte( data[0])
        self.writeByte( data[1])
        self.writeByte( data[2])

        while self.IsBusy() and wait:
            time.sleep(0.25)

        if wait and float:
            self.Float()

        return abspos

    def GoToDir(self, speed, direction = 1, wait=False, float = True):
        data = [(speed*self.SPU >> i & 0xff) for i in (16,8,0)]

        self.writeByte( 0b01101000 + int(direction))
        self.writeByte( data[0])
        self.writeByte( data[1])
        self.writeByte( data[2])

        while self.IsBusy() and wait:
            time.sleep(0.25)

        if wait and float:
            self.Float()

        return speed


    def goto_virtual(self, position, direction = None, wait = False, currentPosition = None, moveDistance = 2**16, speed = 60000):
        status = self.getStatus()
        if not currentPosition:
            currentPosition = status.get('VIRTUAL_POSITION')
        delta = currentPosition-position
        if self.virtualDeltaLast == None:
            print("Prvni delta...")
            self.Float()
            self.virtualDeltaLast = delta
        move = bool(status.get('SPEED_RAW'))

        l = 0b11111111111111111111

        #print("\t \t \t \t \t \t Pos: {:08.0f}, cur: {:08.0f}, err: {:08.0f}, check: {:08.0f}".format(position, currentPosition, delta, abs(delta) >  3000*128))


        if delta > l: delta = l
        if delta < -l: delta = -l
        print("Delta", delta)
        if abs(delta/128) < 500:
            self.virtualDeltaLast = None
            return 0
        self.Move(delta/128.0)


        #print(delta, abs(delta) < 500*128)
        #if not move:
        #    if delta > 0:
        #        print("Hybej se! a")
        #        self.Run(0, 80000)
        #    if delta < 0:
        #        print("Hybej se! b")
        #        self.Run(1, 80000)
        #    return 1
        #else:
        #    if abs(delta) < 500*128:
        ##        self.Float()
        #        print("OK")
        #        return 0
        #    return 1
        '''
        l = 0b111111111111


        print("\t \t \t \t \t \t Pos: {}, cur: {}, err: {}, check: {}".format(position, currentPosition, delta, abs(delta) >  3000*128))
        

        if abs(delta) > 2000*128 and not move:
            if delta > 0:
                #self.Move(2000*128, mode='steps', wait=True)
                self.MoveWait(2100)
                print("Move 1", 2100, position, currentPosition, delta)
            else:
                #self.Move(-2000*128, mode='steps', wait=True)
                self.MoveWait(-2100)
                print("Move 2", -2100, position, currentPosition, delta)
            return 2


        elif abs(delta) < 300:
            print("MOVEVIRTUAL - DONE")            
            self.virtualDeltaLast = None
            return 0


        elif delta < 0 and not move:
            #self.Float()
            #self.Wait()
            currentPosition = self.virtual_position()
            delta = (currentPosition-position)
            self.MoveWait(-50)
            newPosition = self.virtual_position()
            #self.Move(delta, wait = True, mode='steps')
            print("Move 3", position, currentPosition, delta, currentPosition-newPosition)
            return 1

        elif delta > 0 and not move:
            #self.Float()
            #self.Wait()
            currentPosition = self.virtual_position()
            delta = (currentPosition-position)
            self.MoveWait(50)
            newPosition = self.virtual_position()
            #self.Move(delta, wait = True, mode='steps')
            print("Move 3", position, currentPosition, delta, currentPosition-newPosition)
            return 1
        '''

        '''
        elif abs(delta) <= 2000*128 and not move:
            self.Float()
            self.Wait()
            currentPosition = self.virtual_position()
            delta = (currentPosition-position)
            self.MoveWait((delta/200.0))
            newPosition = self.virtual_position()
            #self.Move(delta, wait = True, mode='steps')
            print("Move 3", position, currentPosition, delta, currentPosition-newPosition)
            return 1
        '''

        '''
        # pokud se nam chyba zvetsuje
        if abs(delta) > abs(self.virtualDeltaLast):
            print("Error is getting bigger")
            self.Float()
            virtualGoToState = 4
            self.virtualDeltaLast = delta

        # Pokud motor neni na spravne hodnote
        if self.virtualGoToLastTarget != position:
            print("GOTOV INIT")
            self.virtualGoToState = 4
            self.virtualGoToLastTarget = position

        #print(position, currentPosition, "ERR", currentPosition-position, "dist", abs(currentPosition - position) < moveDistance, moveDistance)
        print("\t \t \t \t \t \t Pos: {}, cur: {}, err: {}, check: {}, dist: {}".format(position, currentPosition, currentPosition-position, abs(currentPosition - position) < moveDistance, moveDistance))
        

        if -200 < delta and 200 > delta:
            if not self.virtualGoToState:
                self.Float()
                print("GOTOV ----- DONE :)")
            self.virtualGoToState = 0
            return 0

        # pokud je motor v rozsahu registru, pouzij metodu MOVE
        elif -1*moveDistance < delta and moveDistance > delta and self.virtualGoToState > 2:
            print("GOTOV ----- BETWEEN")
            self.virtualGoToState = 2
            self.Float()
            self.Wait()
            time.sleep(0.2)
            currentPosition = self.virtual_position()
            self.Move(currentPosition-position, wait = wait, mode='steps')

        # pokud je motor v rozsahu vnitniho registru, ale zastavil se
        elif -1*moveDistance < delta and moveDistance > delta:
            if not self.getStatus().get('BUSY', False):
                print("Should be false:", self.getStatus().get('BUSY', "Err"))
                print("GOTOV ----- BETWEEN - repair")
                self.virtualGoToState = 3
        
        elif delta < (-1*moveDistance) and self.virtualGoToState != -3:
            print("GOTOV ----- ERR -")
            self.Run(True, speed)
            self.virtualGoToState = -3

        elif delta > ( 1*moveDistance) and self.virtualGoToState != 3:
            print("GOTOV ----- ERR +")
            self.Run(False, speed)
            self.virtualGoToState = 3
        
        '''
        return -1


        '''
        if self.virtualGoToLastTarget != position:
            print("\t \t \t \t \t \t \t \t \t \t GOTOV INIT")
            self.virtualGoToState = 4
            self.virtualGoToLastTarget = position

        if abs(currentPosition - position) == 0:
            print("\t \t \t \t \t \t \t \t \t \t GOTOV DONE")
            self.virtualGoToState = 1
            return 0

        if abs(currentPosition - position) < moveDistance and self.virtualGoToState != 2:
            print("\t \t \t \t \t \t \t \t \t \t GOTOV MOVE ..............", position-currentPosition)
            self.virtualGoToState = 2
            #self.Run(0, 0)
            self.Float()
            time.sleep(2)
            self.Wait()
            currentPosition = self.virtual_position()
            self.Move(position-currentPosition, wait = wait)
            return 1
        
        if self.virtualGoToState > 3:
            print('\t \t \t \t \t \t \t \t \t \t GOTOV RUN', (currentPosition-position) < 0)
            self.virtualGoToState = 3
            self.Run((currentPosition-position) < 0, speed)
            return 2
        '''

        return -1


    def GoUntil(self, direction, speed, ACT = True):
        data = [(int(self._IOspeed(speed)) >> i & 0xff) for i in (16,8,0)]

        self.writeByte( (0b10000010 | int(ACT)<<3 | int(direction)<<0))
        self.writeByte( data[0])
        self.writeByte( data[1])
        self.writeByte( data[2])

        return speed

    def ReleaseSW(self, direction = 1, ACT = True):
        ' Go away from Limit Switch '

        self.writeByte( (0b10010010 | int(ACT)<<3 | int(direction)<<0))

        #while self.ReadStatusBit(2) == 1:
        #    self.writeByte( 0b10010010 + int(direction))
        #    time.sleep(0.25)


    def GoZero(self, speed):
        ' Go to Zero position '
        self.ReleaseSW()

        self.writeByte( 0x82 | (self.Dir & 1))       # Go to Zero
        self.writeByte( 0x00)
        self.writeByte( speed)
        while self.IsBusy():
            time.sleep(0.25)
        self.ReleaseSW()


    def GoHome(self, wait = True):
        ' Go to Zero position '
        self.ReleaseSW()

        self.writeByte( 0x70)       # Go to Zero
        self.writeByte( 0x00)
        while self.IsBusy() and wait:
            time.sleep(0.25)


    def ResetPos(self):
        self.writeByte( 0xD8)       # Reset position
        self.writeByte( 0x00)
        #self.writeData(self.CS, [0xD8, 0x00])

    def ResetDevice(self):
        self.writeByte( 0xC0)       # Reset device

    def SoftStop(self):
        self.writeByte( 0xB0)       # SoftStop

    def HardStop(self):
        self.writeByte( 0xB8)       # HardStop


    def Move(self, units = 0, direction = 0, wait = False, mode = 'units'):
        ' Move some distance units from current position'

        if mode == 'steps':
            steps = int(units)
        else:
            steps = int(abs(self._units2steps(units)))
        #steps = units

        print("Move: steps: %s, units %s" %(steps, units))

        direction = bool(direction)
        if units < 0:
            direction = not direction

        data = [(steps >> i & 0xff) for i in (16,8,0)]

        #print "move data arr", data, bin(0b01000000 + int(direction))[2:].zfill(8), bin(data[0])[2:].zfill(8), bin(data[1])[2:].zfill(8), bin(data[2])[2:].zfill(8)

        self.writeByte( 0b01000000 + int(direction))
        self.writeByte( data[0])
        self.writeByte( data[1])
        self.writeByte( data[2])

        time.sleep(0.1)
        if wait: self.Wait()

        return steps


    def MoveWait(self, units, direction = 0):
        ' Move some distance units from current position and wait for execution '
        self.Move(units = units, direction = direction, wait = True)


    def Run(self, direction = 0, speed = 0):
        speed_value = abs(self._IOspeed(speed))

        if speed < 0: direction = not bool(direction)
        #if speed_value > 0x000FFFFF: speed_value = 0x000FFFFF

        data = [(speed_value >> i & 0xff) for i in (16,8,0)]
        self.writeByte( 0b01010000 | bool(direction))
        self.writeByte( data[0])
        self.writeByte( data[1])
        self.writeByte( data[2])

        print("Run(%s, %s [steps/s])" %(bool(direction), int(speed)))

        return self._Speed(speed_value)


    def Float(self):
        ' switch H-bridge to High impedance state '
        #print("switch H-bridge to High impedance state")
        self.writeByte( 0xA0)


    def ReadStatusReg(self):
        ' Read status register '

        return self.getParam(0x19)

        '''
        self.writeByte( 0x20 | 0x19)   # Read from address 0x19 (STATUS)
        self.writeByte( 0x00)
        data = [self.readByte()]
        self.writeByte( 0x00)
        data += [self.readByte()]
        return (data[0] << 8 | data[1])
        '''

    def ReadStatusBit(self, bit):
        ' Report given status bit '

        self.writeByte( 0x20 | 0x19)   # Read from address 0x19 (STATUS)
        self.writeByte( 0x00)
        data = self.readByte() << 8
        self.writeByte( 0x00)
        data |= self.readByte()

        return (data >> bit) & 1


    def getStatus(self):
        try:
            #print "GetStatus"
            self.writeByte( 0x20 | 0x19)   # Read from address 0x19 (STATUS)
            self.writeByte( 0x00)
            data = [self.readByte()]      # 1st byte
            self.writeByte( 0x00)
            data += [self.readByte()]
            #print data

            self.writeByte( 0x20 | 0x04)   # Read from address 0x04 (SPEED)
            self.writeByte( 0x00)
            spd = [self.readByte()]
            self.writeByte( 0x00)
            spd += [self.readByte()]
            self.writeByte( 0x00)
            spd += [self.readByte()]
            #print('spd: {}'.format(spd))

            spd_c = 0
            pos = self.getPosition()
            pos2 = self.getPosition()
            #if abs(pos - pos2) > 10000:
            #    print(pos, pos2)
            #    #raise Exception("Driver read error")
            vpos = self.virtual_position(pos)
            #if (self.last_pos_date + 10) < time.time():
            #    spd_c = (self.last_pos - pos)/(self.last_pos_date - time.time())
            #    self.last_pos = pos
            #    self.last_pos_date = time.time()

            status = dict()
            status = dict([('SCK_MOD',data[0] & 0x80 == 0x80),  #The SCK_MOD bit is an active high flag indicating that the device is working in Step-clock mode. In this case the step-clock signal should be provided through the STCK input pin. The DIR bit indicates the current motor direction
                        ('STEP_LOSS_B',data[0] & 0x40 == 0x40),
                        ('STEP_LOSS_A',data[0] & 0x20 == 0x20),
                        ('OCD',data[0] & 0x10 == 0x10),
                        ('TH_SD',data[0] & 0x08 == 0x08),
                        ('TH_WRN',data[0] & 0x04 == 0x04),
                        ('UVLO',data[0] & 0x02 == 0x02),
                        ('WRONG_CMD',data[0] & 0x01 == 0x01),   #The NOTPERF_CMD and WRONG_CMD flags are active high and indicate, respectively, that the command received by SPI cannot be performed or does not exist at all.
                        ('NOTPERF_CMD',data[1] & 0x80 == 0x80),
                        ('MOT_STATUS',data[1] & 0x60),
                        ('DIR',data[1] & 0x10 == 0x10),
                        ('SW_EVN',data[1] & 0x08 == 0x08),
                        ('SW_F',data[1] & 0x04 == 0x04),        #The SW_F flag reports the SW input status (low for open and high for closed).
                        ('BUSY',data[1] & 0x02 != 0x02),
                        ('HIZ',data[1] & 0x01 == 0x01),
                        ('MSByte', data[0]),
                        ('LSByte', data[1]),
                        ('SPEED', self._Speed(spd[0] << 16 | spd[1] <<8 | spd[2])),
                        ('SPEED_RAW', spd[0] << 16 | spd[1] <<8 | spd[2]),
                        #('SPEED_C', spd_c),
                        ('POSITION', pos),
                        ('VIRTUAL_POSITION', vpos),
                        ('VIRTUAL_ROUND', self.virtualRound),
                        #('POSITION_CLC', pos/(self.microstepping*1.0)),
                        ('DATETIME', time.time())
                        ])

            
            if data[0] == 0 and data[1] == 0:
                print("i2c problem", bin(data[0]), bin(data[1]))

            if self.statusCallback:
                self.statusCallback(status = status)

            self.last_status = status
            return status

        except Exception as e:
            print("GetStatusErr>>", e)
            return dict([('DATETIME', time.time())])


    def setStatusCallback(self, callback = None):
        self.statusCallback = callback

    def SetSwRange(self, minv, maxv):
        self.sw_range = (float(minv), float(maxv))

    def getParam(self, param):          #TODO: toto bude nova verze funkce ReadPara, ReadParam uz nepouzivat. Nyni pouzivejte getParam(param)
        self.writeByte( 0x20 | param)
        self.writeByte( 0x00)
        data0 = self.readByte()           # 1st byte
        self.writeByte( 0x00)
        data1 = self.readByte()           # 2nd byte
        self.writeByte( 0x00)
        data2 = self.readByte()
        return data0 << 16 | data1 <<8 | data2

    def setParam(self, param, value):
        self.writeByte( 0x00 | param)
        self.writeByte( (value & 0xff0000) >> 16)
        self.writeByte( (value & 0xff00) >> 8)
        self.writeByte( (value & 0xff) >> 0)

    #def ReadParam(self, param):
    #    return getParam(param)

    def getPosition(self):
        self.writeByte( 0x20 | 0x01)
        self.writeByte( 0x00)
        data0 = self.readByte()           # 1st byte
        self.writeByte( 0x00)
        data1 = self.readByte()           # 2nd byte
        self.writeByte( 0x00)
        data2 = self.readByte()

        #print (data0 << 16 | data1 <<8 | data2)&0x3fffff, '\t', bin(data0 << 16 | data1 <<8 | data2), bin(data0), bin(data1), bin(data2)
        return (data0 << 16 | data1 <<8 | data2)&0x3fffff

    def setPosition(self, position):
        return self.setParam(0x01, position)

    def setStepMode(self, mode):
        self.setParam(0x16, mode[0])
        self.microstepping = mode[1]
        #self.writeByte( self.L6470_STEP_MODE)      # Microstepping
        #self.writeByte( STEP_MODE[0])      # 0x04 - 1/16
        #self.microstepping = STEP_MODE[1]


    #def ReadPosition(self):
    #    return self.getPosition()

    def Wait(self, delay = 0.2, maximal_time = False, print_pos = False):
        start = time.time()
        while self.IsBusy() and (not maximal_time or maximal_time > time.time() - start):
            time.sleep(delay)
            if print_pos:
                print("Position: ", self.last_status['POSITION'])
        return (time.time() - start)

    def IsBusy(self):
        try:
            return self.getStatus()['BUSY']
        except Exception as e:
            return True
        #if self.ReadStatusBit(1) == 1:
        #    return True
        #else:
        #    return False
    
    def virtual_position(self, position = None):
        '''
            Tato funkce bude hlidat owerflow a bude pocitat, kolikrat na kterou stranu je registr přetečen...
            Je potreba hlidat, aby byla volana dostatecne casto.

            Input:
               position: current stepper position, optional
            Output:
               VirtualPosition
        '''
        regLenght = 2**22
        if not position:
            position = self.getPosition()
            #pos2 = self.getPosition()
            #print("vpos:", position, pos2)
        lastPosition = self.virtualPositionLast
        self.virtualPositionLast = position

        if lastPosition > regLenght*2/3 and position < regLenght*1/3:
            self.virtualRound += 1
            print("# Buffer pretekl nahoru, pouzivame nasobek {}, Position: {}, Last: {}".format(self.virtualRound, position, lastPosition))
        
        elif lastPosition < regLenght*1/3 and position > regLenght*2/3:
            self.virtualRound -= 1
            print("# Buffer pretekl dolu, pouzivame nasobek {}, Position: {}, Last: {}".format(self.virtualRound, position, lastPosition))

        return regLenght*self.virtualRound + position


    def _IOspeed(self, speed):
        return int((speed * 250e-9)/(2**-28))

    def _Speed(self, speed, dir = False):
        return ((speed)*2**-28)/250e-9

    def _units2steps(self, units):
        return units * 1.0 * self.SPU * self.microstepping

    def _steps2units(self, steps):
        return (steps*1.0) / self.SPU / self.microstepping

# End Class axis --------------------------------------------------







class axis_between(axis):

    def search_range(self, def_dir, usbi2c, gpio_pins):
        '''
            Tato funkce najde koncove stupne a snazi se
        '''

        (a,b) = self.validate_switch(usbi2c, gpio_pins)
        print ("Stavy tracitek", a, b)

        if a:
            self.Move(10000, 0)
            self.Wait()
        
        if b:
            self.Move(10000, 1)
            self.Wait()


        if self.getStatus()['SW_F']:
            print("\nERR...\nJsme na koncaku, uvolnete koncak\n")
            self.Float()
            sys.exit(0)
        self.def_dir = def_dir

        # cesta na koncak
        print("Cesta na prvni koncak")
        self.GoUntil(direction = self.def_dir, speed = 100, ACT = False)
        self.Wait()


        (a,b) = self.validate_switch(usbi2c, gpio_pins)
        print ("Stavy tracitek", a, b)

        # opustit koncak + vynulovat registr
        print("Cesta opustit prvni koncak")
        self.ReleaseSW(direction = not self.def_dir, ACT = False)
        self.Wait()


        (a,b) = self.validate_switch(usbi2c, gpio_pins)
        print ("Stavy tracitek", a, b)

        time.sleep(0.5)

        print("Cesta na druhy koncak")
        self.GoUntil(direction = not self.def_dir, speed = 100, ACT = True)
        self.Wait(print_pos = True)


        (a,b) = self.validate_switch(usbi2c, gpio_pins)
        print ("Stavy tracitek", a, b)

        print("Cesta opustit druhy")
        self.ReleaseSW(direction = self.def_dir, ACT = False)
        self.Wait()

        (a,b) = self.validate_switch(usbi2c, gpio_pins)
        print ("Stavy tracitek", a, b)

        if def_dir: first = 0x3fffff
        else: first = 0x0
        second = self.getStatus()['POSITION']

        if def_dir: print("delka drahy je %s kroku" %(first - second))
        else: print("delka drahy je %s kroku" %(second - first))



    def validate_switch(self, gpio_device, pin, polarity = True, delay = 0.01):
        if not isinstance(pin, list):
            pin = [pin]
            
        out = []
        for p in pin:
            gpio_device.output(p, 1)
            time.sleep(delay)
            sw1 = self.getStatus()['SW_F']
            gpio_device.output(p, 0)
            time.sleep(delay)
            sw2 = self.getStatus()['SW_F']
            if sw1 == sw2:
                out.append(False)
            else:
                out.append(True)

        return out
