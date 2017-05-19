import time

class axis:
    def __init__(self, SPI, SPI_CS, Direction, StepsPerUnit = 1, protocol = 'i2c', arom_spi_name = 'spi'):
        ' One axis of robot '
        self.CS = SPI_CS
        self.Dir = Direction
        self.SPU = StepsPerUnit
        self.spi = SPI
        self.protocol = protocol
        self.arom_spi_name = arom_spi_name

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

        #self.Reset()


    def writeByte(self, CS, address):
        if self.protocol == 'i2c':
            return self.spi.SPI_write_byte(self.CS, address)
        elif self.protocol == 'spi':
            return "Err. SPI neni podporovano"
        elif self.protocol == 'arom':
            return eval(self.spi(device=self.arom_spi_name, method="SPI_write_byte", parameters=str((self.CS,address))).value) # self.spi.I2CSPI_MSB_FIRST| self.spi.I2CSPI_MODE_CLK_IDLE_HIGH_DATA_EDGE_TRAILING| self.spi.I2CSPI_CLK_461kHz


    def readByte(self):
        if self.protocol == 'i2c':
            return self.spi.SPI_read_byte(self.CS, address)
        elif self.protocol == 'spi':
            return "Err. SPI neni podporovano"
        elif self.protocol == 'arom':
            return eval(self.spi(device=self.arom_spi_name, method="SPI_read_byte").value)


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
            self.writeByte(self.CS, 0xC0)      # reset

        if stall_th > 4000:
            stall_th = 4000

        self.writeByte(self.CS, self.L6470_STALL_TH)      # Stall Treshold setup
        self.writeByte(self.CS, int(stall_th/31.25)-1)      # 0x70 = 3.5A
        
        if ocd_th > 6000:
            ocd_th = 6000

        self.writeByte(self.CS, self.L6470_OCD_TH)      # Over Current Treshold setup 
        self.writeByte(self.CS, int(ocd_th/375)-1)      # 0x0A = 4A

        self.writeByte(self.CS, self.L6470_FS_SPD)      # Full Step speed, 0x03FF - maximal - always microstepping
        self.writeByte(self.CS, (FS_SPD >>16) & 0xFF)
        self.writeByte(self.CS, (FS_SPD >> 8) & 0xFF)
        self.writeByte(self.CS, (FS_SPD >> 0) & 0xFF)


        self.writeByte(self.CS, self.L6470_ACC)      # ACC (0x008a)
        self.writeByte(self.CS, (ACC >> 8) & 0xFF)
        self.writeByte(self.CS, (ACC >> 0) & 0xFF)
        self.writeByte(self.CS, self.L6470_DEC)      # DEC (0x008a)
        self.writeByte(self.CS, (DEC >> 8) & 0xFF)
        self.writeByte(self.CS, (DEC >> 0) & 0xFF)

        self.writeByte(self.CS, self.L6470_KVAL_HOLD)
        self.writeByte(self.CS, KVAL_HOLD)
        self.writeByte(self.CS, self.L6470_KVAL_RUN)
        self.writeByte(self.CS, KVAL_RUN)
        self.writeByte(self.CS, self.L6470_KVAL_ACC)
        self.writeByte(self.CS, KVAL_ACC)
        self.writeByte(self.CS, self.L6470_KVAL_DEC)
        self.writeByte(self.CS, KVAL_DEC)

        self.MinSpeed(speed = 0x00, LSPD_OPT = True)

        self.setConfig(F_PWM_INT = 0b001, F_PWM_DEC = 0b110, POW_SR = 0b00, OC_SD = 0b0, RESERVED = 0b0, EN_VSCOMP =  0b0, SW_MODE = 0b0, EXT_CLK = 0b0, OSC_SEL = 0b000)
        

        self.writeByte(self.CS, self.L6470_STEP_MODE)      # Microstepping
        self.writeByte(self.CS, 0x04)      # 0x04 - 1/16



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

        print bin(config)

        data = [(config >> i & 0xff) for i in (8,0)]

        self.writeByte(self.CS, self.L6470_CONFIG)
        self.writeByte(self.CS, data[0])
        self.writeByte(self.CS, data[1])

        return config
 

      
    def MaxSpeed(self, speed):
        ' Setup of maximum speed  - 15.25 to 1560 steps/s'

        speed_value = int(speed / 15.25)
        if (speed_value <= 0):
            speed_value = 1
        elif (speed_value >= 1023):
            speed_value = 1023

        data = [(speed_value >> i & 0xff) for i in (8,0)]

        print data
        self.writeByte(self.CS, self.L6470_MAX_SPEED)       # Max Speed setup 
        self.writeByte(self.CS, data[0])
        self.writeByte(self.CS, data[1])
        return (speed_value * 15.25)

      
    def MinSpeed(self, speed, LSPD_OPT = True):
        ' Setup of minimum speed  - 0 to 976 steps/s'

        if LSPD_OPT:
            lspd = 0b10000000
        else:
            lspd = 0

        speed_value = int(speed / 0.238)
        if (speed_value <= 0):
            speed_value = 1
        elif (speed_value >= 976):
            speed_value = 976

        data = [(speed_value >> i & 0xff) for i in (8,0)]
        
        print data
        self.writeByte(self.CS, self.L6470_MAX_SPEED)       # Max Speed setup 
        self.writeByte(self.CS, data[0] | lspd)
        self.writeByte(self.CS, data[1])
        return (speed_value * 0.238)


    '''
          APPLICATION COMMANDS
        ========================
    '''


    def GoTo(self, abspos):
        data = [(abspos >> i & 0xff) for i in (16,8,0)]

        self.writeByte(self.CS, 0b01100000)
        self.writeByte(self.CS, data[0])
        self.writeByte(self.CS, data[1])
        self.writeByte(self.CS, data[2])

        return abspos

    def GoToDir(self, direction, speed):
        data = [(speed >> i & 0xff) for i in (16,8,0)]

        self.writeByte(self.CS, 0b01101000 + int(direction))
        self.writeByte(self.CS, data[0])
        self.writeByte(self.CS, data[1])
        self.writeByte(self.CS, data[2])

        return speed

    def GoUntil(self, direction, speed):
        data = [(speed >> i & 0xff) for i in (16,8,0)]

        self.writeByte(self.CS, 0b10000010 + int(direction))
        self.writeByte(self.CS, data[0])
        self.writeByte(self.CS, data[1])
        self.writeByte(self.CS, data[2])

        return speed

    def ReleaseSW(self, direction = 1):
        ' Go away from Limit Switch '

        while self.ReadStatusBit(2) == 1:
            self.writeByte(self.CS, 0b10010010 + int(direction))
            time.sleep(0.25)
 

    def GoZero(self, speed):
        ' Go to Zero position '
        self.ReleaseSW()

        self.writeByte(self.CS, 0x82 | (self.Dir & 1))       # Go to Zero
        self.writeByte(self.CS, 0x00)
        self.writeByte(self.CS, speed)  
        while self.IsBusy():
            time.sleep(0.25)
        time.sleep(0.3)
        self.ReleaseSW()


    def GoHome(self, wait = True):
        ' Go to Zero position '
        self.ReleaseSW()

        self.writeByte(self.CS, 0x70)       # Go to Zero
        self.writeByte(self.CS, 0x00)
        while self.IsBusy() and wait:
            time.sleep(0.25)


    def ResetPos(self):
        self.writeByte(self.CS, 0xD8)       # Reset position
        self.writeByte(self.CS, 0x00)


    def Move(self, direction = 0, units = 0):
        ' Move some distance units from current position'
        print 'move', units, 'units'
        
        steps = int(abs(units) * self.SPU)

        if units < 0:
            direction = not bool(direction)

        data = [(steps >> i & 0xff) for i in (16,8,0)]

        self.writeByte(self.CS, 0b01000000 + int(direction))
        self.writeByte(self.CS, data[0])
        self.writeByte(self.CS, data[1])
        self.writeByte(self.CS, data[2])

        return steps


    def MoveWait(self, units):
        ' Move some distance units from current position and wait for execution '
        self.Move(units)
        while self.IsBusy():
            pass


    def Run(self, direction = 0, speed = 0):
        speed_value = int(abs(speed) / 0.015)

        if speed < 0:
            direction = not bool(direction)

        data = [(speed_value >> i & 0xff) for i in (16,8,0)]

        self.writeByte(self.CS, 0b01010000 + int(direction))
        self.writeByte(self.CS, data[0])
        self.writeByte(self.CS, data[1])
        self.writeByte(self.CS, data[2])

        print 'run', bool(direction), speed_value, (data)

        return (speed_value * 0.015)


    def Float(self):
        ' switch H-bridge to High impedance state '
        print "switch H-bridge to High impedance state"
        self.writeByte(self.CS, 0xA0)



    def ReadStatusReg(self):
        ' Read status register '

        return self.getParam(0x19)

        '''
        self.writeByte(self.CS, 0x20 | 0x19)   # Read from address 0x19 (STATUS)
        self.writeByte(self.CS, 0x00)
        data = [self.readByte()]
        self.writeByte(self.CS, 0x00)
        data += [self.readByte()] 
        return (data[0] << 8 | data[1])
        '''

    def ReadStatusBit(self, bit):
        ' Report given status bit '

        data = self.getParam(0x19)
        return (data >> bit) & 1 


    def GetStatus(self):
        self.writeByte(self.CS, 0x20 | 0x19)   # Read from address 0x19 (STATUS)
        self.writeByte(self.CS, 0x00)
        data = [self.readByte()]      # 1st byte
        self.writeByte(self.CS, 0x00)
        data += [self.readByte()]

        self.writeByte(self.CS, 0x20 | 0x04)   # Read from address 0x04 (SPEED)
        self.writeByte(self.CS, 0x00)
        spd = [self.readByte()]
        self.writeByte(self.CS, 0x00)
        spd += [self.readByte()]
        self.writeByte(self.CS, 0x00)
        spd += [self.readByte()]


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
                    ('SPEED', spd[0] << 16 | spd[1] <<8 | spd[2])])

        return status


    def ABS_POS(self):
        return 0x01


    def EL_POS(self):
        return 0x02


    def MARK(self):
        return 0x03


    def SPEED(self):
        return 0x04


    def getParam(self, param):          #TODO: toto bude nova verze funkce ReadPara, ReadParam uz nepouzivat. Nyni pouzivejte getParam(param)
        self.writeByte(self.CS, 0x20 | param)
        self.writeByte(self.CS, 0x00)
        data0 = self.readByte()           # 1st byte
        self.writeByte(self.CS, 0x00)
        data1 = self.readByte()           # 2nd byte
        self.writeByte(self.CS, 0x00)
        data2 = self.readByte()
        return data0 << 16 | data1 <<8 | data2


    #def ReadParam(self, param):
    #    return getParam(param)


    def getPosition(self):
        return self.getParam(0x01)


    #def ReadPosition(self):
    #    return self.getPosition()
    

    def IsBusy(self):
        if self.ReadStatusBit(1) == 1:
            return False
        else:
            return True

# End Class axis --------------------------------------------------
