# axis

Axis is Python module for controlling stepper motors with [MLAB](https://www.mlab.cz) [HBSTEP01B](https://github.com/MLAB-project/Modules/blob/master/H_Bridge/HBSTEP01B/DOC/HBSTEP01B.cs.pdf) based on ST L6470 inteligent stepper motor driver.

![HBSTEP](https://raw.githubusercontent.com/MLAB-project/Modules/master/H_Bridge/HBSTEP01B/DOC/SRC/img/HBSTEP01B_Top_Big.jpg)

Class axis is prepared for use with multiple ways of connecting to the control computer. 

##### I2C (PyMLAB)
Connecting [HBSTEP01B](https://github.com/MLAB-project/Modules/blob/master/H_Bridge/HBSTEP01B/DOC/HBSTEP01B.cs.pdf) with control computer with I2C requires I2C-SPI translator supported with pymlab. For example [I2CSPI01A](https://github.com/MLAB-project/Modules/blob/master/CommSerial/I2CSPI01A/DOC/I2CSPI01A.cs.pdf) or SC18IS602B equivalent. 

##### AROM (ROS framework)
Using with AROM requires fully installed and configured [AROM](http://wiki.mlab.cz/doku.php?id=cs:robozor:arom) and running [pymlab_bridge]().


##### SPI

## Examples

#### Example with I2C

```Python
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
motor.Reset(KVAL_RUN = 0x90, KVAL_ACC = 0xF0, KVAL_DEC = 0xF0, FS_SPD = 0xFFFFFF)

#motor.setConfig(EXT_CLK = 0b1, OSC_SEL = 0b110) # Ext clock source: 24 MHz(Crystal/resonator driver disabled)

motor.Float()
print motor.GetStatus()

i = 0
while True:
    print "loop"
    motor.MoveWait(5000)
    motor.MoveWait(-5000)
```


#### Example with AROM


## Using

##### Reset

```Python
def Reset(self, init = True, ACC = 0x00a, DEC = 0x00a, stall_th = 2030, ocd_th = 3380, KVAL_HOLD = 0x29, KVAL_RUN = 0x29, KVAL_ACC = 0x29, KVAL_DEC = 0x29, FS_SPD = 0x027)
```

`init` (default: True) reset driver to initial values

`ACC` (default: 0x00A) Acceleration speed

`DEC` (default: 0x00A) Acceleration speed

`stall_th` (default: 2030mA) Stall current treshold in mA, maximal value is ******

`ocd_th` (default: 3380mA) Over current treshold in mA, maximal value is ******

`KVAL_HOLD` (default: 0x29) 

`KVAL_RUN`(default: 0x29) 

`KVAL_ACC`(default: 0x29) 

`KVAL_DEC`(default: 0x29) 

`FS_SPD`(default: 0x29) 

#### MaxSpeed
```Python
def MaxSpeed(self, speed)
```
Write maximal speed to driver

`speed` (range: 15.25 - 1560) Maximal speed value in stpes/s

#### GoTo
```Python
def GoTo(self, abspos)
```
Brings motor into ABS_POS position (minimum
path) 

#### GoToDir
```Python
def GoToDir(self, direction, abspos)
```
Brings motor into ABS_POS position forcing DIR
direction

## Installation

```
cd ~/repos
git clone git@github.com:MLAB-project/axis.git
cd axis
```
```
sudo python setup.py develop
```
