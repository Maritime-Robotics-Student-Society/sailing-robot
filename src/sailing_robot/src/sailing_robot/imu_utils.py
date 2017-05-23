"""This code reads the data from an LSM303D and an L3GD20H which are both attached to the I2C bus of a Raspberry Pi.
Both can be purchased as a unit from Pololu as their MinIMU-9 v3 Gyro, Accelerometer, and Compass  product.

First follow the procedure to enable I2C on R-Pi.
1. Add the lines "ic2-bcm2708" and "i2c-dev" to the file /etc/modules
2. Comment out the line "blacklist ic2-bcm2708" (with a #) in the file /etc/modprobe.d/raspi-blacklist.conf
3. Install I2C utility (including smbus) with the command "apt-get install python-smbus i2c-tools"
4. Connect the I2C device to the SDA and SCL pins of the Raspberry Pi and detect it using the command "i2cdetect -y 1".  It should show up as 1D (typically) or 1E (if the jumper is set).
"""

from smbus import SMBus

def twos_comp_combine(msb, lsb):
    twos_comp = 256*msb + lsb
    if twos_comp >= 32768:
        return twos_comp - 65536
    else:
        return twos_comp

## LSM303D Registers --------------------------------------------------------------

LSM_WHOAMI_ADDRESS = 0x0F
LSM_WHOAMI_CONTENTS = 0b1001001 #Device self-id

#Control register addresses -- from LSM303D datasheet

LSM_CTRL_0 = 0x1F #General settings
LSM_CTRL_1 = 0x20 #Turns on accelerometer and configures data rate
LSM_CTRL_2 = 0x21 #Self test accelerometer, anti-aliasing accel filter
LSM_CTRL_3 = 0x22 #Interrupts
LSM_CTRL_4 = 0x23 #Interrupts
LSM_CTRL_5 = 0x24 #Turns on temperature sensor
LSM_CTRL_6 = 0x25 #Magnetic resolution selection, data rate config
LSM_CTRL_7 = 0x26 #Turns on magnetometer and adjusts mode

#Registers holding twos-complemented MSB and LSB of magnetometer readings -- from LSM303D datasheet
LSM_MAG_X_LSB = 0x08 # x
LSM_MAG_X_MSB = 0x09
LSM_MAG_Y_LSB = 0x0A # y
LSM_MAG_Y_MSB = 0x0B
LSM_MAG_Z_LSB = 0x0C # z
LSM_MAG_Z_MSB = 0x0D

#Registers holding twos-complemented MSB and LSB of magnetometer readings -- from LSM303D datasheet
LSM_ACC_X_LSB = 0x28 # x
LSM_ACC_X_MSB = 0x29
LSM_ACC_Y_LSB = 0x2A # y
LSM_ACC_Y_MSB = 0x2B
LSM_ACC_Z_LSB = 0x2C # z
LSM_ACC_Z_MSB = 0x2D

LSM_FIELDS = {
    'MAG_X': (LSM_MAG_X_MSB, LSM_MAG_X_LSB),
    'MAG_Y': (LSM_MAG_Y_MSB, LSM_MAG_Y_LSB),
    'MAG_Z': (LSM_MAG_Z_MSB, LSM_MAG_Z_LSB),
    'ACC_X': (LSM_ACC_X_MSB, LSM_ACC_X_LSB),
    'ACC_Y': (LSM_ACC_Y_MSB, LSM_ACC_Y_LSB),
    'ACC_Z': (LSM_ACC_Z_MSB, LSM_ACC_Z_LSB),
}

#Registers holding 12-bit right justified, twos-complemented temperature data -- from LSM303D datasheet
LSM_TEMP_MSB = 0x05
LSM_TEMP_LSB = 0x06

# L3GD20H registers ----------------------------------------------------


LGD_WHOAMI_ADDRESS = 0x0F
LGD_WHOAMI_CONTENTS = 0b11010111 #Device self-id

LGD_CTRL_1 = 0x20 #turns on gyro
LGD_CTRL_2 = 0x21 #can set a high-pass filter for gyro
LGD_CTRL_3 = 0x22
LGD_CTRL_4 = 0x23
LGD_CTRL_5 = 0x24
LGD_CTRL_6 = 0x25

LGD_TEMP = 0x26

#Registers holding gyroscope readings
LGD_GYRO_X_LSB = 0x28
LGD_GYRO_X_MSB = 0x29
LGD_GYRO_Y_LSB = 0x2A
LGD_GYRO_Y_MSB = 0x2B
LGD_GYRO_Z_LSB = 0x2C
LGD_GYRO_Z_MSB = 0x2D

LGD_FIELDS = {
    'GYRO_X': (LGD_GYRO_X_MSB, LGD_GYRO_X_LSB),
    'GYRO_Y': (LGD_GYRO_Y_MSB, LGD_GYRO_Y_LSB),
    'GYRO_Z': (LGD_GYRO_Z_MSB, LGD_GYRO_Z_LSB),
}

class ImuReader(object):
    def __init__(self, bus_num, lsm_addr, lgd_addr):
        self.bus_num = bus_num
        self.bus = SMBus(bus_num)
        self.lsm_addr = lsm_addr
        self.lgd_addr = lgd_addr
    
    def check_status(self):
        if self.bus.read_byte_data(self.lsm_addr, LSM_WHOAMI_ADDRESS) == LSM_WHOAMI_CONTENTS:
            print('LSM303D detected on I2C bus %d.' % self.bus_num)
        else:
            raise Exception('No LSM303D detected on I2C bus %d.' % self.bus_num)

        if self.bus.read_byte_data(self.lgd_addr, LGD_WHOAMI_ADDRESS) == LGD_WHOAMI_CONTENTS:
            print('L3GD20H detected successfully on I2C bus %d.' % self.bus_num)
        else:
            raise Exception('No L3GD20H detected on bus on I2C bus %d.' % self.bus_num)

    def configure_for_reading(self):
        b = self.bus
        b.write_byte_data(self.lsm_addr, LSM_CTRL_1, 0b1010111) # enable accelerometer, 50 hz sampling
        b.write_byte_data(self.lsm_addr, LSM_CTRL_2, 0x00) #set +/- 2g full scale
        b.write_byte_data(self.lsm_addr, LSM_CTRL_5, 0b01100100) #high resolution mode, thermometer off, 6.25hz ODR
        b.write_byte_data(self.lsm_addr, LSM_CTRL_6, 0b00100000) # set +/- 4 gauss full scale
        b.write_byte_data(self.lsm_addr, LSM_CTRL_7, 0x00) #get magnetometer out of low power mode

        b.write_byte_data(self.lgd_addr, LGD_CTRL_1, 0x0F) #turn on gyro and set to normal mode

    def read_lsm_field(self, field_name):
        msb_reg, lsb_reg = LSM_FIELDS[field_name]
        return twos_comp_combine(self.bus.read_byte_data(self.lsm_addr, msb_reg),
                                 self.bus.read_byte_data(self.lsm_addr, lsb_reg))

    def read_mag_field(self):
        return (self.read_lsm_field('MAG_X'),
                self.read_lsm_field('MAG_Y'),
                self.read_lsm_field('MAG_Z'))

    def read_acceleration(self):
        return (self.read_lsm_field('ACC_X'),
                self.read_lsm_field('ACC_Y'),
                self.read_lsm_field('ACC_Z'))

    def read_lgd_field(self, field_name):
        msb_reg, lsb_reg = LGD_FIELDS[field_name]
        return twos_comp_combine(self.bus.read_byte_data(self.lgd_addr, msb_reg),
                                 self.bus.read_byte_data(self.lgd_addr, lsb_reg))

    def read_gyro(self):
        return (self.read_lgd_field('GYRO_X'),
                self.read_lgd_field('GYRO_Y'),
                self.read_lgd_field('GYRO_Z'))
