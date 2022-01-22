###############################################################################
# IMU.py from BerryIMU-master/python-BerryIMU-gyro-accel-compass-filters/IMU.py
# 
# 1 use adafruit_bus_device.i2c_device.I2CDevice api's to access i2c bus 
#   instead of 'import smbus , bus = smbus.SMBus(1)'
#   * this enables using adafruit i2c=board.i2c() or i2c=busio.i2c() api's
#     i2c.try_lock() & i2c.unlock() to avoid bus contention accross multiple
#     threads where each thread is managing i/o to a specific i2c device on the
#     same i2c bus object (ie i2c = board.i2c()).
#     << NOT IMPLEMENTED HERE YET >>
#     - see POC in paramotor-avionics/python-notes/i2c-buslock-2threads.py
# 
# 2 - restructure IMU.py as a proper python class vs inline code
#     ref: adafruit_mpl3115a2.py 
################################################################################
# Revision History:
# Revison   Date       Author                  Description
# 0.1       12-31-21   IAD   POC Class using adafruit i2c i/o w only detectIMU() 
# 0.2       01-04-22   IAD   Chg all of IMU.py to be class w adafruit i2c i/o
################################################################################
import struct
import time
#import smbus
#bus = smbus.SMBus(1)
from adafruit_bus_device import i2c_device
from LSM6DSL import *
from LIS3MDL import *
import i2cdevices

class IMU:

    # Class level buffer to reduce memory usage and allocations.
    # Note this is not thread safe by design!
    # >>> not thread safe if you got more than 1 method in this class
    #     trying to access the I2C bus at the same time using this buffer
    #     but that should never be the case

    _BUFFER = bytearray(4)

    # i2cdevices.py:
    # class I2CDev(Enum):
	#  GYRO = 1
	#  MAG = 2

    def __init__(self, i2c, *, gyro_address=LSM6DSL_ADDRESS, mag_address=LIS3MDL_ADDRESS):
        self._i2cbus = i2c
        self._i2c_gyro_device = i2c_device.I2CDevice(i2c, gyro_address)
        self._i2c_mag_device = i2c_device.I2CDevice(i2c, mag_address)
        self._sensors = i2cdevices.I2CDevs
        self._BerryIMUversion = 99

    def _read_u8(self, device, address):
        # Read an 8-bit unsigned value from the specified 8-bit address of specified device
        self._read_into(device, address, self._BUFFER, count=1)
        return self._BUFFER[0]

    def _read_into(self, device, address, buf, count=None):
        # Read bytes from the specified 8-bit address into the provided buffer.
        # If the count is not specified then the entire buffer is filled,
        # otherwise count bytes are copied in.
        if count is None:
            count = len(buf)
        if device is self._sensors.GYRO:
            with self._i2c_gyro_device as i2c: 
                i2c.write_then_readinto(bytes([address & 0xFF]), buf, in_end=count)
        if device is self._sensors.MAG:
            with self._i2c_mag_device as i2c:
                i2c.write_then_readinto(bytes([address & 0xFF]), buf, in_end=count)

    def _write_u8(self, device, address, val):
        # Write an 8-bit unsigned value to the specified 8-bit address.
        if device is self._sensors.GYRO:
            with self._i2c_gyro_device as i2c:
                self._BUFFER[0] = address & 0xFF
                self._BUFFER[1] = val & 0xFF
                i2c.write(self._BUFFER, end=2)
        if device is self._sensors.MAG:
            with self._i2c_mag_device as i2c:
                self._BUFFER[0] = address & 0xFF
                self._BUFFER[1] = val & 0xFF
                i2c.write(self._BUFFER, end=2)

    def detectIMU(self):
        #LSM6DSL_WHO_AM_I_response = (bus.read_byte_data(LSM6DSL_ADDRESS, LSM6DSL_WHO_AM_I))
        #LIS3MDL_WHO_AM_I_response = (bus.read_byte_data(LIS3MDL_ADDRESS, LIS3MDL_WHO_AM_I))
        LSM6DSL_WHO_AM_I_response = self._read_u8(self._sensors.GYRO, LSM6DSL_WHO_AM_I)
        LIS3MDL_WHO_AM_I_response = self._read_u8(self._sensors.MAG, LIS3MDL_WHO_AM_I)

        if (LSM6DSL_WHO_AM_I_response == 0x6A) and (LIS3MDL_WHO_AM_I_response == 0x3D):
            print("Found BerryIMUv3 (LSM6DSL and LIS3MDL)")
            self._BerryIMUversion = 3
        else:
            print("BerryIMUv3 (LSM6DSL and LIS3MDL) !! NOT FOUND !! - check your wiring ?? ")
            self._BerryIMUversion = 99

        return self._BerryIMUversion

    #def writeByte(device_address,register,value):
    #    bus.write_byte_data(device_address, register, value)
    #                                   \         |       |
    # >>> Replacd with  _write_u8(self, device, address, val) above
    #                                    |         \      /
    #                       self._sensors.GYRO     lv as sis
    #                       self._sensors.MAG  

    def readACCx(self):
        acc_l = 0
        acc_h = 0
        #acc_l = bus.read_byte_data(LSM6DSL_ADDRESS, LSM6DSL_OUTX_L_XL)
        #acc_h = bus.read_byte_data(LSM6DSL_ADDRESS, LSM6DSL_OUTX_H_XL)
        acc_l = self._read_u8(self._sensors.GYRO, LSM6DSL_OUTX_L_XL)
        acc_h = self._read_u8(self._sensors.GYRO, LSM6DSL_OUTX_H_XL)

        acc_combined = (acc_l | acc_h <<8)
        return acc_combined  if acc_combined < 32768 else acc_combined - 65536

    def readACCy(self):
        acc_l = 0
        acc_h = 0
        acc_l = self._read_u8(self._sensors.GYRO, LSM6DSL_OUTY_L_XL)
        acc_h = self._read_u8(self._sensors.GYRO, LSM6DSL_OUTY_H_XL)

        acc_combined = (acc_l | acc_h <<8)
        return acc_combined  if acc_combined < 32768 else acc_combined - 65536

    def readACCz(self):
        acc_l = 0
        acc_h = 0
        acc_l = self._read_u8(self._sensors.GYRO, LSM6DSL_OUTZ_L_XL)
        acc_h = self._read_u8(self._sensors.GYRO, LSM6DSL_OUTZ_H_XL)

        acc_combined = (acc_l | acc_h <<8)
        return acc_combined  if acc_combined < 32768 else acc_combined - 65536

    def readGYRx(self):
        gyr_l = 0
        gyr_h = 0
        gyr_l = self._read_u8(self._sensors.GYRO, LSM6DSL_OUTX_L_G)
        gyr_h = self._read_u8(self._sensors.GYRO, LSM6DSL_OUTX_H_G)

        gyr_combined = (gyr_l | gyr_h <<8)
        return gyr_combined  if gyr_combined < 32768 else gyr_combined - 65536

    def readGYRy(self):
        gyr_l = 0
        gyr_h = 0
        gyr_l = self._read_u8(self._sensors.GYRO, LSM6DSL_OUTY_L_G)
        gyr_h = self._read_u8(self._sensors.GYRO, LSM6DSL_OUTY_H_G)

        gyr_combined = (gyr_l | gyr_h <<8)
        return gyr_combined  if gyr_combined < 32768 else gyr_combined - 65536

    def readGYRz(self):
        gyr_l = 0
        gyr_h = 0
        gyr_l = self._read_u8(self._sensors.GYRO, LSM6DSL_OUTZ_L_G)
        gyr_h = self._read_u8(self._sensors.GYRO, LSM6DSL_OUTZ_H_G)

        gyr_combined = (gyr_l | gyr_h <<8)
        return gyr_combined  if gyr_combined < 32768 else gyr_combined - 65536

    def readMAGx(self):
        mag_l = 0
        mag_h = 0
        mag_l = self._read_u8(self._sensors.MAG, LIS3MDL_OUT_X_L)
        mag_h = self._read_u8(self._sensors.MAG, LIS3MDL_OUT_X_H)

        mag_combined = (mag_l | mag_h <<8)
        return mag_combined  if mag_combined < 32768 else mag_combined - 65536

    def readMAGy(self):
        mag_l = 0
        mag_h = 0
        mag_l = self._read_u8(self._sensors.MAG, LIS3MDL_OUT_Y_L)
        mag_h = self._read_u8(self._sensors.MAG, LIS3MDL_OUT_Y_H)

        mag_combined = (mag_l | mag_h <<8)
        return mag_combined  if mag_combined < 32768 else mag_combined - 65536

    def readMAGz(self):
        mag_l = 0
        mag_h = 0
        mag_l = self._read_u8(self._sensors.MAG, LIS3MDL_OUT_Z_L)
        mag_h = self._read_u8(self._sensors.MAG, LIS3MDL_OUT_Z_H)

        mag_combined = (mag_l | mag_h <<8)
        return mag_combined  if mag_combined < 32768 else mag_combined - 65536

    #def writeByte(device_address,register,value):
    #    bus.write_byte_data(device_address, register, value)
    #                                   \         |       |
    # >>> Replacd with self._write_u8(self, device, address, val) above
    #                                    |         \      /
    #                       self._sensors.GYRO     lv as sis
    #                       self._sensors.MAG  

    def initIMU(self):

        #initialise the accelerometer
        #writeByte(LSM6DSL_ADDRESS,LSM6DSL_CTRL1_XL,0b10011111)           #ODR 3.33 kHz, +/- 8g , BW = 400hz
        #writeByte(LSM6DSL_ADDRESS,LSM6DSL_CTRL8_XL,0b11001000)           #Low pass filter enabled, BW9, composite filter
        #(LSM6DSL_ADDRESS,LSM6DSL_CTRL3_C,0b01000100)                     #Enable Block Data update, increment during multi byte read

        self._write_u8(self._sensors.GYRO,LSM6DSL_CTRL1_XL,0b10011111)           #ODR 3.33 kHz, +/- 8g , BW = 400hz
        self._write_u8(self._sensors.GYRO,LSM6DSL_CTRL8_XL,0b11001000)           #Low pass filter enabled, BW9, composite filter
        self._write_u8(self._sensors.GYRO,LSM6DSL_CTRL3_C,0b01000100)            #Enable Block Data update, increment during multi byte read

        #initialise the gyroscope
        self._write_u8(self._sensors.GYRO,LSM6DSL_CTRL2_G,0b10011100)            #ODR 3.3 kHz, 2000 dps

        #initialise the magnetometer
        self._write_u8(self._sensors.MAG,LIS3MDL_CTRL_REG1, 0b11011100)         # Temp sesnor enabled, High performance, ODR 80 Hz, FAST ODR disabled and Selft test disabled.
        self._write_u8(self._sensors.MAG,LIS3MDL_CTRL_REG2, 0b00100000)         # +/- 8 gauss
        self._write_u8(self._sensors.MAG,LIS3MDL_CTRL_REG3, 0b00000000)         # Continuous-conversion mode


