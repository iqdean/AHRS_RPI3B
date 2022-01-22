# AHRS_RPI3B
Raspberry Pi side code for Attitude Heading Reference System based on berryIMU v3

# Block Diagram

```
     PC              udp socket over wifi         RPI3B
  readIMU_naive.py---------> request -----> reqIMUData.py <--- i2c ---> IMU sensors
 /  Class IMUread \<-------- response <----/       IMU.py               berryIMUv3   
|                                                                       
 \
  readIMUtest.py --> print sensor data every 1sec


berryIMUv3
LSM6DSL - accelerometer and gyro   3D orientation
LIS3MDL - magnetometer             Magnetic heading


IMU.py 
- updated to use "adafruit_bus_device import i2c_device" api vs python smbus api
- upon request via udp socket, reads raw 16bit signed 2's complement sensor data 
  over i2c bus 8bits at a time , combining high/low bytes to formulate 16bit value.
- handles sign extending the resulting 16bit 2's compement sensor value by doing
  if value < (2^15 - 1) return value else return (2^16 - value)
- repeates for all 9 raw sensor values:
  acceleration: ax,ay,az
  rate gyro: gx,gy,gz
  magnetomenter: mx,my,mz
  > this results in 9 python signed integers each being 4bytes
- uses python struct pack to pack the 9 integers into byte array for transmission over udp socket
  
reqIMUData.py
- start server on RPI3B to handle udp socket coms
- implements a request/response com scheme to mimic getSerialData() function 
  - to getIMUData(), the client side sends a single byte to server
  - server responds by reading raw sensor data and sends 9 signed integers back (9i)

TO RUN POC1:

1. On RPI3B side - start the server

iqdean@rpi3ubu2004:~/AHRS_RPI3B$ sudo python3 reqIMUdata.py
[sudo] password for iqdean:
Found BerryIMUv3 (LSM6DSL and LIS3MDL)
reqIMUdata Server started on 0.0.0.0:8090

2. On PC Side - start the client

C:\<path_to>\2022\SWDEV\QuaternionsEKF\3-QUATERNION-ROTATION\PythonIMU_naive>python readIMUtest.py
recording data
------------------------------
gyro radians: x = 0.000, y = -0.061, z = 0.011
accel      g: x = -1.007, y = -0.061, z -0.079
mag    gauss: x = -0.937, y = 0.057, z = 0.079
------------------------------
------------------------------
gyro radians: x = -0.013, y = -0.062, z = 0.017
accel      g: x = -1.006, y = -0.060, z -0.079
mag    gauss: x = -0.938, y = 0.055, z = 0.078
------------------------------
...

```
