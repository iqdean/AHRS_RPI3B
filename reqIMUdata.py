#!/usr/bin/python
#
#    This program  reads the angles from the acceleromteer, gyroscope
#    and mangnetometer on a BerryIMU connected to a Raspberry Pi and
#    sends raw IMU sensor data over UDP socket connection to PC.
#
#    Only BerryIMUv3 is supported
#    Original code is from  http://ozzmaker.com/

################################################################################
# Revision History:
# Revison   Date       Author                  Description
# 0.1       01-09-22   iad1046@gmail.com    sendIMUdata.py
# modified berryIMU.py to transmit raw sensor data over UDP socket connection
# to PC every 1sec
#
# 0.2       01-20-22   IAD                  reqIMUdata.py
# a) update to make udp socket comms req/response vs periodic push every 1st
# b) update to send all 9i of sensor data in order expected by readIMU_naive.py
################################################################################

import sys
import time
import math
import board
import IMU
import datetime
import os
# ---- reqIMUdata.py ----
import socket
from struct import pack
from struct import unpack

i2c = board.I2C()  # uses board.SCL and board.SDA
imu = IMU.IMU(i2c)

BerryIMUversion = imu.detectIMU()     #Detect if BerryIMU is connected.
if(BerryIMUversion == 99):
    print(" No BerryIMU found... exiting ")
    sys.exit()

imu.initIMU()       #Initialise the accelerometer, gyroscope and compass

# Create an UDP based server socket
socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
host, port = '0.0.0.0', 8090        # server on RPI, client on PC

server_address = (host, port)
socket.bind(server_address)
print('reqIMUdata Server started on 0.0.0.0:8090')

while(True):

    rcvMsg, client_address = socket.recvfrom(64)
    print('msg rcvd')
    rcvData = unpack('i', rcvMsg)

    #Read the accelerometer,gyroscope and magnetometer values
    Ax = imu.readACCx()
    Ay = imu.readACCy()
    Az = imu.readACCz()
    Gx = imu.readGYRx()
    Gy = imu.readGYRy()
    Gz = imu.readGYRz()
    Mx = imu.readMAGx()
    My = imu.readMAGy()
    Mz = imu.readMAGz()

    sendMsg = pack('9i',Gx,Gy,Gz,Ax,Ay,Az,Mx,My,Mz)
    socket.sendto(sendMsg, client_address)

   
