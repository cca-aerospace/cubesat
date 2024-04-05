"""
The code you will write for this module should calculate
roll, pitch, and yaw (RPY) and calibrate your measurements
for better accuracy. Your functions are split into two activities.
The first is basic RPY from the accelerometer and magnetometer. The
second is RPY using the gyroscope. Finally, write the calibration functions.
Run plot.py to test your functions, this is important because auto_camera.py 
relies on your sensor functions here.
"""

#import libraries
import time
import numpy as np
import time
import os
import board
import busio
from adafruit_lsm6ds.lsm6dsox import LSM6DSOX as LSM6DS
from adafruit_lis3mdl import LIS3MDL

#imu initialization
i2c = busio.I2C(board.SCL, board.SDA)
accel_gyro = LSM6DS(i2c)
mag = LIS3MDL(i2c)


#Activity 1: RPY based on accelerometer and magnetometer
def roll_am(accelX,accelY,accelZ):
    #TODO
    return (180/np.pi)*np.arctan2(accelY/np.sqrt(accelX ** 2 + accelZ ** 2))

def pitch_am(accelX,accelY,accelZ):
    #TODO
    return (180/np.pi)*np.arctan2(accelX/np.sqrt(accelY ** 2 + accelZ ** 2))

def yaw_am(accelX,accelY,accelZ,magX,magY,magZ):
    #TODO
    roll = roll_am(accelX, accelY, accelZ)
    pitch = pitch_am(accelX, accelY, accelZ)
    mag_x = magX * np.cos(pitch) + magY * np.sin(roll) * np.sin(pitch) + magZ * np.cos(roll) * np.sin(pitch)
    mag_y = magY * np.cos(roll) - magZ * np.sin(roll)
    return (180/np.pi)*np.arctan2(-mag_y, mag_x)

#Activity 2: RPY based on gyroscope
def roll_gy(prev_angle, delT, gyro):
    #TODO
    return prev_angle + gyro * delT
def pitch_gy(prev_angle, delT, gyro):
    #TODO
    return prev_angle + gyro * delT
def yaw_gy(prev_angle, delT, gyro):
    #TODO
    return prev_angle + gyro * delT

#Activity 3: Sensor calibration
def calibrate_mag():
    #TODO: Set up lists, time, etc
    magXData = []
    magYData = []
    magZData = []
    #print("Preparing to calibrate magnetometer. Please wave around.")
    #time.sleep(3)
    #print("Calibrating...")
    while True: # <-- set time
        time.sleep(5)
        for _ in range(50):
            magX, magY, magZ = mag.magnetic
            magXData.append(magX)
            magYData.append(magY)
            magZData.append(magZ)
        break

    #TODO: Calculate calibration constants
    magXOffset = (max(magXData) + min(magXData)) / 2
    magYOffset = (max(magYData) + min(magYData)) / 2
    magZOffset = (max(magZData) + min(magZData)) / 2
   # print("Calibration complete.")
    return [magXOffset, magYOffset, magZOffset]

def calibrate_gyro():
    #TODO
    gyroXData = []
    gyroYData = []
    gyroZData = []
    #print("Preparing to calibrate gyroscope. Put down the board and do not touch it.")
    #time.sleep(3)
    #print("Calibrating...")
    while True:
        time.sleep(5)
        for _ in range(50):
            gyroX, gyroY, gyroZ = accel_gyro.gyro
            gyroXData.append(gyroX)
            gyroYData.append(gyroY)
            gyroZData.append(gyroZ)
        break

    #TODO
    gyroXOffset = (max(gyroXData) + min(gyroXData)) / 2
    gyroYOffset = (max(gyroYData) + min(gyroYData)) / 2
    gyroZOffset = (max(gyroZData) + min(gyroZData)) / 2
    #print("Calibration complete.")
    return [gyroXOffset, gyroYOffset, gyroZOffset]

def set_initial(mag_offset = [0,0,0]):
    """
    This function is complete. Finds initial RPY values.

    Parameters:
        mag_offset (list): magnetometer calibration offsets
    """
    #Sets the initial position for plotting and gyro calculations.
    print("Preparing to set initial angle. Please hold the IMU still.")
    time.sleep(3)
    print("Setting angle...")
    accelX, accelY, accelZ = accel_gyro.acceleration #m/s^2
    magX, magY, magZ = mag.magnetic #gauss
    #Calibrate magnetometer readings. Defaults to zero until you
    #write the code
    magX = magX - mag_offset[0]
    magY = magY - mag_offset[1]
    magZ = magZ - mag_offset[2]
    roll = roll_am(accelX, accelY,accelZ)
    pitch = pitch_am(accelX,accelY,accelZ)
    yaw = yaw_am(accelX,accelY,accelZ,magX,magY,magZ)
    print("Initial angle set.")
    return [roll,pitch,yaw]
