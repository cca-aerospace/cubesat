#!/usr/bin/env python3

# https://forums.raspberrypi.com/viewtopic.php?t=112888

from picamera2 import Picamera2, Preview
# import picamera2.array
import numpy as np
import time
from time import sleep
import math
import cv2
import threading 
import run


import board
from adafruit_lsm6ds.lsm6dsox import LSM6DSOX as LSM6DS
from adafruit_lis3mdl import LIS3MDL
#from git import Repo
#from pathlib import Path

def calibrate_mag():
    i2c = board.I2C()
    mag = LIS3MDL(i2c)
    magXData = []
    magYData = []
    magZData = []
    print("Preparing to calibrate magnetometer. Please wave around.")
    time.sleep(3)
    print("Calibrating...")
    while True: 
        time.sleep(5)
        for _ in range(100):
            magX, magY, magZ = mag.magnetic
            magXData.append(magX)
            magYData.append(magY)
            magZData.append(magZ)
        break
    
    print('magXData:', magXData)
    print('magYData:', magYData)
    print('magZData:', magZData)
    magXOffset = (max(magXData) + min(magXData)) / 2
    magYOffset = (max(magYData) + min(magYData)) / 2
    magZOffset = (max(magZData) + min(magZData)) / 2
    print("Calibration complete.")
    print(magXOffset, magYOffset, magZOffset)
    return [magXOffset, magYOffset, magZOffset]


def get_rotation():
    # imu initialization
    i2c = board.I2C()
    accel_gyro = LSM6DS(i2c)
    mag = LIS3MDL(i2c)
    accelX, accelY, accelZ = accel_gyro.acceleration
    magX, magY, magZ = mag.magnetic
    #mag_offset = calibrate_mag()
    #magX = magX - mag_offset[0]
    #magY = magY - mag_offset[1]
    #magZ = magZ - mag_offset[2]

    #yaw calculation
    roll = (180/np.pi)*np.arctan2(accelY, np.sqrt(accelX ** 2 + accelZ ** 2))
    pitch = (180/np.pi)*np.arctan2(accelX, np.sqrt(accelY ** 2 + accelZ ** 2))
    mag_x = magX * np.cos(pitch) + magY * np.sin(roll) * np.sin(pitch) + magZ * np.cos(roll) * np.sin(pitch)
    mag_y = magY * np.cos(roll) - magZ * np.sin(roll)
    return (180/np.pi)*np.arctan2(-mag_y, mag_x)



def collect_data(camera):
    # idx 0 stores rotational displacement
    # idx 1 stores light 'on' or 'off'
    train_data = []

    while len(train_data) < 3:
        sleep(1)
        array = camera.capture_array()
        hsv = cv2.cvtColor(array, cv2.COLOR_BGR2HSV)
        img = cv2.cvtColor(array, cv2.COLOR_RGB2HSV)
        bw = cv2.cvtColor(array, cv2.COLOR_BGR2GRAY)
        lower_ = np.array([0,0,220])
        upper = np.array([180,255,255])
        mask = cv2.inRange(hsv, lower_, upper)
        (T,binaryImg) = cv2.threshold(bw,200,255,cv2.THRESH_BINARY)
        cv2.imwrite("mask2.png",mask)
        result = cv2.bitwise_and(img,img, mask= mask)


        # img_state: True if on, False if off
        LED_state = bw.max() >= 200
        print('img max:', bw[::,].max())
        if bw.max() >= 200:
            print("light is on")
        else:
            print("light is off")
            
        imgBGR = cv2.cvtColor(img, cv2.COLOR_HSV2BGR)
        #print('imgBGR', imgBGR)
        #print(array.shape)
        cv2.imwrite(filename="test2.png",img=imgBGR)
        print("saved")
        cv2.imwrite(filename="bw.png",img=binaryImg)

        

        # determine rotational displacement
        print('Calculating rotational displacement...')
        rotational_displacement = []
        avg = 0
        for _ in range(50):
            rotational_displacement.append(get_rotation())
        avg = sum(rotational_displacement) / len(rotational_displacement)
        print('curr rotation:', avg)
        #camera.capture_file('light_on.png')
        train_data.append([avg, LED_state])
        print(train_data)


    return train_data


def lightMeter():
    camera = Picamera2()
    config = camera.create_preview_configuration({'format': 'BGR888'}) 
    camera.configure(config)
    camera.exposure_mode = 'spotlight'
    camera.awb_mode = 'auto'
    camera.start()
    test_data = []
    
    while len(test_data) < 3:
        sleep(1)
        array = camera.capture_array()
        hsv = cv2.cvtColor(array, cv2.COLOR_BGR2HSV)
        img = cv2.cvtColor(array, cv2.COLOR_RGB2HSV)
        bw = cv2.cvtColor(array, cv2.COLOR_BGR2GRAY)
        lower_ = np.array([0,0,220])
        upper = np.array([180,255,255])
        mask = cv2.inRange(hsv, lower_, upper)
        (T,binaryImg) = cv2.threshold(bw,200,255,cv2.THRESH_BINARY)
        cv2.imwrite("mask2.png",mask)
        result = cv2.bitwise_and(img,img, mask= mask)


        # img_state: True if on, False if off
        LED_state = img.max() >= 200
        print('img max:', img[::,].max())
        if img.max() >= 200:
            print("light is on")
        else:
            print("light is off")
            
        imgBGR = cv2.cvtColor(img, cv2.COLOR_HSV2BGR)
        #print('imgBGR', imgBGR)
        #print(array.shape)
        cv2.imwrite(filename="test2.png",img=imgBGR)
        print("saved")
        cv2.imwrite(filename="bw.png",img=binaryImg)
    
    
    

def main():

    #configure picamera
    camera = Picamera2()
    config = camera.create_preview_configuration({'format': 'BGR888'})

    camera.configure(config)
    camera.exposure_mode = 'spotlight'
    # camera.brightness
    camera.awb_mode = 'auto'
    camera.start()

    # idx 0 stores rotational displacement
    # idx 1 stores light 'on' or 'off'
    test_data = []

    while len(test_data) < 3:
        sleep(1)
        params = cv2.SimpleBlobDetector_Params()
        params.filterByArea = True
        params.minArea = 150
        params.maxArea = 2000
        detector = cv2.SimpleBlobDetector_create(params)
        
        
        array = camera.capture_array()
        hsv = cv2.cvtColor(array, cv2.COLOR_BGR2HSV)
        img = cv2.cvtColor(array, cv2.COLOR_RGB2HSV)
        bw = cv2.cvtColor(array, cv2.COLOR_BGR2GRAY)
        mask = cv2.inRange(bw, 150, 255)

        (T,binaryImg) = cv2.threshold(bw,200,255,cv2.THRESH_BINARY)
        keypoints = detector.detect(binaryImg)
        
        img_key = cv2.drawKeypoints(img, keypoints, np.array([]), (0,50,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        cv2.imwrite("keypoints.png", img_key)
        number_of_keypoints = len(detector.keypoints)
        
        
        cv2.imwrite("mask2.png",mask)
        result = cv2.bitwise_and(img,img, mask= mask)


        # img_state: True if on, False if off
        LED_state = number_of_keypoints > 0
        print('img max:', img[::,].max())
        if LED_state:
            print("light is on")
        else:
            print("light is off")
            
        imgBGR = cv2.cvtColor(img, cv2.COLOR_HSV2BGR)
        #print('imgBGR', imgBGR)
        #print(array.shape)
        cv2.imwrite(filename="test2.png",img=imgBGR)
        print("saved")
        cv2.imwrite(filename="bw.png",img=binaryImg)

        

        # determine rotational displacement
        print('Calculating rotational displacement...')
        rotational_displacement = []
        avg = 0
        for _ in range(10):
            rotational_displacement.append(get_rotation())
        avg = sum(rotational_displacement) / len(rotational_displacement)
        print('curr rotation:', avg)
        #camera.capture_file('light_on.png')
        test_data.append([avg, LED_state])
        print(test_data)
        

    # compare train data and test data for three positions
    print('comparing data...')
    train_data = collect_data(camera)
    for i in range(len(train_data)):
        if abs(train_data[i][0] - test_data[i][0]) < 20:
            if train_data[i][1] and not test_data[i][1]:
                print('position %s had a power outage' % (i+1))
            else:
                print('position %s: no significant change detected' % (i+1))
        else:
            print('position %s: couldnt pass threshold' % (i+1))

if __name__ == "__main__": 
    main()
