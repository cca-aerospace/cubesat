#!/usr/bin/env python3

# https://forums.raspberrypi.com/viewtopic.php?t=112888

import picamera2
# import picamera2.array
import numpy as np
from time import sleep
import cv2

# import opencv


# def main():
#     with picamera2.Picamera2() as camera:
#         camera.resolution = (320, 240)
#         with camera.capture_array() as stream:
#             camera.exposure_mode = 'auto'
#             camera.awb_mode = 'auto'
#             print("Initializing Pi Camera")
#             sleep(2)
#             print(type(stream))

# if __name__ == "__main__": 
#     main()
camera = picamera2.Picamera2()
config = camera.create_preview_configuration({'format': 'BGR888'})

camera.configure(config)
camera.exposure_mode = 'auto'
# camera.brightness
camera.awb_mode = 'auto'
camera.start()



# print(img)
# img = cv2.cvtColor(img, cv2.COLOR_HSV2BGR)
# cv2.imwrite(filename="test.png",img=img)
# if img.max() >= 200:
#     print("light is on")

# else:
#     print("light is off")

# print(array.shape)
while True:
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
    print('img max:', img[::,].max())
    if img.max() >= 200:
        print("light is on")
    else:
        print("light is off")
    imgBGR = cv2.cvtColor(img, cv2.COLOR_HSV2BGR)
    print(img.max())
    print('imgBGR', imgBGR)
    print(array.shape)
    cv2.imwrite(filename="test2.png",img=imgBGR)
    print("saved")
    cv2.imwrite(filename="bw.png",img=binaryImg)



