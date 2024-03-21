#!/usr/bin/env python3

# https://forums.raspberrypi.com/viewtopic.php?t=112888

import picamera2
# import picamera2.array
import numpy as np
from time import sleep


def main():
    with picamera2.Picamera2() as camera:
        camera.resolution = (320, 240)
        with camera.capture_array() as stream:
            camera.exposure_mode = 'auto'
            camera.awb_mode = 'auto'
            print("Initializing Pi Camera")
            sleep(2)
            camera.exposure_mode = 'off'
            while True:
                try:
                    camera.capture(stream, format='r')
                    # pixAverage = int(np.average(stream.array[...,1]))
                    pixAverage = np.average(stream.array[...,1])
                    print ("Light Meter pixAverage: {:.1f}".format(pixAverage))
                    sleep(1)
                    stream.truncate()
                    stream.seek(0)
                except KeyboardInterrupt:
                    print("\nExiting ..")
                    break

if __name__ == "__main__": main()