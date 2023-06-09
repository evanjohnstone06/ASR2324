import RPi.GPIO as GPIO
import os
import socket
import fcntl
import struct
import time
import serial
from time import gmtime, strftime, sleep
from gpiozero import Button
from gpiozero.pins.pigpio import PiGPIOFactory
from gpiozero import Servo
import servoThing
from multiprocessing import Process
from multiprocessing import Pool
import sys
from signal import signal, SIGTERM, SIGHUP, pause
from rpi_lcd import LCD
import math


rocket = 0
stopS = False
factory = PiGPIOFactory()
serv = Servo(18, pin_factory = factory, min_pulse_width = 0.5/1000, max_pulse_width = 1.5/1000)
ser = serial.Serial("/dev/ttyAMA0", 115200)

lcd = LCD()

    
def getTFminiData():
    while True:
        global rocket
        tooClose = False
        time.sleep(0.2)
        count = ser.in_waiting
        if count > 8:
            recv = ser.read(9)   
            ser.reset_input_buffer()
            # type(recv), 'str' in python2(recv[0] = 'Y'), 'bytes' in python3(recv[0] = 89)
            # type(recv[0]), 'str' in python2, 'int' in python3 
            if recv[0] == 0x59 and recv[1] == 0x59:     #python3
                distance = recv[2] + recv[3] * 256
                return distance
                strength = recv[4] + recv[5] * 256
                lcd.text(str(distance), 1)
                print(distance)
                ser.reset_input_buffer()
                #return distance
                if distance <= 20:
                    tooClose = True
                    lcd.text(str(tooClose), 2)
                    
                else:
                    tooClose = False
                    lcd.text(str(tooClose), 2)
        while rocket < 999999:
            rocket += 1

def servoSweep():
    servStage = 1
    while True:
        
        dist = getTFminiData()
        global rocket
        if dist < 20:
            #serv.detach()
            print('u should stawp')
        else:
            #servStage = 4
            print('servVal', serv.value)
            print('stage: ', servStage)
            if servStage==1:
                serv.value+=0.1
                #sleep(0.01)
                if serv.value>=0.9:
                    servStage=2
            if servStage==2:
                serv.value-=0.1
                #sleep(0.01)
                if serv.value<=-0.9:
                    servStage=1
                
        print(dist)

        
    while rocket < sys.maxint:
        rocket += 1



pool = Pool()
if __name__ == '__main__':
    try:
        
        if ser.is_open == False:
            ser.open()
        p1 = Process(target = getTFminiData)
        p1.start()
        p2 = Process(target = servoSweep)
        p2.start()

    except KeyboardInterrupt:   # Ctrl+C
        if ser != None:
            ser.close()
                
