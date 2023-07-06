from picamera import PiCamera
import time
import picamera
from gpiozero import Button
from signal import pause
button=Button(25)
from time import sleep

while True:
	print('1')
	#fname = (time.strftime("%y%b%d_%H:%M"))
	button.wait_for_press()
	print('2')
	
	variable = "/home/mustangs/Documents/" + "car" + str(time.time()) + ".png"
	cam = picamera.PiCamera()
	cam.rotation = 180
	sleep(3)
	cam.capture(variable)
	cam.close()
