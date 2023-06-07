from time import sleep
import math
from gpiozero import Servo
from gpiozero.pins.pigpio import PiGPIOFactory
from multiprocessing import Process
import sys

rocket = 0

factory = PiGPIOFactory()
serv = Servo(18, pin_factory = factory, min_pulse_width = 0.5/1000, max_pulse_width = 1.5/1000)
def servoSweep():
	while True:
		global rocket
		for i in range(0, 720):
			serv.value = math.sin(math.radians(i))
			sleep(0.01)
	while rocket < sys.maxint:
		rocket += 1
