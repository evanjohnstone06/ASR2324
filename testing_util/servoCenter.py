from gpiozero import Servo
from gpiozero.pins.pigpio import PiGPIOFactory
factory = PiGPIOFactory()
serv = Servo(18, pin_factory = factory, min_pulse_width = 0.5/1000, max_pulse_width = 1.5/1000)
serv.value=0.5

