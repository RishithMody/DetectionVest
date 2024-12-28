import RPi.GPIO as GPIO
import time
import threading
import math

TRIG = 11
ECHO = 12
Buzzer = 13
motor_pin = 7

def setup():
	GPIO.setmode(GPIO.BOARD)  # Numbers GPIOs by physical location
	GPIO.setup(TRIG, GPIO.OUT)
	GPIO.setup(ECHO, GPIO.IN)
	
	#motor setup
	global MotorPin
	MotorPin = motor_pin
	GPIO.setup(motor_pin, GPIO.OUT)
	
	# buzzer setup
	global BuzzerPin
	BuzzerPin = Buzzer
	GPIO.setup(BuzzerPin, GPIO.OUT)
	GPIO.output(BuzzerPin, GPIO.HIGH)

def setupBuzzer(pin):
	global BuzzerPin
	BuzzerPin = pin
	GPIO.setup(BuzzerPin, GPIO.OUT)
	GPIO.output(BuzzerPin, GPIO.HIGH)

def buzzerOn():
	GPIO.output(BuzzerPin, GPIO.LOW)

def buzzerOff():
	GPIO.output(BuzzerPin, GPIO.HIGH)

def setupMotor(pin):
	global MotorPin
	MotorPin = pin
	GPIO.setup(motor_pin, GPIO.OUT)

def motorOn():
	print("yay")
	GPIO.output(MotorPin, GPIO.HIGH)
	
def motorOff():
	GPIO.output(MotorPin, GPIO.LOW)

def activate(x):
	motorOn()
	buzzerOn()
	time.sleep(x)
	motorOff()
	buzzerOff()
	time.sleep(x)

def distance():
	GPIO.output(TRIG, 0)
	time.sleep(0.05)

	GPIO.output(TRIG, 1)
	time.sleep(0.05)
	GPIO.output(TRIG, 0)

	while GPIO.input(ECHO) == 0:
		a = 0
	time1 = time.time()
	while GPIO.input(ECHO) == 1:
		a = 1
	time2 = time.time()

	during = time2 - time1
	return during * 340 / 2 * 100

def loop():
	buzzerOff()
	motorOff()
	while True:
		dis = distance()
		max_distance = 50
		if (dis <= max_distance): activateMotor(dis)

def activateMotor(dis):
	print(dis, "cm")
	motorOn()
	time.sleep(0.5)
	motorOff()
	time.sleep(0.5)

def destroy():
	print("Cleaning up GPIO")
	GPIO.output(BuzzerPin, GPIO.HIGH)
	buzzerOff()
	motorOff()
	GPIO.cleanup()
	
if __name__ == "__main__":
	setup()
	loop()
