import RPi.GPIO as GPIO
import time

TRIG = 11
ECHO = 12
Buzzer = 13
motor_pin = 16

def setup():
	GPIO.setmode(GPIO.BOARD)  # Set to BOARD mode
	GPIO.setup(TRIG, GPIO.OUT)
	GPIO.setup(ECHO, GPIO.IN)

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
	GPIO.setup(MotorPin, GPIO.OUT)

def motorOn():
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
	time.sleep(0.00001)  # 10 microseconds
	GPIO.output(TRIG, 0)

	timeout = time.time() + 1  # 1 second timeout
	while GPIO.input(ECHO) == 0 and time.time() < timeout:
		pass
	time1 = time.time()
	while GPIO.input(ECHO) == 1 and time.time() < timeout:
		pass
	time2 = time.time()

	during = time2 - time1
	return during * 340 / 2 * 100

def loop():
	buzzerOff()
	motorOff()
	while True:
		dis = distance()
		print(f"Distance: {dis:.2f} cm")  # Print the distance
		max_distance = 50
		if 0 < dis <= max_distance:
			interval = getInterval(dis)
			activate(interval)  # Activate for 0.5 seconds

def getInterval(dis):
	interval = (dis/50)
	print(interval)
	return interval

def destroy():
	print("Cleaning up GPIO")
	GPIO.output(BuzzerPin, GPIO.HIGH)
	buzzerOff()
	motorOff()
	GPIO.cleanup()

if __name__ == "__main__":
	try:
		setup()
		setupBuzzer(Buzzer)
		setupMotor(motor_pin)
		loop()
	except KeyboardInterrupt:  # Exit when Ctrl+C is pressed
		destroy()



