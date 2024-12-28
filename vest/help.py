import RPi.GPIO as GPIO
import time

TRIG_1 = 29
ECHO_1 = 31
TRIG_2 = 32
ECHO_2 = 33
TRIG_3 = 35
ECHO_3 = 36
TRIG_4 = 37
ECHO_4 = 38

Buzzer = 13

motor_pin_1 = 16
motor_pin_2 = 18
motor_pin_3 = 22
motor_pin_4 = 12

def setup():
	GPIO.setmode(GPIO.BOARD)  # Set to BOARD mode
	GPIO.setup(TRIG_1, GPIO.OUT)
	GPIO.setup(ECHO_1, GPIO.IN)
	GPIO.setup(TRIG_2, GPIO.OUT)
	GPIO.setup(ECHO_2, GPIO.IN)
	GPIO.setup(TRIG_3, GPIO.OUT)
	GPIO.setup(ECHO_3, GPIO.IN)
	GPIO.setup(TRIG_4, GPIO.OUT)
	GPIO.setup(ECHO_4, GPIO.IN)
	
	setupBuzzer(Buzzer)
	
	setupMotor(motor_pin_1)
	setupMotor(motor_pin_2)
	setupMotor(motor_pin_3)
	setupMotor(motor_pin_4)

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
	GPIO.setup(pin, GPIO.OUT)

def motorOn(pin):
	GPIO.output(pin, GPIO.HIGH)

def motorOff(pin):
	GPIO.output(pin, GPIO.LOW)

def allOff():
	buzzerOff()
	motorOff(motor_pin_1)
	motorOff(motor_pin_2)
	motorOff(motor_pin_3)
	motorOff(motor_pin_4)

def activate(x, motornum):
	motorOn(motornum)
	buzzerOn()
	time.sleep(x)
	motorOff(motornum)
	buzzerOff()
	time.sleep(x)
	
def distance(trig, echo):
	GPIO.output(trig, 0)
	time.sleep(0.05)

	GPIO.output(trig, 1)
	time.sleep(0.00001)  # 10 microseconds
	GPIO.output(trig, 0)

	timeout = time.time() + 1  # 1 second timeout
	while GPIO.input(echo) == 0 and time.time() < timeout:
		pass
	time1 = time.time()
	while GPIO.input(echo) == 1 and time.time() < timeout:
		pass
	time2 = time.time()

	during = time2 - time1
	return during * 340 / 2 * 100

def loop():
	allOff()
	while True:
		dis1 = distance(TRIG_1, ECHO_1)
		dis2 = distance(TRIG_2, ECHO_2)
		dis3 = distance(TRIG_3, ECHO_3)
		dis4 = distance(TRIG_4, ECHO_4)
		
		print(f"Distance 1: {dis1:.2f} cm")  # Print the distance
		print(f"Distance 2: {dis2:.2f} cm")  # Print the distance
		print(f"Distance 3: {dis3:.2f} cm")  # Print the distance
		print(f"Distance 4: {dis4:.2f} cm")  # Print the distance
		
		motornum = motor_pin_1
		if dis2 < dis1:
			dis1=dis2
			motornum = motor_pin_2
		if dis3 < dis1:
			dis1=dis3
			motornum = motor_pin_3
		if dis4 < dis1:
			dis1=dis4
			motornum = motor_pin_4
		
		max_distance = 50
		if 0 < dis1 <= max_distance:
			activate(0.5, motornum)  # Activate for 0.5 seconds

def destroy():
	print("Cleaning up GPIO")
	GPIO.output(BuzzerPin, GPIO.HIGH)
	allOff()
	GPIO.cleanup()

if __name__ == "__main__":
	try:
		setup()
		loop()
	except KeyboardInterrupt:  # Exit when Ctrl+C is pressed
		destroy()



