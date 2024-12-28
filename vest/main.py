import RPi.GPIO as GPIO
import time
import threading

TRIG_1 = 11
ECHO_1 = 12
TRIG_2 = 31
ECHO_2 = 33

ultrasonic_sensors = [(11,12), (31,33)] # TRIG, ECHO TUPLE - (LEFT) (MIDDLE) (RIGHT) (BACK)

Buzzer = 13
motor_pin_1 = 16
# motor_pin_2 = x
# motor_pin_3
# motor_pin_4
#

def setup():
	GPIO.setmode(GPIO.BOARD)  # Set to BOARD mode
	#trig/echo setup
	GPIO.setup(TRIG_1, GPIO.OUT)
	GPIO.setup(ECHO_1, GPIO.IN)
	GPIO.setup(TRIG_2, GPIO.OUT)
	GPIO.setup(ECHO_2, GPIO.IN)
	#buzzer/motor setup
	setupBuzzer(Buzzer)
	setupMotor(motor_pin_1)
	
def setupBuzzer(pin):
	GPIO.setup(pin, GPIO.OUT)
	GPIO.output(pin, GPIO.HIGH)

def buzzerOn(pin):
	GPIO.output(pin, GPIO.LOW)

def buzzerOff(pin):
	GPIO.output(pin, GPIO.HIGH)

def setupMotor(pin):
	global MotorPin
	MotorPin = pin
	GPIO.setup(pin, GPIO.OUT)

def motorOn(pin):
	GPIO.output(pin, GPIO.HIGH)

def motorOff(pin):
	GPIO.output(pin, GPIO.LOW)

def activate(x):
	motorOn(motor_pin_1)
	buzzerOn(Buzzer)
	time.sleep(x)
	motorOff(motor_pin_1)
	buzzerOff(Buzzer)
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
	
    
distances = {}
def distance_thread(trig, echo, dist):
	while True:
		time.sleep(0.25)
		dist = distance(trig, echo)
		print(f"Distance: {dist:.2f} cm")

	
	
	
	

def loop():
	buzzerOff(Buzzer)
	motorOff(motor_pin_1)
	while True:
		dis1 = distance(TRIG_1, ECHO_1)
		dis2 = distance(TRIG_2, ECHO_2)
		if dis2 <= dis1:
			dis1 = dis2
		print(f"Distance: {dis1:.2f} cm")  # Print the distance
		max_distance = 50
		if 0 < dis1 <= max_distance:
			activate(0.5)  # Activate for 0.5 seconds

def destroy():
	print("Cleaning up GPIO")
	GPIO.output(Buzzer, GPIO.HIGH)
	buzzerOff(Buzzer)
	motorOff(motor_pin_1)
	GPIO.cleanup()

if __name__ == "__main__":
	try:
		setup()
		#threads = []
		DISTANCE_1 = 0
		t = threading.Thread(target=distance_thread, args=(TRIG_1, ECHO_1, DISTANCE_1))
		t.start()
		#loop()
	except KeyboardInterrupt:  # Exit when Ctrl+C is pressed
		destroy()



