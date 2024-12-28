import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BOARD) 
trig, echo = 37, 38

def distance(trig, echo):
	GPIO.setup(trig, GPIO.OUT)
	GPIO.setup(echo, GPIO.IN)
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

while True:
    print(distance(trig, echo))
    time.sleep(0.25)