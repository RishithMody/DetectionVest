import RPi.GPIO as GPIO
import time
GPIO.setmode(GPIO.BOARD) 
def motorOn(pin):
    GPIO.output(pin, GPIO.HIGH)

def motorOff(pin):
    GPIO.output(pin, GPIO.LOW)

pinnum=22
GPIO.setup(pinnum, GPIO.OUT) # Right
while True:
    motorOn(pinnum)
    print("motor on")
    time.sleep(1)

    motorOff(pinnum)
    print("motor off")
    time.sleep(1)