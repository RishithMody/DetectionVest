#!/usr/bin/env python3

import RPi.GPIO as GPIO
import time
import threading
import math

TRIG = 11
ECHO = 12
Buzzer = 13
Motor = 7

def setup():
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(TRIG, GPIO.OUT)
    GPIO.setup(ECHO, GPIO.IN)
    
def setupBuzzer(pin):
    global buzzer_pin
    buzzer_pin = pin
    GPIO.setmode(GPIO.BOARD)  # Numbers GPIOs by physical location
    GPIO.setup(buzzer_pin, GPIO.OUT)

def buzzerOn():
    GPIO.output(buzzer_pin, GPIO.LOW)

def buzzerOff():
    GPIO.output(buzzer_pin, GPIO.HIGH)

def setupMotor(pin):
    global motor_pin
    motor_pin = pin
    GPIO.setmode(GPIO.BOARD)  # Numbers GPIOs by physical location
    GPIO.setup(motor_pin, GPIO.OUT)
    GPIO.output(motor_pin, GPIO.HIGH)

def motorOn():
    GPIO.output(motor_pin, GPIO.LOW)

def motorOff():
    GPIO.output(motor_pin, GPIO.HIGH)

def activate(x):
    motorOn()
    buzzerOn()
    time.sleep(x)
    motorOff()
    buzzerOff()
    time.sleep(x)
    
def activateMotor(interval):
	motorOn()
	time.sleep(interval)
	motorOff()
	time.sleep(interval)
    
def destroy():
    print("Cleaning up GPIO")
    GPIO.output(buzzer_pin, GPIO.HIGH)
    buzzerOff()
    motorOff()
    GPIO.cleanup()

def distance():
    GPIO.output(TRIG, 0)
    time.sleep(0.000002)

    GPIO.output(TRIG, 1)
    time.sleep(0.00001)
    GPIO.output(TRIG, 0)

    while GPIO.input(ECHO) == 0:
        a = 0
    time1 = time.time()
    while GPIO.input(ECHO) == 1:
        a = 1
    time2 = time.time()

    during = time2 - time1
    return during * 340 / 2 * 100

ultrasonic_distance = 0

def manage_distance():
    global ultrasonic_distance
    while True:
        time.sleep(0.5)
        if kill_threads:
            break
        ultrasonic_distance = distance()
        print(str(ultrasonic_distance) + " cm")

def manage_buzzer():
    while True:
        min_distance = 1
        rate_of_change = 0.05
        base_interval = 0.05  # minimum distance between
        max_distance = 10

        if ultrasonic_distance <= min_distance:
            buzzer_interval = base_interval
        elif ultrasonic_distance >= max_distance:
            buzzerOff()
            continue
        else:
            buzzer_interval = base_interval * math.exp(
                rate_of_change * (ultrasonic_distance - min_distance)
            )
        buzzer_interval = round(buzzer_interval, 3)
        buzzer_interval = 1/ultrasonic_distance
        
        buzzerOff()
        print("Buzzer interval: " + str(buzzer_interval))
        time.sleep(buzzer_interval)
        buzzerOn()
        time.sleep(0.05)

def manage_motor(motor_pin):
    while True:
        min_distance = 1
        rate_of_change = 0.05
        base_interval = 0.05  # minimum distance between
        max_distance = 100

        if ultrasonic_distance <= min_distance:
            motor_interval = base_interval
        elif ultrasonic_distance > max_distance:
            continue
        else:
            motor_interval = base_interval * math.exp(
                rate_of_change * (ultrasonic_distance - min_distance)
            )
        motor_interval = round(motor_interval, 3)
        print("Motor interval: " + str(motor_interval))
        activateMotor(motor_interval)
        
        
kill_threads = False

if __name__ == "__main__":
    setup()
    setupBuzzer(Buzzer)
    setupMotor(Motor)
    threads = []
    try:
        distance_thread = threading.Thread(target=manage_distance, args=())
        threads.append(distance_thread)
        #buzzer_thread = threading.Thread(target=manage_buzzer, args=())
        #threads.append(buzzer_thread)
        motor_thread = threading.Thread(target=manage_motor, args=(motor_pin,))
        threads.append(motor_thread)

        for t in threads:
           t.start()

    except KeyboardInterrupt:
        kill_threads = True
        for t in threads:
            t.join()  # wait for the thread to finish
        destroy()
