#!/usr/bin/env python3

import RPi.GPIO as GPIO
import time
import threading
import math

TRIG = 11
ECHO = 12
Buzzer = 13
motor_pin = 7


def setup():
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(TRIG, GPIO.OUT)
    GPIO.setup(ECHO, GPIO.IN)


def setupBuzzer(pin):
    global BuzzerPin
    BuzzerPin = pin
    GPIO.setmode(GPIO.BOARD)  # Numbers GPIOs by physical location
    GPIO.setup(BuzzerPin, GPIO.OUT)
    GPIO.output(BuzzerPin, GPIO.HIGH)


def buzzerOn():
    GPIO.output(BuzzerPin, GPIO.LOW)


def buzzerOff():
    GPIO.output(BuzzerPin, GPIO.HIGH)


def setupMotor(motor_pin):
    GPIO.setmode(GPIO.BOARD)  # Numbers GPIOs by physical location
    GPIO.setup(motor_pin, GPIO.OUT)


def motorOn(motor_pin):
    GPIO.output(motor_pin, GPIO.HIGH)


def motorOff(motor_pin):
    GPIO.output(motor_pin, GPIO.LOW)


def activate(x):
    motorOn()
    buzzerOn()
    time.sleep(x)
    motorOff()
    buzzerOff()
    time.sleep(x)


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


def loop(): # UNUSED
    buzzerOff()
    motorOff(motor_pin)
    while True:
        dis = distance()
        print(dis, "cm")
        print("")
        min_distance = 1
        rate_of_change = 0.05
        base_interval = 0.05  # minimum distance between
        max_distance = 75

        buzzer_min_distance = 1
        buzzer_rate_of_change = 0.05
        buzzer_base_interval = 0.05  # minimum distance between
        buzzer_max_distance = 75
        if dis <= min_distance:
            buzzer_interval = buzzer_base_interval * math.exp(
                buzzer_rate_of_change * (buzzer_dis - buzzer_min_distance)
            )
            motor_interval = base_interval
        elif dis >= max_distance:
            buzzerOff()
            motorOff(motor_pin)
            continue
        else:
            buzzerOff()
            motor_interval = base_interval * math.exp(
                rate_of_change * (dis - min_distance)
            )
        motor_interval = round(motor_interval, 3)
        print(motor_interval)
        motorOff(motor_pin)
        time.sleep(motor_interval)
        motorOn(motor_pin)
        time.sleep(0.15)


def destroy():
    print("Cleaning up GPIO")
    GPIO.output(BuzzerPin, GPIO.HIGH)
    buzzerOff()
    motorOff()
    GPIO.cleanup()


ultrasonic_distance = 0


def manage_distance():
    global ultrasonic_distance
    while True:
        time.sleep(0.25)
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
        buzzerOff()
        print("Buzzer interval: " + str(buzzer_interval))
        time.sleep(buzzer_interval)
        buzzerOn()
        time.sleep(0.15)


def manage_motor(motor_pin):
    while True:
        min_distance = 1
        rate_of_change = 0.05
        base_interval = 0.05  # minimum distance between
        max_distance = 100

        if ultrasonic_distance <= min_distance:
            #print("yes 1")
            motor_interval = base_interval
        elif ultrasonic_distance >= max_distance:
            continue
        else:
            motor_interval = base_interval * math.exp(
                rate_of_change * (ultrasonic_distance - min_distance)
            )
        motor_interval = round(motor_interval, 3)
        print("Motor interval: " + str(motor_interval))
        time.sleep(buzzer_interval)
        motorOn(motor_pin)
        time.sleep(0.25)
        motorOff(motor_pin)
        
        
kill_threads = False
if __name__ == "__main__":
    setup()
    setupBuzzer(Buzzer)
    setupMotor(motor_pin)
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
