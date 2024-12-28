import RPi.GPIO as GPIO
import time
import threading
import math
import os

# TODO:
# Configure exponential function in motor_thread to what feels natural while wearing
# Buzzer integration (exponential beeping from minimum motor value to 0? or other)

toggle_print = True

# Ultrasonic Sensors - (3rd person perspective, not of the wearer)
TRIG_1, ECHO_1 = 29, 31 # Right
TRIG_2, ECHO_2 = 32, 33 # Middle 
TRIG_3, ECHO_3 = 35, 16 # Left
TRIG_4, ECHO_4 = 37, 38 # Back
ultrasonic_sensors = [(TRIG_1, ECHO_1), (TRIG_2, ECHO_2), (TRIG_3, ECHO_3), (TRIG_4, ECHO_4)] # Used to iterate to create threads
distance_calculation_interval = 0.05 # Time to sleep between distance calculations (per thread)

# Buzzer Pin
buzzer_pin = 15
buzzer_toggle_duration = 0.1

# Tilt Switch Pin
tilt_pin = 7 # Placed closest to center of mass
tilt_inactivity_after_sec = 10 # Amount of seconds of detected inactivity to wait before stopping outputs

# Vibration Motors
motor_pin_1 = 22 # Right
motor_pin_2 = 18 # Middle
motor_pin_3 = 40 # Left
motor_pin_4 = 12 # Back
motor_toggle_duration = 0.15 # Length of pulse from motor

def setup():
	GPIO.setmode(GPIO.BOARD)  # Set to BOARD mode
	GPIO.setwarnings(False)
	
	### Ultrasonic Setup
	
	# Right
	GPIO.setup(TRIG_1, GPIO.OUT)
	GPIO.setup(ECHO_1, GPIO.IN)
	
	# Middle
	GPIO.setup(TRIG_2, GPIO.OUT)
	GPIO.setup(ECHO_2, GPIO.IN)
	
	# Left
	GPIO.setup(TRIG_3, GPIO.OUT)
	GPIO.setup(ECHO_3, GPIO.IN)
	
	# Back
	GPIO.setup(TRIG_4, GPIO.OUT)
	GPIO.setup(ECHO_4, GPIO.IN)
	
	### Buzzer Setup
	GPIO.setup(buzzer_pin, GPIO.OUT)
	GPIO.output(buzzer_pin, GPIO.HIGH)
	
	### Vibration Motor Setup
	GPIO.setup(motor_pin_1, GPIO.OUT) # Right
	GPIO.setup(motor_pin_2, GPIO.OUT) # Middle
	GPIO.setup(motor_pin_3, GPIO.OUT) # Left
	GPIO.setup(motor_pin_4, GPIO.OUT) # Back
	
	### Tilt Switch Setup
	GPIO.setup(tilt_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)    # Set BtnPin's mode is input, and pull up to high level(3.3V)
	#GPIO.add_event_detect(tilt_pin, GPIO.BOTH, callback=on_tilt_status_change, bouncetime=200) # Set the event


def allOff():
	buzzerOff(buzzer_pin)
	motorOff(motor_pin_1)
	motorOff(motor_pin_2)
	motorOff(motor_pin_3)
	motorOff(motor_pin_4)

	
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
	return cmToFt(during * 340 / 2 * 100)


def cmToFt(cm_dist):
    return (cm_dist/30.48)

distances = [] # [Right, Middle, Left, Back] - Contains float. Also manages buzzer output
buzzers = []
def distance_thread(TRIG, ECHO, sensor_number):
    global distances
    buzzer_status = False
    while True:
        dist = distance(TRIG, ECHO)
        if len(distances) >= sensor_number: # Avoids error, assures everything's placed properly in the list.
            distances[sensor_number - 1] = dist # Set distance value to correct placement in list 
        else:
            distances.append(dist) # Appending distance to list as none was found in placement (sensor_number) specified
            
        if len(buzzers) <= sensor_number:
            buzzers.append(False)
            
        time.sleep(distance_calculation_interval)

### Tilt Switch Functions
last_tilt_status = 0
last_tilt_unix = round(time.time())
tilt_toggle = False
tilt_status = 0
last_tilt_seconds = 0
def tilt_thread(pin):
    global last_tilt_status, last_tilt_unix, tilt_toggle, tilt_status, last_tilt_seconds
    while True:
        tilt_status = GPIO.input(pin)
        if last_tilt_status != tilt_status:
            last_tilt_status = tilt_status
            last_tilt_unix = round(time.time())
            
            
        time.sleep(0.25)
        last_tilt_seconds = round(time.time()) - last_tilt_unix # how long ago last tilt occured
        tilt_toggle = last_tilt_seconds < tilt_inactivity_after_sec # if ten seconds elapsed since last tilt = False below = True
        
        
### Buzzer Functions
def buzzerOn(pin):
    GPIO.output(pin, GPIO.LOW)

def buzzerOff(pin):
    GPIO.output(pin, GPIO.HIGH)


### Vibration Motor Functions
def motorOn(pin):
    GPIO.output(pin, GPIO.HIGH)

def motorOff(pin):
    GPIO.output(pin, GPIO.LOW)
    
def buzzer_thread(buzzer_pin):
    while True:
        for dist in distances:
            if dist < 2:
                print(dist < 2)
                buzzerOn(buzzer_pin)
                time.sleep(0.25)
                buzzerOff(buzzer_pin)
                time.sleep(0.1)

motor_status = [] # [Right, Middle, Left, Back] - Contains bool (if motor currently active)
def motor_thread(motor_pin, motor_number):
    min_distance = 1 # Minimum distance, below = Constant
    rate_of_change = 0.009 # Rate of change
    base_interval = 0.05  # The base speed at minimum distance
    max_distance = 4 # The maximum distance, above = silent
    
    if len(motor_status) < motor_number: # Set status of motor in motor_status to false if no value for it exists
        motor_status.append(0) 
    while True:
        motor_distance = distances[motor_number - 1]
        
        motor_status[motor_number - 1] = 0 # Set motor status to Off
        motorOff(motor_pin) # Toggle motor off
        if motor_distance <= min_distance:
            motor_interval = base_interval
        elif motor_distance >= max_distance:
            motor_interval = 0
            continue
        else:
            motor_interval = base_interval * math.exp(
                rate_of_change * (motor_distance - min_distance)
            )
            
        if motor_interval > 0:
            motor_status[motor_number - 1] = round(motor_interval, 2)
            motorOn(motor_pin) # Toggle motor on
            time.sleep(motor_toggle_duration) # Turns motor on for X
            motorOff(motor_pin)
                
        time.sleep(motor_interval)
                  
if __name__ == "__main__":
    try:
        setup()
        
        ### Define Distance Threads
        distance_threads = [] # Stores threads to be started later
        
        sensor_number = 0 # Iterator
        for TRIG, ECHO in ultrasonic_sensors:
            sensor_number += 1 # Passed to distance_thread to determine location & placement in 'distances'
            t = threading.Thread(target=distance_thread, args=(TRIG, ECHO, sensor_number)) # Define thread (not start)
            distance_threads.append(t) # Append thread to list

        ### Define Vibration Motor Threads
        vibration_motor_list = [motor_pin_1, motor_pin_2, motor_pin_3, motor_pin_4] # Used to iterate to create threads
        vibration_motor_threads = [] # Stores threads to be started later
        
        motor_number = 0 # Iterator
        for motor in vibration_motor_list:
            motor_number += 1 # Passed to 'motor_thread' to determine location & placement in 'motor_status'
            t = threading.Thread(target=motor_thread, args=(motor, motor_number)) # Define thread (not start)
            vibration_motor_threads.append(t) # Append thread to list
        
        ## Define Tilt Switch Thread
        tilt_thread = threading.Thread(target=tilt_thread, args=(tilt_pin,))
        
            
        # Start Distance Threads
        for t in distance_threads:
            t.start() # Start thread
            
        while len(distances) < len(ultrasonic_sensors):
            time.sleep(0.01) # Wait until all ultrasonic sensors have been activated
        
        # Start Vibration Motor Threads (checks distance determines output)
        for t in vibration_motor_threads:
            t.start() # Start thread
            
        while len(motor_status) < len(ultrasonic_sensors):
            time.sleep(0.01) # Wait until all vibration motors have been activated
        
        # Start Outputs
        buzzer_thread = threading.Thread(target = buzzer_thread, args=(buzzer_pin,))
        buzzer_thread.start()
        tilt_thread.start()
        
            
        while True:
            if toggle_print:
                print(f"DISTANCES   : RIGHT: {distances[0]} ft MIDDLE: {distances[1]} ft LEFT: {distances[2]} ft BACK: {distances[3]} ft")
                print(f"MOTOR STATUS: RIGHT: {motor_status[0]} MIDDLE: {motor_status[1]} LEFT: {motor_status[2]} BACK: {motor_status[3]}")
                print(f"TILT STATUS : {tilt_status} TOGGLE : {str(tilt_toggle)} LAST CHANGE : {last_tilt_seconds}")
                print("-----------------------------------------------------------------------------------------------------------------")
            time.sleep(distance_calculation_interval)
        
    except KeyboardInterrupt:  # Exit when Ctrl+C is pressed
        print("Cleaning up GPIO")
        GPIO.output(buzzer_pin, GPIO.HIGH)
        allOff()
        GPIO.cleanup()



