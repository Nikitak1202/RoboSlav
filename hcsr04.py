#1 sensor
import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

TRIG = 23
ECHO = 24
print('Distance measurement in progress')

GPIO.setup(TRIG,GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)
GPIO.output(TRIG,False)
print('Waiting for sensor to settle')
time.sleep(2)

GPIO.output(TRIG, True)
time.sleep(0.00001)
GPIO.output(TRIG,False)

while GPIO.input(ECHO)==0:
    pulse_start = time.time()

while GPIO.input(ECHO)==1:
    pulse_end = time.time()
    
pulse_duration = pulse_end - pulse_start
distance = pulse_duration * 17150
distance = round(distance,2)
print('Distance: ', distance, 'cm')

#4 sensor
import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

TRIG = 23
ECHO = 12

TRIG1 = 24
ECHO1 = 16

TRIG2 = 25
ECHO2 = 20

TRIG3 = 08
ECHO3 = 21

print('Distance measurement in progress')

GPIO.setup(TRIG,GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

GPIO.setup(TRIG1,GPIO.OUT)
GPIO.setup(ECHO1, GPIO.IN)

GPIO.setup(TRIG2,GPIO.OUT)
GPIO.setup(ECHO2, GPIO.IN)

GPIO.setup(TRIG3,GPIO.OUT)
GPIO.setup(ECHO3, GPIO.IN)

GPIO.output(TRIG,False)
GPIO.output(TRIG1,False)
GPIO.output(TRIG2,False)
GPIO.output(TRIG3,False)

print('Waiting for sensor 1 to send a signal')
time.sleep(2)

GPIO.output(TRIG, True)
time.sleep(0.00001)
GPIO.output(TRIG,False)

while GPIO.input(ECHO)==0:
    pulse_start = time.time()

while GPIO.input(ECHO)==1:
    pulse_end = time.time()
    
pulse_duration = pulse_end - pulse_start
distance = pulse_duration * 17150
distance = round(distance,2)
print('Distance: ', distance, 'cm')
print('Waiting for sensor 2 to send signal')
time.sleep(2)

GPIO.output(TRIG1, True)
time.sleep(0.00001)
GPIO.output(TRIG1,False)

while GPIO.input(ECHO1)==0:
    pulse_start = time.time()

while GPIO.input(ECHO1)==1:
    pulse_end = time.time()
    
pulse_duration = pulse_end - pulse_start
distance = pulse_duration * 17150
distance = round(distance,2)
print('Distance: ', distance, 'cm')
print('Waiting for sensor 3 to send signal')
time.sleep(2)

GPIO.output(TRIG2, True)
time.sleep(0.00001)
GPIO.output(TRIG2,False)

while GPIO.input(ECHO2)==0:
    pulse_start = time.time()

while GPIO.input(ECHO2)==1:
    pulse_end = time.time()
    
pulse_duration = pulse_end - pulse_start
distance = pulse_duration * 17150
distance = round(distance,2)
print('Distance: ', distance, 'cm')
print('Waiting for sensor 3 to send signal')
time.sleep(2)

GPIO.output(TRIG3, True)
time.sleep(0.00001)
GPIO.output(TRIG3,False)

while GPIO.input(ECHO3)==0:
    pulse_start = time.time()

while GPIO.input(ECHO3)==1:
    pulse_end = time.time()
    
pulse_duration = pulse_end - pulse_start
distance = pulse_duration * 17150
distance = round(distance,2)
print('Distance: ', distance, 'cm')
