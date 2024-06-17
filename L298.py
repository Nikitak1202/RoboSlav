import pigpio
from time import sleep


# Class for Motor control
class Move():
    def __init__(self, ENA_right, IN1_right, IN2_right, ENB_right, IN3_right, IN4_right,
                 ENA_left, IN1_left, IN2_left, ENB_left, IN3_left, IN4_left):
        # Set pins
        # Front right
        self.PWM_FR = ENA_right
        self.IN1_FR = IN1_right
        self.IN2_FR = IN2_right
        # Back right
        self.PWM_BR = ENB_right
        self.IN1_BR = IN3_right
        self.IN2_BR = IN4_right
        # Back left
        self.PWM_BL = ENA_left
        self.IN1_BL = IN2_left
        self.IN2_BL = IN1_left
        # Front left
        self.PWM_FL = ENB_left
        self.IN1_FL = IN4_left
        self.IN2_FL = IN3_left

        # Declaring PWM pins
        pi.set_mode(self.PWM_FR, pigpio.ALT5)
        pi.set_mode(self.PWM_BL, pigpio.ALT5)
        # PWM range: 0-255
        self.pwmFR = pi.set_PWM_dutycycle(self.PWM_FR, 0)
        self.pwmBR = pi.set_PWM_dutycycle(self.PWM_BR, 0)
        self.pwmFL = pi.set_PWM_dutycycle(self.PWM_FL, 0)
        self.pwmBL = pi.set_PWM_dutycycle(self.PWM_BL, 0)

        # Declaring as outputs
        pi.set_mode(self.IN1_FR, pigpio.OUTPUT)
        pi.set_mode(self.IN2_FR, pigpio.OUTPUT)
        pi.set_mode(self.IN1_BR, pigpio.OUTPUT)
        pi.set_mode(self.IN2_BR, pigpio.OUTPUT)
        pi.set_mode(self.IN1_BL, pigpio.OUTPUT)
        pi.set_mode(self.IN2_BL, pigpio.OUTPUT)
        pi.set_mode(self.IN1_FL, pigpio.OUTPUT)
        pi.set_mode(self.IN2_FL, pigpio.OUTPUT)

    def forward(self, speed):
        # left side
        pi.write(self.IN1_FL, 1)
        pi.write(self.IN2_FL, 0)
        self.pwmFL = pi.set_PWM_dutycycle(self.PWM_FL, speed)

        pi.write(self.IN1_BL, 1)
        pi.write(self.IN2_BL, 0)
        self.pwmBL = pi.set_PWM_dutycycle(self.PWM_BL, speed)

        # right side
        pi.write(self.IN1_BR, 1)
        pi.write(self.IN2_BR, 0)
        self.pwmBR = pi.set_PWM_dutycycle(self.PWM_BR, speed)

        pi.write(self.IN1_FR, 1)
        pi.write(self.IN2_FR, 0)
        self.pwmFR = pi.set_PWM_dutycycle(self.PWM_FR, speed)

        '''
    def back(self, speed_left, speed_right):
        GPIO.output(self.IN1_right, GPIO.LOW)  
        GPIO.output(self.IN2_right, GPIO.HIGH) 
        self.pwmA.ChangeDutyCycle(speed_left)

        GPIO.output(self.IN3_right, GPIO.LOW)
        GPIO.output(self.IN4_right, GPIO.HIGH)
        self.pwmB.ChangeDutyCycle(speed_right)


    def left(self, speed_left, speed_right):
        GPIO.output(self.IN1_right, GPIO.HIGH)
        GPIO.output(self.IN2_right, GPIO.LOW)
        self.pwmA.ChangeDutyCycle(speed_left)

        GPIO.output(self.IN3_right, GPIO.LOW)
        GPIO.output(self.IN4_right, GPIO.HIGH)
        self.pwmB.ChangeDutyCycle(speed_right)


    def right(self, speed_left, speed_right):
        GPIO.output(self.IN1_right, GPIO.LOW)
        GPIO.output(self.IN2_right, GPIO.HIGH)
        self.pwmA.ChangeDutyCycle(speed_left)

        GPIO.output(self.IN3_right, GPIO.HIGH)
        GPIO.output(self.IN4_right, GPIO.LOW)
        self.pwmB.ChangeDutyCycle(speed_right)

    def stop(self):
        self.pwmA.ChangeDutyCycle(0)
        self.pwmB.ChangeDutyCycle(0)
        #sleep(time)
'''

# TEST
Motor = Move(19, 22, 27, 13, 17, 4, 18, 0, 11, 12, 9, 10) # GPIO numbers
# Pin's num: 35, 15, 13, 33, 11, 7, 12, 27, 23, 32, 21, 19

import curses
screen = curses.initscr()
curses.noecho()
curses.cbreak()
curses.halfdelay(3)
screen.keypad(True)

# Initialize pigpio
pi = pigpio.pi()
try: 
    while True:
        char = screen.getch()
        if char == ord('q'): break
        elif char == curses.KEY_UP:
            Motor.forward(26)
        '''
        elif char == curses.KEY_DOWN:
            Motor.back(50)
        elif char == curses.KEY_LEFT:
            Motor.left(100)
        elif char == curses.KEY_RIGHT:
            Motor.right(50)
        elif char == 0:
            Motor.stop()
        '''
finally:
    curses.nocbreak(); screen.keypad(0); curses.echo()
    curses.endwin()
    GPIO.cleanup()