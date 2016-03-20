#! /usr/bin/python
"""
Created 3/22/16 by Greg Griffes based on the hackster.io rover pages at
https://www.hackster.io/peejster/rover-c42139 
"""

import time, os
import RPi.GPIO as GPIO     # GPIO is the handle to control pins
from random import randint, choice

class motor(object):

    def __init__(self, gpio_forward, gpio_backward):
        self.forward = gpio_forward
        self.backward = gpio_backward
      
        print 'forward GPIO # =  '+str(self.forward)
        print 'backward GPIO # =  '+str(self.backward)
        
        GPIO.output(self.forward, 0)
        GPIO.output(self.backward, 0)

# Done with initialization
                    
    def __del__(self):
        print 'motor '+str(self.forward)+' closed'

    def move_forward(self):
#        print 'motor '+str(self.forward)+' forward'
        GPIO.output(self.forward, 1)
        GPIO.output(self.backward, 0)

    def move_backward(self):
#        print 'motor '+str(self.forward)+' backward'
        GPIO.output(self.forward, 0)
        GPIO.output(self.backward, 1)

    def stop(self):
        print 'motor '+str(self.forward)+' stop'
        GPIO.output(self.forward, 0)
        GPIO.output(self.backward, 0)

class distance(object):

    def __init__(self, trigger, echo):
        self.trigger_gpio = trigger
        GPIO.output(self.trigger_gpio, 0)

        self.echo_gpio = echo
        self.distance = 1000    # in cm
      
    def get_distance(self):
        GPIO.output(self.trigger_gpio, 0)
        time.sleep(0.1)
        GPIO.output(self.trigger_gpio, 1)
        time.sleep(0.00001)
        GPIO.output(self.trigger_gpio, 0)

        while (GPIO.input(self.echo_gpio) == 0):
            pulse_start = time.time()
            
        while (GPIO.input(self.echo_gpio) == 1):
            pulse_end = time.time()

        pulse_duration = pulse_end - pulse_start

        self.distance = pulse_duration * 17150
        self.distance = round(self.distance, 2)

        print 'distance = '+str(self.distance)+' in cm'
        return self.distance


###############################################################
# Main program
###############################################################
if __name__ == '__main__':

    MP3_LOC = "/home/pi/Downloads/"
    MP3_FILES_STOP = "hasta", "a13prob", "alright2", "do_not", "failure", "school", "iceburg", "seeingthis", "whatinthename"
    MP3_FILES_GO = "bond_theme", "hatecomp", "letsgo", "nproblem", "whoohoo", "bbthm"

    TOO_CLOSE_IN_CM = 30    # stop and turn when this close to an object

    GPIO.setmode(GPIO.BCM)  # set GPIO to use BCM pin numbers
    GPIO.setwarnings(False) # warnings off

    right_motor_forward = 27    # GPIO pin
    right_motor_backward = 22    # GPIO pin
    left_motor_forward = 5    # GPIO pin
    left_motor_backward = 6    # GPIO pin

    GPIO.setup(right_motor_forward, GPIO.OUT)
    GPIO.setup(right_motor_backward, GPIO.OUT)
    GPIO.setup(left_motor_forward, GPIO.OUT)
    GPIO.setup(left_motor_backward, GPIO.OUT)

    right_motor = motor(right_motor_forward, right_motor_backward)
    left_motor = motor(left_motor_forward, left_motor_backward)

    ultra1_trigger_gpio_pin = 23    # GPIO pin
    ultra1_echo_gpio_pin = 24       # GPIO pin

    GPIO.setup(ultra1_trigger_gpio_pin, GPIO.OUT)
    GPIO.setup(ultra1_echo_gpio_pin, GPIO.IN)

    distance_sensor = distance(ultra1_trigger_gpio_pin, ultra1_echo_gpio_pin)

    print('*********************')
    print('      Running        ')
    print('*********************')
    os.system('mpg123 -q '+MP3_LOC+'online.mp3 &')

    try:
        # Main loop
        while True:

            if (distance_sensor.get_distance() >= TOO_CLOSE_IN_CM):

                right_motor.move_forward()
                left_motor.move_forward()
                time.sleep(0.1)

            else:

                right_motor.stop()
                left_motor.stop()
                MP3_FILE = MP3_LOC+choice(MP3_FILES_STOP)
                os.system('mpg123 -q '+MP3_FILE+'.mp3 &')
                time.sleep(2)
                right_motor.move_backward()
                left_motor.move_backward()
                time.sleep(0.5)

                if (randint(0,9) < 5):
                    right_motor.move_forward()
                    left_motor.move_backward()
                    time.sleep(0.5)
                else:
                    right_motor.move_backward()
                    left_motor.move_forward()
                    time.sleep(0.5)
                
                time.sleep(2)
                MP3_FILE = MP3_LOC+choice(MP3_FILES_GO)
                os.system('mpg123 -q '+MP3_FILE+'.mp3 &')


    except KeyboardInterrupt:
        right_motor.stop()
        left_motor.stop()

    except:
        right_motor.stop()
        left_motor.stop()
        GPIO.cleanup()



