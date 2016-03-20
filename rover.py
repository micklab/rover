#! /usr/bin/python
"""
Created 3/22/16 by Greg Griffes based on the hackster.io rover pages at
https://www.hackster.io/peejster/rover-c42139 
"""

import time
import RPi.GPIO as GPIO     # GPIO is the handle to control pins

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
        print 'motor '+str(self.forward)+' forward'
        GPIO.output(self.forward, 1)
        GPIO.output(self.backward, 0)

    def move_backward(self):
        print 'motor '+str(self.forward)+' backward'
        GPIO.output(self.forward, 0)
        GPIO.output(self.backward, 1)

    def stop(self):
        print 'motor '+str(self.forward)+' stop'
        GPIO.output(self.forward, 0)
        GPIO.output(self.backward, 0)


###############################################################
# Main program
###############################################################
if __name__ == '__main__':

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

    print('*********************')
    print('      Test Code      ')
    print('*********************')

    # Main loop
    while True:

        right_motor.move_forward()
        time.sleep(3)

        right_motor.move_backward()
        time.sleep(3)

        right_motor.stop()
        time.sleep(3)

        left_motor.move_forward()
        time.sleep(3)

        left_motor.move_backward()
        time.sleep(3)

        left_motor.stop()
        time.sleep(3)

