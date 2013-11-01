#!/usr/bin/python

import RPi.GPIO as GPIO
import time

def switchIsPressed(channel):
	print "SWITCH PRESSED"
	time.sleep(5)

# GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
PIN_SWITCH = 23
GPIO.setup(PIN_SWITCH, GPIO.IN, pull_up_down=GPIO.PUD_UP)

GPIO.add_event_detect(PIN_SWITCH, GPIO.RISING, callback=switchIsPressed)

while True:
	print GPIO.input(PIN_SWITCH)
