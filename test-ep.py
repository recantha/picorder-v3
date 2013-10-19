#!/usr/bin/python

from __future__ import division
import RPi.GPIO as GPIO
import time

###############################################################
# General A2D read
# read SPI data from MCP3008 chip, 8 possible adc's (0 thru 7)
def readadc(adcnum, clockpin, mosipin, misopin, cspin):
        try:
                if ((adcnum > 7) or (adcnum < 0)):
                        return -1
                GPIO.output(cspin, True)
                GPIO.output(clockpin, False)  # start clock low
                GPIO.output(cspin, False)     # bring CS low

                commandout = adcnum
                commandout |= 0x18  # start bit + single-ended bit
                commandout <<= 3    # we only need to send 5 bits here
                for i in range(5):
                        if (commandout & 0x80):
                                GPIO.output(mosipin, True)
                        else:
                                GPIO.output(mosipin, False)
                        commandout <<= 1
                        GPIO.output(clockpin, True)
                        GPIO.output(clockpin, False)

                adcout = 0
                # read in one empty bit, one null bit and 10 ADC bits
                for i in range(12):
                        GPIO.output(clockpin, True)
                        GPIO.output(clockpin, False)
                        adcout <<= 1
                        if (GPIO.input(misopin)):
                                adcout |= 0x1

                GPIO.output(cspin, True)

                adcout /= 2      #first bit is 'null' so drop it

        except:
                adcout = 0

        return adcout

# GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Analog-to-digital converter
SPICLK = 11
SPIMISO = 9
SPIMOSI = 10
SPICS = 8

GPIO.setup(SPIMOSI, GPIO.OUT)
GPIO.setup(SPIMISO, GPIO.IN)
GPIO.setup(SPICLK, GPIO.OUT)
GPIO.setup(SPICS, GPIO.OUT)

PIN_EPULSE = 6

# Easy Pulse
def computeHeartrate(beats):
	intervals = []
	for i in range(1, len(beats)):
		intervals.append(beats[i]-beats[i-1])

	# Take off the top and bottom
	intervals.sort()
	intervals.pop(0)
	intervals.pop(len(intervals)-1)

	average_interval = sum(intervals) / len(intervals)
	heartrate = 60 / average_interval

	return heartrate

THRESH=1010
READING_INTERVAL = 0.1
beats = []
beats_to_capture = 5
pulse = False
pulse_timer = 0

while True:
	reading = readadc(PIN_EPULSE, SPICLK, SPIMOSI, SPIMISO, SPICS)

	for i in range(int(reading / 100)):
		print ".",

	if (reading > THRESH):
		if not pulse:
			pulse = True
			if len(beats) == beats_to_capture:
				print "Beat"
				print "Heartrate: " + str(computeHeartrate(beats))
				beats.pop(0)
			else:
				print "Beat"
			beats.append(pulse_timer)
		else:
			print ""
			pass
	else:
		pulse = False
		print ""

	time.sleep(READING_INTERVAL)
	pulse_timer = pulse_timer + READING_INTERVAL

