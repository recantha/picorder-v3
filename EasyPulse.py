#!/usr/bin/python

import RPi.GPIO as GPIO
import time

class EasyPulse:
	def __init__(self, debug=0, pin_epulse=6):
		self.DEBUG = debug

		# GPIO
		GPIO.setmode(GPIO.BCM)
		GPIO.setwarnings(False)

		# Analog-to-digital converter
		self.SPICLK = 11
		self.SPIMISO = 9
		self.SPIMOSI = 10
		self.SPICS = 8

		GPIO.setup(self.SPIMOSI, GPIO.OUT)
		GPIO.setup(self.SPIMISO, GPIO.IN)
		GPIO.setup(self.SPICLK, GPIO.OUT)
		GPIO.setup(self.SPICS, GPIO.OUT)

		# MCP3008 pin
		self.PIN_EPULSE = pin_epulse

	###############################################################
	# General A2D read
	# read SPI data from MCP3008 chip, 8 possible adc's (0 thru 7)
	def readadc(self, adcnum, clockpin, mosipin, misopin, cspin):
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

	def computeHeartrate(self, beats):
		intervals = []
		for i in range(1, len(beats)):
			intervals.append(beats[i]-beats[i-1])

		if len(intervals) < 3:
			heartrate = -1
		else:
			# Take off the top and bottom
			intervals.sort()
			intervals.pop(0)
			intervals.pop(len(intervals)-1)

			average_interval = sum(intervals) / len(intervals)
			heartrate = round(60 / average_interval, 2)

		return heartrate

	def readPulse(self):
		THRESHOLD=900
		READING_INTERVAL = 0.1
		READ_FOR_SECONDS = 4
		CURRENT_READING_TIME = 0
		beats = []
		pulse = False

		if self.DEBUG:
			print "Total reading time will be " + str(READ_FOR_SECONDS) + "s"
	
		while CURRENT_READING_TIME < READ_FOR_SECONDS:
			reading = self.readadc(self.PIN_EPULSE, self.SPICLK, self.SPIMOSI, self.SPIMISO, self.SPICS)

			for i in range(int(reading / 100)):
				if self.DEBUG:
					print ".",

			if (reading > THRESHOLD):
				if not pulse:
					pulse = True
					if self.DEBUG:
						print "Beat"
					beats.append(CURRENT_READING_TIME)
				else:
					if self.DEBUG:
						print ""
			else:
				pulse = False
				if self.DEBUG:
					print ""

			time.sleep(READING_INTERVAL)
			CURRENT_READING_TIME = CURRENT_READING_TIME + READING_INTERVAL
			if self.DEBUG:
				print "Current reading time is " + str(CURRENT_READING_TIME)

		return beats

if __name__ == '__main__':
	easypulse = EasyPulse()
	beats = easypulse.readPulse()
	print "Number of beats: " + str(len(beats))
	heartrate = easypulse.computeHeartrate(beats)
	print "Heartrate: " + str(heartrate)
