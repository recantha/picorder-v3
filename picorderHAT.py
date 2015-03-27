#!/usr/bin/python

from __future__ import division
import RPi.GPIO as GPIO
#from PyComms import hmc5883l
#from PyComms import mpu6050
#import TMP102
import lcddriver
import math
import commands
import os
import time
import threading
import sys
import termios
import tty
import twitter
import RTIMU
import random
from smbus import SMBus
from datetime import datetime
from gps import *
from Adafruit_BMP085 import BMP085
from EasyPulse import EasyPulse
from Adafruit_7Segment import SevenSegment
from Adafruit_8x8 import EightByEight
from Adafruit_Bargraph import Bargraph

# This doesn't work. It conflicts with something that is being imported
# If it's a choice between sensors and sound, I choose sound any day
#import pygame

#pygame.init()
#pygame.mixer.init()
#scan = pygame.mixer.Sound("sounds/tricorder.wav")
#scan.set_volume(1.0)

#scan.play(-1)
#
#while True:
#        print "Playing"

#exit(0)





LCD_ENABLED = True

###############################################################
# LCD
def display(line_1, line_2="                    ", line_3="                    ", line_4="                    "):
	line_1 = line_1.ljust(20, " ")
	line_2 = line_2.ljust(20, " ")
	line_3 = line_3.ljust(20, " ")
	line_4 = line_4.ljust(20, " ")

	if LCD_ENABLED:
		lcd.lcd_display_string(line_1, 1)
		lcd.lcd_display_string(line_2, 2)
		lcd.lcd_display_string(line_3, 3)
		lcd.lcd_display_string(line_4, 4)

	if DEBUG == 1:
		print "--------------------"
		print line_1
		print line_2
		print line_3
		print line_4
		print "--------------------"


###############################################################
# SESSION
def currentSession():
        ts = time.time()
        st = datetime.fromtimestamp(ts).strftime('%Y%m%d%H%M%S')

        return st

def readHostname():
        local_hostname=socket.gethostname()
        return local_hostname

def readIPaddresses():
        ips = commands.getoutput("/sbin/ifconfig | grep -i \"inet\" | grep -iv \"inet6\" | " + "awk {'print $2'} | sed -ne 's/addr\:/ /p'")
        addrs = ips.split('\n')

        return addrs


###############################################################
# Clock
def sevenSegmentClock():
	while True:
		now = datetime.now()
		hour = now.hour
		minute = now.minute
		second = now.second
		# Set hours
		segment.writeDigit(0, int(hour / 10))	 # Tens
		segment.writeDigit(1, hour % 10)         # Ones
		# Set minutes
		segment.writeDigit(3, int(minute / 10))  # Tens
		segment.writeDigit(4, minute % 10)       # Ones
		# Toggle colon
		segment.setColon(second % 2)             # Toggle colon at 1Hz
		# Wait one second
		time.sleep(1)

		if thread_keep_alive == 0:
			print "Clock shutdown"
			break

################################################################
# Matrix
def eightByEightDemo():
	# Continually update the 8x8 display one pixel at a time
	while(True):
		for x in range(0, 8):
			for y in range(0, 8):
				grid.setPixel(x, y)
				time.sleep(0.05)
		time.sleep(0.5)
		grid.clear()
		time.sleep(0.5)

		if thread_keep_alive == 0:
			print "Matrix shutdown"
			break


def eightByEightHeartbeat():
	while True:
		for x in range(0,8):
			for y in range(0,8):
				grid.setPixel(x,y)
		time.sleep(1)
		grid.clear()

		if eightByEightHeartbeatAlive == 0:
			print "Heartbeat shutdown"
			break

def matrix(mx):
	x = 0
	for line in mx:
		y=0
		for cell in line:
			#print Z[x][y],
			if cell == 1:
				grid.setPixel(x,y)
			else:
				grid.clearPixel(x,y)
			y=y+1
		x=x+1
		#print ""

###############################################################
# Strip
def bargraphDemo():
	while(True):
		for color in range(1, 4):
			for i in range(24):
				bargraph.setLed(i, color)
				time.sleep(0.02)

		if thread_keep_alive == 0:
			print "Bargraph shutdown"
			break


###############################################################
# Game of Life
# Code originally taken from http://dana.loria.fr/doc/game-of-life.html
def GOLiterate(Z):
	previous = Z

	try:
		shape = len(Z), len(Z[0])
		N  = [[0,]*(shape[0]+2)  for i in range(shape[1]+2)]
		# Compute number of neighbours for each cell
		for x in range(1,shape[0]-1):
			for y in range(1,shape[1]-1):
				N[x][y] = Z[x-1][y-1]+Z[x][y-1]+Z[x+1][y-1] \
					+ Z[x-1][y]+Z[x+1][y]   \
					+ Z[x-1][y+1]+Z[x][y+1]+Z[x+1][y+1]
		# Update cells
		for x in range(1,shape[0]-1):
			for y in range(1,shape[1]-1):
				if Z[x][y] == 0 and N[x][y] == 3:
					Z[x][y] = 1
				elif Z[x][y] == 1 and not N[x][y] in [2,3]:
					Z[x][y] = 0
	except:
		Z = previous

def GOLdisplay(Z):
	shape = len(Z), len(Z[0])
	for x in range(1,shape[0]-1):
		for y in range(1,shape[1]-1):
			print Z[x][y],
		print
	print

def GOLgenerate(width, height):
	build = []
	for x in range (0,width+2):
		build.append([])
		for y in range(0,height+2):
			build[x].append(random.randint(0,1))
	return build

Z = GOLgenerate(8,8)
def gameOfLife():
	generation = 0
	while True:
		generation = generation + 1
		if generation == 10:
			Z = GOLgenerate(8,8)
			generation = 0
		#os.system('clear')
		#display(Z)
		try:
			matrix(Z)
			GOLiterate(Z)
		except:
			pass
		time.sleep(1)


###############################################################
# HMC reading
def readHMC5883L():
        try:
                data = hmc.getHeading()
                reading = {}
                reading['yaw'] = "%.0f" % data['z']
                reading['pitch'] = "%.0f" % data['y']
                reading['roll'] = "%.0f" % data['x']

        except:
                reading = {}
                reading['yaw'] = -1
                reading['pitch'] = -1
                reading['roll'] = -1

        return reading

###############################################################
# MPU reading
def readMPU6050():
        # Sensor initialization

        # get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize()

        try:
                while True:
                        # Get INT_STATUS byte
                        mpuIntStatus = mpu.getIntStatus()

                        if mpuIntStatus >= 2: # check for DMP data ready interrupt (this should happen frequently)
                                # get current FIFO count
                                fifoCount = mpu.getFIFOCount()

                                # check for overflow (this should never happen unless our code is too inefficient)
                                if fifoCount == 1024:
                                        # reset so we can continue cleanly
                                        mpu.resetFIFO()
                                        #print('FIFO overflow!')


                                # wait for correct available data length, should be a VERY short wait
                                fifoCount = mpu.getFIFOCount()
                                while fifoCount < packetSize:
                                        fifoCount = mpu.getFIFOCount()

                                result = mpu.getFIFOBytes(packetSize)
                                q = mpu.dmpGetQuaternion(result)
                                g = mpu.dmpGetGravity(q)
                                ypr = mpu.dmpGetYawPitchRoll(q, g)

                                reading = {}
                                reading['yaw'] = "%.2f" % (ypr['yaw'] * 180 / math.pi)
                                reading['pitch'] = "%.2f" % (ypr['pitch'] * 180 / math.pi)
                                reading['roll'] = "%.2f" % (ypr['roll'] * 180 / math.pi)

                                # track FIFO count here in case there is > 1 packet available
                                # (this lets us immediately read more without waiting for an interrupt)
                                fifoCount -= packetSize
                                # break when you have a reading
                                return reading
        except:
                reading = {}
                reading['yaw'] = -1
                reading['pitch'] = -1
                reading['roll'] = -1

                return reading


###############################################################
# GPS
class GpsPoller(threading.Thread):
        def __init__(self):
                threading.Thread.__init__(self)
                global gpsd #bring it in scope
                gpsd = gps(mode=WATCH_ENABLE) #starting the stream of info
                self.current_value = None
                self.running = True #setting the thread running to true

        def run(self):
                global gpsd
                while gpsp.running:
                        gpsd.next() #this will continue to loop and grab EACH set of gpsd info to clear the buffer


###########################################################
def readBarometer():
	try:
		temp = bmp.readTemperature()
		pressure = bmp.readPressure()
		altitude = bmp.readAltitude()

		str_temp = "Temperature:%.2f C" % temp
		str_pressure = "Pressure:%.2f hPa" % (pressure/100.0)
		str_altitude = "Altitude:%.2f m" % altitude

		reading = {}
		reading['temperature'] = str_temp
		reading['pressure'] = str_pressure
		reading['altitude'] = str_altitude

	except:
		pass

	return reading






###########################################################
def readCoordinates():
        try:
                lat = gpsd.fix.latitude
                lon = gpsd.fix.longitude
                speed = gpsd.fix.speed
                utc = gpsd.utc
                alt = gpsd.fix.altitude

                if (math.isnan(lat)):
                        lat = "No fix"

                if (math.isnan(lon)):
                        lon = "No fix"

                if (math.isnan(speed)):
                        speed = "No fix"
                else:
                        speed = "%s m/s" % speed

                if (utc):
                        pass
                else:
                        utc = "No fix"

                if (math.isnan(alt)):
                        alt = "No fix"
                else:
                        alt = "%s metres" % alt

                sats = gpsd.satellites

                coords = [lat, lon, utc, alt, speed, sats]

        except (KeyboardInterrupt, SystemExit):
                pass

        return coords




###############################################################
# ULTRASONIC
def readUltrasonic():
        try:
                number_of_readings=10
                number_of_samples=10
                ping_timeout=200000
                debug = False

                # Timing constants
                us_trigger_pulse = 0.00001
                us_read_sleep = 0.00002

                all_readings = []
                for j in range(number_of_readings):

                        reading_list = []
                        readings_used = 0
                        for i in range(number_of_samples):
                                # 50 ms is the max timeout if nothing in range.
                                # time.sleep(0.005)
                                timeout_flag = False

                                # set our trigger high, triggering a pulse to be sent.
                                GPIO.output(US_PIN_TRIGGER, GPIO.HIGH)
                                time.sleep(us_trigger_pulse)
                                GPIO.output(US_PIN_TRIGGER, GPIO.LOW)

                                timeout_start = datetime.now()

                                # Wait for our pin to go high, waiting for a response.
                                while not GPIO.input(US_PIN_ECHO):
                                        timeout_end = datetime.now()
                                        timeout_delta = timeout_end - timeout_start
                                        if timeout_delta.microseconds > ping_timeout:
                                                if debug:
                                                        print "Timeout A"
                                                timeout_flag = True
                                                break
                                        pass

                                # Now its high, get our start time
                                timeout_start = datetime.now()
                                start = datetime.now()

                                # wait for our input to go low
                                while GPIO.input(US_PIN_ECHO):
                                        timeout_end = datetime.now()
                                        timeout_delta = timeout_end - timeout_start
                                        if timeout_delta.microseconds > ping_timeout:
                                                if debug:
                                                        print "Timeout B"
                                                timeout_flag = True
                                                break
                                        pass

                                # Now its low, grab our end time
                                end = datetime.now()

                                # Store our delta.
                                if not timeout_flag:
                                        delta = end - start
                                        reading_list.append(delta.microseconds)
                                        readings_used = readings_used + 1

                                        if debug:
                                                print "Microseconds %1.f" % delta.microseconds

                                # take a little break, it appears to help stabalise readings
                                # I suspect due to less interference with previous readings
                                time.sleep(us_read_sleep)

                        average_reading = sum(reading_list)/len(reading_list)

                        all_readings.append(average_reading)

                average_of_all_readings = sum(all_readings)/len(all_readings)
                average_distance=average_of_all_readings * 340
                average_distance=average_distance/20000
                return_text = "%s cm" % average_distance

        except Exception as err:
		print type(err)
                return_text = "No reading"

        return return_text


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

###############################################################
# Convert A2D readings
def readAnalogSensor(PIN):
	try:
		reading = readadc(PIN, SPICLK, SPIMOSI, SPIMISO, SPICS)
		perc_of_max = round((reading/1023)*100, 2)
		percentage = str(perc_of_max) + "%"

		number_of_bars = (reading/1023)*16
		number_of_bars = int(math.ceil(number_of_bars))
		bars = "#" * number_of_bars

	except:
		percentage = "No reading"
		bars = ""

	return [percentage, bars]


###############################################################
# MICROPHONE/SOUND LEVEL
def readSoundLevel():
	return readAnalogSensor(PIN_MICR)


###############################################################
# Read Galvanic Skin Response sensor
def readGSR():
	return readAnalogSensor(PIN_GSR)


###############################################################
# Read MQ7
def readMQ7():
	return readAnalogSensor(PIN_MQ7)

###############################################################
# Read MQ2
def readMQ2():
	return readAnalogSensor(PIN_MQ2)

###############################################################
# Read MQ3
def readMQ3():
	return readAnalogSensor(PIN_MQ3)

###############################################################
# Read Moisture
def readMoisture():
	return readAnalogSensor(PIN_MOISTURE)

###############################################################
# Read system temperatures
def readSystemTemperatures():
	try:
		reading = commands.getoutput("./getSystemTemperature.sh")
		readings = reading.splitlines();

	except:
		readings = [-1, -1]

	return readings

###############################################################
# Read IMU board
def readIMU():
	imu_readings = {}
	imu_readings['roll'] = "%.0f" % -1
	imu_readings['pitch'] = "%.0f" % -1
	imu_readings['yaw'] = "%.0f" % -1
	if imu.IMURead():
	# x, y, z = imu.getFusionData()
	# print("%f %f %f" % (x,y,z))
		data = imu.getIMUData()
		fusionPose = data["fusionPose"]
		imu_readings['roll'] = "%.0f" % math.degrees(fusionPose[0])
		imu_readings['pitch'] = "%.0f" % math.degrees(fusionPose[1])
		imu_readings['yaw'] = "%.0f" % math.degrees(fusionPose[2])
		time.sleep(imu_poll_interval*1.0/1000.0)

	return imu_readings

###############################################################
###############################################################
# SYS
stop_counter = 0

def iterateOperation():
	global operation

	operation = operation + 1
	display("################", "Next operation", "", "################")

	if DEBUG:
		print "Next operation (" + str(operation) + ")"

def getKey():
	fd = sys.stdin.fileno()
	old = termios.tcgetattr(fd)
	new = termios.tcgetattr(fd)
	new[3] = new[3] & ~termios.ICANON & ~termios.ECHO
	new[6][termios.VMIN] = 1
	new[6][termios.VTIME] = 0
	termios.tcsetattr(fd, termios.TCSANOW, new)
	key = None
	try:
		key = os.read(fd, 3)
	finally:
		termios.tcsetattr(fd, termios.TCSAFLUSH, old)
	return key

key_press = "^"
def readKey():
	global key_press
	lock = threading.Lock()
	while True:
		with lock:
			key_press = str(getKey())
			if key_press != '^':
				iterateOperation()

###############################################################
# TWITTER
def sendTweet(message):
	try:
		twitter_api.PostUpdate(message)
	except:
		print "Failed to send tweet"
		pass

def readLastTweet():
	try:
		twitter_statuses = twitter_api.GetUserTimeline('picorder')
		last_status = twitter_statuses[0].text
	except:
		last_status = "Check connection"

	return last_status

###############################################################
def playTricorderSound():
	sound = pygame.mixer.Sound("sounds/tricorder.wav")
	sound.play(loops=-1)

###############################################################
# INIT SECTION
DEBUG=1

# TWITTER SET-UP
CONSUMER_KEY = '4dg3D0FRULTUMCWyUytmsg'
CONSUMER_SECRET = 'yqgk7hqpBxkQwJzvtUnK2E0rDPLqI5JSuv54qsA8'
ACCESS_KEY = '1911720504-8audXV5cs0Wt2cmFaKmAyPbytSc3vFnonpOd9Tg'
ACCESS_SECRET = 'T1yNOP1ZAeI0MGriTB8ERBDYDhrfJxorRZeqUWV3q10'
try:
	twitter_api = twitter.Api(consumer_key=CONSUMER_KEY, consumer_secret=CONSUMER_SECRET, access_token_key=ACCESS_KEY, access_token_secret=ACCESS_SECRET)
except:
	pass

###############################################################
picorder_version_no = "1 HAT"
print "Picorder version " + picorder_version_no
print "Michael Horne - March 2015"
print "Using RPi.GPIO version " + GPIO.VERSION

time_stamp = time.time()
session_id = currentSession()

sendTweet("Picorder Activated at " + str(time_stamp))

# GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# LCD
try:
	lcd = lcddriver.lcd()
except:
	print "LCD failed to initialise"

# 7-segment
try:
	segment = SevenSegment(address=0x70)
except:
	print "Seven segment failed to initialise"

# Matrix
try:
	grid = EightByEight(address=0x72)
except:
	print "Matrix failed to initialize"

# Strip
try:
	bargraph = Bargraph(address=0x71)
except:
	print "Bargraph failed to initialize"


# ACCEL NUMBER 1
#try:
#	hmc = hmc5883l.HMC5883L()
#
#except:
#	print "HMC failed to initialise"

# ACCEL NUMBER 2
#try:
#	mpu = mpu6050.MPU6050()
#	mpu.dmpInitialize()
#	mpu.setDMPEnabled(True)

#except:
#	print "MPU failed to initialise"

#######################################################
# BerryIMU
SETTINGS_FILE = "RTIMULib"
#print("Setting up RTIMULib using settings file " + SETTINGS_FILE + ".ini")
if not os.path.exists(SETTINGS_FILE + ".ini"):
	print("RTIMULib settings file does not exist, will be created")

s = RTIMU.Settings(SETTINGS_FILE)
imu = RTIMU.RTIMU(s)
imu_poll_interval = imu.IMUGetPollInterval()

print("IMU Name: " + imu.IMUName())

if (not imu.IMUInit()):
	print("IMU Init Failed");
	sys.exit(1)
else:
	print("IMU Init Succeeded");

#######################################################
# Barometer BMP085
bmp = BMP085(0x77, 0)

#######################################################
# Ultrasonic
US_PIN_TRIGGER=20
US_PIN_ECHO=19
GPIO.setup(US_PIN_TRIGGER, GPIO.OUT)
GPIO.setup(US_PIN_ECHO, GPIO.IN)

#######################################################
# Analog-to-digital converter
SPICLK = 11
SPIMISO = 9
SPIMOSI = 10
SPICS = 8

GPIO.setup(SPIMOSI, GPIO.OUT)
GPIO.setup(SPIMISO, GPIO.IN)
GPIO.setup(SPICLK, GPIO.OUT)
GPIO.setup(SPICS, GPIO.OUT)

# Analog sensors
PIN_MOISTURE = 0
PIN_MQ7 = 1 # Carbon monoxide sensor
PIN_MQ3 = 2 # Alcohol sensor
PIN_MQ2 = 3

PIN_MICR = 4
PIN_GSR = 5
PIN_EPULSE = 6

#######################################################
# I2C bus
bus = SMBus(1)

#######################################################
# Button
PIN_SWITCH = 5
GPIO.setup(PIN_SWITCH, GPIO.IN, pull_up_down=GPIO.PUD_UP)

#easypulse = EasyPulse()


###############################################################
# MAIN

if __name__ == "__main__":
	#############################################################
	# Start-up routine

	eightByEightHeartbeatAlive = 1
	threading.Thread(target = eightByEightHeartbeat).start()
	
	display("####################", "Picorder v" + picorder_version_no, "Starting up...", "####################")
	time.sleep(0.2)
	
	display("####################", "Picorder v" + picorder_version_no, session_id, "####################")
	time.sleep(0.2)
	
	# GPS
	ENABLE_GPS = True
	if ENABLE_GPS:
		display("####################", "Picorder v" + picorder_version_no, "Initialising GPS", "####################")
		os.system("./enable_gps.sh")
		time.sleep(3)
		display("####################", "Picorder v" + picorder_version_no, "GPS should be up", "####################")
		gpsd = None
		gpsp = GpsPoller()
		gpsp.start()

	# Kill heartbeat
	eightByEightHeartbeatAlive = 0

	# Start independent processes
	thread_keep_alive = 1
	threading.Thread(target = sevenSegmentClock).start()
	#threading.Thread(target = eightByEightDemo).start()
	threading.Thread(target = gameOfLife).start()
	threading.Thread(target = bargraphDemo).start()

	operation = 0

	# A key press also advances the operation
	#threading.Thread(target = readKey).start()

	SHUTDOWN_FLAG = 0
	while True:
		print "Current operation: " + str(operation)

		try:
			if operation == -1:
				SHUTDOWN_FLAG = 1
				thread_keep_alive = 0
				break

			if operation == 0:
				hostname = readHostname()
				for addr in readIPaddresses():
					display("Hostname:", hostname, "IP addresses", addr)
					time.sleep(1)

			elif operation == 1:
				now = datetime.now()
				curDate = now.strftime("%d %B %Y")
				curTime = now.strftime("%H:%M:%S")
				display("Current date is:", curDate, "Current time is:", curTime)
				time.sleep(1)

			elif operation == 2:
				coords = readCoordinates()
				display("Latitude %s" % coords[0], "Longitude %s" % coords[1], "Altitude %s" % coords[3], "Speed %s" % coords[4])
				time.sleep(0.5)

			elif operation == 3:
				mq7 = readMQ7()
				display("MQ7 sensor", "Carbon monoxide", mq7[0], mq7[1])
				time.sleep(0.5)

			elif operation == 4:
				imu_readings = readIMU()
				display("IMU", "Roll: " + imu_readings['roll'], "Pitch: " + imu_readings['pitch'], "Yaw: " + imu_readings['yaw'])
				#time.sleep(0.5)

			elif operation == 4:
				display("Operation 4", "Not enabled", "", "")
				#ypr = readMPU6050()
				#display("MPU accelerometer", "Yaw: " + ypr['yaw'], "Pitch: " + ypr['pitch'], "Roll: " + ypr['roll'])
				#time.sleep(0.05)

			elif operation == 5:
				tpa = readBarometer()
				display("Barometer", tpa['temperature'], tpa['pressure'], tpa['altitude'])
				time.sleep(0.5)

			elif operation == 6:
				ultrasonic = readUltrasonic()
				display("Ultrasonic", "Distance", "Measurement", ultrasonic)
				time.sleep(0.05)

			elif operation == 7:
				display("Operation 7", "Not enabled", "", "")
				#reading = readSoundLevel()
				#display("Microphone", "Sound level", reading[0], reading[1])
				#time.sleep(0.5)

			elif operation == 8:
				display("Operation 8", "Not enabled", "", "")
				#reading = readGSR()
				#display("Galvanic Skin Resp.", reading[0], reading[1], "")
				#time.sleep(0.05)

			elif operation == 9:
				mq2 = readMQ2()
				display("MQ2 sensor", "Combustible gas", mq2[0], mq2[1])
				time.sleep(0.5)

			elif operation == 10:
				reading = readMoisture()
				display("Moisture", reading[0], reading[1], "")
				time.sleep(0.5)

			elif operation == 11:
				readings = readSystemTemperatures()
				display("System temperatures", readings[0], readings[1], "")
				time.sleep(1)

			elif operation == 12:
				display("Operation 12", "Not enabled", "", "")
				#beats = easypulse.readPulse()
				#heartrate = easypulse.computeHeartrate(beats)
				#display("Heartbeat/Pulse", "Beats: " + str(len(beats)), "Heartrate: " + str(heartrate), "")

			elif operation == 13:
				mq3 = readMQ3()
				display("MQ3 sensor", "Alcohol sensor", mq3[0], mq3[1])
				time.sleep(0.5)

			#elif operation == 13:
			#	reading = readLastTweet()
			#	display("Last tweet to @picorder", reading, "", "")
			#	time.sleep(5)
			#	iterateOperation()

			else:
				operation = 0

			if GPIO.input(PIN_SWITCH) == 0:
				#################################################################################
				# Detect physical switch presses and iterate the operation
				iterateOperation()
				time.sleep(0.5)

				start = time.time()
				while not GPIO.input(PIN_SWITCH):
					end = time.time()
					duration = end - start

					if duration > 5:
						display("################", "Kill switch detected", "", "################")
						operation = -1
						break


		except KeyboardInterrupt:
			gpsp.running = False
			gpsp.join()
			GPIO.cleanup()
			raise

		except Exception as exc:
			print exc
			print "Error"
			exit(0)
			raise

if SHUTDOWN_FLAG == 1:
	display("################", "System shutdown", "", "################")
	os.system("sudo halt")
	display("################", "Halt initiated", "", "################")
	sys.exit(1)

