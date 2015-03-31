#!/usr/bin/python

from __future__ import division
import RPi.GPIO as GPIO
import lcddriver
import math
import commands
import os
import time
import threading
import sys
import termios
import tty
import RTIMU
import random
import tweepy
from smbus import SMBus
from datetime import datetime
from gps import *
from Adafruit_BMP085 import BMP085
from Adafruit_7Segment import SevenSegment
from Adafruit_8x8 import EightByEight
from Adafruit_Bargraph import Bargraph
from BerryIMU import BerryIMU
import xively

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
# Barometer BMP085
bmp = BMP085(0x77, 0)
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
		reading['raw_temperature'] = temp
		reading['raw_pressure'] = pressure
		reading['raw_altitude'] = altitude

	except Exception as err:
		print err
		reading['temperature'] = -1
		reading['pressure'] = -1
		reading['altitude'] = -1

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
# Using RTIMULib
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
		twitter_api.update_status(status=message)

        except Exception as err:
		print "Failed to send tweet"
		print err

def readLastTweet():
	try:
		tweets = twitter_api.mentions_timeline()
		last_tweet = tweets[0].text

	except Exception as err:
		print err
		last_tweet = "Unable to read tweet - " + str(err.message)

	return last_tweet

###############################################################
def playTricorderSound():
	sound = pygame.mixer.Sound("sounds/tricorder.wav")
	sound.play(loops=-1)

###############################################################
# Data streaming
FEED_ID = "482486897"
API_KEY = "nBBS1N1dIYIzHVAj7DuXjJrReUGK9xBdCWfZBXkgeVYTRsno"
xively = xively.XivelyAPIClient(API_KEY)

def get_datastream(feed, stream_name):
	try:
		datastream = feed.datastreams.get(stream_name)
		#print "Found existing datastream"

	except:
		datastream = feed.datastreams.create(stream_name, tags=stream_name + "_01")
		#print "Creating new datastream"
 
	return datastream

def streamReadings():
	print "Streaming readings to Xively feed " + FEED_ID
	feed = xively.feeds.get(FEED_ID)

	datastreams = {}
	datastreams["temperature"] = get_datastream(feed, "Temperature")
	datastreams["temperature"].min_value = None
	datastreams["temperature"].max_value = None
	datastreams["altitude"] = get_datastream(feed, "Altitude")
	datastreams["altitude"].min_value = None
	datastreams["altitude"].max_value = None
	datastreams["pressure"] = get_datastream(feed, "Pressure")
	datastreams["pressure"].min_value = None
	datastreams["pressure"].max_value = None

	while True:
		tpa = readBarometer()
		datastreams["temperature"].current_value = tpa["raw_temperature"]
		datastreams["temperature"].at = datetime.utcnow()
		datastreams["altitude"].current_value = tpa["raw_altitude"]
		datastreams["altitude"].at = datetime.utcnow()
		datastreams["pressure"].current_value = tpa["raw_pressure"]
		datastreams["pressure"].at = datetime.utcnow()

		try:
			datastreams["temperature"].update()
			datastreams["altitude"].update()
			datastreams["pressure"].update()

		except requests.HTTPError as e:
			print "HTTPError({0}): {1}".format(e.errno, e.strerror)

		if thread_keep_alive == 0:
			print "Streaming shutdown"
			break

		time.sleep(10)

###############################################################
# INIT SECTION
DEBUG=1


# Consumer keys and access tokens, used for OAuth
consumer_key = '4dg3D0FRULTUMCWyUytmsg'
consumer_secret = 'yqgk7hqpBxkQwJzvtUnK2E0rDPLqI5JSuv54qsA8'
access_token = '1911720504-8audXV5cs0Wt2cmFaKmAyPbytSc3vFnonpOd9Tg'
access_token_secret = 'T1yNOP1ZAeI0MGriTB8ERBDYDhrfJxorRZeqUWV3q10'

# OAuth process, using the keys and tokens
auth = tweepy.OAuthHandler(consumer_key, consumer_secret)
auth.set_access_token(access_token, access_token_secret)

# Creation of the actual interface, using authentication
twitter_api = tweepy.API(auth)

###############################################################
picorder_version_no = "1 HAT"
print "Picorder version " + picorder_version_no
print "Michael Horne - March 2015"
print "Using RPi.GPIO version " + GPIO.VERSION

# Tweet out the current timestamp and the IP addresses
time_stamp = time.time()
session_id = currentSession()

ip_tweet = ""
for addr in readIPaddresses():
	ip_tweet = ip_tweet + addr + ", "

try:
	sendTweet("Picorder activated at " + str(time_stamp) + ". " + ip_tweet)

except Exception as err:
	print "Unable to send tweet"
	print err

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

#######################################################
# BerryIMU

BERRYIMU = "BerryIMU" # BerryIMU or RTIMULib
if BERRYIMU == "RTIMULib":
	SETTINGS_FILE = "RTIMULib"
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

if BERRYIMU == "BerryIMU":
	imu = BerryIMU()


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

#######################################################
# I2C bus
bus = SMBus(1)

#######################################################
# Button
PIN_SWITCH = 5
GPIO.setup(PIN_SWITCH, GPIO.IN, pull_up_down=GPIO.PUD_UP)

###############################################################
# MAIN

if __name__ == "__main__":
	#############################################################
	# Start-up routine

	# Start a 'heartbeat' on the matrix to show something is working
	eightByEightHeartbeatAlive = 1
	threading.Thread(target = eightByEightHeartbeat).start()

	display("####################", "Picorder v" + picorder_version_no, "Starting up...", "####################")
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

	# Kill heartbeat (thread looks for this)
	eightByEightHeartbeatAlive = 0

	# Start independent processes
	thread_keep_alive = 1
	threading.Thread(target = streamReadings).start()
	threading.Thread(target = sevenSegmentClock).start()
	#threading.Thread(target = eightByEightDemo).start()
	threading.Thread(target = gameOfLife).start()
	threading.Thread(target = bargraphDemo).start()

	# A key press also advances the operation
	# Trouble with enabling this is you need to type 'reset' when you've exited to grab the keyboard back
	#threading.Thread(target = readKey).start()

	SHUTDOWN_FLAG = 0
	operation = 0
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
				if BERRYIMU == "RTIMULib":
					imu_readings = readIMU()
					display("IMU", "Roll: " + imu_readings['roll'], "Pitch: " + imu_readings['pitch'], "Yaw: " + imu_readings['yaw'])
					#time.sleep(0.5)

				elif BERRYIMU == "BerryIMU":
					imu_readings = imu.getReadings()
					display("Gyro", "X: " + str(imu_readings["gyroXangle"]), "Y: " + str(imu_readings["gyroYangle"]), "Z: " + str(imu_readings["gyroZangle"]))

			elif operation == 5:
				if BERRYIMU == "BerryIMU":
					imu_readings = imu.getReadings()
					display("CF", "X: " + str(imu_readings["CFangleX"]), "Y: " + str(imu_readings["CFangleY"]), "")

				else:
					display("Operation not enabled", "", "", "")

			elif operation == 6:
				if BERRYIMU == "BerryIMU":
					imu_readings = imu.getReadings()
					display("Accelerometer", "Roll (X): %.3f" % imu_readings["accXangle"], "Pitch (Y): %.3f" % imu_readings["accYangle"], "")

				else:
					display("Operation not enabled", "", "", "")

			elif operation == 7:
				if BERRYIMU == "BerryIMU":
					imu_readings = imu.getReadings()
					display("Heading", str(imu_readings["heading"]), "", "")

				else:
					display("Operation not enabled", "", "", "")

			elif operation == 8:
				tpa = readBarometer()
				display("Barometer", tpa['temperature'], tpa['pressure'], tpa['altitude'])
				time.sleep(0.5)

			elif operation == 9:
				ultrasonic = readUltrasonic()
				display("Ultrasonic", "Distance", "Measurement", ultrasonic)
				time.sleep(0.05)

			elif operation == 10:
				mq2 = readMQ2()
				display("MQ2 sensor", "Combustible gas", mq2[0], mq2[1])
				time.sleep(0.5)

			elif operation == 11:
				reading = readMoisture()
				display("Moisture", reading[0], reading[1], "")
				time.sleep(0.5)

			elif operation == 12:
				readings = readSystemTemperatures()
				display("System temperatures", readings[0], readings[1], "")
				time.sleep(1)

			elif operation == 13:
				mq3 = readMQ3()
				display("MQ3 sensor", "Alcohol sensor", mq3[0], mq3[1])
				time.sleep(0.5)

			elif operation == 14:
				reading = readLastTweet()
				display("Last tweet to @picorder", reading, "", "")
				time.sleep(5)
				operation = operation + 1

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

