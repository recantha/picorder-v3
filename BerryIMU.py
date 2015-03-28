#!/usr/bin/python
#
#	This program  reads the angles from the acceleromter, gyrscope
#	and mangnetometeron a BerryIMU connected to a Raspberry Pi.
#
#	http://ozzmaker.com/
#
#    Copyright (C) 2014  Mark Williams
#    This library is free software; you can redistribute it and/or
#    modify it under the terms of the GNU Library General Public
#    License as published by the Free Software Foundation; either
#    version 2 of the License, or (at your option) any later version.
#    This library is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
#    Library General Public License for more details.
#    You should have received a copy of the GNU Library General Public
#    License along with this library; if not, write to the Free
#    Software Foundation, Inc., 59 Temple Place - Suite 330, Boston,
#    MA 02111-1307, USA


import smbus
import time
import math
from LSM9DS0 import *
import datetime
bus = smbus.SMBus(1)

class BerryIMU:
	RAD_TO_DEG = 57.29578
	M_PI = 3.14159265358979323846
	G_GAIN = 0.070  # [deg/s/LSB]  If you change the dps for gyro, you need to update this value accordingly
	LP = 0.041    	# Loop period = 41ms.   This needs to match the time it takes each loop to run
	AA =  0.90      # Complementary filter constant


	def __init__(self):
		#initialise the accelerometer
		self.writeACC(CTRL_REG1_XM, 0b01100111) #z,y,x axis enabled, continuos update,  100Hz data rate
		self.writeACC(CTRL_REG2_XM, 0b00100000) #+/- 16G full scale

		#initialise the magnetometer
		self.writeMAG(CTRL_REG5_XM, 0b11110000) #Temp enable, M data rate = 50Hz
		self.writeMAG(CTRL_REG6_XM, 0b01100000) #+/-12gauss
		self.writeMAG(CTRL_REG7_XM, 0b00000000) #Continuous-conversion mode

		#initialise the gyroscope
		self.writeGRY(CTRL_REG1_G, 0b00001111) #Normal power mode, all axes enabled
		self.writeGRY(CTRL_REG4_G, 0b00110000) #Continuos update, 2000 dps full scale

	def writeACC(self,register,value):
	        bus.write_byte_data(ACC_ADDRESS , register, value)
	        return -1
	
	def writeMAG(self,register,value):
	        bus.write_byte_data(MAG_ADDRESS, register, value)
	        return -1

	def writeGRY(self,register,value):
	        bus.write_byte_data(GYR_ADDRESS, register, value)
	        return -1

	def readACCx(self):
	        acc_l = bus.read_byte_data(ACC_ADDRESS, OUT_X_L_A)
	        acc_h = bus.read_byte_data(ACC_ADDRESS, OUT_X_H_A)
		acc_combined = (acc_l | acc_h <<8)

		return acc_combined  if acc_combined < 32768 else acc_combined - 65536

	def readACCy(self):
	        acc_l = bus.read_byte_data(ACC_ADDRESS, OUT_Y_L_A)
	        acc_h = bus.read_byte_data(ACC_ADDRESS, OUT_Y_H_A)
		acc_combined = (acc_l | acc_h <<8)
		
		return acc_combined  if acc_combined < 32768 else acc_combined - 65536

	def readACCz(self):
	        acc_l = bus.read_byte_data(ACC_ADDRESS, OUT_Z_L_A)
	        acc_h = bus.read_byte_data(ACC_ADDRESS, OUT_Z_H_A)
		acc_combined = (acc_l | acc_h <<8)

		return acc_combined  if acc_combined < 32768 else acc_combined - 65536

	def readMAGx(self):
	        mag_l = bus.read_byte_data(MAG_ADDRESS, OUT_X_L_M)
	        mag_h = bus.read_byte_data(MAG_ADDRESS, OUT_X_H_M)
	        mag_combined = (mag_l | mag_h <<8)
	
	        return mag_combined  if mag_combined < 32768 else mag_combined - 65536

	def readMAGy(self):
	        mag_l = bus.read_byte_data(MAG_ADDRESS, OUT_Y_L_M)
	        mag_h = bus.read_byte_data(MAG_ADDRESS, OUT_Y_H_M)
	        mag_combined = (mag_l | mag_h <<8)
	
	        return mag_combined  if mag_combined < 32768 else mag_combined - 65536

	def readMAGz(self):
	        mag_l = bus.read_byte_data(MAG_ADDRESS, OUT_Z_L_M)
	        mag_h = bus.read_byte_data(MAG_ADDRESS, OUT_Z_H_M)
	        mag_combined = (mag_l | mag_h <<8)
	
	        return mag_combined  if mag_combined < 32768 else mag_combined - 65536

	def readGYRx(self):
	        gyr_l = bus.read_byte_data(GYR_ADDRESS, OUT_X_L_G)
	        gyr_h = bus.read_byte_data(GYR_ADDRESS, OUT_X_H_G)
	        gyr_combined = (gyr_l | gyr_h <<8)

	        return gyr_combined  if gyr_combined < 32768 else gyr_combined - 65536
  
	def readGYRy(self):
	        gyr_l = bus.read_byte_data(GYR_ADDRESS, OUT_Y_L_G)
	        gyr_h = bus.read_byte_data(GYR_ADDRESS, OUT_Y_H_G)
	        gyr_combined = (gyr_l | gyr_h <<8)

	        return gyr_combined  if gyr_combined < 32768 else gyr_combined - 65536

	def readGYRz(self):
	        gyr_l = bus.read_byte_data(GYR_ADDRESS, OUT_Z_L_G)
	        gyr_h = bus.read_byte_data(GYR_ADDRESS, OUT_Z_H_G)
	        gyr_combined = (gyr_l | gyr_h <<8)

	        return gyr_combined  if gyr_combined < 32768 else gyr_combined - 65536

	def getReadings(self):
		gyroXangle = 0.0
		gyroYangle = 0.0
		gyroZangle = 0.0
		CFangleX = 0.0
		CFangleY = 0.0

		AccXangle =  (math.atan2(self.readACCy(),self.readACCz())+self.M_PI)*self.RAD_TO_DEG;
		AccYangle =  (math.atan2(self.readACCz(),self.readACCx())+self.M_PI)*self.RAD_TO_DEG;

		a = datetime.datetime.now()

		#Convert Gyro raw to degrees per second
		rate_gyr_x =  self.readGYRx() * self.G_GAIN
		rate_gyr_y =  self.readGYRy() * self.G_GAIN
		rate_gyr_z =  self.readGYRz() * self.G_GAIN
	
		#Calculate the angles from the gyro. LP = loop period 
		gyroXangle+=rate_gyr_x*self.LP;
		gyroYangle+=rate_gyr_y*self.LP;
		gyroZangle+=rate_gyr_z*self.LP;

	        # Change the rotation value of the accelerometer to -/+ 180 and move the Y axis '0' point to up.
	        # Two different pieces of code are used depending on how your IMU is mounted.
	        # If IMU is upside down
		#
	        # if AccXangle >180:
	        #         AccXangle -= 360.0
	        # AccYangle-=90
	        # if (AccYangle >180):
	        #         AccYangle -= 360.0
	

	        #//If IMU is up the correct way, use these lines
	        AccXangle -= 180.0
		if AccYangle > 90:
		        AccYangle -= 270.0
		else:
			AccYangle += 90.0


	        #Complementary filter used to combine the accelerometer and gyro values.
	        CFangleX=self.AA*(CFangleX+rate_gyr_x*self.LP) +(1 - self.AA) * AccXangle;
	        CFangleY=self.AA*(CFangleY+rate_gyr_y*self.LP) +(1 - self.AA) * AccYangle;

		heading = 180 * math.atan2(self.readMAGy(),self.readMAGx())/self.M_PI

		if heading < 0:
		 	heading += 360;

	
 		#print ("\033[1;34;40mACCX Angle %5.2f ACCY Angle %5.2f\033[1;31;40m\tGRYX Angle %5.2f  GYRY Angle %5.2f  GYRZ Angle %5.2f \033[1;35;40m    \tCFangleX Angle %5.2f \033[1;36;40m  CFangleY Angle %5.2f \33[1;32;40m  HEADING  %5.2f\033[0m  " % (AccXangle, AccYangle,gyroXangle,gyroYangle,gyroZangle,CFangleX,CFangleY,heading))

		time.sleep(0.03)
		b = datetime.datetime.now()
		c = b - a

		#print "Loop Time |",  c.microseconds/1000,"|",

		readings = {}
		readings["accXangle"] = AccXangle
		readings["accYangle"] = AccYangle
		readings["gyroXangle"] = gyroXangle
		readings["gyroYangle"] = gyroYangle
		readings["gyroZangle"] = gyroZangle
		readings["CFangleX"] = CFangleX
		readings["CFangleY"] = CFangleY
		readings["heading"] = heading
		readings["loopTime"] = c.microseconds/1000

		return readings

if __name__ == '__main__':
	berry = BerryIMU()
	readings = berry.getReadings()
	for key in readings:
		print "Reading " + key + ": " + str(readings[key])
