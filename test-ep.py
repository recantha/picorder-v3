#!/usr/bin/python

from EasyPulse import EasyPulse

easypulse = EasyPulse(True)
beats = easypulse.readPulse()
print "Number of beats: " + str(len(beats))
heartrate = easypulse.computeHeartrate(beats)
print "Heartrate: " + str(heartrate)


