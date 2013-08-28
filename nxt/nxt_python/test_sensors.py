#!/usr/bin/env python

import roslib; roslib.load_manifest("nxt_python")
import rospy
import time
import nxt.locator
from nxt.sensor import *

def test_sensors(b):
        rospy.init_node('test_sensor')

#	print 'Touch:',
#	if TouchSensor(b, PORT_4).get_sample():
#		print 'yes'
#	else:
#		print 'no'
	#print 'Sound:', SoundSensor(b, PORT_2).get_sample()
#	print 'RED:', ColorSensor(b, PORT_2).set_illuminated('green')
#        time.sleep(1.0)
#	print 'BLUE:', ColorSensor(b, PORT_2).set_illuminated('blue')
#        time.sleep(1.0)
#	print 'GREEN:', ColorSensor(b, PORT_2).set_illuminated('green')
#        time.sleep(1.0)
#	print 'WHITE:', ColorSensor(b, PORT_2).set_illuminated('white')
#        time.sleep(1.0)
#	print 'OFF:', ColorSensor(b, PORT_2).set_illuminated(None)
        color_sensor = ColorSensor(b, PORT_4)
#        accel_sensor = AccelerometerSensor(b, PORT_2)
#        gyro_sensor = GyroSensor(b, PORT_3)
        while not rospy.is_shutdown():
#          print 'Gyro', gyro_sensor.get_sample()
          #print 'Accelerometer', accel_sensor.get_sample()
          #color_sensor.set_light_color('red')
          print 'COLOR:', color_sensor.get_reflected_light('blue')
          #print 'COLOR:', color_sensor.get_color()
          time.sleep(0.1)

#	print 'Ultrasonic:', UltrasonicSensor(b, PORT_3).get_sample()

sock = nxt.locator.find_one_brick()
if sock:
	test_sensors(sock.connect())
	sock.close()
else:
	print 'No NXT bricks found'
