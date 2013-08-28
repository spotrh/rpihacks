#!/usr/bin/env python

import roslib; roslib.load_manifest("nxt_python")
import rospy
import time
import nxt.locator
from nxt.sensor import *

def test_sensors(b):
        rospy.init_node('test_sensor')
        cs = ColorSensor(b, PORT_1)
        print "This is the color sensor test make sure that your color sensor is plugged into port 1:"
	print 'RED:'
        cs.set_light_color('red')
        time.sleep(1.0)
	print 'BLUE:'
        cs.set_light_color('blue')
        time.sleep(1.0)
	print 'GREEN:'
        cs.set_light_color('green')
        time.sleep(1.0)
	print 'WHITE:'
        cs.set_light_color('white')
        time.sleep(1.0)
	print 'OFF:'
        cs.set_light_color(None)
        print "INTENSITY READING:"
        start =rospy.Time.now()
        while rospy.Time.now()<start+rospy.Duration(5.0):
          print 'INTENSITY:', cs.get_reflected_light('blue')
          time.sleep(0.1)
        print "COLOR READING:"
        start =rospy.Time.now()
        while rospy.Time.now()<start+rospy.Duration(5.0):
          print 'COLOR:', cs.get_color()
          time.sleep(0.1)



sock = nxt.locator.find_one_brick()
if sock:
	test_sensors(sock.connect())
	sock.close()
else:
	print 'No NXT bricks found'
