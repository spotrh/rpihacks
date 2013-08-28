#!/usr/bin/env python

import roslib; roslib.load_manifest("nxt_python")
import rospy
import time
import nxt.locator
from nxt.sensor import *

def test_sensors(b):
        rospy.init_node('test_sensor')

        t = TouchSensor(b, PORT_1)
        print "This is the touch sensor test. Make sure that your touch sensor is plugged into port 1:"
        print "TOUCH READING:"
        start =rospy.Time.now()
        while rospy.Time.now()<start+rospy.Duration(5.0):
          print 'TOUCH:', t.get_sample()
          time.sleep(0.1)


sock = nxt.locator.find_one_brick()
if sock:
	test_sensors(sock.connect())
	sock.close()
else:
	print 'No NXT bricks found'
