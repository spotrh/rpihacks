#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2010, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import roslib; roslib.load_manifest('nxt_controllers')  
import rospy
import math
import thread
import tf
from PyKDL import *
from sensor_msgs.msg import JointState, Range
from nav_msgs.msg import Odometry
from nxt_msgs.msg import JointCommand
from tf_conversions import posemath

PUBLISH_TF = True

class BaseOdometry:
    def __init__(self):
        self.initialized = False

        self.ns =rospy.get_namespace() + 'base_parameters/'
        # get joint name
        self.l_joint = rospy.get_param(self.ns +'l_wheel_joint')
        self.r_joint = rospy.get_param(self.ns +'r_wheel_joint')

        self.wheel_radius = rospy.get_param(self.ns +'wheel_radius', 0.022)
        self.wheel_basis = rospy.get_param(self.ns +'wheel_basis', 0.055)

        # joint interaction
        rospy.Subscriber('joint_states', JointState, self.jnt_state_cb)

        # tf broadcaster
        if PUBLISH_TF:
            self.br = tf.TransformBroadcaster()

        # publish results on topic
        self.pub = rospy.Publisher('odom', Odometry)

        self.initialized = False

    def jnt_state_cb(self, msg):
        # crates map
        position = {}
        for name, pos in zip(msg.name, msg.position):
            position[name] = pos
        self.frame_id = '/odom'      
        # initialize
        if not self.initialized:
            self.r_pos = position[self.r_joint]
            self.l_pos = position[self.l_joint]
            self.pose = Frame()
            self.initialized = True
        else:
            delta_r_pos = position[self.r_joint] - self.r_pos
            delta_l_pos = position[self.l_joint] - self.l_pos
            delta_trans = (delta_r_pos + delta_l_pos)*self.wheel_radius/2.0
            delta_rot   = (delta_r_pos - delta_l_pos)*self.wheel_radius/(2.0*self.wheel_basis)
            twist = Twist(Vector(delta_trans, 0, 0),  Vector(0, 0, delta_rot))
            self.r_pos = position[self.r_joint]
            self.l_pos = position[self.l_joint]
            self.pose = addDelta(self.pose, self.pose.M * twist)
            if PUBLISH_TF:
                self.br.sendTransform(self.pose.p, self.pose.M.GetQuaternion(), rospy.Time.now(), 'base_link', 'odom')

            
            self.rot_covar = 1.0
            if delta_rot == 0:
                self.rot_covar = 0.00000000001
        
            odom = Odometry()
            odom.header.frame_id = self.frame_id
            odom.header.stamp = rospy.Time.now()
            odom.pose.pose = posemath.toMsg(self.pose)
            odom.pose.covariance = [0.00001, 0, 0, 0, 0, 0,
                                    0, 0.00001, 0, 0, 0, 0, 
                                    0, 0, 10.0000, 0, 0, 0,
                                    0, 0, 0, 1.00000, 0, 0,
                                    0, 0, 0, 0, 1.00000, 0,
                                    0, 0, 0, 0, 0, self.rot_covar]   
            self.pub.publish(odom)

def main():
    rospy.init_node('nxt_base_odometry')
    base_odometry = BaseOdometry()
    rospy.spin()



if __name__ == '__main__':
    main()
