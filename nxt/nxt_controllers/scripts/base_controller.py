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
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from nxt_msgs.msg import Range, JointCommand

class BaseController:
    def __init__(self):
        self.initialized = False
        self.vel_rot_desi = 0
        self.vel_trans_desi = 0
        self.vel_trans = 0
        self.vel_rot = 0

        self.ns =rospy.get_namespace() + 'base_parameters/' 
        # get joint name
        self.l_joint = rospy.get_param(self.ns +'l_wheel_joint')
        self.r_joint = rospy.get_param(self.ns +'r_wheel_joint')
        self.wheel_radius = rospy.get_param(self.ns +'wheel_radius', 0.022)
        self.wheel_basis = rospy.get_param(self.ns +'wheel_basis', 0.055)
        self.vel_to_eff = rospy.get_param(self.ns +'vel_to_eff', 0.5)
        self.k_rot = 0.075/self.vel_to_eff
        self.k_trans = 0.055/self.vel_to_eff

        # joint interaction
        self.pub = rospy.Publisher('joint_command', JointCommand)
        rospy.Subscriber('joint_states', JointState, self.jnt_state_cb)

        # base commands
        rospy.Subscriber('cmd_vel', Twist, self.cmd_vel_cb)
        
    def cmd_vel_cb(self, msg):
        self.vel_rot_desi = msg.angular.z
        self.vel_trans_desi = msg.linear.x

    def jnt_state_cb(self, msg):
        velocity = {}
        for name, vel in zip(msg.name, msg.velocity):
            velocity[name] = vel

        # lowpass for measured velocity
        self.vel_trans = 0.5*self.vel_trans + 0.5*(velocity[self.r_joint] + velocity[self.l_joint])*self.wheel_radius/2.0
        self.vel_rot =   0.5*self.vel_rot   + 0.5*(velocity[self.r_joint] - velocity[self.l_joint])*self.wheel_radius/(2.0*self.wheel_basis)

        # velocity commands
        vel_trans = self.vel_trans_desi + self.k_trans*(self.vel_trans_desi - self.vel_trans)
        vel_rot = self.vel_rot_desi + self.k_rot*(self.vel_rot_desi - self.vel_rot)
        
        # wheel commands
        l_cmd = JointCommand()
        l_cmd.name = self.l_joint
        l_cmd.effort = self.vel_to_eff*(vel_trans/self.wheel_radius - vel_rot*self.wheel_basis/self.wheel_radius)
        self.pub.publish(l_cmd)

        r_cmd = JointCommand()
        r_cmd.name = self.r_joint
        r_cmd.effort = self.vel_to_eff*(vel_trans/self.wheel_radius + vel_rot*self.wheel_basis/self.wheel_radius)
        self.pub.publish(r_cmd)


def main():
    rospy.init_node('nxt_base_controller')
    base_controller = BaseController()
    rospy.spin()



if __name__ == '__main__':
    main()
