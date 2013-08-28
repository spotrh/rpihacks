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
from nxt_msgs.msg import Range, JointCommand



class JointPositionController:
    def __init__(self):
        self.initialized = False
        self.vel = 0

        # get joint name
        self.name = rospy.get_param('name', 'motor_1')
        
        # joint interaction
        self.pub = rospy.Publisher('joint_command', JointCommand)
        rospy.Subscriber('joint_states', JointState, self.jnt_state_cb)

        # desired joint position
        rospy.Subscriber('joint_position', JointCommand, self.jnt_pos_cb)
        


    def jnt_pos_cb(self, msg):
        if msg.name == self.name:
            self.pos_desi = msg.effort


    def jnt_state_cb(self, msg):
        for name, pos, vel in zip(msg.name, msg.position, msg.velocity):
            if name == self.name:
                self.vel = 0.5 * self.vel + 0.5 * vel
                if not self.initialized:
                    self.pos_desi = pos
                    self.initialized = True
                cmd = JointCommand()
                cmd.name = self.name
                cmd.effort = 190.0 * (self.pos_desi - pos) - 4.0 * self.vel
                print 'Joint at %f, going to %f, commanding joint %f'%(pos,self.pos_desi, cmd.effort)
                self.pub.publish(cmd)



def main():
    rospy.init_node('jnt_pos_controller')
    jnt_pos_controller = JointPositionController()
    rospy.spin()



if __name__ == '__main__':
    main()
