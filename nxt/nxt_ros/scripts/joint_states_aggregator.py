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


import roslib; roslib.load_manifest('nxt_ros')  
import rospy
import math
from sensor_msgs.msg import JointState


class JS:
    def __init__(self, name, header, position, velocity, effort):
        self.name = name
        self.header = header
        self.position = position
        self.velocity = velocity 
        self.effort = effort


class JSAggregator:
    def __init__(self):
        # create motor
        self.sub = rospy.Subscriber('joint_state', JointState, self.callback)
        # create publisher
        self.pub = rospy.Publisher('joint_states', JointState)
        self.observed_states = {}
        self.updates_since_publish = 0
        
    def callback(self, data):
        num_joints = len(data.name)
        if len(data.position) < num_joints:
            rospy.logerr("Position array shorter than names %s < %d"%(len(data.position), num_joints))
            return
        elif len(data.velocity) < num_joints:
            rospy.logerr("Velocity array shorter than names %s < %d"%(len(data.velocity), num_joints))
            return
        elif len(data.effort) < num_joints:
            rospy.logerr("Effort array shorter than names %s < %d"%(len(data.effort), num_joints))
            return

        for i in xrange(0, num_joints):
            self.observed_states[data.name[i]] = JS(data.name[i], 
                                                    data.header, 
                                                    data.position[i], 
                                                    data.velocity[i], 
                                                    data.effort[i])
            
        todelete = [k for k, v in self.observed_states.iteritems() if  data.header.stamp - v.header.stamp > rospy.Duration().from_sec(10.0) ] #hack parametersize
        for td in todelete:
            del self.observed_states[td]


        # Only publish if there has been as many updates as there are joints, otherwise odom, gets zero deltas and the robot jerks around. 
        if self.updates_since_publish < len(self.observed_states.keys()):
            self.updates_since_publish += 1
            return

        self.updates_since_publish = 0
        msg_out = JointState()
        msg_out.header = data.header        
        for k, v in self.observed_states.iteritems():
            #print k, v
            msg_out.name.append(v.name)
            msg_out.position.append(v.position)
            msg_out.velocity.append(v.velocity)
            msg_out.effort.append(v.effort)
            
        self.pub.publish(msg_out)

def main():
    rospy.init_node("joint_state_aggregator")

    agg = JSAggregator()

    rospy.spin()



if __name__ == '__main__':
    main()
