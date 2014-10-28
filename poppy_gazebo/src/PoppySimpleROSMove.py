#!/usr/bin/python
# -*- coding: utf-8 -*-

########################################################################
#  File Name	: 'PoppySimpleROSMove.py'
#  Author	: Steve NGUYEN
#  Contact      : steve.nguyen@inria.fr
#  Created	: Wednesday, June  4 2014
#  Revised	:
#  Version	:
#  Target MCU	:
#
#  This code is distributed under the GNU Public License
# 		which can be found at http://www.gnu.org/licenses/gpl.txt
#
#
#  Notes:	notes
########################################################################

from math import radians
import time


#ros stuff

import roslib
import rospy
import math
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64







class Poppy():
    def __init__(self):

        rospy.init_node("poppy_controller")

        self.joints_state_sub = rospy.Subscriber("/poppy/joint_states", JointState, self.state_cb)

        #retrieve the joints names loaded in ROS
        #they are of the form: l_hip_x_position_controller

        self.poppy_param = rospy.get_param("/poppy")
        self.joints_ctrl_name = []
        self.joints_ctrl_pos = []
        self.joints_pub = {}
        self.joints_state = {}

        for k, d in self.poppy_param.iteritems():
            if k != 'joint_state_controller': #just ignore this one
                self.joints_ctrl_name.append(k)
                self.joints_ctrl_pos.append(0.0)
                #for each joint we create a publisher
                self.joints_pub[k.replace('_position_controller', '')] = rospy.Publisher("/poppy/" + k + "/command", Float64)

    def zero(self):
        for k, d in self.joints_pub.iteritems():
            d.publish(0.0)
            time.sleep(0.5)
        print "Zero done"

    def state_cb(self, data):
        for i, n in enumerate(data.name):
            self.joints_state[n] = data.position[i] #there is also velocity and effort

    def move(self):
        while not rospy.is_shutdown():
            for k, d in self.joints_pub.iteritems():
                d.publish(self.joints_state[k] + 0.01)

            time.sleep(0.1)

if __name__ ==  '__main__':


    poppy = Poppy()
    poppy.zero()

    poppy.move()
    # rospy.spin()
