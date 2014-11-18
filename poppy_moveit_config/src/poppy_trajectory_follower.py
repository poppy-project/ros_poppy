#!/usr/bin/python
# -*- coding: utf-8 -*-

########################################################################
#  File Name	: 'poppy_trajectory_follower.py'
#  Author	: Yoan Mollard
#  Contact      : yoan.mollard@inria.fr
#  Created	: Saturday, Nov  1 2014
#  Revised	:
#  Version	:
#  Target MCU	:
#
#  This code is distributed under the GNU Public License
# 		which can be found at http://www.gnu.org/licenses/gpl.txt
#
#
#  Notes:	This simple trajectory follower allows to send trajectories
#       to Poppy in joint-space and is position-controlled only
########################################################################
import rospy
import actionlib

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryFeedback, FollowJointTrajectoryResult
from std_msgs.msg import Float64

class PoppyTrajectoryFollower(object):
  def __init__(self, name):
    self._feedback = FollowJointTrajectoryFeedback()
    self._result   = FollowJointTrajectoryResult()

    self._action_name = name
    self._as = actionlib.SimpleActionServer(self._action_name, FollowJointTrajectoryAction, execute_cb=self.execute_cb, auto_start = False)
    self._as.start()

    # Getting publishers for all joints
    self._poppy = rospy.get_param("/poppy")
    self._joints_pub = {}
    for k, d in self._poppy.iteritems():
      if k != 'joint_state_controller': #just ignore this one
        self._joints_pub[k.replace('_position_controller', '')] = rospy.Publisher("/poppy/" + k + "/command", Float64)

    rospy.loginfo('%s: is ready' % self._action_name)


  def execute_cb(self, goal):
    """
      This callback is called when a trajectory in joint space is received, it executes the corresponding motion
      :param goal: a message of type control_msgs/FollowJointTrajectoryGoal
    """
    self._result.error_code = FollowJointTrajectoryResult.SUCCESSFUL

    # For each point of the trajectory
    t_start = rospy.Time.now()

    for i, p in enumerate(goal.trajectory.points):
      # For each joint
      for ji, jp in enumerate(p.positions):
        try:
          joint = goal.trajectory.joint_names[ji]
          self._joints_pub[joint].publish(jp)
        except KeyError:
          self._result.error_code = FollowJointTrajectoryResult.INVALID_JOINTS

        self._feedback.header.stamp = rospy.Time.now()
        self._as.publish_feedback(self._feedback)

      while rospy.Time.now()-t_start<p.time_from_start:
        rospy.sleep(0.001) # Waiting the next point at 1000Hz
        if self._as.is_preempt_requested():
          rospy.loginfo('%s: Preempted' % self._action_name)
          self._as.set_preempted()
          break
      
    if self._result.error_code == FollowJointTrajectoryResult.SUCCESSFUL:
      rospy.loginfo('%s: Succeeded' % self._action_name)
      self._as.set_succeeded(self._result)
    else:
      self._as.set_aborted(self._result)
      
if __name__ == '__main__':
  name = "poppy_trajectory_follower"
  rospy.init_node(name)
  PoppyTrajectoryFollower("/poppy/"+name)
  rospy.spin()
