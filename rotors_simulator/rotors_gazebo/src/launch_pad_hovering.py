#!/usr/bin/env python

import rospy

from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState

from geometry_msgs.msg import Point, Pose

from math import cos, pi

A = 7
w = 0.025

def main():

    rospy.init_node("hoverLanding")
    rospy.wait_for_service('/gazebo/set_model_state')
    movePlatform = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

    while not rospy.is_shutdown() :
        t = rospy.get_time()
        pos = Point(7, A*cos(2*pi*w*t), 1)

        pose_ = Pose(position  = pos)

        state = ModelState(model_name="landing_pad", pose = pose_)

        movePlatform(state)

if __name__ == "__main__":
    main()