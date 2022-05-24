#!/usr/bin/env python

import rospy

from geometry_msgs.msg import PoseStamped, Pose, Vector3, Quaternion

class controllerCheck():
    
    def __init__(self):
        rospy.init_node("controller_check_node")
        
        self.posePub = rospy.Publisher( "command/pose", PoseStamped, queue_size=1)

    def publish(self):

        rospy.sleep(7)
        rospy.loginfo("Controller Check Beginning.")

        rospy.sleep(3)
        pose = PoseStamped( pose = Pose( position = Vector3(0,0,4), orientation=Quaternion(0,0,0,1) ))
        self.posePub.publish(pose)
        
        rospy.sleep(3)
        pose = PoseStamped( pose = Pose( position = Vector3(2,0,4), orientation=Quaternion(0,0,0,1) ))
        self.posePub.publish(pose)
        
        rospy.sleep(3)
        pose = PoseStamped( pose = Pose( position = Vector3(2,2,4), orientation=Quaternion(0,0,0,1) ))
        self.posePub.publish(pose)
        
        rospy.sleep(3)
        pose = PoseStamped( pose = Pose( position = Vector3(0,0,4), orientation=Quaternion(0,0,0,1) ))
        self.posePub.publish(pose)
        

if __name__ == "__main__":
    checker = controllerCheck()
    checker.publish()