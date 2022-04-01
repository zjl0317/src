#!/usr/bin/env python

import rospy
from robot305_gazebo.robot305 import Robot305


if __name__=="__main__":
    rospy.init_node("walker_demo")
    
    rospy.loginfo("Instantiating Robot305 Client")
    robot305=Robot305()
    rospy.sleep(1)
 
    rospy.loginfo("Robot305 Walker Demo Starting")


    robot305.set_walk_velocity(0.2,0,0)
    rospy.sleep(3)
    robot305.set_walk_velocity(1,0,0)
    rospy.sleep(3)
    robot305.set_walk_velocity(0,1,0)
    rospy.sleep(3)
    robot305.set_walk_velocity(0,-1,0)
    rospy.sleep(3)
    robot305.set_walk_velocity(-1,0,0)
    rospy.sleep(3)
    robot305.set_walk_velocity(1,1,0)
    rospy.sleep(5)
    robot305.set_walk_velocity(0,0,0)
    
    rospy.loginfo("Robot305 Walker Demo Finished")
