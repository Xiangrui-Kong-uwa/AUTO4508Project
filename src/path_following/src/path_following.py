#!/usr/bin/env python3
'''
This node is for path following. 
It reads the path from a file which path is given as a parameter.
It subscribes to the {GPS topic} and publishes {/cmd_vel}.


'''
import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)

def follower():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('path_following', anonymous=True)

    rospy.Subscriber('/gps', String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
     follower()