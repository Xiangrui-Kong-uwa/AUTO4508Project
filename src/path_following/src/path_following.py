#!/usr/bin/env python3
'''
This node is for path following. 
It reads the path from a file which path is given as a parameter.
It subscribes to the {GPS topic} and publishes {/cmd_vel}.

TODO: discuss with the team how to implement this node.
'''
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix

def callback(data):
    pass 
    #rospy.loginfo(rospy.get_caller_id() + ' lat: %s, lon: %s', data.latitude, data.longitude)

def follower():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('path_following', anonymous=True)
    rospy.Subscriber('/fix', NavSatFix, callback)

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        #rospy.loginfo(hello_str)
        #pub.publish(hello_str)
        rate.sleep()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
     follower()
