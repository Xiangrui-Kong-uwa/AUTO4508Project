#!/usr/bin/env python3
'''
This node is for taking photos including way point markers and traffic signs.
It subscribes to the {GPS topic, orientation, velocoty, etc} and describes the status of the robot.
If the robot is face to a way point marker, it will take a photo and save it to a file.
After that, publish a msg to tell other node you finish the task. The robot will move to the next point.

For the traffic sign, it will take a photo when the robot is close enough to the sign.

TODO: discuss with the team how to implement this node.
'''
import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + ' I heard %s', data.data)

def node():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('photo_taking', anonymous=True)
    rospy.Subscriber('/some_status', String, callback)
    pub = rospy.Publisher('chatter', String, queue_size=10)
    pub.publish(rospy.loginfo(rospy.get_caller_id() + ' Start take photo'))

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
     node()