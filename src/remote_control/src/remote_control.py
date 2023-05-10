#!/usr/bin/env python3
'''
For safety reasons, implement a Bluetooth link between the robot’s on-board PC
and a gamepad controller:
a. Button ‘A’ enable automated mode.
In automated mode, use the back pedals as a dead-man switch.
If released, the robot has to stop.
b. Button ‘B’ enable manual mode (disable automated mode).
In manual mode, the steering controls can be used to manually drive the
robot forward/backward and left/right.

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
    rospy.init_node('remote control', anonymous=True)
    rospy.Subscriber('/some_status', String, callback)
    pub = rospy.Publisher('chatter', String, queue_size=10)
    pub.publish(rospy.loginfo(rospy.get_caller_id() + ' Start remote control'))

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
     node()