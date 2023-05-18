#!/usr/bin/env python3
'''
This node is for taking photos including way point markers and traffic signs.
It subscribes to the {GPS topic, orientation, velocoty, etc} and describes the status of the robot.
If the robot is face to a way point marker, it will take a photo and save it to a file.
After that, publish a msg to tell other node you finish the task. The robot will move to the next point.

For the traffic sign, it will take a photo when the robot is close enough to the sign.
'''
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def img_callback(img_msg):
    '''
    if cone detected, save image to a cone folder
    
    if traffic sign detected, save image to a traffic sign folder
    '''
    img_orig = CvBridge().imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
    pass
    #rospy.loginfo(rospy.get_caller_id() + ' I heard %s', data.data)

def node():
    rospy.init_node('photo_taking', anonymous=True)
    rospy.Subscriber('/img', String, img_callback)
    pub = rospy.Publisher('chatter', String, queue_size=10)
    #pub.publish(rospy.loginfo(rospy.get_caller_id() + ' Start take photo'))

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
     node()