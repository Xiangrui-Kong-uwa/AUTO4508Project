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
import numpy as np
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from skimage import measure
t_min = 0
i = 0
class imgNode():
    def __init__(self):
        self.dmap = []
        rospy.init_node('photo_taking', anonymous=True)
        rospy.Subscriber('/oak/rgb/image_raw', Image, self.rgb_callback)
        rospy.Subscriber('/oak/stereo/image_raw', Image, self.depth_callback)
        self.pub = rospy.Publisher('/cone/captured', String, queue_size=10)

    def depth_callback(self, msg):
        img_depth = CvBridge().imgmsg_to_cv2(msg, desired_encoding='passthrough')
        self.dmap = img_depth


    def rgb_callback(self, img_msg):
        '''
        if cone detected, save image to a cone folder
        
        if traffic sign detected, save image to a traffic sign folder
        '''
        global i
        img_orig = CvBridge().imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        retval, result = self.Cone_detection(img_orig)
        if(retval == True):
            x = int(result.centroid[0])
            y = int(result.centroid[1])
            item = self.dmap
            dist = item[x,y]
            #cv2.imwrite("/home/group6/img/"+str(i)+".png", img_orig)
            self.pub.publish('Found cone')
            i+=1

    def Cone_detection(self,img):
        global t_min
        t_min = 8000
        img = cv2.blur(img,(1,1))
        hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
        low_filter = np.array([0,120,60])
        high_filter = np.array([15,255,255])
        low_filter1 = np.array([165,120,60])
        high_filter1 = np.array([180,255,255])
        mask = cv2.inRange(hsv,low_filter,high_filter)
        mask1 = cv2.inRange(hsv,low_filter1,high_filter1)
        mask2 = cv2.bitwise_or(mask,mask1)
    
        labels, num = measure.label(mask2,connectivity=1, return_num=1)
    
        p = measure.regionprops(labels)
        candidate = []
        for item in p:
            if item.area>t_min:
                candidate.append(item)
        if len(candidate)!=0:
            return (True, candidate[0])
        else:
            return (False,None)

'''        
def node():
    rospy.init_node('photo_taking', anonymous=True)
    rospy.Subscriber('/oak/rgb/image_raw', Image, img_callback)
    rospy.Subscriber('/oak/stereo/image_raw', Image, img_callback)
    pub = rospy.Publisher('chatter', String, queue_size=10)
    #pub.publish(rospy.loginfo(rospy.get_caller_id() + ' Start take photo'))

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
'''
if __name__ == '__main__':
    node = imgNode()
    rospy.spin()
