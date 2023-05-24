#!/usr/bin/env python3
'''
This node is to take photos of the cone and bucket, along with notifying other nodes that these objects are detected.
It subscribes to the camera topic and will use contour mapping to see if the image from the camera contains a cone or bucket. 

Need to test and fine tune the parameters of this code. At the time of writing this, there are no available machines. 
'''
import rospy
import cv2
import numpy as np
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from skimage import measure
import random as rng
t_min = 0
i = 0

def detect_cones(image):
    global t
    global seg_t
    global i
    bridge = CvBridge()
    cone_detected = False
    
    hsv_img = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)

    lower_orange1 = np.array([0, 135, 135])
    lower_orange2 = np.array([15, 255, 255])
    upper_orange1 = np.array([159, 135, 80])
    upper_orange2 = np.array([179, 255, 255])

    lower_yellow1 = np.array([0, 100, 100])
    lower_yellow2 = np.array([30, 255, 255])
    upper_yellow1 = np.array([170, 100, 100])
    upper_yellow2 = np.array([180, 255, 255])

    imgThreshLow = cv2.inRange(hsv_img, lower_orange1, lower_orange2)
    imgThreshHigh = cv2.inRange(hsv_img, upper_orange1, upper_orange2)

    yelow = cv2.inRange(hsv_img, lower_yellow1, lower_yellow2)
    yehigh = cv2.inRange(hsv_img, upper_yellow1, upper_yellow2)

    t_im = cv2.bitwise_or(yelow, yehigh)
    threshed_img = cv2.bitwise_or(imgThreshLow, imgThreshHigh)
    threshed_img = cv2.bitwise_or(t_im, threshed_img)
    kernel = np.ones((5,5),np.uint8)
    threshed_img_smooth = cv2.erode(threshed_img, kernel, iterations = 6)
    threshed_img_smooth = cv2.dilate(threshed_img_smooth, kernel, iterations = 5)

    smoothed_img = cv2.dilate(threshed_img_smooth, kernel, iterations = 50) #50
    smoothed_img = cv2.copyMakeBorder(src=smoothed_img, top=1, bottom=1, left=1, right=1, borderType=cv2.BORDER_CONSTANT, value=[0,0,0])
    smoothed_img = cv2.erode(smoothed_img, kernel, iterations = 49) 
    seg_t = bridge.cv2_to_imgmsg(smoothed_img, "8UC1")

    edges_img = cv2.Canny(smoothed_img, 100, 200)
    contours, hierarchy = cv2.findContours(edges_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    coordinates = []
    color1 = (rng.randint(0,256), rng.randint(0,256), rng.randint(0,256))
    tmp_obj = image
    font = cv2.FONT_HERSHEY_SIMPLEX
    fontScale = 2
    fontColor = (0, 0, 255)
    lineType = 2

    for cnt in contours:
        boundingRect = cv2.boundingRect(cnt)
        approx = cv2.approxPolyDP(cnt, 0.07 * cv2.arcLength(cnt, True), True)
        if len(approx) == 3:
            x, y, w, h = cv2.boundingRect(approx)
            rect = (x, y, w, h)
            cv2.rectangle(tmp_obj, (x, y), (x+w, y+h), (0, 255, 0), 3)
            bottomLeftCornerOfText = (x, y)
            cv2.putText(tmp_obj,'traffic_cone', 
                bottomLeftCornerOfText, 
                font, 
                fontScale,
                fontColor,
                lineType)
            cv2.imwrite("/home/group6/img/"+str(i)+".png", tmp_obj)
            i+=1

    t = bridge.cv2_to_imgmsg(tmp_obj, "bgr8")
    detected_cones = []
    # for contour in contours:
    #     approx = cv2.approxPolyDP(contour, 0.03 * cv2.arcLength(contour, True), True)
    #     if len(approx) >= 3:
    #         (x, y), radius = cv2.minEnclosingCircle(contour)
    #         if 1 < radius < 500:
    #             detected_cones.append((int(x), int(y), int(radius)))
    #             cone_detected = True

    return detected_cones, False

def image_callback(msg):
    global i
    global cone_detected
    global img
    try:
        # Convert ROS Image message to OpenCV format
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        detect = False
        # Detect cones
        detected_cones, detect = detect_cones(cv_image)
        # Draw bounding circles on the image
        for (x, y, radius) in detected_cones:
            cv2.circle(cv_image, (x, y), radius, (0, 255, 0), 2)
        if detect:
            cv2.imwrite("/home/group6/img/"+str(i)+".png", cv_image)
            i += 1
        # Convert the OpenCV image to ROS Image message
        img_msg = bridge.cv2_to_imgmsg(cv_image, "bgr8")
        # Publish the image with bounding circles
        img = img_msg
        cone_detected.data = detect
        
    except CvBridgeError as e:
        print(e)
        img = Image()

if __name__ == '__main__':
    cone_detect = Bool()
    i = 0
    img = Image()
    t = Image()
    seg_t = Image()
    rospy.init_node('cone_detection')
    rospy.Subscriber('/oak/rgb/image_raw', Image, image_callback)
    image_pub = rospy.Publisher('cone_detection/image_with_circles', Image, queue_size=10)
    cone_detected = rospy.Publisher('cone_detection/check', Bool, queue_size=10)
    test = rospy.Publisher('test/box', Image, queue_size=10)
    s = rospy.Publisher('test/highlight', Image, queue_size=10)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        image_pub.publish(img)
        test.publish(t)
        s.publish(seg_t)
        cone_detected.publish(cone_detect)
        rate.sleep()
