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
import random as rng
cone_distance = 10000000
bucket_distance = 10000000
cone_count = 0
bucket_count = 0
cone_captured = 0
cone_detected = 0
bucket_detected = 0
bucket_captured = 0
cone_msg = Image()
bucket_msg = Image()
img_depth = []
seg_msg = Image()




def detect_cones(image):
    global cone_msg
    global cone_count
    global cone_detected
    global seg_msg
    global cone_captured
    global cone_distance

    bridge = CvBridge()
    
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

    seg_msg = bridge.cv2_to_imgmsg(smoothed_img, "8UC1")

    edges_img = cv2.Canny(smoothed_img, 100, 200)
    contours, hierarchy = cv2.findContours(edges_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    tmp_obj = image
    font = cv2.FONT_HERSHEY_SIMPLEX
    fontScale = 2
    fontColor = (0, 0, 255)
    lineType = 2

    area = []
    for cnt in contours:
        boundingRect = cv2.boundingRect(cnt)
        approx = cv2.approxPolyDP(cnt,0.07*cv2.arcLength(cnt,True),True)
        area.append((boundingRect[2]*boundingRect[3],approx))
        area.sort(key=lambda x:x[0],reverse=True)

    if(len(area)!=0):
        approx = area[0][1]
        if len(approx) == 3 and area[0][0]>100000:
            x, y, w, h = cv2.boundingRect(approx)
            cv2.rectangle(tmp_obj, (x, y), (x+w, y+h), (0, 255, 0), 3)
            bottomLeftCornerOfText = (x, y)
            cv2.putText(tmp_obj,'traffic_cone'+str(cone_distance), 
                bottomLeftCornerOfText, 
                font, 
                fontScale,
                fontColor,
                lineType)
            cone_count+=1

                #for debug
            print(cone_count)
            if(cone_count>20):
                cv2.imwrite("/home/group6/img/"+"Cone"+str(cone_count)+".png", tmp_obj)
                cone_detected = 1
                cone_count = 0

            return int(x+w/2),int(y+h/2)
    cone_msg = bridge.cv2_to_imgmsg(tmp_obj, "bgr8")
    return 0,0
    

def bucket_detection(img):
    global bucket_count
    global bucket_detected
    global bucket_msg
    global bucket_distance
    

    bridge = CvBridge()

    blurred = cv2.blur(img,(5,5))
    hsv = cv2.cvtColor(blurred,cv2.COLOR_BGR2HSV)
    
    high_filter_blue = np.array([140,230,180])
    low_filter_blue = np.array([90,30,30])
    
    high_filter_green = np.array([100,255,120])
    low_filter_green = np.array([70,100,30])
    
    low_filter_orange = np.array([0,120,60])
    high_filter_orange = np.array([15,255,255])
    
    blue_mask = cv2.inRange(hsv,low_filter_blue,high_filter_blue)
    green_mask = cv2.inRange(hsv,low_filter_green,high_filter_green)
    orange_mask = cv2.inRange(hsv,low_filter_orange,high_filter_orange)
    
    final_mask = cv2.bitwise_or(blue_mask,green_mask)
    final_mask = cv2.bitwise_or(final_mask,orange_mask)
    
    kernel = np.ones((7,7),np.uint8)
    mask2_smooth = cv2.erode(final_mask,kernel,iterations=4)
    mask2_smooth = cv2.dilate(final_mask,kernel,iterations=4)

    
    edges_img = cv2.Canny(mask2_smooth,100,200)
    contours, hierarchy = cv2.findContours(edges_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    tmp_obj = img
    font = cv2.FONT_HERSHEY_SIMPLEX
    fontScale = 2
    fontColor = (0, 0, 255)
    lineType = 2    
    area = []
    for cnt in contours:
        boundingRect = cv2.boundingRect(cnt)
        approx = cv2.approxPolyDP(cnt,0.1*cv2.arcLength(cnt,True),True)
        area.append((boundingRect[2]*boundingRect[3],approx))
        area.sort(key=lambda x:x[0],reverse=True)
    
    if (len(area) != 0 and area[0][0]>100000):
        approx = area[0][1]
        x, y, w, h = cv2.boundingRect(approx)
        cv2.rectangle(tmp_obj, (x, y), (x+w, y+h), (0, 255, 0), 3)
        bottomLeftCornerOfText = (x, y)
        cv2.putText(tmp_obj,'bucket'+str(bucket_distance), 
            bottomLeftCornerOfText, 
            font, 
            fontScale,
            fontColor,
            lineType)
        bucket_count+=1
        print(bucket_count)
        if(bucket_count>20):
            cv2.imwrite("/home/group6/img/"+"Bucket"+str(bucket_count)+".png", tmp_obj)
            bucket_detected=1
            bucket_count=0
        return int(x+w/2),int(y+h/2)
    bucket_msg = bridge.cv2_to_imgmsg(tmp_obj, "bgr8")
    return 0,0
    
    

def image_callback(msg):
    global img_depth
    global cone_msg
    global seg_msg
    global cone_distance
    global bucket_distance
    global cone_detected
    global bucket_detected
    global bucket_msg
    
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(msg,"bgr8")
    print(cone_detected)
    if(cone_detected==0):
        y,x = detect_cones(cv_image)
        if(len(img_depth)!=0):
            cone_distance = img_depth[x][y]
    if(cone_detected==1):
        y,x = bucket_detection(cv_image)
        if(len(img_depth)!=0):
            bucket_distance = img_depth[x][y]

    detection_pub.publish(str(cone_detected)+","+str(cone_distance)+","+str(bucket_detected)+","+str(bucket_distance))

    if(bucket_detected==1 and cone_detected==1):
        cone_detected=0
        bucket_detected=0 
    
    if(cone_msg!=None and cone_detected==0):
        image_pub.publish(cone_msg)
    if(seg_msg!=None):
        seg_pub.publish(seg_msg)
    if(bucket_msg!=None and cone_detected==1):
        image_pub.publish(bucket_msg)





        
def depth_callback(msg):
    global img_depth
    img_depth = CvBridge().imgmsg_to_cv2(msg, desired_encoding='passthrough')

    



if __name__ == '__main__':
    rospy.init_node('cone_detection')
    rospy.Subscriber('/oak/rgb/image_raw', Image, image_callback)
    rospy.Subscriber('/oak/stereo/image_raw', Image, depth_callback)
    detection_pub = rospy.Publisher('/detection_msg', String,queue_size=10)
    image_pub = rospy.Publisher('/ObjectDetection/Detected_Object', Image, queue_size=10)
    seg_pub = rospy.Publisher('/ObjectDetection/seg_pic',Image,queue_size=10)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()