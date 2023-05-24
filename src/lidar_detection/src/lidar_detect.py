import rospy
import math
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray, Bool
def laser_scan_callback(msg):
    global m
    angle_min = msg.angle_min
    angle_increment = msg.angle_increment
    r = []
    min_index = math.inf
    min_range = math.inf

    for angle in range(-80, 25, 1):
        desired_angle_radians = math.radians(angle)
        desired_angle_index = int((desired_angle_radians - angle_min) / angle_increment)
        r.append(msg.ranges[desired_angle_index])

    for e in r:
        if e < min_range and not (e < 0.2):
            min_index = r.index(e) - 5
            min_range = e

    m.data = [min_index, min_range]

    
if __name__ == '__main__':
    m = Float32MultiArray()
    m.data = [0.0,0.0]
    rospy.init_node('Lidar_Detect')
    rospy.Subscriber('/scan', LaserScan, laser_scan_callback)
    pub = rospy.Publisher('/scan/data', Float32MultiArray, queue_size=10)
    pub_avoid = rospy.Publisher('/avoid_object/front', Bool, queue_size=10)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if m.data[1] < 1.5:
            pub_avoid.publish(True)
        else:
            pub_avoid.publish(False)
        pub.publish(m)
        rate.sleep()
    