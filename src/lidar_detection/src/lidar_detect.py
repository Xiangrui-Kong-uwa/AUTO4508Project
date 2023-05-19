import rospy
import math
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
def laser_scan_callback(msg):
    global m
    angle_min = msg.angle_min
    angle_increment = msg.angle_increment
    r = []
    min_index = math.inf
    min_range = math.inf

    for angle in range(-5, 6, 1):
        desired_angle_radians = math.radians(angle)
        desired_angle_index = int((desired_angle_radians - angle_min) / angle_increment)
        r.append(msg.ranges[desired_angle_index])

    for e in r:
        if e < min_range:
            min_index = r.index(e) - 5
            min_range = e

    m.data = [min_index, min_range]

    
if __name__ == '__main__':
    m = Float32MultiArray()
    rospy.init_node('Lidar_Detect')
    rospy.Subscriber('/scan', LaserScan, laser_scan_callback)
    pub = rospy.Publisher('/scan/data', Float32MultiArray, queue_size=10)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        pub.publish(m)
        rate.sleep()
    