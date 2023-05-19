#!/usr/bin/env python3
'''
Use imu to plot path since the GPS is weak.
'''
import rospy
import math
import matplotlib.pyplot as plt
from sensor_msgs.msg import Imu, NavSatFix
from tf.transformations import euler_from_quaternion

class recorderNode:
    def __init__(self):
        self.x = []
        self.y = []
        rospy.init_node('record_display', anonymous=True)
        rospy.Subscriber('/imu/data', Imu, self.imu_callback)
        rospy.Subscriber('/fix', NavSatFix, self.gps_callback)
        #rate = rospy.Rate(10)  # Set the desired receiving rate to 10 Hz


    def imu_callback(self, msg):
        orientation = msg.orientation
        euler = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        heading = euler[2]  # Extracting the yaw angle from Euler angles

        # Calculate the position using heading information and assume constant velocity
        delta_x = 0.1 * math.cos(heading)  # Assuming a constant time interval of 0.1s
        delta_y = 0.1 * math.sin(heading)  # Assuming a constant time interval of 0.1s

        if len(self.x) == 0 and len(self.y) == 0:
            self.x.append(0)
            self.y.append(0)
        else:
            self.x.append(self.x[-1] + delta_x)
            self.y.append(self.y[-1] + delta_y)

    def gps_callback(self, msg):
        pass

    def plot_path(self):
        while not rospy.is_shutdown():
            plt.plot(self.x, self.y, 'b-')
            plt.xlabel('X')
            plt.ylabel('Y')
            plt.title('Path Plot')
            plt.grid(True)
            plt.pause(0.1)
            rospy.spin()
    

if __name__ == '__main__':
     recorder = recorderNode()
     recorder.plot_path()
