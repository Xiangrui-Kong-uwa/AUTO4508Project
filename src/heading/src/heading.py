#!/usr/bin/env python

import rospy
from sensor_msgs.msg import MagneticField
from std_msgs.msg import Float64
import math

class CompassNode:
    def __init__(self):
        rospy.init_node('compass_node')
        self.heading_pub = rospy.Publisher('compass/heading', Float64, queue_size=10)
        rospy.Subscriber('imu/mag', MagneticField, self.mag_callback)

    def mag_callback(self, msg):
        # Extract magnetic field components
        mag_x = msg.magnetic_field.x
        mag_y = msg.magnetic_field.y

        # Calculate compass heading in degrees
        compass_heading = math.degrees(math.atan2(mag_y, mag_x)) + 90

        if compass_heading < -180:
            compass_heading += 360
        elif compass_heading > 180:
            compass_heading -= 360
        # Publish the compass heading on the Compass/Heading topic
        self.heading_pub.publish(compass_heading)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    compass_node = CompassNode()
    compass_node.run()
