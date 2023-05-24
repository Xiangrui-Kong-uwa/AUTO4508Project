#!/usr/bin/env python3

import xml.etree.ElementTree as ET
import rospy
from sensor_msgs.msg import Joy, LaserScan, NavSatFix
from geometry_msgs.msg import Twist, Vector3
from math import sin, cos, sqrt, atan2, radians, pi, isnan
from std_msgs.msg import String, Float64, Bool
import math
import os
import csv
# Here we will have the main control flow of the robot.
# This node takes in all the appropriate sensor data and subscribes to the approptiate topics to make informed decisions
# This node then publishes the cmd_vel topic to the robot to make it move.

#kml_file_path = '../ws/src/path_following/src/Pioneer-coordinates.kml'
kml_file_path = os.path.dirname(os.path.abspath(__file__))+'/Pioneer-coordinates.kml'
wp_file_path = os.path.dirname(os.path.abspath(__file__))+'/waypoints.csv'

class WaypointNavigation:
    def __init__(self, waypoints):

        self.waypoints = waypoints
        self.current_waypoint_index = 0
        self.currentState = 'WaypointFollowing'
        self.nextState = 'WaypointFollowing'
        self.atWaypoint = False
        self.ManualControl = True
        self.stop = True
        self.currentHeading = 0.0
        self.lid_msg = LaserScan()
        self.collide_detect = True
        self.goal_bearing = 0.0
        self.object_avoid_front = rospy.Subscriber('/avoid_object/front', Bool,self.object_avoid_front_callback)
        self.heading_sub = rospy.Subscriber('/compass/heading', Float64, self.heading_callback)
        self.nav_fix_sub = rospy.Subscriber('/fix', NavSatFix, self.nav_fix_callback)
        self.joy_sub = rospy.Subscriber('/joy', Joy, self.joy_callback)
        self.cmd_vel_pub = rospy.Publisher('RosAria/cmd_vel', Twist, queue_size=1)
        self.state_pub = rospy.Publisher('/state', String, queue_size=10)
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

    def object_avoid_front_callback(self,object_avoid):
        self.collide_detect = object_avoid.data
        #print(self.collide_detect)

    def heading_callback(self,heading_msg):
        self.currentHeading = heading_msg.data

    def joy_callback(self, joy_msg):
        # Check if the A button is pressed

        if joy_msg.buttons[8] == 1 or joy_msg.buttons[9] == 1:
            self.stop = False
        else:
            self.stop = True

        if joy_msg.buttons[13] == 1:
            self.ManualControl = True
        # Check if the B button is pressed
        elif joy_msg.buttons[14] == 1:
            self.ManualControl = False

        if self.ManualControl:
            self.linear_velocity = float(joy_msg.axes[1])
            self.angular_velocity = float(joy_msg.axes[0])

    def nav_fix_callback(self, nav_fix_msg):

        current_latitude = nav_fix_msg.latitude
        current_longitude = nav_fix_msg.longitude
        if isnan(current_longitude):
           return
        waypoint = self.waypoints[self.current_waypoint_index]
        waypoint_latitude = waypoint[0]
        waypoint_longitude = waypoint[1]

        distance, bearing = self.calculate_distance_and_bearing(
            current_latitude, current_longitude, waypoint_latitude, waypoint_longitude)
        
        self.goal_bearing = bearing
        # Check if the distance is less than a threshold (e.g., 1 meter)
        if distance < 1.0:
            # Stop the robot
            self.atWaypoint = True
            self.nextState = 'AtWaypoint'
            print('aT waypoint')
            # Move to the next waypoint
            self.current_waypoint_index += 1

            if self.current_waypoint_index == len(self.waypoints):
                rospy.loginfo("Reached all waypoints")
                rospy.signal_shutdown("Reached all waypoints")

    def calculate_distance_and_bearing(self, lat1, lon1, lat2, lon2):
        # Calculate distance and bearing between two GPS coordinates
        # Modify this function according to your desired distance and bearing calculation method

        # Distance calculation (example using Haversine formula)
        R = 6371.0  # Earth's radius in kilometers
        
        lat1_rad = math.radians(lat1)
        lon1_rad = math.radians(lon1)
        lat2_rad = math.radians(lat2)
        lon2_rad = math.radians(lon2)

        dlon = lon2_rad - lon1_rad
        dlat = lat2_rad - lat1_rad

        a = math.sin(dlat / 2)**2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon / 2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

        distance = R * c * 1000.0  # Convert to meters

        # Bearing calculation (example using spherical trigonometry)
        # y = math.sin(lon2_rad - lon1_rad) * math.cos(lat2_rad)
        # x = math.cos(lat1_rad) * math.sin(lat2_rad) - math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(lon2_rad - lon1_rad)
        y = lat2_rad - lat1_rad
        x = lon2_rad - lon1_rad

        bearing = math.atan2(y, x)
        #bearing = math.degrees(bearing)
        print(distance)

        return distance, bearing

    def calculate_angular_velocity(self, desired_heading):
        # Adjust the code below to compute the angular velocity based on desired heading
        # This is just an example, you can implement your own logic
        # Obtain the current heading of the robot (e.g., from IMU or compass)
        current_heading = radians(self.currentHeading)

        # Compute the difference between desired and current headings
        heading_error = desired_heading - current_heading

        # Normalize the heading error to the range [-pi, pi]
        if heading_error > pi:
            heading_error -= 2 * pi
        elif heading_error < -pi:
            heading_error += 2 * pi

        # Set the maximum angular velocity based on the desired heading error
        max_angular_velocity = 0.6  # Adjust as needed

        # Compute the angular velocity proportional to the heading error
        #print(desired_heading)
        #print(current_heading)
        #print(heading_error)
        return max_angular_velocity * heading_error
    

    def main_loop(self):


        # Emergency STOP
        # If the deadmanswitch is not pressed, stop the robot
        # skips all other checks
        if self.stop:
            self.linear_velocity = 0.0
            self.angular_velocity = 0.0
            pub_msg = Twist()
            pub_msg.linear.x = self.linear_velocity
            pub_msg.angular.z = float(self.angular_velocity)
            self.cmd_vel_pub.publish(pub_msg)
            return

        # Update the state
        self.currentState = self.nextState

        if self.ManualControl:
            self.state_pub.publish('ManualControl')
        elif self.currentState == 'Idle':
            self.linear_velocity = 0.0
            self.angular_velocity = 0.0
            self.nextState = 'WaypointFollowing'
        elif self.currentState == "WaypointFollowing":
            #print(self.collide_detect)
            if self.collide_detect:
                #print('avoiding_object')
                self.linear_velocity = 0.0
                self.angular_velocity = 0.5
                self.currentState = 'AvoidiingObject'
                self.nextState = "WaypointFollowing"
            else:
                #print('waypoint_following')
                self.linear_velocity = 0.7
                self.angular_velocity = self.calculate_angular_velocity(self.goal_bearing)
        elif self.currentState == 'AtWaypoint':
            print('waypoint')
            # TODO: implement this state
            self.nextState = 'WaypointFollowing'
            pass
        elif self.currentState == 'FindingCone':
            print('finding cone')
            # TODO: implement this state
            # Spin until cone is found
            # cone detector to run on different node
            # Take photo
            # if cone found nextstate = 'FindingObject'
        elif self.currentState == 'FindingObject':
            print('finding object')
            # TODO: implement this state
            # Spin until object is found
            # if object found nextstate = 'AvoidingObject'
            # Take photo
            # point at cone
            # next heading

        elif self.currentState == 'AvoidingObject':
            print('avoiding object')
            # TODO: implement this state
            # drive around the object until heading is free

        # publishes the drive command
        # THIS SHOULD BE THE ONLY THING THAT PUBLISHES TO THE CMD_VEL TOPIC
        # exception is the emergency stop

        state = String()
        state.data = self.currentState

        self.state_pub.publish(state)

        pub_msg = Twist()
        pub_msg.linear.x = self.linear_velocity
        pub_msg.angular.z = float(self.angular_velocity if self.angular_velocity is not None else 0.0)
        self.cmd_vel_pub.publish(pub_msg)

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.main_loop()
            rate.sleep()


def extract_coordinates_from_kml(kml_file):
    with open(kml_file):
        tree = ET.parse(kml_file)
    root = tree.getroot()
    coordinates = []
    # Find all <coordinates> elements
    for element in root.iter('{http://www.opengis.net/kml/2.2}coordinates'):
        # Extract and split the coordinate values
        coords = element.text.strip().split()
        # Process each coordinate value
        for coord in coords:
            # Split the coordinate string into latitude, longitude, and optional altitude
            parts = coord.split(',')
            # Append the latitude and longitude to the coordinates array
            latitude = float(parts[1])
            longitude = float(parts[0])
            coordinates.append((latitude, longitude))
    return coordinates

def read_gps_positions(file_path):
    positions = []
    with open(file_path, 'r') as csv_file:
        reader = csv.reader(csv_file)
        for row in reader:
            if len(row) >= 2:
                try:
                    latitude = float(row[0])
                    longitude = float(row[1])
                    positions.append([latitude, longitude])
                except ValueError:
                    print("Invalid GPS position:", row)
    return positions


if __name__ == '__main__':
    # waypoints = extract_coordinates_from_kml(
    #     kml_file_path)  # Add your GPS waypoints here
    waypoints = read_gps_positions(
        wp_file_path)  # Add your GPS waypoints here
    waypoints.append(waypoints[0]) # Add the first waypoint to the end of the list
    #waypoints = [[-31.9801990,115.8179322],[-31.9802811,115.8180582], [-31.9803828,115.8174952]]
    rospy.init_node('waypoint_navigation')
    #print("Test")
    navigation = WaypointNavigation(waypoints)
    navigation.run()
