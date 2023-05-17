#!/usr/bin/env python3
import rospy 
from sensor_msgs.msg import Joy, LaserScan, NavSatFix
from geometry_msgs.msg import Twist, Vector3
from math import sin, cos, sqrt, atan2, radians, pi

# Here we will have the main control flow of the robot.
# This node takes in all the appropriate sensor data and subscribes to the approptiate topics to make informed decisions
# This node then publishes the cmd_vel topic to the robot to make it move.

kml_file_path = 'Pioneer-coordinates.kml'



class WaypointNavigation:
    def __init__(self, waypoints):
        rospy.init_node('waypoint_navigation')
        self.waypoints = waypoints
        self.current_waypoint_index = 0
        self.currentState = 'Idle'
        self.nextState = 'Idle'
        self.atWaypoint = False
        self.stop = True
        self.currentHeading = 0.0
        self.imu_sub = rospy.Subscriber('/imu', Imu, self.imu_callback)
        self.nav_fix_sub = rospy.Subscriber('/fix', NavSatFix, self.nav_fix_callback)
        self.joy_sub = rospy.Subscriber('/joy', Joy, self.joy_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
    
    def joy_callback(self, joy_msg):
        # Check if the A button is pressed
        if joy_msg.buttons[13] == 1:
            self.nextState = 'ManualControl'
        # Check if the B button is pressed
        elif joy_msg.buttons[14] == 1:
            self.nextState = 'Idle'

        if joy_msg.buttons[8] == 1 or joy_msg.buttons[9] == 1 or self.currentState == 'ManualControl':
            self.stop = False
        else:
            self.stop = True

        if self.currentState == 'ManualControl':
            self.linear_velocity = joy_msg.axes[1]
            self.angular_velocity = joy_msg.axes[0]
    
    def imu_callback(self, imu_msg):
        #TODO: implement this function
        # Get the current heading from the IMU
        # Store in self.currentHeading
        return

    def nav_fix_callback(self, nav_fix_msg):
        current_latitude = nav_fix_msg.latitude
        current_longitude = nav_fix_msg.longitude
        
        waypoint = self.waypoints[self.current_waypoint_index]
        waypoint_latitude = waypoint[0]
        waypoint_longitude = waypoint[1]
        
        distance = self.calculate_distance(current_latitude, current_longitude, waypoint_latitude, waypoint_longitude)
        
        # Check if the distance is less than a threshold (e.g., 1 meter)
        if distance < 1.0:
            # Stop the robot
            self.atWaypoint = True
            self.nextState = 'AtWaypoint'
            # cmd_vel_msg = Twist()
            # cmd_vel_msg.linear.x = 0.0
            # cmd_vel_msg.angular.z = 0.0
            #self.cmd_vel_pub.publish(cmd_vel_msg)
            
            # Move to the next waypoint
            self.current_waypoint_index += 1
            
            if self.current_waypoint_index == len(self.waypoints):
                rospy.loginfo("Reached all waypoints")
                rospy.signal_shutdown("Reached all waypoints")
        else:
            # Compute the desired heading towards the current waypoint
            desired_heading = self.calculate_heading(current_latitude, current_longitude, waypoint_latitude, waypoint_longitude)
            
            # Turn the robot towards the desired heading
            #cmd_vel_msg = Twist()
            self.linear_velocity = 0.9  # Forward linear velocity (adjust as needed)
            self.angular_velocity = self.calculate_angular_velocity(desired_heading)
            #self.cmd_vel_pub.publish(cmd_vel_msg)

    def calculate_distance(self, lat1, lon1, lat2, lon2):
        # approximate radius of earth in km
        R = 6371.0

        lat1 = radians(lat1)
        lon1 = radians(lon1)
        lat2 = radians(lat2)
        lon2 = radians(lon2)

        dlon = lon2 - lon1
        dlat = lat2 - lat1

        a = sin(dlat / 2)**2 + cos(lat1) * cos(lat2) * sin(dlon / 2)**2
        c = 2 * atan2(sqrt(a), sqrt(1 - a))

        distance = R * c
        return distance * 1000  # Convert to meters

    def calculate_heading(self, lat1, lon1, lat2, lon2):
        lat1 = radians(lat1)
        lon1 = radians(lon1)
        lat2 = radians(lat2)
        lon2 = radians(lon2)

        dlon = lon2 - lon1

        y = sin(dlon) * cos(lat2)
        x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dlon)

        heading = atan2(y, x)
        return heading

    def calculate_angular_velocity(self, desired_heading):
        # Adjust the code below to compute the angular velocity based on desired heading
        # This is just an example, you can implement your own logic
        current_heading = self.currentHeading  # Obtain the current heading of the robot (e.g., from IMU or compass)

        # Compute the difference between desired and current headings
        heading_error = desired_heading - current_heading

        # Normalize the heading error to the range [-pi, pi]
        if heading_error > pi:
            heading_error -= 2 * pi
        elif heading_error < -pi:
            heading_error += 2 * pi

        # Set the maximum angular velocity based on the desired heading error
        max_angular_velocity = 1.0  # Adjust as needed

        # Compute the angular velocity proportional to the heading error
        angular_velocity = max_angular_velocity * heading_error

        # Create and publish the Twist message with the computed angular velocity
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = 0.2  # Forward linear velocity (adjust as needed)
        cmd_vel_msg.angular.z = angular_velocity
        self.cmd_vel_pub.publish(cmd_vel_msg)

    def main_loop(self):

        # Emergency STOP
        # If the deadmanswitch is not pressed, stop the robot
        # skips all other checks
        if self.stop:
            self.linear_velocity == 0.0
            self.angular_velocity == 0.0
            pub_msg = Twist()
            pub_msg.linear.x = self.linear_velocity
            pub_msg.angular.z = self.angular_velocity
            self.cmd_vel_pub.publish(pub_msg)
            return
        
        # Update the state
        self.currentState = self.nextState

        if self.currentState == 'Idle':
            self.linear_velocity == 0.2
            self.angular_velocity == 0.0
        elif self.currentState == 'ManualControl':
            pass
        elif self.currentState == 'WaypointFollowing':
            self.waypoint_following()
        elif self.currentState == 'AtWaypoint':
            print('waypoint')
            #TODO: implement this state
            #Confirm at waypoint by finding cone
        elif self.currentState == 'FindingCone':
            print('finding cone')
            #TODO: implement this state
            #Spin until cone is found
            #cone detector to run on different node
            #Take photo
            #if cone found nextstate = 'FindingObject'
        elif self.currentState == 'FindingObject':
            print('finding object')
            #TODO: implement this state
            #Spin until object is found
            #if object found nextstate = 'AvoidingObject'
            #Take photo
            #point at cone
            #next heading
            
        elif self.currentState == 'AvoidingObject':
            print('avoiding object')
            #TODO: implement this state
            #drive around the object until heading is free 


        #publishes the drive command
        # THIS SHOULD BE THE ONLY THING THAT PUBLISHES TO THE CMD_VEL TOPIC
        #exception is the emergency stop
        pub_msg = Twist()
        pub_msg.linear.x = self.linear_velocity
        pub_msg.angular.z = self.angular_velocity
        self.cmd_vel_pub.publish(pub_msg)

    def run(self):
        while not rospy.is_shutdown():
            self.main_loop()
            rate.sleep()


import xml.etree.ElementTree as ET

def extract_coordinates_from_kml(kml_file):
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


if __name__ == '__main__':
    waypoints = extract_coordinates_from_kml(kml_file_path) # Add your GPS waypoints here
    navigation = WaypointNavigation(waypoints)
    rate = rospy.Rate(10)
    navigation.run()





