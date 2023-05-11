# AUTO4508Project
 This repo is for group project using Pioneer 3-AT Outdoor Mobile Robot Platform

# Equipment
● Pioneer 3-AT Outdoor Mobile Robot Platform    
● Industrial Linux PC with touch screen display    
● GPS: built-in    
● IMU: Phidget Spatial 3/3/3    
● Camera: Stereo Camera OAK-D V2    
● Lidar: TBA    
● Software: Ubuntu, ARIA or ROS    

# Tasks to complete 1/3
● Each team will be given a number of GPS waypoints that
the robot has to visit, before returning to its starting position.    
● Whenever a waypoint has been reached (within reasonable
accuracy), the robot must take a photo of the marker object at
the waypoint (e.g. an orange cone) and then head towards the
next waypoint. Always leave this marker to the robot’s right
side.    
● At each waypoint, an additional object of interest, e.g. a
traffic sign or a large coloured bucket, will be displayed at a
fixed height, but at an undetermined distance. Identify the
object of interest and calculate its distance from the waypoint
marker.    
● Upon completion of the waypoint course, print all marker
photos and object distance measurements on the screen.     

# Tasks to complete 2/3
● Record the robot’s driving path and display it graphically on the
robot’s display with all detected markers, objects of interest and
any obstacles.   
● Implement a user interface with graphics and text on the robot’s
display that always displays the robot’s internal status and intended
actions.   
● At all times, avoid collisions with markers, objects of interest and
any other stationary or moving obstacles, such as walls, vehicles,
people, bikes, etc.    
● For safety reasons, implement a Bluetooth link between the robot’s
on-board PC and a gamepad controller:
Button ‘A’ enable automated mode. In automated mode, use the back pedals
as a dead-man switch. If released, the robot has to stop.
Button ‘B’ enable manual mode (disable automated mode). In manual mode,
the steering controls can be used to manually drive the robot
forward/backward and left/right.   

# Tasks to complete 3/3
Project design report (pdf), which includes    
● Report on which team member did what    
● Software design description    
● Diagrams, photos, screenshots, plots, etc.   
● Include page numbers   
● Max 10 pages plus 1 Title page   
Do NOT include Program code, Table of contents, Half-empty pages, etc.
User Manual (pdf)   
● Max 5 pages, no Title page   
● As if it was sold to a customer   
Source code, via email to lab demonstrator, clearly marking any imported code
with referencing the source.    

# Build and run
'''
$ git clone git@github.com:Xiangrui-Kong-uwa/AUTO4508Project.git
$ cd AUTO4508Project
$ source /opt/ros/noetic/setup.bash
$ catkin_make -j2
$ source devel/setup.bash
$ roslaunch path_following path_following.launch
'''
