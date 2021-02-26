#!/usr/bin/env python
import rospy, numpy as np,math,matplotlib.pyplot as plt
# from ParametersInitialization import *
# from EKF_estimation_algorithm import *
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose, Point, Quaternion
from ackermann_msgs.msg import AckermannDriveStamped,AckermannDrive
from tf.transformations import quaternion_from_euler, euler_from_quaternion
#  globaling variables
global new_msg1, new_msg2,EKF,fpositionx,fpositiony,fnewxhat1,fnewyhat1,yaw,fnewthetahat1;
mp = np.array([[1,0,0],[0,1,0],[0,0,0.3]]);
fpositionx = list();fpositiony = list();yaw=list();fnewxhat1 = [];fnewyhat1 = [];fnewthetahat1=[]
# Plotting Variables
def plot_x(fpositionx,fpositiony,fnewxhat1,fnewyhat1):
    global counter
    if counter % 10 == 0:
        plt.subplot(1,2,1)
        plt.plot(fpositionx,fpositiony,'b')
        plt.title('odom')
        plt.subplot(1,2,2)
        plt.plot(fnewxhat1,fnewyhat1,'r')
        plt.title('gps_by_tf')
        plt.draw()
        plt.pause(0.000000001)

    counter += 1
# declaring the data types
new_msg1 = Odometry()
new_msg2 = Odometry()
EKF = Odometry()
EKF_estimated_Pose = Odometry()

# subscriber call back function
def robot_to_map_callback(msg):
    global new_msg1
    new_msg1 = msg
def cmd_vel_callback(msg):
    global new_msg2
    #print(msg)
    new_msg2 = msg

#initializing node and subscribing and publishing
rospy.init_node('gps')
sub1 = rospy.Subscriber('odom',Odometry, robot_to_map_callback,queue_size=10)
sub2 = rospy.Subscriber('gps_tf',Odometry, cmd_vel_callback,queue_size=1)


rate = rospy.Rate(100)
while not rospy.is_shutdown():
    counter = 0
    fnewxhat = new_msg2.pose.pose.position.x;
    fnewyhat = new_msg2.pose.pose.position.y;
        
    ##plotting data
    fpositionx.append(new_msg1.pose.pose.position.x);
    fpositiony.append(new_msg1.pose.pose.position.y);
    
    fnewxhat1.append(fnewxhat)
    fnewyhat1.append(fnewyhat)
    fnewthetahat1.append(fnewthetahat1)
    plot_x(fpositionx,fpositiony,fnewxhat1,fnewyhat1)
    plt.ion()
    rate.sleep()
rospy.spin()