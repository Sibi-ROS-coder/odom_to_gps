#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, TwistStamped, Quaternion, Twist, Transform
from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool
from sensor_msgs.msg import NavSatFix, Imu
from tf.transformations import quaternion_matrix, euler_from_quaternion
import message_filters
import math
from pyproj import Proj

global init,x0,y0,z0,init_transformation_matrix,pub;
init=0;x0=0;y0=0;z0=0;
init_transformation_matrix=np.eye(4);
transformation = np.eye(4);
odom = Odometry()
gps = NavSatFix()
pub = Odometry()
imu = Imu()
def longlat_to_utm(long,lat):
    p = Proj(proj='utm', zone=48, ellps='WGS84')
    x,y = p(long,lat)
    return x,y
def callback(odom, gps,imu):
    global init,x0,y0,z0,pub,init_transformation_matrix;
    if init ==0:
        x0,y0 = longlat_to_utm(gps.longitude,gps.latitude)
        z0 = gps.altitude
        init=1;
        init_transformation_matrix = quaternion_matrix([imu.orientation.x,imu.orientation.y,imu.orientation.z,imu.orientation.w])
        init_transformation_matrix[0][3]=x0
        init_transformation_matrix[1][3]=y0
        init_transformation_matrix[2][3]=z0
    elif init==1:
        x,y = longlat_to_utm(gps.longitude,gps.latitude)
        z = gps.altitude
        gps_coordinate = np.array([[x],[y],[z],[1]])
        inverse_init = np.linalg.inv(init_transformation_matrix)
        final_tf = np.dot(inverse_init,gps_coordinate)
        pub.header = imu.header
        pub.pose.pose.position.x = -1*final_tf[1][0]
        pub.pose.pose.position.y = -1*final_tf[0][0]
        pub.pose.pose.position.z = final_tf[2][0]
        pub1.publish(pub)
if __name__ == "__main__":
    rospy.init_node("odom_to_gps") 
    odom_sub = message_filters.Subscriber("/odom", Odometry)
    gnss_sub = message_filters.Subscriber("/sensors/gnss/swift_gnss/navsatfix_best_fix",NavSatFix)
    imu_sub = message_filters.Subscriber("/sensors/imu/front_mid",Imu)
    ts = message_filters.ApproximateTimeSynchronizer([odom_sub, gnss_sub,imu_sub], 10,0.1, allow_headerless=True)
    ts.registerCallback(callback)
    pub1 = rospy.Publisher('/gps_tf',Odometry)
rospy.spin()