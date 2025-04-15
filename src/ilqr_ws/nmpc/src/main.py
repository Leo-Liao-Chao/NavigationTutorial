#!/usr/bin/python2.7
# -*- coding: utf-8 -*-


import rospy
from std_msgs.msg import String
from vehiclepub.msg import VehicleInfo, VehicleInfoArray 
from vehiclepub.msg  import Experiment
from nmpc_all import nmpc
from params import Params
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
import time
import random

from nav_msgs.msg import Path, Odometry

import tf2_ros
import geometry_msgs.msg
from geometry_msgs.msg import TransformStamped, Pose
import tf2_geometry_msgs
import tf.transformations
from tf.transformations import euler_from_quaternion
from ackermann_msgs.msg import AckermannDrive




global_path = []
obstacles = []

def publish_nmpc_path(X):
    """
    Publishes a MarkerArray representing the NMPC path.

    Parameters:
    - X: A 2D numpy array where each column represents a waypoint [x, y, ...].
    - path_pub: A ROS publisher to publish the MarkerArray.
    - frame_id: The coordinate frame ID (default is "map").
    - namespace: The namespace for the markers (default is "nmpc_path").
    """
    global_path_pub = rospy.Publisher("nmpc_path", MarkerArray, queue_size=10)
    
    marker_array = MarkerArray()

    for i in range(X.shape[1]):  
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "nmpc_path"
        marker.id = i
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        # Set marker's position and other attributes
        marker.pose.position.x = X[0, i]
        marker.pose.position.y = X[1, i]
        marker.pose.position.z = 0.0
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.01

        # Set marker color
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.lifetime = rospy.Duration(1.0)
        
        # Add the marker to the MarkerArray
        marker_array.markers.append(marker)

    # Publish the MarkerArray message
    global_path_pub.publish(marker_array)

def publish_reference_path(X):
    """
    Publishes a MarkerArray representing the NMPC path.

    Parameters:
    - X: A 2D numpy array where each column represents a waypoint [x, y, ...].
    - path_pub: A ROS publisher to publish the MarkerArray.
    - frame_id: The coordinate frame ID (default is "map").
    - namespace: The namespace for the markers (default is "nmpc_path").
    """
    reference_path_pub = rospy.Publisher("reference_path", MarkerArray, queue_size=10)
    
    marker_array = MarkerArray()

    for i in range(X.shape[0]):  
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "nmpc_path"
        marker.id = i
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        # Set marker's position and other attributes
        marker.pose.position.x = X[i,0]
        marker.pose.position.y = X[i,1]
        marker.pose.position.z = 0.0
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.01

        # Set marker color
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.lifetime = rospy.Duration(1.0)
        
        # Add the marker to the MarkerArray
        marker_array.markers.append(marker)

    # Publish the MarkerArray message
    reference_path_pub.publish(marker_array)

def publish_nmpc_cmd(speed, accel, angle_velocity):
    """
    Publishes an AckermannDrive command with the given parameters.

    Parameters:
    - speed: The current speed of the vehicle (m/s).
    - angle: The current steering angle (rad).
    - accel: The acceleration to be applied (m/s^2).
    - angle_velocity: The steering angle velocity (rad/s).
    - vehicle_cmd_pub: The ROS publisher for AckermannDrive messages.
    """
    vehicle_cmd_pub = rospy.Publisher("/carla/ego_vehicle/ackermann_cmd", AckermannDrive, queue_size=10)
    
    vehicle_cmd = AckermannDrive()
    
    vehicle_cmd.steering_angle = angle_velocity  # rad
    vehicle_cmd.steering_angle_velocity = 0      # rad/s
    vehicle_cmd.speed = speed + accel            # m/s
    vehicle_cmd.acceleration = 0                 # m/s^2
    vehicle_cmd.jerk = 0                         # m/s^3
    
    vehicle_cmd_pub.publish(vehicle_cmd)

def publish_experiment_data(start_time, start_pos, planning_time, X, U):
    experiment_data_pub = rospy.Publisher("experiment", Experiment, queue_size=10)
    
    # 创建消息对象
    msg = Experiment()
    msg.start_time = start_time
    msg.start_pos = start_pos
    msg.planning_time = planning_time

    # 将X（4×n）和U（2×n）展平成一维列表，并赋值给消息字段
    # 将X按列优先顺序展开为一维列表
    msg.X = [X[row][col] for col in range(len(X[0])) for row in range(len(X))]

    # 将U按列优先顺序展开为一维列表
    msg.U = [U[row][col] for col in range(len(U[0])) for row in range(len(U))]

    # 发布消息
    experiment_data_pub.publish(msg)

def odomCallback(odom_msg):
    global current_ego_vehicle_state, ilqrplanner, global_path

    current_state_x = 0.0
    current_state_y = 0.0
    current_state_v = 0.0
    current_state_yaw = 0.0

    # Calculate the current velocity
    current_state_v = np.sqrt(odom_msg.twist.twist.linear.x ** 2 +
                              odom_msg.twist.twist.linear.y ** 2 +
                              odom_msg.twist.twist.linear.z ** 2)

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    try:
        transform_stamped = tf_buffer.lookup_transform("map", odom_msg.header.frame_id, rospy.Time(0))
    except tf2_ros.TransformException as ex:
        rospy.logwarn("%s", ex)
        return

    # Create a PoseStamped object
    pose_stamped = geometry_msgs.msg.PoseStamped()
    pose_stamped.header = odom_msg.header
    pose_stamped.pose = odom_msg.pose.pose

    # Perform the transformation
    pose_in_map = tf2_geometry_msgs.do_transform_pose(pose_stamped, transform_stamped)
    # Current XY of the robot (map frame)
    current_state_x = pose_in_map.pose.position.x
    current_state_y = pose_in_map.pose.position.y

    # Convert quaternion to roll, pitch, and yaw
    quaternion = (pose_in_map.pose.orientation.x,
                  pose_in_map.pose.orientation.y,
                  pose_in_map.pose.orientation.z,
                  pose_in_map.pose.orientation.w)
    roll, pitch, current_state_yaw = euler_from_quaternion(quaternion)

    current_ego_vehicle_state = np.array([current_state_x, current_state_y, current_state_v, current_state_yaw])

    # Planning parameters
    global global_path, obstacles
    x0 = [current_state_x,current_state_y,current_state_v,current_state_yaw]
    params = Params()

    vehicle_sigma_x = rospy.get_param('vehicle_sigma_x', 0.0)
    vehicle_sigma_y = rospy.get_param('vehicle_sigma_y', 0.0)
    vehicle_sigma_theta = rospy.get_param('vehicle_sigma_theta', 0.0)

    obstacle_sigma_x = rospy.get_param('obstacle_sigma_x', 0.0)
    obstacle_sigma_y = rospy.get_param('obstacle_sigma_y', 0.0)
    obstacle_sigma_theta = rospy.get_param('obstacle_sigma_theta', 0.0)

    safe_length = rospy.get_param('safe_length', 0.0)
    safe_width = rospy.get_param('safe_width', 0.0)

    params.safe_prolong_length = safe_length
    params.safe_prolong_width = safe_width

    print(vehicle_sigma_x,vehicle_sigma_y,vehicle_sigma_theta)

    delta_x = np.random.normal(0,vehicle_sigma_x)
    delta_y = np.random.normal(0,vehicle_sigma_y)
    delta_theta = np.random.normal(0,vehicle_sigma_theta)

    x0_noise = [current_state_x + delta_x,current_state_y + delta_y,current_state_v,current_state_yaw + delta_theta]

    params.sigma_i_2 = vehicle_sigma_x**2
    params.sigma_o_2 = obstacle_sigma_x**2

    begin_time = rospy.Time.now()
    start_time = time.time()
    U,X,local_ref_path = nmpc(x0_noise, global_path,obstacles,params)
    end_time = time.time()
    execution_time = end_time - start_time

    print(execution_time)

    publish_nmpc_path(X)
    publish_reference_path(local_ref_path)
    publish_nmpc_cmd(current_state_v,U[0,1],U[1,1])
    publish_experiment_data(begin_time,x0,execution_time,X,U)

def laneInfoCallback(data):
    global global_path
    global_path_receive = np.zeros((len(data.poses),2))  # 初始化global_path
    for i in range(len(data.poses)):
        global_path_receive[i,0] = data.poses[i].pose.position.x 
        global_path_receive[i,1] = data.poses[i].pose.position.y 
    global_path = global_path_receive

    # print("global_path_pub.shape[1]", len(global_path))
    # print("global_path_pub.shape[0]", len(global_path[0]))
  
def obstaclecallback(data):
    global obstacles
    obstacles[:] = []
    
    # 将新的车辆数据添加到全局变量中
    for vehicle in data.vehicles:
        x = vehicle.pose.position.x
        y = -vehicle.pose.position.y
        yaw = -vehicle.pose.orientation.z
        length = vehicle.size.x  # Assuming 'length' is stored in the 'x' attribute
        width = vehicle.size.y   # Assuming 'width' is stored in the 'y' attribute
        
        obstacles.append([x, y, yaw, length, width])
    # print(len(obstacles))
def simple_node():
    # 初始化ROS节点
    
    rospy.init_node('nmpc', anonymous=True)
    rospy.Subscriber("/carla/ego_vehicle/waypoints", Path, laneInfoCallback)
    rospy.Subscriber("/carla/ego_vehicle/odometry", Odometry, odomCallback)
    rospy.Subscriber("/static_obstacle/vehicle_info", VehicleInfoArray, obstaclecallback)

    # # 节点的主要循环
    # rospy.spin()
    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    try:
        simple_node()
    except rospy.ROSInterruptException:
        pass

