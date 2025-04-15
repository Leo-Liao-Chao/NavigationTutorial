#include "cilqr_planner.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(local_planner::LocalPlanner, nav_core::BaseLocalPlanner)

namespace local_planner
{

  LocalPlanner::LocalPlanner() : costmap_ros(NULL), tf(NULL), initialized(false), nh("~/CILQRPlanner"), ilqrplanner(params) {}

  LocalPlanner::LocalPlanner(std::string name, tf2_ros::Buffer *tf,
                             costmap_2d::Costmap2DROS *costmap_ros)
      : costmap_ros(NULL), tf(NULL), initialized(false), nh("~/CILQRPlanner"), ilqrplanner(params)
  {
    initialize(name, tf, costmap_ros);
  }

  LocalPlanner::~LocalPlanner() {}

  void LocalPlanner::initialize(std::string name, tf2_ros::Buffer *tf,
                                costmap_2d::Costmap2DROS *costmap_ros)
  {
    if (initialized)
    {
      ROS_WARN("CILQR local planner: Already initilized");
      return;
    }
    if (!initialized)
    {
      this->tf = tf;
      this->costmap_ros = costmap_ros;
      this->initialized = true;

      this->costmap = costmap_ros->getCostmap();
      this->costmap_ros->getRobotPose(currentPose);
      this->odometry_sub = oh.subscribe<nav_msgs::Odometry>("odom", 1, boost::bind(&LocalPlanner::odomCallback, this, _1));
      this->ilqr_path_pub = nh.advertise<visualization_msgs::MarkerArray>("ILQR_Path", 10);
      this->debugPub = nh.advertise<std_msgs::String>("debug_info", 1);
    }
  }
  // set global path
  bool LocalPlanner::setPlan(
      const std::vector<geometry_msgs::PoseStamped> &orig_global_plan)
  {
    if (!initialized)
    {
      ROS_ERROR("CILQR planner has not been initialized");
      return false;
    }
    if (initialized)
    {
      globalPlan = orig_global_plan;
      return true;
    }
  }
  // generate control
  bool LocalPlanner::computeVelocityCommands(geometry_msgs::Twist &cmd_vel)
  {
    t = ros::Time::now();
    ros::Duration delta_time = (t - oldT);
    oldT = t;
    if (!initialized)
    {
      ROS_ERROR("CILQR planner has not been initialized");
      return false;
    }
    if (costmap_ros->getRobotPose(currentPose) == false)
    {
      ROS_WARN("CILQR local planner: Failed to get current local pose");
      return false;
    }

    // costmap,currentPose
    costmap = costmap_ros->getCostmap();
    costmap_ros->getRobotPose(currentPose);

    double robotVx = getLinearVelocityX();
    double robotVy = getLinearVelocityY();
    double robotWz = getAngularVelocityZ();
    // get velocity ranges
    double minVx = clamp(robotVx - DT * MAX_LINEAR_ACCELERATION, -MAX_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY);
    double maxVx = clamp(robotVx + DT * MAX_LINEAR_ACCELERATION, -MAX_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY);
    double minVy = clamp(robotVy - DT * MAX_LINEAR_ACCELERATION, -MAX_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY);
    double maxVy = clamp(robotVy + DT * MAX_LINEAR_ACCELERATION, -MAX_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY);
    double minWz = clamp(robotWz - DT * MAX_ANGULAR_ACCELERATION, -MAX_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY);
    double maxWz = clamp(robotWz + DT * MAX_ANGULAR_ACCELERATION, -MAX_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY);

    // 没有到达终点

    errGoalPose = getPoseDifference(globalPlan.back(), currentPose);
    if (hypot(errGoalPose.pose.position.x, errGoalPose.pose.position.y) > XY_GOAL_TOLERANCE)
    {
      // global path convert
      {
        global_path.resize(2, globalPlan.size());
        for (int i = 0; i < globalPlan.size(); i++)
        {
          // ROS_INFO("Received odometry message: Position(x=%f, y=%f, z=%f), Orientation(x=%f, y=%f, z=%f, w=%f)",
          //          msg->poses[i].pose.position.x, msg->poses[i].pose.position.y, msg->poses[i].pose.position.z,
          //          msg->poses[i].pose.orientation.x, msg->poses[i].pose.orientation.y, msg->poses[i].pose.orientation.w);
          // ROS_INFO("Received odometry message: Position(x=%f, y=%f, z=%f)",
          //          msg->poses[i].pose.position.x, msg->poses[i].pose.position.y, msg->poses[i].pose.position.z);
          global_path(0, i) = globalPlan[i].pose.position.x;
          global_path(1, i) = globalPlan[i].pose.position.y;
        }
        ilqrplanner.set_global_plan(global_path);
      }
      //  MAP convert
      {
        // ilqrplanner.set_uncertainty_map(map);
      }
      // State convert
      {
        current_ego_vehicle_state = Eigen::VectorXd::Zero(4);
        // current_ego_vehicle_state << current_state_x, current_state_y, current_state_v, current_state_yaw;
        current_ego_vehicle_state << currentPose.pose.position.x, currentPose.pose.position.y, sqrt(currentVx * currentVx + currentVy * currentVy), getHeading(currentPose);
      }

      auto start_time = std::chrono::high_resolution_clock::now();
      ilqrplanner.run_step(current_ego_vehicle_state);
      auto end_time = std::chrono::high_resolution_clock::now();
      std::chrono::duration<double> duration = end_time - start_time;
      double planning_time = duration.count();

      std::stringstream ss;
      ss << "Planning finished, use time " << planning_time << " s";
      std_msgs::String msg;
      msg.data = ss.str();
      debugPub.publish(msg);

      PathVisualization(ilqrplanner.X_result);
      // PathVisualization(ilqrplanner.ref_traj_result);
      // publishVehicleCmd(current_state_v, current_state_yaw, ilqrplanner.U_result(0, 1), ilqrplanner.U_result(1, 1));
      // publishVehicleCmd(current_state_v, current_state_yaw, ilqrplanner.U_result(0, 0), ilqrplanner.U_result(1, 0));
      // publishExperimentData(begin_time,current_ego_vehicle_state,planning_time,ilqrplanner.X_result,ilqrplanner.U_result);
      // std::cout<<"compute reference path in vehicle axis"<<std::endl;
      // std::cout<<ilqrplanner.ref_traj_result<<std::endl;

      double v = current_ego_vehicle_state(2);
      double a = ilqrplanner.U_result(0, 0);
      double angle_velocity = ilqrplanner.U_result(1, 0);
      double speed = v + a * ilqrplanner.params.timestep;
      double angle = current_ego_vehicle_state(3) + angle_velocity * ilqrplanner.params.timestep;

      //
      cmd_vel.linear.x = clamp(speed, -MAX_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY);
      // cmd_vel.linear.x = clamp(speed*std::cos(angle), -MAX_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY);
      // cmd_vel.linear.y = clamp(speed*std::sin(angle), -MAX_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY);
      cmd_vel.linear.z = 0;
      cmd_vel.angular.x = 0;
      cmd_vel.angular.y = 0;
      cmd_vel.angular.z = clamp(-angle_velocity, -MAX_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY);
      // cmd_vel.angular.z = clamp(angle, -MAX_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY);
    }
    else
    {
      // 到达终点。
    }

    // 发布调试信息
    std_msgs::String msg;
    msg.data = "Final positioning or rotation. Velocities: linear.x=" + std::to_string(cmd_vel.linear.x) +
               ", linear.y=" + std::to_string(cmd_vel.linear.y) +
               ", angular.z=" + std::to_string(cmd_vel.angular.z);
    debugPub.publish(msg);
    return true;
  }

  bool LocalPlanner::isGoalReached()
  {
    if (!initialized)
    {
      ROS_ERROR("CILQR planner has not been initialized");
      return false;
    }
    if (costmap_ros->getRobotPose(currentPose) == false)
    {
      ROS_WARN("CILQR local planner: Failed to get current local pose");
      return false;
    }
    // 算距离，距离小于容忍值，就return true
    errGoalPose = getPoseDifference(globalPlan.back(), currentPose);
    if (hypot(errGoalPose.pose.position.x, errGoalPose.pose.position.y) <= XY_GOAL_TOLERANCE)
    {
      std_msgs::String msg;
      msg.data = "Goal reach.";
      debugPub.publish(msg); // 发布调试信息
      return true;
    }
    return false;
  }

  void LocalPlanner::odomCallback(const nav_msgs::Odometry::ConstPtr &odom)
  {
    currentVx = odom->twist.twist.linear.x;
    currentVy = odom->twist.twist.linear.y;
    currentWz = odom->twist.twist.angular.z;
  }

  // get velocities
  const double LocalPlanner::getLinearVelocityX() const
  {
    return this->currentVx;
  }

  const double LocalPlanner::getLinearVelocityY() const
  {
    return this->currentVy;
  }

  const double LocalPlanner::getAngularVelocityZ() const
  {
    return this->currentWz;
  }
  // // °
  // double LocalPlanner::getHeading(geometry_msgs::PoseStamped robotPose)
  // {
  // double angleToGoal = atan2(nearestPlanPose.pose.position.y - robotPose.pose.position.y, nearestPlanPose.pose.position.x - robotPose.pose.position.x );
  // angleToGoal = angles::normalize_angle((angles::normalize_angle_positive(angleToGoal) - angles::normalize_angle_positive(tf::getYaw(robotPose.pose.orientation))));
  // return 180 -  abs(angleToGoal)/PI * 180;
  // }
  // rad
  double LocalPlanner::getHeading(geometry_msgs::PoseStamped robotPose)
  {
    double angleToGoal = atan2(nearestPlanPose.pose.position.y - robotPose.pose.position.y, nearestPlanPose.pose.position.x - robotPose.pose.position.x);
    angleToGoal = angles::normalize_angle((angles::normalize_angle_positive(angleToGoal) - angles::normalize_angle_positive(tf::getYaw(robotPose.pose.orientation))));
    return angleToGoal;
  }

  geometry_msgs::PoseStamped LocalPlanner::getNewRobotPose(geometry_msgs::PoseStamped robotPose, double velocityVx, double velocityVy, double velocityWz)
  {
    geometry_msgs::PoseStamped newRobotPose;
    double robotYaw = tf::getYaw(robotPose.pose.orientation);
    newRobotPose.pose.position.x = robotPose.pose.position.x + (velocityVx * cos(robotYaw) - velocityVy * sin(robotYaw)) * DT;
    newRobotPose.pose.position.y = robotPose.pose.position.y + (velocityVx * sin(robotYaw) + velocityVy * cos(robotYaw)) * DT;
    newRobotPose.pose.position.z = 0;
    double newRobotYaw = robotYaw + velocityWz * DT;
    newRobotPose.pose.orientation = tf::createQuaternionMsgFromYaw(newRobotYaw);
    return newRobotPose;
  }

  geometry_msgs::PoseStamped LocalPlanner::getNewRobotGoal(geometry_msgs::PoseStamped robotPose)
  {
    int closestPointInPath = 0;
    double shortestDistance = INF;

    for (int i = 0; i < globalPlan.size(); i++)
    {
      double dx = globalPlan[i].pose.position.x - robotPose.pose.position.x;
      double dy = globalPlan[i].pose.position.y - robotPose.pose.position.y;
      double newDistance = sqrt(dx * dx + dy * dy);
      if (newDistance < shortestDistance)
      {
        shortestDistance = newDistance;
        closestPointInPath = i;
      }
    }
    if (closestPointInPath + N_STEPS_AHEAD > globalPlan.size() - 1)
    {
      return globalPlan.back();
    }
    return globalPlan[closestPointInPath + N_STEPS_AHEAD];
  }

  geometry_msgs::PoseStamped LocalPlanner::getPoseDifference(geometry_msgs::PoseStamped pose1, geometry_msgs::PoseStamped pose2)
  {
    geometry_msgs::PoseStamped diffPose;
    diffPose.pose.position.x = pose1.pose.position.x - pose2.pose.position.x;
    diffPose.pose.position.y = pose1.pose.position.y - pose2.pose.position.y;
    double tempYaw = tf::getYaw(pose1.pose.orientation) - tf::getYaw(pose2.pose.orientation);
    diffPose.pose.orientation = tf::createQuaternionMsgFromYaw(tempYaw);
    return diffPose;
  }
  // 碰撞检测
  bool LocalPlanner::isImpactAroundLocation(geometry_msgs::PoseStamped currentPoseTemp)
  {
    unsigned int x, y, indX, indY;
    double yaw = tf::getYaw(currentPoseTemp.pose.orientation);
    costmap->worldToMap(currentPoseTemp.pose.position.x, currentPoseTemp.pose.position.y, x, y);

    double resolution = costmap->getResolution();

    for (int i = -(int)(ROBOT_LENGTH / resolution); i <= (int)(ROBOT_LENGTH / resolution); i++)
    {
      for (int j = -(int)(ROBOT_WIDTH / resolution); j <= (int)(ROBOT_WIDTH / resolution); j++)
      {
        indX = (unsigned int)clamp((x + (i * cos(yaw) - j * sin(yaw))), 0, costmap->getSizeInCellsX());
        indY = (unsigned int)clamp((y + (i * sin(yaw) + j * cos(yaw))), 0, costmap->getSizeInCellsY());
        if (costmap->getCost(indX, indY) != 0)
        {
          return true;
        }
      }
    }
    return false;
  }

  double LocalPlanner::clamp(double value, double minValue, double maxValue)
  {
    if (value < minValue)
      return minValue;
    if (value > maxValue)
      return maxValue;
    return value;
  }

  void LocalPlanner::PathVisualization(MatrixXd &path)
  {

    visualization_msgs::MarkerArray path_markerarray;

    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;  // 矩形的宽度
    marker.scale.y = 0.1;  // 矩形的长度
    marker.scale.z = 0.01; // 矩形的高度
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.lifetime = ros::Duration(1.0); // 显示时间为1秒
    for (int i = 0; i < path.cols(); i++)
    {
      marker.header.stamp = ros::Time::now();
      marker.id = i;
      marker.pose.position.x = path(0, i);
      marker.pose.position.y = path(1, i);
      marker.pose.position.z = 0.0;
      path_markerarray.markers.emplace_back(marker);
    }
    this->ilqr_path_pub.publish(path_markerarray);
  }
}