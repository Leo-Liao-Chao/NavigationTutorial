#ifndef LOCAL_CILQR_PLANNER_H_
#define LOCAL_CILQR_PLANNER_H_

// abstract class from which our plugin inherits
#include <string>
#include <cmath>
#include <limits>
#include <algorithm>
#include <iterator>
#include <fstream>
#include <iostream>
#include <vector>
#include <stdio.h>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/GetPlan.h>
#include <tf/tf.h>
#include <tf2/utils.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_local_planner.h>
#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>

#include <std_msgs/String.h>

// CILQR

#define EIGEN_USE_MKL_ALL
#include "mkl.h"

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <time.h>
#include <iostream>
#include <chrono>

#include <tf/tf.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int32.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/GridMapRosConverter.hpp>

#include <autoware_msgs/VehicleCmd.h>
#include <autoware_msgs/DetectedObjectArray.h>

#include "carla_msgs/CarlaEgoVehicleControl.h"
#include "ackermann_msgs/AckermannDrive.h"

#include "map_engine/map_param.h"
#include <dynamic_reconfigure/client.h>
#include <map_engine/map_engine_Config.h>

#include "iLQR.h"
#include "Parameters.h"
#include "vehiclepub/VehicleInfo.h"
#include "vehiclepub/VehicleInfoArray.h"
#include "vehiclepub/Experiment.h"

#include "Uncertainty.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

using namespace std;

namespace local_planner{

class LocalPlanner : public nav_core::BaseLocalPlanner{
public:

    LocalPlanner();
    LocalPlanner(std::string name, tf2_ros::Buffer* tf,
                 costmap_2d::Costmap2DROS* costmap_ros);

    ~LocalPlanner();

    void initialize(std::string name, tf2_ros::Buffer* tf,
                    costmap_2d::Costmap2DROS* costmap_ros);

    bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

    bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

    bool isGoalReached();

    // Custom process functions
    void odomCallback(const nav_msgs::Odometry::ConstPtr& odom);
    const double getLinearVelocityX() const;
    const double getLinearVelocityY() const;
    const double getAngularVelocityZ() const;
    double getHeading(geometry_msgs::PoseStamped robotPose);
    geometry_msgs::PoseStamped getNewRobotPose(geometry_msgs::PoseStamped robotPose, double velocityVx, double velocityVy, double velocityWz);
    geometry_msgs::PoseStamped getNewRobotGoal(geometry_msgs::PoseStamped robotPose);
    geometry_msgs::PoseStamped getPoseDifference(geometry_msgs::PoseStamped pose1, geometry_msgs::PoseStamped pose2);
    bool isImpactAroundLocation(geometry_msgs::PoseStamped currentPoseTemp);
    double clamp(double value, double minValue, double maxValue);
    void PathVisualization(MatrixXd &path);
private:
    bool initialized;
    tf2_ros::Buffer* tf;
    costmap_2d::Costmap2DROS* costmap_ros;
    costmap_2d::Costmap2D* costmap;
    geometry_msgs::PoseStamped currentPose;
    geometry_msgs::PoseStamped nearestPlanPose;
    geometry_msgs::PoseStamped errGoalPose;
    vector<geometry_msgs::PoseStamped> globalPlan;
    ros::Subscriber odometry_sub;
    ros::Publisher waypointPub;
    ros::NodeHandle oh;
    
    double currentVx;
    double currentVy;
    double currentWz;

    double sumDistPlan = 0;
    double sumHeading = 0;
    double sumVelocity = 0;

    struct ScoringHelper
    {
        double vx;
        double vy;
        double wz;
        double distPlan;
        double heading;
        double velocity;
        double score;
    };

    // params 
    const double INF                        = std::numeric_limits<double>::infinity();
    const double EPSILON                    = std::numeric_limits<double>::epsilon();
    const double PI                         = 3.141592653589793238463;

    const double STEP_LINEAR_VELOCITY       = 0.01;
    const double STEP_ANGULAR_VELOCITY      = 5/180.0 * PI;
    const double MAX_LINEAR_VELOCITY        = 0.5;
    const double MAX_ANGULAR_VELOCITY       = PI*50.0/180.0;
    const double MAX_LINEAR_ACCELERATION    = 0.5;
    const double MAX_ANGULAR_ACCELERATION   = PI*50.0/180.0;

    const double DT                         = 0.1;

    const double WEIGHT_DISTPLAN            = 0.1;
    const double WEIGHT_HEADING             = 0.4;
    const double WEIGHT_VELOCITY            = 0.0;

    const int N_SAMPLES_TRAJ                = 30;
    const int N_STEPS_AHEAD                 = 30;

    const double FINE_POS_TOLERANCE         = 0.2;
    const double XY_GOAL_TOLERANCE          = 0.01;
    const double YAW_GOAL_TOLERANCE         = 10*PI/180;

    const double ROBOT_LENGTH               = 0.665;
    const double ROBOT_WIDTH                = 0.445;
    bool ROT_STARTED                        = false;

    ros::Time t,oldT;

    ros::NodeHandle nh;
    ros::Publisher debugPub;
    Parameters params;
    iLQR ilqrplanner;
    MatrixXd global_path;
    VectorXd current_ego_vehicle_state;
    VectorXd current_ego_vehicle_state_noise;

    // ros::Subscriber lane_info_sub;
    // ros::Subscriber odom_sub;
    // ros::Subscriber map_sub;
    // ros::Subscriber static_obstacle_sub;
    // ros::Subscriber map_param_sub;
    // ros::Subscriber grid_map_sub;
    ros::Publisher ilqr_path_pub;
    // ros::Publisher vehicle_cmd_pub;
    // ros::Publisher experiment_data_pub;

    // tf2_ros::Buffer tf_buffer;
    // tf2_ros::TransformListener tf_listener;

    // uncertainty params

    // double SIGMA_X;
    // double SIGMA_Y;
    // double SIGMA_THETA;

    // vechicle map params

    // double X_LENGTH;
    // double Y_LENGTH;
    // double X_POSITION;
    // double Y_POSITION;
    // double RESOLUTION;

    // uncertainty map = occupancy map
    // nav_msgs::OccupancyGrid map_msg;
    // double x_center;
    // double y_center;

    // // uncertainty map ->grid map
    // grid_map_msgs::GridMap grid_map_msg;

};
};

#endif