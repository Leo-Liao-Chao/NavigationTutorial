#include <vector>
#include <cmath>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <ilqr/VehicleArray.h> // Replace with your package name
#include <ilqr/Vehicle.h>      // Replace with your package name

std::vector<ilqr::Vehicle> obstacle_vehicles;
ilqr::Vehicle ego_vehicle;

// Callback function that will be called when a new message is received
void obstaclevehicleCallback(const ilqr::VehicleArray::ConstPtr &msg)
{
    obstacle_vehicles = msg->vehicles;
}

void egovehicleCallback(const ilqr::Vehicle::ConstPtr &msg)
{
    ego_vehicle = *msg;
}

bool isCollision(const ilqr::Vehicle &v1, const ilqr::Vehicle &v2)
{
    // Helper lambda to rotate points
    auto rotatePoint = [](double x, double y, double yaw)
    {
        double xr = x * cos(yaw) - y * sin(yaw);
        double yr = x * sin(yaw) + y * cos(yaw);
        return std::make_pair(xr, yr);
    };

    // Get vehicle corners
    auto getCorners = [&](const ilqr::Vehicle &v)
    {
        double half_width = v.width / 2.0;
        double half_length = v.length / 2.0;
        std::vector<std::pair<double, double>> corners;
        corners.push_back(rotatePoint(v.x - half_length, v.y - half_width, v.yaw));
        corners.push_back(rotatePoint(v.x + half_length, v.y - half_width, v.yaw));
        corners.push_back(rotatePoint(v.x + half_length, v.y + half_width, v.yaw));
        corners.push_back(rotatePoint(v.x - half_length, v.y + half_width, v.yaw));
        return corners;
    };

    std::vector<std::pair<double, double>> corners1 = getCorners(v1);
    std::vector<std::pair<double, double>> corners2 = getCorners(v2);

    // Check for separating axis theorem
    auto axes = {v1.yaw, v1.yaw + M_PI_2, v2.yaw, v2.yaw + M_PI_2};

    for (auto axis : axes)
    {
        double min1 = std::numeric_limits<double>::infinity();
        double max1 = -std::numeric_limits<double>::infinity();
        double min2 = std::numeric_limits<double>::infinity();
        double max2 = -std::numeric_limits<double>::infinity();

        for (auto corner : corners1)
        {
            double projection = corner.first * cos(axis) + corner.second * sin(axis);
            min1 = std::min(min1, projection);
            max1 = std::max(max1, projection);
        }

        for (auto corner : corners2)
        {
            double projection = corner.first * cos(axis) + corner.second * sin(axis);
            min2 = std::min(min2, projection);
            max2 = std::max(max2, projection);
        }

        if (max1 < min2 || max2 < min1)
        {
            return false; // No collision
        }
    }

    return true; // Collision detected
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "experiment");
    ros::NodeHandle nh;
    ros::Subscriber sub_obstacle = nh.subscribe("/vehicle_data", 1, obstaclevehicleCallback);
    ros::Subscriber sub_ego = nh.subscribe("/ego_vehicle_data", 1, egovehicleCallback);

    ros::Publisher result_pub = nh.advertise<std_msgs::String>("/experiment_result", 1);

    ros::Rate loop_rate(60);

    while (ros::ok())
    {
        ros::spinOnce();

        for (const auto &obstacle : obstacle_vehicles)
        {
            if (isCollision(ego_vehicle, obstacle))
            {
                std_msgs::String msg;
                std::stringstream ss;
                ss << "Collision detected with obstacle at " << obstacle.x <<" , "<< obstacle.y << std::endl;
                msg.data = ss.str();
                result_pub.publish(msg);
                break;
            }
        }

        loop_rate.sleep();
    }

    return 0;
}
