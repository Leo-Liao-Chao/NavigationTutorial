#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nmpc/VehicleArray.h> // Replace with your package name
#include <nmpc/Vehicle.h> // Replace with your package name

void createVehicleMarker(const std::string& frame_id, int id, double x, double y, double yaw, double length, double width, visualization_msgs::Marker& marker) {
    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();
    marker.ns = "vehicles";
    marker.id = id;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = 0.0;

    tf::Quaternion q;
    q.setRPY(0, 0, yaw);
    tf::quaternionTFToMsg(q, marker.pose.orientation);

    marker.scale.x = length;
    marker.scale.y = width;
    marker.scale.z = 1.0; // Height of the vehicle

    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration(0.0); // Infinite lifetime
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "vehicle_obstacle_publisher");
    ros::NodeHandle nh;
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("vehicle_obstacles", 10);
    ros::Publisher vehicle_data_pub = nh.advertise<nmpc::VehicleArray>("vehicle_data", 10); // Replace with your package name

    ros::Rate rate(10);

    while (ros::ok()) {
        visualization_msgs::MarkerArray marker_array;
        nmpc::VehicleArray vehicle_array; // Replace with your package name

        // Example vehicle data
        std::vector<std::tuple<double, double, double, double, double>> vehicles = {
            {123.32, -306.74, 0, 3.63, 1.840},
            {103.32, -306.74, 0, 3.63, 1.840},
            {193.9, -230.74, -M_PI/2.0, 3.63, 1.840},// No
            {192.9, -190.74, M_PI*4.0/3.0, 3.63, 1.840}, // No
            {189.6, -210.74, M_PI/2.0, 3.63, 1.840}, //No
            {191, -110.6, M_PI*4.0/3.0, 3.63, 1.840}, //No
            {123.4, -105.6, M_PI, 3.63, 1.840},// No
            {103.4, -105.6, M_PI, 3.63, 1.840},// No    
            {83.4, -105.6, M_PI, 3.63, 1.840} // No
        };

        for (size_t i = 0; i < vehicles.size(); ++i) {
            const auto& [x, y, yaw, length, width] = vehicles[i];

            visualization_msgs::Marker marker;
            createVehicleMarker("map", i, x, y, yaw, length, width, marker);
            marker_array.markers.push_back(marker);

            nmpc::Vehicle vehicle; // Replace with your package name
            vehicle.x = x;
            vehicle.y = y;
            vehicle.yaw = yaw;
            vehicle.length = length;
            vehicle.width = width;
            vehicle_array.vehicles.push_back(vehicle);
        }

        marker_pub.publish(marker_array);
        vehicle_data_pub.publish(vehicle_array);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
