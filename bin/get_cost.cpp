#include <iostream>
#include <Eigen/Geometry>
#include <gz/transport/Node.hh>
#include <gz/msgs.hh>
#include "octo.hpp"
#include <pcl/visualization/pcl_visualizer.h>
Eigen::Vector3d position;
Eigen::Quaterniond orientation;


void imuCallback(const gz::msgs::IMU &imu_msg) {
    //std::cout << "IMU";

    orientation.x() = imu_msg.orientation().x();
    orientation.y() = imu_msg.orientation().y();
    orientation.z() = imu_msg.orientation().z();
    orientation.w() = imu_msg.orientation().w();
}

void odometryCallback(const gz::msgs::Odometry &odom) {
    //td::cout << "Odometry";

    position.x() = odom.pose().position().x();
    position.y() = odom.pose().position().y();
    position.z() = odom.pose().position().z();
}

void calculate_octomap(const gz::msgs::PointCloudPacked &packed_msg) {

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    int point_step = packed_msg.point_step();
    //std::cout << "Point step: " << point_step << std::endl;
    int num_points = packed_msg.data().size() / point_step;


    cloud->width = num_points;
    cloud->height = 1;
    cloud->is_dense = false;
    cloud->points.resize(cloud->width * cloud->height);

    /*for (int i = 0; i < packed_msg.field_size(); ++i) {
        const auto& field = packed_msg.field(i);
        std::cout << "Field " << i << ": name = " << field.name() 
                << ", offset = " << field.offset() << std::endl;
    }*/

    for (int i = 0; i < num_points; ++i) {
    
        const char* point_data = packed_msg.data().c_str() + i * point_step;    
        const float* point_fields = reinterpret_cast<const float*>(point_data);

        //std::cout << "Raw float values: " << point_fields[0] << ", " << point_fields[1] << ", " << point_fields[2] << std::endl;

        cloud->points[i].x = point_fields[0];
        cloud->points[i].y = point_fields[1];
        cloud->points[i].z = point_fields[2];
    }

    /*
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));

    viewer->setBackgroundColor(0, 0, 0);

    viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");

    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");

    viewer->addCoordinateSystem(1.0);

    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
    }*/

    Eigen::Vector3d direction = orientation * Eigen::Vector3d::UnitX();
    float cost = pointCloudCallback(cloud, position, direction , orientation);
    std::cout << "Cost: " << cost << std::endl;
}

int main() {
    gz::transport::Node node;
    node.Subscribe("/world/default/model/rover/link/base_link/sensor/imu_sensor/imu", imuCallback);
    node.Subscribe("/model/rover/odometry", odometryCallback);
    node.Subscribe("/depth_camera/points", calculate_octomap);
    gz::transport::waitForShutdown();   

    return 0;
}