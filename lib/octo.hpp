#include <iostream>
#include <gz/transport/Node.hh>
#include <gz/msgs.hh>
#include <Eigen/Dense>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <pcl/visualization/pcl_visualizer.h>

octomap::OcTree tree(0.05);


void viewOctree() {

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for(octomap::OcTree::leaf_iterator it = tree.begin_leafs(), end=tree.end_leafs(); it!= end; ++it) {
        if(tree.isNodeOccupied(*it)) {
            cloud->push_back(pcl::PointXYZ(it.getX(), it.getY(), it.getZ()));
        }
    }

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));

    viewer->setBackgroundColor (0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");

    viewer->addCoordinateSystem (1.0);
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

/*
void imuCallback(const gz::msgs::IMU &imu_msg) {
    orientation.x() = imu_msg.orientation().x();
    orientation.y() = imu_msg.orientation().y();
    orientation.z() = imu_msg.orientation().z();
    orientation.w() = imu_msg.orientation().w();
}

void odometryCallback(const gz::msgs::Odometry &odom) {
    position.x() = odom.pose().position().x();
    position.y() = odom.pose().position().y();
    position.z() = odom.pose().position().z();
}*/

void updateOctomapWithPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, Eigen::Vector3d position, Eigen::Quaterniond orientation) {

    Eigen::Affine3d transform(Eigen::Translation3d(position) * orientation);
    Eigen::Affine3d inverse_transform = transform.inverse();

    for (const auto& point : cloud->points) {
        Eigen::Vector4d point_homogeneous(point.x, point.y, point.z, 1.0);
        Eigen::Vector4d transformed_point_homogeneous = inverse_transform * point_homogeneous;

        octomap::point3d point3d(transformed_point_homogeneous.x(), transformed_point_homogeneous.y(), transformed_point_homogeneous.z());
        tree.updateNode(point3d, true);
    }

    tree.updateInnerOccupancy();
}

float search(const octomap::OcTree& tree, octomap::point3d position, octomap::point3d direction) {
    
    octomap::point3d position_octo(position.x(), position.y(), position.z());
    octomap::point3d direction_octo(direction.x(), direction.y(), direction.z());
    octomap::point3d end;

    bool hit = tree.castRay(position_octo, direction_octo, end);
    
    if(hit){
        float cost = (end-position_octo).norm();
        std::cout << "Hit: " << end << ", Cost: " << cost << std::endl;
        return cost;
    }
    else{
        std::cout << "No hit" << std::endl;
        return std::numeric_limits<float>::max();
    }
    
}

float pointCloudCallback(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, Eigen::Vector3d position, Eigen::Vector3d direction, Eigen::Quaterniond orientation) {

    octomap::point3d direction_point(direction.x(), direction.y(), direction.z());
    octomap::point3d position_point(position.x(), position.y(), position.z());

    //std::cout << "Position: " << position.transpose() << std::endl;
    //std::cout << "Direction: " << direction.transpose() << std::endl;
    //std::cout << "Orientation: " << orientation.coeffs().transpose() << std::endl;

    updateOctomapWithPointCloud(cloud, position, orientation);

    //std::cout << "Octomap size: " << tree.size() << std::endl;

    float cost = search(tree, position_point, direction_point);

    //viewOctree();
    return cost;
}