#include <librealsense2/rs.hpp>
#include <iostream>
#include <Eigen/Dense>
#include <MadgwickAHRS.h>

int main()
{

    rs2::pipeline pipe;

    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
    cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);
    pipe.start(cfg);

    Madgwick filter;
    filter.begin(30);

    while (true)
    {
        rs2::frameset frames = pipe.wait_for_frames();

        rs2::motion_frame accel_frame = frames.first_or_default(RS2_STREAM_ACCEL);
        rs2_vector accel_data = accel_frame.get_motion_data();
        Eigen::Vector3f linear_acceleration(accel_data.x, accel_data.y, accel_data.z);

        rs2::motion_frame gyro_frame = frames.first_or_default(RS2_STREAM_GYRO);
        rs2_vector gyro_data = gyro_frame.get_motion_data();
        Eigen::Vector3f angular_velocity(gyro_data.x, gyro_data.y, gyro_data.z);

        filter.updateIMU(angular_velocity.x(), angular_velocity.y(), angular_velocity.z(),
                        linear_acceleration.x(), linear_acceleration.y(), linear_acceleration.z());

        float roll = filter.getRollRadians();
        float pitch = filter.getPitchRadians();
        float yaw = filter.getYawRadians();

        Eigen::Quaternionf orientation = Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX())
                                        * Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY())
                                        * Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ());

        std::cout << "Linear Acceleration: " << linear_acceleration.transpose() << std::endl;
        std::cout << "Angular Velocity: " << angular_velocity.transpose() << std::endl;
        std::cout << "Orientation: " << orientation.coeffs().transpose() << std::endl;
    }

    return 0;
}