#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "airsim_ros_wrapper.h"
#include <math.h>

class PID {
public:
    PID(double kp, double ki, double kd)
        : kp_(kp), ki_(ki), kd_(kd), prev_error_(0), integral_(0) {}

    double compute(double setpoint, double actual) {
        double error = setpoint - actual;
        integral_ += error;
        double derivative = error - prev_error_;
        prev_error_ = error;
        return kp_ * error + ki_ * integral_ + kd_ * derivative;
    }

private:
    double kp_, ki_, kd_;
    double prev_error_, integral_;
};

class GimbalTrackingNode : public rclcpp::Node {
public:
    GimbalTrackingNode()
        : Node("gimbal_tracking_node"),
          tfBuffer(this->get_clock()),
          tfListener(tfBuffer),
          pid_yaw(0.1, 0.01, 0.01),
          pid_pitch(0.1, 0.01, 0.01)
    {
        target_point_.setValue(0, 0, 0);

        gimbal_pub = this->create_publisher<airsim_interfaces::msg::GimbalAngleQuatCmd>("/airsim_node/gimbal_angle_quat_cmd", 10);
        timer = this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&GimbalTrackingNode::timer_callback, this));
    }

private:
    void timer_callback() {
        try {
            auto transform = tfBuffer.lookupTransform("world_ned", "PX4/odom_local_ned", tf2::TimePointZero);

            tf2::Vector3 gimbal_position(transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z);
            tf2::Vector3 direction_vector = target_point_ - gimbal_position;
            direction_vector.normalize();
            
            tf2::Quaternion current_orientation(transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w);
            tf2::Quaternion target_orientation = getQuaternionFromDirection(direction_vector);

            tf2::Quaternion error_quat = target_orientation * current_orientation.inverse();

            // Manual axis-angle extraction from quaternion
            double error_angle = 2 * acos(error_quat.w());
            tf2::Vector3 error_axis(error_quat.x(), error_quat.y(), error_quat.z());
            error_axis.normalize();

            double corrected_yaw = pid_yaw.compute(0, error_angle * error_axis.z());
            double corrected_pitch = pid_pitch.compute(0, error_angle * error_axis.y());

            tf2::Quaternion adjusted_orientation;
            adjusted_orientation.setRPY(0, corrected_pitch, corrected_yaw);

            airsim_interfaces::msg::GimbalAngleQuatCmd gimbal_cmd;
            gimbal_cmd.header.stamp = this->now();
            gimbal_cmd.header.frame_id = "world_ned";
            gimbal_cmd.camera_name = "GimbalCam";
            gimbal_cmd.vehicle_name = "PX4";
            gimbal_cmd.orientation.x = adjusted_orientation.x();
            gimbal_cmd.orientation.y = adjusted_orientation.y();
            gimbal_cmd.orientation.z = adjusted_orientation.z();
            gimbal_cmd.orientation.w = adjusted_orientation.w();

            gimbal_pub->publish(gimbal_cmd);

        } 
        catch (const tf2::TransformException &e) {
            RCLCPP_WARN(this->get_logger(), "Failed to compute gimbal orientation: %s", e.what());
        }
    }

    tf2::Quaternion getQuaternionFromDirection(const tf2::Vector3& direction) {
        tf2::Vector3 up(0, 0, 1);
        tf2::Vector3 right = up.cross(direction);
        right.normalize();
        up = direction.cross(right);
        up.normalize();

        tf2::Matrix3x3 rotation_matrix;
        rotation_matrix[0] = right;     // First column
        rotation_matrix[1] = up;       // Second column
        rotation_matrix[2] = direction; // Third column

        tf2::Quaternion orientation;
        rotation_matrix.getRotation(orientation);
        return orientation;
    }

    tf2::Vector3 target_point_;
    rclcpp::Publisher<airsim_interfaces::msg::GimbalAngleQuatCmd>::SharedPtr gimbal_pub;
    rclcpp::TimerBase::SharedPtr timer;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;
    PID pid_yaw;
    PID pid_pitch;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GimbalTrackingNode>());
    rclcpp::shutdown();
    return 0;
}
