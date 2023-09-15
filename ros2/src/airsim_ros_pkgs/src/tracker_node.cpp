#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "airsim_ros_wrapper.h"
#include "visualization_msgs/msg/marker.hpp"
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
        target_point.header.frame_id = "world_enu";
        target_point.point.x = 0;
        target_point.point.y = 0;
        target_point.point.z = 0;

        gimbal_pub = this->create_publisher<airsim_interfaces::msg::GimbalAngleQuatCmd>("/airsim_node/gimbal_angle_quat_cmd", 10);
        marker_pub = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);
        timer = this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&GimbalTrackingNode::timer_callback, this));
    }

private:
    void timer_callback() {
        try {
            auto transform = tfBuffer.lookupTransform("world_ned", "PX4/odom_local_ned", tf2::TimePointZero);

            tf2::Vector3 gimbal_position(transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z);
            tf2::Vector3 direction_vector = tf2::Vector3(target_point.point.x, target_point.point.y, target_point.point.z) - gimbal_position;
            direction_vector.normalize();

            tf2::Vector3 up_vector(0, 0, 1);
            tf2::Vector3 right_vector = tf2::tf2Cross(direction_vector, up_vector);
            right_vector.normalize();
            tf2::Vector3 camera_up_vector = tf2::tf2Cross(right_vector, direction_vector);
            tf2::Matrix3x3 rotation_matrix(
                direction_vector.x(), right_vector.x(), camera_up_vector.x(),
                direction_vector.y(), right_vector.y(), camera_up_vector.y(),
                direction_vector.z(), right_vector.z(), camera_up_vector.z()
            );
            tf2::Quaternion target_orientation;
            rotation_matrix.getRotation(target_orientation);

            tf2::Quaternion current_orientation(transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w);
            double current_yaw = tf2::getYaw(current_orientation);
            double target_yaw = tf2::getYaw(target_orientation);
            double corrected_yaw = pid_yaw.compute(target_yaw, current_yaw);

            double current_pitch = tf2::getPitch(current_orientation);
            double target_pitch = tf2::getPitch(target_orientation);
            double corrected_pitch = pid_pitch.compute(target_pitch, current_pitch);

            target_orientation.setRPY(0, corrected_pitch, corrected_yaw);

            airsim_interfaces::msg::GimbalAngleQuatCmd gimbal_cmd;
            gimbal_cmd.header.stamp = this->now();
            gimbal_cmd.header.frame_id = "world_ned";
            gimbal_cmd.camera_name = "GimbalCam";
            gimbal_cmd.vehicle_name = "PX4";
            gimbal_cmd.orientation.x = target_orientation.x();
            gimbal_cmd.orientation.y = target_orientation.y();
            gimbal_cmd.orientation.z = target_orientation.z();
            gimbal_cmd.orientation.w = target_orientation.w();

            gimbal_pub->publish(gimbal_cmd);

            visualization_msgs::msg::Marker line_target, line_current;

            tf2::Vector3 target_end = gimbal_position + direction_vector * 2.0;
            setVisualization(line_target, gimbal_position, target_end, 1.0, 0.0, 0.0, 1);

            tf2::Vector3 current_direction = tf2::quatRotate(current_orientation, tf2::Vector3(1, 0, 0));
            tf2::Vector3 current_end = gimbal_position + current_direction * 2.0;
            setVisualization(line_current, gimbal_position, current_end, 0.0, 0.0, 1.0, 2);

            marker_pub->publish(line_target);
            marker_pub->publish(line_current);

        } catch (const std::exception &e) {
            RCLCPP_WARN(this->get_logger(), "Failed to compute gimbal orientation: %s", e.what());
        }
    }

    void setVisualization(visualization_msgs::msg::Marker &line, tf2::Vector3 start, tf2::Vector3 end, float r, float g, float b, int id) {
        line.header.frame_id = "world_ned";
        line.header.stamp = this->now();
        line.ns = "orientation_vectors";
        line.action = visualization_msgs::msg::Marker::ADD;
        line.pose.orientation.w = 1.0;
        line.id = id;
        line.type = visualization_msgs::msg::Marker::ARROW;
        line.scale.x = 0.1;
        line.scale.y = 0.2;
        line.scale.z = 0.2;
        line.color.a = 1.0;
        line.color.r = r;
        line.color.g = g;
        line.color.b = b;

        geometry_msgs::msg::Point p_start, p_end;
        p_start.x = start.x();
        p_start.y = start.y();
        p_start.z = start.z();
        p_end.x = end.x();
        p_end.y = end.y();
        p_end.z = end.z();

        line.points.push_back(p_start);
        line.points.push_back(p_end);
    }

    geometry_msgs::msg::PointStamped target_point;
    rclcpp::Publisher<airsim_interfaces::msg::GimbalAngleQuatCmd>::SharedPtr gimbal_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub;
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
