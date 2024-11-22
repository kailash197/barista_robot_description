#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include <chrono>
using namespace std::chrono_literals;

class RobotChaser : public rclcpp::Node {
    private:
        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;

        const std::string target_frame = "rick/base_link";
        const std::string source_frame = "morty/base_link";

        rclcpp::TimerBase::SharedPtr timer_;
        void getTransform();

    public:
        RobotChaser():
        Node("robot_chaser_node"),
        tf_buffer_(this->get_clock()),
        tf_listener_(tf_buffer_){
            timer_ = this->create_wall_timer(500ms, std::bind(&RobotChaser::getTransform, this));
            RCLCPP_INFO(this->get_logger(), "Robot chaser ready!!!");
        }
};

void RobotChaser::getTransform(){
    geometry_msgs::msg::TransformStamped transform;
    try {
        transform = tf_buffer_.lookupTransform(
            this->target_frame, // the frame to which data should be transformed
            this->source_frame,
            tf2::TimePointZero);
    } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", ex.what());
        return;
    }

    // Calculate distance
    double dx = transform.transform.translation.x;
    double dy = transform.transform.translation.y;
    double dz = transform.transform.translation.z;
    double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

    // Calculate yaw (rotation about Z-axis)
    tf2::Quaternion q(
        transform.transform.rotation.x,
        transform.transform.rotation.y,
        transform.transform.rotation.z,
        transform.transform.rotation.w);
    tf2::Matrix3x3 mat(q);
    double roll, pitch, yaw;
    mat.getRPY(roll, pitch, yaw);

    RCLCPP_INFO(this->get_logger(),
    "Transform Distance: %.2f m, Yaw: %.2f rads", distance, yaw);
}

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotChaser>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
