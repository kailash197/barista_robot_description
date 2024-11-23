#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include <chrono>
using namespace std::chrono_literals;
constexpr double PROXIMITY_LIMIT = 0.37;

class RobotChaser : public rclcpp::Node {
    private:
        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;

        const std::string target_frame = "rick/base_link";
        const std::string source_frame = "morty/base_link";

        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
        geometry_msgs::msg::Twist cmd_vel_;
        const std::string pub_topic_ = "rick/cmd_vel";

        rclcpp::TimerBase::SharedPtr timer_;
        void getTransform();

    public:
        RobotChaser():
        Node("robot_chaser_node"),
        tf_buffer_(this->get_clock()),
        tf_listener_(tf_buffer_){
            timer_ = this->create_wall_timer(100ms, std::bind(&RobotChaser::getTransform, this));
            cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(pub_topic_, 10);
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

    // Calculate angle (rotation about Z-axis)
    double yaw = atan2(dy, dx);

    // assign velocities
    double kp_distance = 1.0;
    double kp_yaw = 1.0;
    double linear_vel = 0.0;
    double angular_vel = 0.0;

    if (distance > PROXIMITY_LIMIT){
        if(distance > 1.0 ){
            kp_distance = 1.0;
            kp_yaw = 1.0;
        } else {
            kp_distance = 0.5;
            kp_yaw=0.5;
        }
        if (fabs(yaw) > M_PI / 6.0){
            kp_distance *= 0.5;
            kp_yaw *= 1.2;
        }

        linear_vel = kp_distance * distance;
        angular_vel = kp_yaw * yaw;
    }

    this->cmd_vel_.linear.x = linear_vel;
    this->cmd_vel_.angular.z = angular_vel;
    this->cmd_vel_pub_->publish(this->cmd_vel_);
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
