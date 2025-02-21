#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <chrono>
#include <thread>


using namespace std::chrono_literals;

class IMUTransform : public rclcpp::Node
{
public:
    IMUTransform() : Node("imu_transform")
    {
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", 10, std::bind(&IMUTransform::imuCallback, this, std::placeholders::_1));

        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data/transformed", 10);

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
        
        this->declare_parameter("yaw_offset_", 1.57079632679);

        checkImumsg(imu_data_);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;

    bool imu_data_ = false;

    double yaw_offset_;

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        if (!imu_data_) {
            imu_data_ = true;
            checkImumsg(imu_data_);
        }

        tf2::Quaternion imu_quat(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);

        tf2::Quaternion yaw_rotation;

        yaw_rotation.setRPY(0, 0, yaw_offset_);

        tf2::Quaternion corrected_quat = yaw_rotation * imu_quat;
        corrected_quat.normalize();

        publishCorrectedImu(msg, corrected_quat);

        publishImuTransform(corrected_quat);
    }

    void checkImumsg(bool imu_data_)
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));

        if (!imu_data_) {
            RCLCPP_INFO(this->get_logger(), "Waiting for Imu data.");
        } else {
            RCLCPP_INFO(this->get_logger(), "start transforming Imu data.");
        }
    }
    

    void publishCorrectedImu(const sensor_msgs::msg::Imu::SharedPtr msg, const tf2::Quaternion &quat)
    {
        auto corrected_msg = std::make_shared<sensor_msgs::msg::Imu>(*msg);

        corrected_msg->orientation.x = quat.x();
        corrected_msg->orientation.y = quat.y();
        corrected_msg->orientation.z = quat.z();
        corrected_msg->orientation.w = quat.w();

        imu_pub_->publish(*corrected_msg);
    }

    void publishImuTransform(const tf2::Quaternion &quat)
    {
        geometry_msgs::msg::TransformStamped transformStamped;

        transformStamped.header.stamp = this->get_clock()->now();
        transformStamped.header.frame_id = "base_link";
        transformStamped.child_frame_id = "imu_link";

        transformStamped.transform.translation.x = 0.0;
        transformStamped.transform.translation.y = 0.0;
        transformStamped.transform.translation.z = 0.0;

        transformStamped.transform.rotation.x = quat.x();
        transformStamped.transform.rotation.y = quat.y();
        transformStamped.transform.rotation.z = quat.z();
        transformStamped.transform.rotation.w = quat.w();

        tf_broadcaster_->sendTransform(transformStamped);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<IMUTransform>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
