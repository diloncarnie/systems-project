#include <memory>
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cstdlib>
#include <cstdio>

#include "sensor_msgs/msg/imu.hpp"
#include <rclcpp/rclcpp.hpp>
#include "mpu9250_driver/Mpu9250Class.h" // Make sure this include path is correct


// namespace mpu9250_node
// {

class Mpu9250Node : public rclcpp::Node
{
public:
    Mpu9250Node() : Node("mpu9250_node"), mpu9250_(4,0x68)
    {
        RCLCPP_INFO(this->get_logger(), "Kinematics Node Initialized");
        // Declare parameters
        declareParameters();

        int bus = this->get_parameter("bus").as_int();
        unsigned int address = this->get_parameter("address").as_int();
        
        // Create publisher
        publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", rclcpp::QoS(100));

        int status = mpu9250_.begin();

        if (status < 0) {
            RCLCPP_INFO(this->get_logger(), "IMU Initialization failed. Error codeL %d", status);
            rclcpp::shutdown();
            return;
        }


        const float period_ms = 1000.0 / this->get_parameter("frequency").as_double();
        timer = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int64_t>(period_ms)),
            std::bind(&Mpu9250Node::update, this));
          RCLCPP_INFO_STREAM(this->get_logger(), "Timer triggering every " << period_ms << " ms");
        
    }

    ~Mpu9250Node() override {
        RCLCPP_WARN(this->get_logger(), "Shutting down");
      }

private:
    // The callback uses the incoming Twist message to calculate the RPM for each motor.
    void update()
    {
        if(mpu9250_.readSensor()){
            auto message = sensor_msgs::msg::Imu();
            message.header.stamp = this->get_clock()->now();
            message.header.frame_id = "imu_link";
            // Direct measurements
            message.linear_acceleration_covariance = {0};
            message.linear_acceleration.x = mpu9250_.getAccelX_mss();
            message.linear_acceleration.y = mpu9250_.getAccelY_mss();
            message.linear_acceleration.z = mpu9250_.getAccelZ_mss();
            message.angular_velocity_covariance = {0};
            message.angular_velocity.x = mpu9250_.getGyroX_rads();
            message.angular_velocity.y = mpu9250_.getGyroY_rads();
            message.angular_velocity.z = mpu9250_.getGyroZ_rads();
            // Calculate euler angles, convert to quaternion and store in message
            message.orientation_covariance = {-1};
            publisher_->publish(message);
        }else{
            RCLCPP_ERROR(this->get_logger(), "Failed to read sensor data");
        }
    }

    void declareParameters()
    {
        this->declare_parameter("bus", 4);
        this->declare_parameter("address", 0x68);
        this->declare_parameter("frequency", 250.0);
    }

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
    // Instance of your Kinematics class.
    MPU9250::MPU9250 mpu9250_;
    rclcpp::TimerBase::SharedPtr timer;
};


int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Mpu9250Node>());
  rclcpp::shutdown();
  return 0;
}



