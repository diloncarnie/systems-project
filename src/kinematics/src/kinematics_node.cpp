#include <memory>
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cstdlib>
#include <cstdio>

#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include "kinematics/KinematicsClass.h"  // Make sure this include path is correct

#define MOTOR_MAX_RPM 90        // Motor's maximum RPM
#define WHEEL_DIAMETER 0.2      // Robot's wheel diameter in meters
#define FR_WHEEL_DISTANCE 0.6   // Front-rear wheel distance in meters
#define LR_WHEEL_DISTANCE 0.5   // Left-right wheel distance in meters
#define PWM_BITS 8              // PWM resolution bits

class KinematicsNode : public rclcpp::Node
{
public:
  KinematicsNode()
  : Node("kinematics_node"),
    kinematics_(MOTOR_MAX_RPM, WHEEL_DIAMETER, FR_WHEEL_DISTANCE, LR_WHEEL_DISTANCE, PWM_BITS)
  {
    RCLCPP_INFO(this->get_logger(), "Kinematics Node Initialized");

    // Create a subscription to the cmd_vel topic
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10,
      std::bind(&KinematicsNode::rpm_callback, this, std::placeholders::_1));
      
    motor_rpm_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("motor_rpm", 10);

    // Initialize I2C connection with the Arduino
    const char* device = "/dev/i2c-4";  // Adjust if your device is on a different bus
    i2c_file_ = open(device, O_RDWR);
    if (i2c_file_ < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open the I2C bus");
      exit(1);
    }

    int addr = 0x28;  // Arduino I2C slave address
    if (ioctl(i2c_file_, I2C_SLAVE, addr) < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to acquire bus access and/or talk to slave");
      close(i2c_file_);
      exit(1);
    }
  }

  ~KinematicsNode() override {
    if(i2c_file_ >= 0) {
      close(i2c_file_);
    }
  }

private:
  // The callback uses the incoming Twist message to calculate the RPM for each motor.
  void rpm_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    // Calculate motor RPMs using the linear and angular velocities from the message.
    auto rpm = kinematics_.getRPM(msg->linear.x, msg->linear.y, msg->angular.z);

    // Define a lambda function to clamp the motor RPM value between -127 and 127.
    auto clamp = [](int value) -> int {
      if (value > 127)
        return 127;
      else if (value < -127)
        return -127;
      else
        return value;
    };

    // Clamp each motor RPM value.
    int motor1 = clamp(rpm.motor1);
    int motor2 = clamp(rpm.motor2);
    int motor3 = clamp(rpm.motor3);
    int motor4 = clamp(rpm.motor4);

    // Create and publish the RPM message on the "motor_rpm" topic.
    std_msgs::msg::Int32MultiArray rpm_msg;
    rpm_msg.data = {motor1, motor2, motor3, motor4};
    motor_rpm_pub_->publish(rpm_msg);

    // Log the calculated motor RPM values.
    RCLCPP_INFO(
      this->get_logger(),
      "Motor RPM: FrontR: %d, FrontL: %d, BackR: %d, BackL: %d", motor1, motor2, motor3, motor4);

    // Pack the four RPM values into an array of bytes.
    // Note: This assumes the RPM values can be represented as a single byte.
    // Adjust conversion if your values exceed 0-255.
    char data[4];
    data[0] = static_cast<char>(rpm.motor1);  
    data[1] = static_cast<char>(rpm.motor2);
    data[2] = static_cast<char>(rpm.motor3);
    data[3] = static_cast<char>(rpm.motor4);

    // Write the array to the I2C bus.
    int bytes_written = write(i2c_file_, data, sizeof(data));
    if (bytes_written != sizeof(data)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to write RPM data to the I2C bus");
    } else {
      RCLCPP_INFO(this->get_logger(), "Sent RPM data via I2C: %d, %d, %d, %d",
                  rpm.motor1, rpm.motor2, rpm.motor3, rpm.motor4);
    }
  }

  // Subscription to the "cmd_vel" topic.
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  // Publisher to the "motor_rpm" topic.
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr motor_rpm_pub_;
  // Instance of your Kinematics class.
  KINEMATICS::Kinematics kinematics_;
  // File descriptor for the I2C device.
  int i2c_file_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KinematicsNode>());
  rclcpp::shutdown();
  return 0;
}
