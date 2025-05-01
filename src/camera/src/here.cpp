#define M_PI 3.14159265358979323846
#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <vector>

class ColorDetectionNode : public rclcpp::Node {
public:
    ColorDetectionNode() : Node("color_detection_node"), camera_(0) {
        this->declare_parameter<std::string>("target_color", "yellow");

        if (!camera_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open the camera");
            return;
        }

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(30),
            std::bind(&ColorDetectionNode::process_frame, this)
        );

        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/detected_object", 10);
        RCLCPP_INFO(this->get_logger(), "Node started.");
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    cv::VideoCapture camera_;
    const double CAMERA_FOV_DEG = 60.0;
    const double MIN_OBJECT_SIZE = 0.03;  // Smallest detectable object
    const double MAX_OBJECT_SIZE = 0.1;   // Largest object before stopping

    void process_frame() {
        cv::Mat frame;
        camera_ >> frame;
        if (frame.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Empty frame");
            return;
        }

        cv::Mat hsvFrame;
        cv::cvtColor(frame, hsvFrame, cv::COLOR_BGR2HSV);

        std::string target_color;
        this->get_parameter("target_color", target_color);

        cv::Scalar lowerColor, upperColor;
        if (target_color == "yellow") {
            lowerColor = cv::Scalar(20, 100, 100);
            upperColor = cv::Scalar(40, 255, 255);
        }
        else if (target_color == "blue") {
            lowerColor = cv::Scalar(100, 50, 50);
            upperColor = cv::Scalar(140, 255, 255);
        }
        else if (target_color == "black") {
            lowerColor = cv::Scalar(0, 0, 0);
            upperColor = cv::Scalar(180, 255, 50);
        }
        else if (target_color == "white") {
            lowerColor = cv::Scalar(0, 0, 200);
            upperColor = cv::Scalar(180, 30, 255);
        }
        else if (target_color == "red") {
            cv::Scalar lowerRed1(0, 50, 50), upperRed1(10, 255, 255);
            cv::Scalar lowerRed2(160, 50, 50), upperRed2(180, 255, 255);
            cv::Mat redMask1, redMask2, colorMask;
            cv::inRange(hsvFrame, lowerRed1, upperRed1, redMask1);
            cv::inRange(hsvFrame, lowerRed2, upperRed2, redMask2);
            cv::bitwise_or(redMask1, redMask2, colorMask);
            detect_color(frame, colorMask, "Red");
            return;
        }
        else {
            RCLCPP_WARN(this->get_logger(), "Invalid color! Defaulting to yellow.");
            lowerColor = cv::Scalar(20, 100, 100);
            upperColor = cv::Scalar(40, 255, 255);
        }

        cv::Mat colorMask;
        cv::inRange(hsvFrame, lowerColor, upperColor, colorMask);
        detect_color(frame, colorMask, target_color);
    }

    void detect_color(cv::Mat& frame, cv::Mat& mask, const std::string& color_name) {
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(mask, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        if (!contours.empty()) {
            double maxArea = 0;
            std::vector<cv::Point> largestContour;
            for (const auto& contour : contours) {
                double area = cv::contourArea(contour);
                if (area > maxArea) {
                    maxArea = area;
                    largestContour = contour;
                }
            }

            if (!largestContour.empty()) {
                cv::Moments M = cv::moments(largestContour);
                if (M.m00 != 0) {
                    int cx = static_cast<int>(M.m10 / M.m00);
                    int cy = static_cast<int>(M.m01 / M.m00);

                    cv::circle(frame, cv::Point(cx, cy), 5, cv::Scalar(0, 255, 0), -1);

                    double object_size = cv::contourArea(largestContour) / (frame.cols * frame.rows);
                    int frameWidth = frame.cols;
                    double angle = ((cx - frameWidth / 2.0) / (frameWidth / 2.0)) * (CAMERA_FOV_DEG / 2.0);

                    control_robot(angle, object_size);
                    visualize_in_rviz(angle);
                }
            }
        }

        cv::imshow("Camera Stream", frame);
        cv::imshow(color_name + " Detection", mask);

        if (cv::waitKey(1) == 'q') {
            rclcpp::shutdown();
        }
    }

    void control_robot(double angle_deg, double object_size) {
        geometry_msgs::msg::Twist cmd_vel;
        double angular_gain = 0.5;
        cmd_vel.angular.z = -angular_gain * (angle_deg * M_PI / 180.0);

        if (object_size > MIN_OBJECT_SIZE) {
            cmd_vel.linear.x = std::max(0.1, 0.5 * (1.0 - object_size / MAX_OBJECT_SIZE));
        }
        else {
            cmd_vel.linear.x = 0.0;  // Stop if object is too small
        }

        cmd_vel_pub_->publish(cmd_vel);
    }

    void visualize_in_rviz(double angle_deg) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "base_link";
        marker.header.stamp = this->now();
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.position.x = 2.0;
        marker.pose.position.y = -2.0 * tan(angle_deg * M_PI / 180.0);
        marker.pose.position.z = 0.5;

        marker.scale.x = 0.3;
        marker.scale.y = 0.3;
        marker.scale.z = 0.3;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;

        marker_pub_->publish(marker);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ColorDetectionNode>());
    rclcpp::shutdown();
    return 0;
}
//#include <rclcpp/rclcpp.hpp>
//#include <opencv2/opencv.hpp>
//
//class ColorDetectionNode : public rclcpp::Node {
//public:
//    ColorDetectionNode() : Node("color_detection_node") {
//        // Timer to process frames
//        timer_ = this->create_wall_timer(
//            std::chrono::milliseconds(30),
//            std::bind(&ColorDetectionNode::process_frame, this)
//        );
//
//        RCLCPP_INFO(this->get_logger(), "Color Detection Node started.");
//    }
//
//private:
//    rclcpp::TimerBase::SharedPtr timer_;
//    cv::VideoCapture camera_;
//
//    void process_frame() {
//        if (!camera_.isOpened()) {
//            // Open camera if not already opened
//            camera_.open(0);
//            if (!camera_.isOpened()) {
//                RCLCPP_ERROR(this->get_logger(), "Unable to open camera.");
//                return;
//            }
//        }
//
//        cv::Mat frame;
//        camera_ >> frame; // Capture a frame
//        if (frame.empty()) {
//            RCLCPP_ERROR(this->get_logger(), "Captured empty frame.");
//            return;
//        }
//
//        // Convert to HSV
//        cv::Mat hsvFrame;
//        cv::cvtColor(frame, hsvFrame, cv::COLOR_BGR2HSV);
//
//        // Define HSV ranges
//        cv::Scalar lowerRed1(0, 50, 50), upperRed1(10, 255, 255);
//        cv::Scalar lowerRed2(160, 50, 50), upperRed2(180, 255, 255);
//        cv::Scalar lowerBlue(100, 50, 50), upperBlue(140, 255, 255);
//        cv::Scalar lowerYellow(20, 100, 100), upperYellow(40, 255, 255);
//        cv::Scalar lowerWhite(0, 0, 200), upperWhite(180, 30, 255);
//        cv::Scalar lowerBlack(0, 0, 0), upperBlack(180, 255, 50);
//
//        // Create masks
//        cv::Mat redMask1, redMask2, blueMask, yellowMask, whiteMask, blackMask;
//        cv::inRange(hsvFrame, lowerRed1, upperRed1, redMask1);
//        cv::inRange(hsvFrame, lowerRed2, upperRed2, redMask2);
//        cv::addWeighted(redMask1, 1.0, redMask2, 1.0, 0.0, redMask1);
//        cv::inRange(hsvFrame, lowerBlue, upperBlue, blueMask);
//        cv::inRange(hsvFrame, lowerYellow, upperYellow, yellowMask);
//        cv::inRange(hsvFrame, lowerWhite, upperWhite, whiteMask);
//        cv::inRange(hsvFrame, lowerBlack, upperBlack, blackMask);
//
//        // Optionally apply masks
//        cv::Mat redDetected, blueDetected, yellowDetected, whiteDetected, blackDetected;
//        cv::bitwise_and(frame, frame, redDetected, redMask1);
//        cv::bitwise_and(frame, frame, blueDetected, blueMask);
//        cv::bitwise_and(frame, frame, yellowDetected, yellowMask);
//        cv::bitwise_and(frame, frame, whiteDetected, whiteMask);
//        cv::bitwise_and(frame, frame, blackDetected, blackMask);
//
//        // Display frames
//        cv::imshow("Camera Stream", frame);
//        cv::imshow("Red Detection", redDetected);
//        cv::imshow("Blue Detection", blueDetected);
//        cv::imshow("Yellow Detection", yellowDetected);
//        cv::imshow("White Detection", whiteDetected);
//        cv::imshow("Black Detection", blackDetected);
//
//        // Exit on 'q' key press
//        if (cv::waitKey(30) == 'q') {
//            rclcpp::shutdown();
//        }
//    }
//};
//
//int main(int argc, char** argv) {
//    rclcpp::init(argc, argv);
//    rclcpp::spin(std::make_shared<ColorDetectionNode>());
//    rclcpp::shutdown();
//    return 0;
//}
//////#include <rclcpp/rclcpp.hpp>
//////#include <sensor_msgs/msg/image.hpp>
//////#include <geometry_msgs/msg/twist.hpp>
//////#include <opencv2/opencv.hpp>
//////
//////class ColorTrackingNode : public rclcpp::Node
//////{
//////public:
//////    ColorTrackingNode() : Node("color_tracking_node")
//////    {
//////        // Image subscriber
//////        image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
//////            "camera/image_raw", 10, std::bind(&ColorTrackingNode::image_callback, this, std::placeholders::_1));
//////
//////        // cmd_vel publisher
//////        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
//////    }
//////
//////private:
//////    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
//////    {
//////        try
//////        {
//////            // Convert the ROS image message data to a cv::Mat (assuming it's an 8-bit grayscale or color image)
//////            cv::Mat frame;
//////            if (msg->encoding == "bgr8") {
//////                frame = cv::Mat(msg->height, msg->width, CV_8UC3, (void*)msg->data.data());
//////            }
//////            else if (msg->encoding == "mono8") {
//////                frame = cv::Mat(msg->height, msg->width, CV_8UC1, (void*)msg->data.data());
//////            }
//////            else {
//////                RCLCPP_ERROR(this->get_logger(), "Unsupported image encoding: %s", msg->encoding.c_str());
//////                return;
//////            }
//////
//////            // Process the image
//////            detect_color(frame);
//////        }
//////        catch (const std::exception& e)
//////        {
//////            RCLCPP_ERROR(this->get_logger(), "Exception: %s", e.what());
//////        }
//////    }
//////
//////    void detect_color(const cv::Mat& frame)
//////    {
//////        cv::Mat hsv;
//////        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
//////
//////        // Define color range for red in HSV
//////        cv::Scalar lower_red(0, 120, 70);
//////        cv::Scalar upper_red(10, 255, 255);
//////
//////        // Create a mask for red color
//////        cv::Mat mask;
//////        cv::inRange(hsv, lower_red, upper_red, mask);
//////
//////        // Find the center of the red color
//////        cv::Moments m = cv::moments(mask, true);
//////        if (m.m00 > 0)
//////        {
//////            double cx = m.m10 / m.m00;
//////            double cy = m.m01 / m.m00;
//////
//////            double image_center_x = frame.cols / 2.0;
//////            double error_x = cx - image_center_x;
//////
//////            // Generate the velocity commands
//////            geometry_msgs::msg::Twist cmd_vel_msg;
//////            cmd_vel_msg.linear.x = 0.1;  // Move forward
//////            cmd_vel_msg.angular.z = -0.1 * error_x / image_center_x;  // Rotate to correct position
//////
//////            // Publish the control message
//////            cmd_vel_publisher_->publish(cmd_vel_msg);
//////        }
//////    }
//////
//////    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
//////    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
//////};
//////
//////int main(int argc, char* argv[])
//////{
//////    rclcpp::init(argc, argv);
//////    rclcpp::spin(std::make_shared<ColorTrackingNode>());
//////    rclcpp::shutdown();
//////    return 0;
//////}
//#ifndef M_PI
//#define M_PI 3.14159265358979323846
//#endif
//
//#include <rclcpp/rclcpp.hpp>
//#include <geometry_msgs/msg/twist.hpp>
//#include <nav_msgs/msg/odometry.hpp>
//#include <nav_msgs/msg/path.hpp>
//#include <visualization_msgs/msg/marker.hpp>
//#include <opencv2/opencv.hpp>
//#include <cmath>
//
//using namespace std;
//
//struct State {
//    double x, y, theta; // Position and orientation
//};
//
//class ColorDetectionAndControlNode : public rclcpp::Node {
//public:
//    ColorDetectionAndControlNode()
//        : Node("color_detection_control_node"), current_state_{ 0.0, 0.0, 0.0 }, dt_(0.1), control_weight_(0.1) {
//        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
//        marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);
//        path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("path", 10);
//        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
//
//        timer_ = this->create_wall_timer(
//            chrono::milliseconds(static_cast<int>(dt_ * 1000)),
//            std::bind(&ColorDetectionAndControlNode::control_loop, this));
//
//        path_msg_.header.frame_id = "map";
//        RCLCPP_INFO(this->get_logger(), "Color Detection and Control Node started.");
//    }
//
//private:
//    State current_state_;
//    cv::VideoCapture camera_;
//    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
//    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
//    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
//    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
//    rclcpp::TimerBase::SharedPtr timer_;
//    nav_msgs::msg::Path path_msg_;
//    double dt_;
//    double control_weight_;
//
//    // Detect red object and return its position in the camera frame
//    cv::Point detect_red_object(cv::Mat& frame) {
//        cv::Mat hsvFrame, redMask1, redMask2;
//        cv::cvtColor(frame, hsvFrame, cv::COLOR_BGR2HSV);
//
//        // Define red color range
//        cv::Scalar lowerRed1(0, 120, 70), upperRed1(10, 255, 255);
//        cv::Scalar lowerRed2(170, 120, 70), upperRed2(180, 255, 255);
//
//        // Create red masks
//        cv::inRange(hsvFrame, lowerRed1, upperRed1, redMask1);
//        cv::inRange(hsvFrame, lowerRed2, upperRed2, redMask2);
//        cv::addWeighted(redMask1, 1.0, redMask2, 1.0, 0.0, redMask1);
//
//        // Find contours
//        vector<vector<cv::Point>> contours;
//        cv::findContours(redMask1, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
//
//        if (!contours.empty()) {
//            // Find largest red object
//            auto maxContour = max_element(contours.begin(), contours.end(),
//                [](const vector<cv::Point>& c1, const vector<cv::Point>& c2) {
//                    return cv::contourArea(c1) < cv::contourArea(c2);
//                });
//            cv::Rect boundingBox = cv::boundingRect(*maxContour);
//            cv::Point center(boundingBox.x + boundingBox.width / 2, boundingBox.y + boundingBox.height / 2);
//            cv::circle(frame, center, 5, cv::Scalar(0, 255, 0), -1); // Draw the detected point
//            return center;
//        }
//
//        return { -1, -1 }; // No red object detected
//    }
//
//    void control_loop() {
//        // Open camera
//        if (!camera_.isOpened()) {
//            camera_.open(0);
//            if (!camera_.isOpened()) {
//                RCLCPP_ERROR(this->get_logger(), "Unable to open camera.");
//                return;
//            }
//        }
//
//        cv::Mat frame;
//        camera_ >> frame; // Capture frame
//        if (frame.empty()) {
//            RCLCPP_ERROR(this->get_logger(), "Captured empty frame.");
//            return;
//        }
//
//        cv::Point redPosition = detect_red_object(frame);
//        cv::imshow("Camera Feed", frame);
//        cv::waitKey(1);
//
//        if (redPosition.x == -1 && redPosition.y == -1) {
//            RCLCPP_INFO(this->get_logger(), "No red object detected. Stopping the robot.");
//            stop_robot();
//            return;
//        }
//
//        // Convert red position to relative target
//        double target_x = current_state_.x + (redPosition.x - frame.cols / 2) * 0.01;
//        double target_y = current_state_.y - (redPosition.y - frame.rows / 2) * 0.01;
//
//        State target = { target_x, target_y, 0.0 };
//        Control control = compute_control(target);
//
//        // Update state
//        current_state_ = predict_state(current_state_, control, dt_);
//
//        // Publish cmd_vel
//        geometry_msgs::msg::Twist cmd_vel_msg;
//        cmd_vel_msg.linear.x = control.v;
//        cmd_vel_msg.angular.z = control.omega;
//        cmd_vel_publisher_->publish(cmd_vel_msg);
//
//        publish_path();
//        publish_odometry();
//    }
//
//    // Predict the next state based on control inputs
//    State predict_state(const State& state, const Control& control, double dt) {
//        State predicted_state = state;
//        predicted_state.x += control.v * cos(state.theta) * dt;
//        predicted_state.y += control.v * sin(state.theta) * dt;
//        predicted_state.theta += control.omega * dt;
//        return predicted_state;
//    }
//
//    // Compute control inputs using NMPC
//    Control compute_control(const State& target) {
//        Control control = { 0.05, 0.05 }; // Initial control inputs
//        double learning_rate = 0.1;
//        int max_iterations = 50;
//
//        for (int i = 0; i < max_iterations; ++i) {
//            State predicted_state = predict_state(current_state_, control, dt_);
//            double cost = compute_cost(predicted_state, control, target);
//
//            // Compute gradients
//            double grad_v = (compute_cost(predict_state(current_state_, { control.v + 0.01, control.omega }, dt_), { control.v + 0.01, control.omega }, target) - cost) / 0.01;
//            double grad_omega = (compute_cost(predict_state(current_state_, { control.v, control.omega + 0.01 }, dt_), { control.v, control.omega + 0.01 }, target) - cost) / 0.01;
//
//            // Update control inputs
//            control.v -= learning_rate * grad_v;
//            control.omega -= learning_rate * grad_omega;
//
//            // Limit control inputs
//            control.v = std::clamp(control.v, 0.0, 1.0);
//            control.omega = std::clamp(control.omega, -M_PI / 4, M_PI / 4);
//        }
//
//        return control;
//    }
//
//    // Compute cost function for NMPC
//    double compute_cost(const State& state, const Control& control, const State& target) {
//        double distance = std::hypot(state.x - target.x, state.y - target.y);
//        double control_effort = control.v * control.v + control.omega * control.omega;
//        return distance + control_weight_ * control_effort;
//    }
//
//    void stop_robot() {
//        geometry_msgs::msg::Twist cmd_vel_msg;
//        cmd_vel_msg.linear.x = 0.0;
//        cmd_vel_msg.angular.z = 0.0;
//        cmd_vel_publisher_->publish(cmd_vel_msg);
//    }
//
//    void publish_path() {
//        geometry_msgs::msg::PoseStamped pose_stamped;
//        pose_stamped.header.stamp = now();
//        pose_stamped.header.frame_id = "map";
//        pose_stamped.pose.position.x = current_state_.x;
//        pose_stamped.pose.position.y = current_state_.y;
//        path_msg_.poses.push_back(pose_stamped);
//        path_publisher_->publish(path_msg_);
//    }
//
//    void publish_odometry() {
//        nav_msgs::msg::Odometry odom_msg;
//        odom_msg.header.stamp = now();
//        odom_msg.header.frame_id = "odom";
//        odom_msg.pose.pose.position.x = current_state_.x;
//        odom_msg.pose.pose.position.y = current_state_.y;
//        odom_msg.pose.pose.orientation.z = sin(current_state_.theta / 2);
//        odom_msg.pose.pose.orientation.w = cos(current_state_.theta / 2);
//        odom_publisher_->publish(odom_msg);
//    }
//};
//
//int main(int argc, char** argv) {
//    rclcpp::init(argc, argv);
//    rclcpp::spin(std::make_shared<ColorDetectionAndControlNode>());
//    rclcpp::shutdown();
//    return 0;
//}
//#include <rclcpp/rclcpp.hpp>
//#include <std_msgs/msg/string.hpp>
//#include <opencv2/opencv.hpp>
//
//class ColorDetectionNode : public rclcpp::Node {
//public:
//    ColorDetectionNode() : Node("color_detection_node") {
//        // Timer to process frames
//        timer_ = this->create_wall_timer(
//            std::chrono::milliseconds(30),
//            std::bind(&ColorDetectionNode::process_frame, this)
//        );
//
//        // Publisher for detected color
//        color_publisher_ = this->create_publisher<std_msgs::msg::String>("detected_color", 10);
//
//        RCLCPP_INFO(this->get_logger(), "Color Detection Node started.");
//    }
//
//private:
//    rclcpp::TimerBase::SharedPtr timer_;
//    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr color_publisher_;
//    cv::VideoCapture camera_;
//
//    void process_frame() {
//        if (!camera_.isOpened()) {
//            // Open camera if not already opened
//            camera_.open(0);
//            if (!camera_.isOpened()) {
//                RCLCPP_ERROR(this->get_logger(), "Unable to open camera.");
//                return;
//            }
//        }
//
//        cv::Mat frame;
//        camera_ >> frame; // Capture a frame
//        if (frame.empty()) {
//            RCLCPP_ERROR(this->get_logger(), "Captured empty frame.");
//            return;
//        }
//
//        // Convert to HSV
//        cv::Mat hsvFrame;
//        cv::cvtColor(frame, hsvFrame, cv::COLOR_BGR2HSV);
//
//        // Define HSV ranges
//        cv::Scalar lowerRed1(0, 50, 50), upperRed1(10, 255, 255);
//        cv::Scalar lowerRed2(160, 50, 50), upperRed2(180, 255, 255);
//        cv::Scalar lowerBlue(100, 50, 50), upperBlue(140, 255, 255);
//        cv::Scalar lowerYellow(20, 100, 100), upperYellow(40, 255, 255);
//        cv::Scalar lowerWhite(0, 0, 200), upperWhite(180, 30, 255);
//        cv::Scalar lowerBlack(0, 0, 0), upperBlack(180, 255, 50);
//
//        // Create masks
//        cv::Mat redMask1, redMask2, blueMask, yellowMask, whiteMask, blackMask;
//        cv::inRange(hsvFrame, lowerRed1, upperRed1, redMask1);
//        cv::inRange(hsvFrame, lowerRed2, upperRed2, redMask2);
//        cv::addWeighted(redMask1, 1.0, redMask2, 1.0, 0.0, redMask1);
//        cv::inRange(hsvFrame, lowerBlue, upperBlue, blueMask);
//        cv::inRange(hsvFrame, lowerYellow, upperYellow, yellowMask);
//        cv::inRange(hsvFrame, lowerWhite, upperWhite, whiteMask);
//        cv::inRange(hsvFrame, lowerBlack, upperBlack, blackMask);
//
//        // Check for detected colors
//        std_msgs::msg::String msg;
//        if (cv::countNonZero(redMask1) > 500) {
//            msg.data = "red";
//        }
//        else if (cv::countNonZero(blueMask) > 500) {
//            msg.data = "blue";
//        }
//        else if (cv::countNonZero(yellowMask) > 500) {
//            msg.data = "yellow";
//        }
//        else if (cv::countNonZero(whiteMask) > 500) {
//            msg.data = "white";
//        }
//        else if (cv::countNonZero(blackMask) > 500) {
//            msg.data = "black";
//        }
//        else {
//            msg.data = "none";
//        }
//
//        // Publish detected color
//        color_publisher_->publish(msg);
//
//        // Display frames
//        cv::imshow("Camera Stream", frame);
//        cv::imshow("Red Mask", redMask1);
//        cv::imshow("Blue Mask", blueMask);
//        cv::imshow("Yellow Mask", yellowMask);
//        cv::imshow("White Mask", whiteMask);
//        cv::imshow("Black Mask", blackMask);
//
//        // Exit on 'q' key press
//        if (cv::waitKey(30) == 'q') {
//            rclcpp::shutdown();
//        }
//    }
//};
//
//int main(int argc, char** argv) {
//    rclcpp::init(argc, argv);
//    rclcpp::spin(std::make_shared<ColorDetectionNode>());
//    rclcpp::shutdown();
//    return 0;
//}

//#include <rclcpp/rclcpp.hpp>
//#include <opencv2/opencv.hpp>
//#include <std_msgs/msg/float32_multi_array.hpp>
//using namespace std;
//using namespace cv;
//
//class ColorDetectionNode : public rclcpp::Node {
//public:
//    ColorDetectionNode() : Node("color_detection") {
//        this->declare_parameter<std::string>("target_color", "yellow"); // Default color
//        timer_ = this->create_wall_timer(chrono::milliseconds(30),
//            bind(&ColorDetectionNode::process_frame, this)
//        );
//
//        position_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("target_position", 10);
//        RCLCPP_INFO(this->get_logger(), "Color detection with path planning started");
//    }
//
//private:
//    rclcpp::TimerBase::SharedPtr timer_;
//    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr position_publisher_;
//    VideoCapture camera_;
//
//    const double CAMERA_FOV_DEG = 60.0;
//    const int FRAME_WIDTH = 640; // Default camera resolution
//    const int FRAME_HEIGHT = 480;
//
//    void process_frame() {
//        if (!camera_.isOpened()) {
//            camera_.open(0, CAP_DSHOW);
//            if (!camera_.isOpened()) {
//                RCLCPP_ERROR(this->get_logger(), "Camera not opening");
//                return;
//            }
//        }
//
//        Mat frame;
//        camera_ >> frame;
//        if (frame.empty()) {
//            RCLCPP_ERROR(this->get_logger(), "Empty frame");
//            return;
//        }
//
//        Mat hsvFrame;
//        cvtColor(frame, hsvFrame, cv::COLOR_BGR2HSV);
//
//        string target_color;
//        this->get_parameter("target_color", target_color);
//
//        Scalar lowerColor, upperColor;
//        if (target_color == "yellow") {
//            lowerColor = Scalar(20, 100, 100);
//            upperColor = Scalar(40, 255, 255);
//        }
//        else if (target_color == "blue") {
//            lowerColor = Scalar(100, 50, 50);
//            upperColor = Scalar(140, 255, 255);
//        }
//        else if (target_color == "red") {
//            Scalar lowerRed1(0, 50, 50), upperRed1(10, 255, 255);
//            Scalar lowerRed2(160, 50, 50), upperRed2(180, 255, 255);
//            Mat redMask1, redMask2;
//            inRange(hsvFrame, lowerRed1, upperRed1, redMask1);
//            inRange(hsvFrame, lowerRed2, upperRed2, redMask2);
//            addWeighted(redMask1, 1.0, redMask2, 1.0, 0.0, redMask1);
//            detect_objects(frame, redMask1, "Red");
//            return;
//        }
//        else {
//            RCLCPP_WARN(this->get_logger(), "Invalid color! Defaulting to yellow");
//            lowerColor = Scalar(20, 100, 100);
//            upperColor = Scalar(40, 255, 255);
//        }
//
//        Mat colorMask;
//        inRange(hsvFrame, lowerColor, upperColor, colorMask);
//        detect_objects(frame, colorMask, target_color);
//    }
//
//    const double REAL_OBJECT_SIZE = 0.1; // Actual size of the object in meters (adjust accordingly)
//    const double FOCAL_LENGTH = 500.0; // Focal length in pixels (you need to determine this via calibration)
//
//    double calculate_real_distance(double pixel_y_position) {
//        // Assuming the object size remains constant, we can approximate the distance based on pixel position
//        double object_pixel_height = FRAME_HEIGHT - pixel_y_position;
//        double real_distance = (REAL_OBJECT_SIZE * FOCAL_LENGTH) / object_pixel_height;
//        return real_distance;
//    }
//
//    void detect_objects(Mat& frame, Mat& mask, const string& color_name) {
//        vector<vector<Point>> contours;
//        findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
//
//        if (contours.empty()) return;
//
//        vector<Point2f> centroids;
//        for (const auto& contour : contours) {
//            Moments M = moments(contour);
//            if (M.m00 != 0) {
//                int cx = static_cast<int>(M.m10 / M.m00);
//                int cy = static_cast<int>(M.m01 / M.m00);
//                centroids.push_back(Point2f(cx, cy));
//                circle(frame, Point(cx, cy), 5, Scalar(0, 255, 0), -1);
//            }
//        }
//
//        if (!centroids.empty()) {
//            Point2f nearest = find_nearest_object(centroids);
//            double angle = ((nearest.x - FRAME_WIDTH / 2.0) / (FRAME_WIDTH / 2.0)) * (CAMERA_FOV_DEG / 2.0);
//
//            // Convert pixel position to real-world distance
//            double real_distance = calculate_real_distance(nearest.y);
//
//            auto msg = std_msgs::msg::Float32MultiArray();
//            msg.data = { static_cast<float>(angle), static_cast<float>(real_distance) };
//            position_publisher_->publish(msg);
//
//            RCLCPP_INFO(this->get_logger(), "%s detected at angle: %.2f degrees, distance: %.2f meters", color_name.c_str(), angle, real_distance);
//        }
//
//        imshow("Stream", frame);
//        imshow(color_name + " Detection", mask);
//
//        if (waitKey(30) == 'q') {
//            rclcpp::shutdown();
//        }
//    }
//
//    Point2f find_nearest_object(const vector<Point2f>& centroids) {
//        Point2f nearest = centroids[0];
//        double minDistance = FRAME_HEIGHT - centroids[0].y;
//        for (const auto& pt : centroids) {
//            double distance = FRAME_HEIGHT - pt.y;
//            if (distance < minDistance) {
//                minDistance = distance;
//                nearest = pt;
//            }
//        }
//        return nearest;
//    }
//};
//
//int main(int argc, char** argv) {
//    rclcpp::init(argc, argv);
//    rclcpp::spin(std::make_shared<ColorDetectionNode>());
//    rclcpp::shutdown();
//    return 0;
//}