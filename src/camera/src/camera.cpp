
#define M_PI 3.14159265358979323846

#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>  //for Point message
#include <vector>
#include <algorithm>
using namespace std;
using namespace cv;

class ColorDetectionNode : public rclcpp::Node {
public:
    ColorDetectionNode() : Node("color_detection_node"), camera_(0) {
        this->declare_parameter<string>("target_color", "yellow");

        if (!camera_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open camera.");
            return;
        }

        timer_ = this->create_wall_timer(
            chrono::milliseconds(30),
            bind(&ColorDetectionNode::process_frame, this)
        );

        target_position_pub_ = this->create_publisher<geometry_msgs::msg::Point>("/target_position", 10);
        RCLCPP_INFO(this->get_logger(), "Node started.");
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr target_position_pub_;
    VideoCapture camera_;
    const double CAMERA_FOV_DEG = 60.0;
    const double REAL_HEIGHT = 6.0;
    const double MIN_DISTANCE = 20.0;
    int object_counter;

    void process_frame() {
        Mat frame;
        camera_ >> frame;
        if (frame.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Empty frame");
            return;
        }

        Mat hsvFrame;
        cvtColor(frame, hsvFrame, COLOR_BGR2HSV);

        string target_color;
        this->get_parameter("target_color", target_color);

        Scalar lowerColor, upperColor;
        if (target_color == "yellow") {
            lowerColor = Scalar(35, 40, 40);
            upperColor = Scalar(85, 255, 255);
        }
        else if (target_color == "blue") {
            lowerColor = Scalar(100, 50, 50);
            upperColor = Scalar(140, 255, 255);
        }
        else if (target_color == "black") {
            lowerColor = Scalar(0, 0, 0);
            upperColor = Scalar(180, 255, 50);
        }
        else if (target_color == "white") {
            lowerColor = Scalar(0, 0, 200);
            upperColor = Scalar(180, 30, 255);
        }
        else if (target_color == "red") {
            Scalar lowerRed1(0, 50, 50), upperRed1(10, 255, 255);
            Scalar lowerRed2(160, 50, 50), upperRed2(180, 255, 255);
            Mat redMask1, redMask2, colorMask;
            inRange(hsvFrame, lowerRed1, upperRed1, redMask1);
            inRange(hsvFrame, lowerRed2, upperRed2, redMask2);
            bitwise_or(redMask1, redMask2, colorMask);
            detect_color(frame, colorMask, "Red");
            return;
        }
        else {
            RCLCPP_WARN(this->get_logger(), "Invalid color! Defaulting to yellow.");
            lowerColor = Scalar(20, 100, 100);
            upperColor = Scalar(40, 255, 255);
        }

        Mat colorMask;
        inRange(hsvFrame, lowerColor, upperColor, colorMask);
        detect_color(frame, colorMask, target_color);
    }

    void detect_color(Mat& frame, Mat& mask, const string& color_name) {
        vector<vector<Point>> contours;
        findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        vector<pair<double, Point>> detected_objects;

        for (const auto& contour : contours) {
            Rect bounds = boundingRect(contour);
            if (bounds.area() < 100) continue;

            Moments m = moments(contour);
            Point center(m.m10 / m.m00, m.m01 / m.m00);

            double distance = estimate_distance(bounds.height, frame.cols);
            double angle = calculate_angle(center.x, frame.cols);

            detected_objects.emplace_back(distance, center);

            rectangle(frame, bounds, Scalar(0, 255, 0), 2);
            circle(frame, center, 5, Scalar(0, 255, 0), -1);

            string dist_text = format("%.1f cm", distance);
            putText(frame, dist_text, Point(bounds.x, bounds.y - 10),
                FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 255, 0), 2);
        }

        if (!detected_objects.empty()) {
            std::sort(detected_objects.begin(), detected_objects.end(),
                [](const std::pair<double, cv::Point>& a, const std::pair<double, cv::Point>& b) {
                    if (a.first == b.first) {
                        return a.second.x < b.second.x;
                    }
                    return a.first < b.first;
                });

            Point best_target = detected_objects.front().second;
            double best_distance = detected_objects.front().first;
            double best_angle = calculate_angle(best_target.x, frame.cols);

            RCLCPP_INFO(this->get_logger(), "Detected %s object #%d at %.2f cm", color_name.c_str(), object_counter++, best_distance);

            // angle and distance in x, y coordinates
            double x = best_distance * cos(best_angle * M_PI / 180.0);
            double y = best_distance * sin(best_angle * M_PI / 180.0);

            // target position
            geometry_msgs::msg::Point target_position;
            target_position.x = x;
            target_position.y = y;
            target_position.z = 0.0;
            target_position_pub_->publish(target_position);
        }

        // Stopping opencv imshow display
        // imshow("Detection", frame);
        // if (waitKey(1) == 'q') rclcpp::shutdown();
    }

    double calculate_angle(int x_center, int frame_width) {
        return ((x_center - frame_width / 2.0) / (frame_width / 2.0)) * (CAMERA_FOV_DEG / 2.0);
    }

    double estimate_distance(int pixel_height, int image_width) {
        double focal_length = (image_width / 2.0) / tan((CAMERA_FOV_DEG * M_PI / 180) / 2);
        return (focal_length * REAL_HEIGHT) / pixel_height;
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ColorDetectionNode>());
    rclcpp::shutdown();
    return 0;
}