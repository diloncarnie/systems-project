import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String, Float32, UInt8
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
import pickle
import os

THRESHOLD = 15
NOMINAL_TRACKING_SPEED = 0.85
MIN_OBSTACLE_DISTANCE = 0.1778
REACHING_DISTANCE = 0.1
REACHING_SPEED = 0.1
GRIPPING_DISTANCE = 0.085
DROPPING_DISTANCE = 0.01

MIN_OBJECT_SIZE = 100
MAX_OBJECT_SIZE = 200

MIN_BOX_SIZE = 200
MAX_BOX_SIZE = 400

class ObjectTracker(Node):
    def __init__(self):
        super().__init__('object_tracker')
        # Tracker subscribers
        self.subscription = self.create_subscription(Image, 'image_raw', self.track_object, 10)
        self.subscription = self.create_subscription(String, 'colour', self.get_colour, 10)
        self.subscription = self.create_subscription(Float32, 'ultrasonic', self.get_distance, 10)
        self.subscription = self.create_subscription(String, 'state', self.get_state, 10)
        
        # Tracker publishers
        self.pub_cmd_vel_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pub_detected_objects_ = self.create_publisher(UInt8, 'detected_objects', 10)
        self.pub_tracker_state_ = self.create_publisher(String, 'tracker_state', 10)

        # Internal States
        self.tracker_state = "deactivated"
        self.min_tracking_size = 100
        self.max_tracking_size = 200
        
        #System States
        self.state = None
        self.colour = None
        self.distance = float('inf')
        
        # Function Variables
        self.br = CvBridge()
        self.linear_velocity_gain = 1.5
        self.angular_velocity_gain = 0.03

    def get_colour(self, data):
        self.colour = data.data
        
    def get_distance(self, data):
        self.distance = data.data
    
    def get_state(self, data):
        self.state = data.data

        
        
    def track_object(self, data):
        twist = Twist()
        if(self.state=="pickup"):
            self.min_tracking_size = MIN_OBJECT_SIZE
            self.max_tracking_size = MAX_OBJECT_SIZE
        else:
            self.get_logger().info('Tracking is deactivated')
            self.tracker_state = "deactivated"
            self.pub_tracker_state_.publish(String(data=self.tracker_state))
            return
            
        if (self.colour == "yellow"):
            # Mask for yellow colour
            lower_target = np.array([20, 35, 185])
            upper_target = np.array([55, 255, 255])
        elif (self.colour == "green"):
            # Mask for yellow colour
            lower_target = np.array([36,0,0])
            upper_target = np.array([86,255,255])
        else:
            self.get_logger().info('No color set')
            return
        
        # Open and load the calibration data
        with open("/home/rosdev/ros2_ws/calibration_data.pkl", 'rb') as f:
            calibration_data = pickle.load(f)

        mtx = calibration_data['camera_matrix']
        dist = calibration_data['distortion_coefficients']
    
        img = self.br.imgmsg_to_cv2(data, 'bgr8')
        
        # Get image dimensions: height and width
        height, width = img.shape[:2]
        # Calculate optimal camera matrix
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (width, height), 1, (width, height))
        # Create undistortion maps
        mapx, mapy = cv2.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (width, height), 5)
        img = cv2.remap(img, mapx, mapy, cv2.INTER_LINEAR)
        # Crop the image (optional)
        x, y, w, h = roi
        img = img[y:y+h, x:x+w]
        
        img = cv2.resize(img, None, 1, 0.25, 0.25, cv2.INTER_CUBIC)

        
        # Calculate the center coordinates
        height, width = img.shape[:2]
        center_x = int(width / 2)
        center_y = int(height / 2)   
        # Draw a small circle at the center
        img = cv2.circle(img, (center_x, center_y), 5, (255, 0, 0), -1)

        
        
        imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(imgHSV, lower_target, upper_target)
        # cv2.imshow("Original Mask", mask)
        
        
        
        kernel = np.ones((THRESHOLD, THRESHOLD), "uint8") 
        mask = cv2.dilate(mask, kernel) 
        res = cv2.bitwise_and(img, img, 
                            mask = mask) 
        
        # Creating contour to track red color 
        contours, hierarchy = cv2.findContours(mask, 
                                            cv2.RETR_TREE, 
                                            cv2.CHAIN_APPROX_SIMPLE) 


        if(self.tracker_state == "reaching" or self.tracker_state == "waiting"):   
                        if(self.distance>GRIPPING_DISTANCE):
                            self.get_logger().info(f'Making pickup move {self.distance}')
                            self.tracker_state = "reaching"
                            self.pub_tracker_state_.publish(String(data=self.tracker_state))
                            twist.linear.x = REACHING_SPEED
                            twist.angular.z = 0.0
                            self.pub_cmd_vel_.publish(twist)
                        else:
                            twist.linear.x = 0.0
                            twist.angular.z = 0.0
                            self.pub_cmd_vel_.publish(twist)
                            self.get_logger().info(f'Waiting for pickup')
                            self.tracker_state = "waiting"
                            self.pub_tracker_state_.publish(String(data=self.tracker_state))       
                        return

        # Publish detected objects
        self.pub_detected_objects_.publish(UInt8(data=len(contours)))        
        if(len(contours)>0 and  self.tracker_state != "waiting"):
            area = cv2.contourArea(contours[0]) 
            if(area>100): 
                # Update tracker state
                self.get_logger().info(f'Tracking {len(contours)} {self.colour} Objects ')
                x, y, w, h = cv2.boundingRect(contours[0]) 
                # Compute the center of the bounding rectangle
                midx = int(x + w / 2)
                midy = int(y + h / 2)
                img = cv2.rectangle(img, (x, y), (x + w, y + h), (0, 0, 255), 2) 
                # Draw a filled circle at the center
                img = cv2.circle(img, (midx, midy), 5, (0, 255, 0), -1)
                # Calculate linear and angular velocities based on centroid position
                error = center_x - midx
                #self.get_logger().info('Error: "%d"' % error)
                
                if(self.state=="pickup"):
                        
                    # Adjust speed according to distance in meters
                    mapped_distance = np.interp(self.distance, [0.055, 0.3], [0.0, 0.3])
                    if (self.distance) < 0.3:
                        # self.get_logger().info(f'Mapped Distance {mapped_distance}')
                        if(self.distance<REACHING_DISTANCE):    
                            self.tracker_state = "reaching"
                            self.pub_tracker_state_.publish(String(data=self.tracker_state))
                            return
                        else:
                            linear_velocity = self.linear_velocity_gain * self.distance  
                            self.tracker_state = "close"
                            self.pub_tracker_state_.publish(String(data=self.tracker_state))     
                    else:
                        linear_velocity = NOMINAL_TRACKING_SPEED
                        self.tracker_state = "tracking"
                        self.pub_tracker_state_.publish(String(data=self.tracker_state))

                    # Keep the centroid in the center of the ROI
                    if abs(error) > 50:
                        linear_velocity = 0.0
                        angular_velocity = self.angular_velocity_gain * error
                        # angular_velocity = np.clip(angular_velocity, -1.0, 1.0)
                    else:
                        angular_velocity = self.angular_velocity_gain * error
                    
            
                # Publish the velocity commands
                self.get_logger().info(f'Linear Velocity: {linear_velocity} |  Angular Velocity {angular_velocity}')
                
                twist.linear.x = linear_velocity
                twist.angular.z = -angular_velocity
                self.pub_cmd_vel_.publish(twist)
        else:
            if(self.tracker_state == "reaching" or self.tracker_state == "waiting"):
                return
            # No objects are detected
            self.get_logger().info('No objects detected')
            self.tracker_state = "looking"
            self.pub_tracker_state_.publish(String(data=self.tracker_state))
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.pub_cmd_vel_.publish(twist)
        
        cv2.imshow("Centroid Indicator", img)
        cv2.imshow("Line Threshold", mask)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    object_tracker = ObjectTracker()
    rclpy.spin(object_tracker)
    object_tracker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()