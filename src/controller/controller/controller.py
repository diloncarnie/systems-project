import rclpy
from rclpy.node import Node
import time

from std_msgs.msg import String, Bool, UInt8, Float32
from geometry_msgs.msg import Twist

MIN_OBSTACLE_DISTANCE = 0.1
GRIPPER_CLOSE_ANGLE = 0
GRIPPER_OPEN_ANGLE = 60
GRIPPER_DELAY = 0.5
ACTUATOR_PULLUP_TIME = -3.0
ACTUATOR_PUSHDOWN_TIME = 3.0

class Controller(Node):

    
    
    def __init__(self):
        super().__init__('controller_node')
        # State subscriptions
        self.subscription = self.create_subscription(UInt8, 'detected_objects', self.get_detected_objects, 10)
        self.subscription = self.create_subscription(Float32, 'ultrasonic', self.get_distance, 10)
        self.subscription = self.create_subscription(String, 'tracker_state', self.get_tracker_state, 10)
        
        
        # State publishers
        self.pub_colour_ = self.create_publisher(String, 'colour', 10)
        self.pub_state_ = self.create_publisher(String, 'state', 10)
        self.pub_gripper_ = self.create_publisher(UInt8, 'servo_gripper', 10)
        self.pub_linear_actuator_ = self.create_publisher(Float32, 'linear_actuator', 10)
        self.pub_speaker_ = self.create_publisher(Bool, 'speaker', 10)
        self.pub_cmd_vel_ = self.create_publisher(Twist, 'cmd_vel', 10)
        
        
        # State of the ROS System
        self.state = "-"
        self.tracker_state = "-"
        self.colour = None
        self.colours = ["yellow","green"]
        self.detected_objects = 0
        self.distance = float('inf')
        
        # Start main loop timer
        self.timer = self.create_timer(0.01, self.main_control_loop)
        
    def get_detected_objects(self, msg):
        self.start = msg.data
    
    def get_distance(self, data):
        self.distance = data.data
        
    def get_tracker_state(self, data):
        self.tracker_state = data.data

    def main_control_loop(self):
        if (self.state == "-"):
            while rclpy.ok():

                state_input = input("Enter state: ")
                colour_input = "yellow"
                self.colour = colour_input
                self.state = state_input
                self.get_logger().info(f"Updated state to: {self.state}")
                return
    
        if (self.state == "close"):
            self.pub_gripper_.publish(UInt8(data=GRIPPER_CLOSE_ANGLE))
            time.sleep(GRIPPER_DELAY)
            self.state = "-"
            self.pub_state_.publish(String(data=self.state))
            return
            
        if (self.state == "open"):
            self.pub_gripper_.publish(UInt8(data=GRIPPER_OPEN_ANGLE))
            self.state = "-"
            self.pub_state_.publish(String(data=self.state))
            return
        
            
        if (self.state == "move"): 
            value = input("how long: ")
            self.pub_linear_actuator_.publish(Float32(data=float(value)))
            self.state = "-"
            self.pub_state_.publish(String(data=self.state))
            return
            
            
            
            
        # Pickup workflowmove 
        if(self.state == "pickup"):
            self.get_logger().info(f'Picking up {self.colour}')
            # Publish pending colours
            self.pub_colour_.publish(String(data=self.colour))
            # Publish state to activate tracker 
            self.pub_state_.publish(String(data=self.state))
            
            self.get_logger().info(f'Tracker state: {self.tracker_state}')
            if(self.tracker_state == "waiting"):
                # Run pickup workflow
                self.get_logger().info('Waiting for pickup!')
                self.state = "-"
                self.pub_state_.publish(String(data=self.state))
                self.get_logger().info('Closing gripper')
                self.pub_gripper_.publish(UInt8(data=GRIPPER_CLOSE_ANGLE))
                time.sleep(GRIPPER_DELAY)
                self.get_logger().info('Pulling actuator up')
                self.pub_linear_actuator_.publish(Float32(data=-ACTUATOR_PULLUP_TIME))
                time.sleep(-ACTUATOR_PULLUP_TIME)
                #Move bacwards
                cmd_vel_msg = Twist()  # All fields default to 0.0
                cmd_vel_msg.linear.x = -0.5  # Set backwards motion
                self.pub_cmd_vel_.publish(cmd_vel_msg)
                time.sleep(1.0)
                cmd_vel_msg.linear.x = 0.0  # Stop motion
                self.pub_cmd_vel_.publish(cmd_vel_msg)
                time.sleep(2)
                #time.sleep(ACTUATOR_PULLUP_TIME)
                self.get_logger().info('Pick up success')
                return
                
        if (self.state == "place"):   
            self.get_logger().info(f'Placing {self.colour}')
            self.pub_colour_.publish(String(data=self.colour))
            self.pub_state_.publish(String(data=self.state))
            if(self.tracker_state == "waiting"):
                self.get_logger().info('Waiting for place!')
                # Set state to none
                self.state = "-"
                self.pub_state_.publish(String(data=self.state))
                # Open gripper to drop object
                self.pub_gripper_.publish(UInt8(data=GRIPPER_OPEN_ANGLE))
                time.sleep(GRIPPER_DELAY)
                # Move bacwards to push down actuator to base
                cmd_vel_msg = Twist()  # All fields default to 0.0
                cmd_vel_msg.linear.x = -0.1  # Set backwards motion
                self.pub_cmd_vel_.publish(cmd_vel_msg)
                time.sleep(1.0)
                cmd_vel_msg.linear.x = 0.0  # Stop motion
                self.pub_cmd_vel_.publish(cmd_vel_msg)
                return
        
        if (self.state == "open"):
            self.get_logger().info('Opening gripper')
            self.pub_gripper_.publish(UInt8(data=GRIPPER_OPEN_ANGLE))
            time.sleep(GRIPPER_DELAY)
            self.state = "-"
            self.pub_state_.publish(String(data=self.state))
            return
        
        # Objective workflow
        if(self.distance < MIN_OBSTACLE_DISTANCE ):
            if(self.tracker_state == "close" or self.tracker_state == "waiting" or self.tracker_state == "reaching" ):
                return
            
            # Run obstacle detected command
            self.get_logger().info('Obstacle detected')
            self.pub_speaker_.publish(Bool(data=True))
            self.state = "-"
            self.pub_state_.publish(String(data=self.state))
            
    
    
            
        
        

def main(args=None):
    rclpy.init(args=args)

    controller_node = Controller()

    rclpy.spin(controller_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()