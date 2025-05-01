import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial

class ESP32AngleControl(Node):
    def __init__(self):
        super().__init__('esp32_angle_control')
        self.subscription = self.create_subscription(
            Float32,
            'target_angle', 
            self.listener_callback,
            10)
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10)
        
        self.serial_port = serial.Serial('COM4', 115200, timeout=1)

    # def listener_callback(self, msg):
    #     angle = msg.data  # Target angle 
    #     self.serial_port.write((str(angle) + "\n").encode())  # Send to ESP32
    #     self.get_logger().info(f"Sent target angle: {angle}")
    def listener_callback(self, msg):
        # Interpret Twist message (linear.x for forward/backward, angular.z for left/right)
        command = "STOP"
        if msg.linear.x > 0:
            command = "FORWARD"
        elif msg.linear.x < 0:
            command = "BACKWARD"
        elif msg.angular.z > 0:
            command = "LEFT"
        elif msg.angular.z < 0:
            command = "RIGHT"
        
        # Send command over Serial
        self.serial_port.write((command + "\n").encode())
        self.get_logger().info(f"Sent command: {command}")

def main(args=None):
    rclpy.init(args=args)
    esp32_angle_control = ESP32AngleControl()
    rclpy.spin(esp32_angle_control)
    esp32_angle_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
