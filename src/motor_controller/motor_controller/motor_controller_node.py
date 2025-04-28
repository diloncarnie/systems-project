import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import smbus
from time import sleep

class MotorController(Node):

    def __init__(self):
        super().__init__('motor_controller_node')
        self.subscription = self.create_subscription(Int32MultiArray,'motor_rpm', self.motor_callback, 10)
        self.bus = smbus.SMBus(4)
        self.front_motor_address = 0x08 # bus address
        #self.back_motor_address = 0x55 # bus address

    def motor_callback(self, msg):
        if msg.data:
            motor1 = msg.data[0]
            motor2 = msg.data[1]
            motor3 = msg.data[2]
            motor4 = msg.data[3]
            
        # Write motor rpms byte by byte
        self.bus.write_byte(self.front_motor_address, motor1)
        sleep(0.01)
        self.bus.write_byte(self.front_motor_address, motor2)
        sleep(0.01)
        self.bus.write_byte(self.front_motor_address, motor3)
        sleep(0.01)
        self.bus.write_byte(self.front_motor_address, motor4)
        
        self.get_logger().info(f'Motor RPMs: FrontR: {motor1}, FrontL: {motor2}, BackR: {motor3}, BackL: {motor4}')


def main(args=None):
    rclpy.init(args=args)
    motor_controller_node = MotorController()
    rclpy.spin(motor_controller_node)
    motor_controller_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()