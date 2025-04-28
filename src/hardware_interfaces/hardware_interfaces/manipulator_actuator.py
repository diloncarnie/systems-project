import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8, Float32
from gpiozero import AngularServo, Motor


class ManipulatorActuator(Node):

    def __init__(self):
        super().__init__('manipulator_actuator')
        self.subscription = self.create_subscription(UInt8,'servo_base', self.servo_base_callback, 10)
        self.subscription = self.create_subscription(UInt8,'servo_gripper', self.servo_gripper_callback, 10)
        self.subscription = self.create_subscription(Float32,'linear_actuator', self.linear_actuator_callback, 10)
        self.timer = None
        self.servo_base = AngularServo(12, min_angle=0, max_angle=180, min_pulse_width=0.5/1000, max_pulse_width=2.5/1000)
        self.servo_gripper = AngularServo(18, min_angle=0, max_angle=180, min_pulse_width=0.5/1000, max_pulse_width=2.5/1000)
        self.linear_actuator = Motor(21,20)
        self.servo_base.angle = 120
        
    def set_servo(self):
        self.servo_base.angle = 60

    def servo_base_callback(self, msg):
        self.servo_base.angle = msg.data
        self.get_logger().info('Ran servo base callback to angle %d' % msg.data)
        
    def servo_gripper_callback(self, msg):
        self.servo_gripper.angle = msg.data
        self.get_logger().info('Ran servo gripper callback %d' % msg.data)
        
    def linear_actuator_callback(self, msg):
        self.get_logger().info('Ran linear actuator callback')
        
        # Stop any previous timer if it's still running
        if self.timer is not None:
            self.destroy_timer(self.timer)
            self.timer = None
            
        delay = abs(msg.data)
        if (msg.data>0):
            self.linear_actuator.forward()
            self.get_logger().info('Pushing Down')
        else:
            self.linear_actuator.backward()
            self.get_logger().info('Pulling Up')
            
        
        self.timer = self.create_timer(delay, self.stop_linear_actuator)
        
    def stop_linear_actuator(self):
        self.get_logger().info('Stopping linear actuator')
        self.linear_actuator.stop()
        self.destroy_timer(self.timer)
        self.timer = None
        


def main(args=None):
    rclpy.init(args=args)
    manipulator_actuator = ManipulatorActuator()
    rclpy.spin(manipulator_actuator)
    manipulator_actuator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()