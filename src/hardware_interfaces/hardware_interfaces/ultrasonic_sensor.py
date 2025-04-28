import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from gpiozero import DistanceSensor




class UltrasonicSensor(Node):

    def __init__(self):
        super().__init__('ultrasonic_sensor')
        self.frequency = 30
        self.publisher_ = self.create_publisher(Float32, 'ultrasonic', 10)
        self.timer = self.create_timer(1/ self.frequency, self.publish_distance)
        self.ultrasonic = DistanceSensor(echo=27, trigger=17, max_distance=2)

    def publish_distance(self):
        msg = Float32()
        msg.data = self.ultrasonic.distance
        self.publisher_.publish(msg)
        self.get_logger().info(f'Distance: {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    ultrasonic_sensor = UltrasonicSensor()
    rclpy.spin(ultrasonic_sensor)
    ultrasonic_sensor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()