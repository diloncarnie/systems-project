import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from gpiozero import LED
from time import sleep




class VoiceSensor(Node):

    def __init__(self):
        super().__init__('voice_sensor')
        self.subscription = self.create_subscription(Bool,'speaker', self.speaker_callback, 10)
        self.play = LED(13)

    def speaker_callback(self, msg):
        if (msg.data):
            self.play.on()
            sleep(0.05)
            self.play.off()
        self.get_logger().info('Running voice sensor callback')


def main(args=None):
    rclpy.init(args=args)
    voice_sensor = VoiceSensor()
    rclpy.spin(voice_sensor)
    voice_sensor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()