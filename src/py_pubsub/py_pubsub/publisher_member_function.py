import rclpy
from py_pubsub.Sensor import *
from rclpy.node import Node
from std_msgs.msg import String

from py_pubsub.utils import load_json, load_sensors, load_errors, get_timestamp


class LoraNode(Node):

    def __init__(self):
        super().__init__('lora_node')
        # Get the JSON path
        self.declare_parameter('json_path', rclpy.Parameter.Type.STRING)
        # Get the parameters
        self.parameters = load_json(self.get_parameter('json_path').value)
        self.id = self.parameters['id']
        # Load the sensors
        self.sensors = load_sensors(self.parameters['sensors'])
        # Load the errors
        self.error_list = load_errors(self.parameters['errors'])
        # Create the publisher
        self.publisher_ = self.create_publisher(String, 'topic', self.parameters['queue_size'])
        # Set the publisher frequency
        timer_period = 1.0 / self.parameters['frequency']  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        for error in self.error_list:
            if error.stop_message():
                self.get_logger().info('Intentionally dropping message')
                return
        msg = String()
        msg.data = f"{self.id}-{get_timestamp()}: "
        for sensor in self.sensors:
            msg.data = f"{msg.data} {sensor.read()}"
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    lora_node = LoraNode()

    rclpy.spin(lora_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    lora_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
