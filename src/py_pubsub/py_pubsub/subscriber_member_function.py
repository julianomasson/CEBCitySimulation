import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from py_pubsub.utils import load_json, load_errors

class Gateway(Node):

    def __init__(self):
        super().__init__('gateway')
        # Get the JSON path
        self.declare_parameter('json_path', rclpy.Parameter.Type.STRING)
        # Get the parameters
        self.parameters = load_json(self.get_parameter('json_path').value)
        self.id = self.parameters['id']
        # Load the errors
        self.error_list = load_errors(self.parameters['errors'])
        # Create the publisher
        self.subscription = self.create_subscription(String, 'topic', self.listener_callback, self.parameters['queue_size'])

    def listener_callback(self, msg):
        for error in self.error_list:
            if error.stop_message():
                self.get_logger().info('Intentionally discarding message')
                return
        self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    gateway = Gateway()

    rclpy.spin(gateway)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    gateway.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
