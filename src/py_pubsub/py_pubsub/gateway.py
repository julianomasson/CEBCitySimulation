import logging
import time

import rclpy
from paho.mqtt import client as mqtt_client
from py_pubsub.utils import load_json, load_errors
from rclpy.node import Node
from std_msgs.msg import String


def connect_mqtt(logger, broker: str, port: int, client_id: str, username: str, password: str):
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            logger.info("Connected to MQTT Broker!")
        else:
            logger.error("Failed to connect, return code %d\n", rc)

    client = mqtt_client.Client(client_id)
    if username != "" and password != "":
        client.username_pw_set(username, password)
    client.on_connect = on_connect
    client.connect(broker, port)
    return client


FIRST_RECONNECT_DELAY = 1
RECONNECT_RATE = 2
MAX_RECONNECT_COUNT = 12
MAX_RECONNECT_DELAY = 60


def on_disconnect(client, userdata, rc):
    logging.info("Disconnected with result code: %s", rc)
    reconnect_count, reconnect_delay = 0, FIRST_RECONNECT_DELAY
    while reconnect_count < MAX_RECONNECT_COUNT:
        logging.info("Reconnecting in %d seconds...", reconnect_delay)
        time.sleep(reconnect_delay)
        try:
            client.reconnect()
            logging.info("Reconnected successfully!")
            return
        except Exception as err:
            logging.error("%s. Reconnect failed. Retrying...", err)
        reconnect_delay *= RECONNECT_RATE
        reconnect_delay = min(reconnect_delay, MAX_RECONNECT_DELAY)
        reconnect_count += 1
    logging.info("Reconnect failed after %s attempts. Exiting...", reconnect_count)


class Gateway(Node):

    def __init__(self):
        super().__init__('gateway')
        # Get the JSON path
        self.declare_parameter('json_path', rclpy.Parameter.Type.STRING)
        # Get the parameters
        self.parameters = load_json(self.get_parameter('json_path').value)
        # Load the errors
        self.error_list = load_errors(self.parameters['errors'])
        # Create the publisher
        self.subscription = self.create_subscription(String, 'topic', self.listener_callback,
                                                     self.parameters['queue_size'])
        # MQTT settings
        self.mqtt_parameters = self.parameters['MQTT']
        self.mqtt_topic = self.mqtt_parameters['topic']
        self.mqtt_client = connect_mqtt(self.get_logger(),
            self.mqtt_parameters['broker'],
            self.mqtt_parameters['port'],
            self.mqtt_parameters['client_id'],
            self.mqtt_parameters['username'],
            self.mqtt_parameters['password']
        )
        self.mqtt_client.on_disconnect = on_disconnect

    def listener_callback(self, msg):
        for error in self.error_list:
            if error.stop_message():
                self.get_logger().info('Intentionally discarding message')
                return
        self.get_logger().debug('I heard: "%s"' % msg.data)
        # Send to MQTT server
        result = self.mqtt_client.publish(self.mqtt_topic, msg.data)
        status = result[0]
        if status == 0:
            self.get_logger().info(f"Send {msg} to topic {self.mqtt_topic}")
        else:
            self.get_logger().error(f"Failed to send message to topic {self.mqtt_topic}")


def main(args=None):
    rclpy.init(args=args)

    gateway = Gateway()

    gateway.mqtt_client.loop_start()
    rclpy.spin(gateway)
    gateway.mqtt_client.loop_stop()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    gateway.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
