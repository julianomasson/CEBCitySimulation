# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

import importlib

from py_pubsub.Sensor import *

import random
import json


class MinimalPublisher(Node):

    def load_json(self, json_path: str) -> dict:
        try:
            with open(json_path, "r") as f:
                data = json.load(f)
        except FileNotFoundError:
            raise ValueError(f"Could not find file on path {json_path}. Is it correct?")
        except json.JSONDecodeError:
            raise ValueError(f"Incorrect JSON encoding on file {json_path}")
        self.get_logger().debug(f"Loaded the {data}")
        return data

    def load_sensors(self, sensors_parameters: dict) -> list:
        sensor_list = list()
        for sensor in sensors_parameters:
            try:
                module = importlib.import_module("py_pubsub.Sensor")
                self.get_logger().info(f"Module {module}")
                class_ = getattr(module, sensor['class_name'])
                self.get_logger().info(f"Class {class_}")
                sensor_instance = class_(sensor['class_name'], sensor['id'], sensor['parameters'])
                sensor_list.append(sensor_instance)
                self.get_logger().debug(f"Loaded the sensor {sensor}")
            except Exception as e:
                self.get_logger().error(f"{e}")
                self.get_logger().error(f"Could not load the sensor {sensor}")
        return sensor_list

    def __init__(self):
        super().__init__('minimal_publisher')
        # Get the JSON path
        self.declare_parameter('json_path', rclpy.Parameter.Type.STRING)
        # Get the parameters
        self.parameters = self.load_json(self.get_parameter('json_path').value)
        # Load the sensors
        self.sensors = self.load_sensors(self.parameters['sensors'])
        self.publisher_ = self.create_publisher(String, 'topic', self.parameters['publisher_queue_size'])
        timer_period = 1.0 / self.parameters['publisher_frequency']  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.publisher_error_rate_threshold = 1.0 - self.parameters['publisher_error_rate']

    def timer_callback(self):
        if random.random() > self.publisher_error_rate_threshold:
            self.get_logger().info('Intentionally dropping message')
            return
        msg = String()
        msg.data = 'Sent message '
        for sensor in self.sensors:
            msg.data = f"{msg.data} {sensor.read()}"
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
