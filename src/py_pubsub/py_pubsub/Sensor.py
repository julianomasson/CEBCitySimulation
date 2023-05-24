from abc import ABC, abstractmethod

import random
from datetime import datetime


def get_timestamp() -> str:
    return datetime.now().strftime("%d-%m-%Y %H:%M:%S")


class Sensor(ABC):
    def __init__(self, name: str, unique_id: str, parameters: dict):
        self.name = name
        self.unique_id = unique_id
        self.parameters = parameters

    @abstractmethod
    def read(self) -> dict:
        pass


# Temperature sensor
class TemperatureSensor(Sensor):
    def __init__(self, name: str, unique_id: str, parameters: dict):
        super().__init__(name, unique_id, parameters)
        self.min_temp = self.parameters['min_temp']
        self.max_temp = self.parameters['max_temp']
        self.error_rate_threshold = 1.0 - self.parameters['error_rate']

    def read(self) -> dict:
        temp = random.uniform(self.min_temp, self.max_temp)
        if random.random() > self.error_rate_threshold:
            temp = temp * random.uniform(0, 2)
        # Create the default output
        output = {
            "name": self.name,
            "id": self.unique_id,
            "value": temp,
            "timestamp": get_timestamp()
        }
        return output
