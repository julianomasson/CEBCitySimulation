import random
from abc import ABC, abstractmethod
from py_pubsub.utils import load_errors


class Sensor(ABC):
    def __init__(self, name: str, unique_id: str, parameters: dict, errors_to_load: dict):
        self.name = name
        self.unique_id = unique_id
        self.parameters = parameters
        self.error_list = load_errors(errors_to_load)

    @abstractmethod
    def read(self) -> dict:
        pass


# Temperature sensor
class TemperatureSensor(Sensor):
    def __init__(self, name: str, unique_id: str, parameters: dict, errors_to_load: dict):
        super().__init__(name, unique_id, parameters, errors_to_load)
        self.min_temp = self.parameters['min_temp']
        self.max_temp = self.parameters['max_temp']

    def read(self) -> dict:
        for error in self.error_list:
            if error.stop_message():
                return dict()
        value = random.uniform(self.min_temp, self.max_temp)
        for error in self.error_list:
            value = error.change_value(value)
        # Create the default output
        return {
            "name": self.name,
            "id": self.unique_id,
            "value": value,
        }
