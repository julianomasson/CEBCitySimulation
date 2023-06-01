import random
from abc import ABC, abstractmethod


class Error(ABC):
    # TODO we could add an ID here as well, to find the error origin, but it is necessary?
    def __init__(self, name: str, parameters: dict):
        self.name = name
        self.parameters = parameters

    # This method should be used to make a test that will interrupt the message
    @abstractmethod
    def stop_message(self) -> bool:
        pass

    # This method should be used to change the input value
    @abstractmethod
    def change_value(self, value: type) -> type:
        pass


# Random uniform deviation error, apply random.uniform(self.min_value, self.max_value) in the sensor value
class RandomUniformDeviation(Error):
    def __init__(self, name: str, parameters: dict):
        super().__init__(name, parameters)
        self.min_value = self.parameters['min_value']
        self.max_value = self.parameters['max_value']
        self.error_rate_threshold = 1.0 - self.parameters['error_rate']

    def stop_message(self) -> bool:
        return False

    def change_value(self, value: float) -> float:
        if random.random() > self.error_rate_threshold:
            value = value * random.uniform(self.min_value, self.max_value)
        return value


# Critical network error, can make the message be lost
class CriticalNetwork(Error):
    def __init__(self, name: str, parameters: dict):
        super().__init__(name, parameters)
        self.error_rate_threshold = 1.0 - self.parameters['error_rate']

    def stop_message(self) -> bool:
        return random.random() > self.error_rate_threshold

    def change_value(self, value: type) -> type:
        return value
