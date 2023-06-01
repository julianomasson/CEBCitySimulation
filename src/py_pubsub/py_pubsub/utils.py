import importlib
import json
import logging
from datetime import datetime


def get_timestamp() -> str:
    return datetime.now().strftime("%d-%m-%Y %H:%M:%S")


def load_json(json_path: str) -> dict:
    try:
        with open(json_path, "r") as f:
            data = json.load(f)
    except FileNotFoundError:
        raise ValueError(f"Could not find file on path {json_path}. Is it correct?")
    except json.JSONDecodeError:
        raise ValueError(f"Incorrect JSON encoding on file {json_path}")
    logging.debug(f"Loaded the {data}")
    return data


def load_sensors(sensors_parameters: dict) -> list:
    sensor_list = list()
    for sensor in sensors_parameters:
        try:
            module = importlib.import_module("py_pubsub.sensor")
            logging.info(f"Module {module}")
            class_ = getattr(module, sensor['class_name'])
            logging.info(f"Class {class_}")
            sensor_instance = class_(sensor['class_name'], sensor['id'], sensor['parameters'], sensor['errors'])
            sensor_list.append(sensor_instance)
            logging.debug(f"Loaded the Sensor {sensor}")
        except Exception as e:
            logging.error(f"{e}")
            logging.error(f"Could not load the Sensor {sensor}")
    return sensor_list


def load_errors(errors_parameters: dict) -> list:
    error_list = list()
    for error in errors_parameters:
        try:
            module = importlib.import_module("py_pubsub.error")
            logging.info(f"Module {module}")
            class_ = getattr(module, error['class_name'])
            logging.info(f"Class {class_}")
            error_instance = class_(error['class_name'], error['parameters'])
            error_list.append(error_instance)
            logging.debug(f"Loaded the Error {error}")
        except Exception as e:
            logging.error(f"{e}")
            logging.error(f"Could not load the Error {error}")
    return error_list
