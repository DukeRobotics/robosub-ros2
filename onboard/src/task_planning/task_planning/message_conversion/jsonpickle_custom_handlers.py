import builtins
import importlib
import traceback

import jsonpickle

from rclpy.logging import get_logger

import rosidl_runtime_py

from task_planning.message_conversion.ros_message_converter import \
    convert_dictionary_to_ros_message, convert_ros_message_to_dictionary


logger = get_logger('jsonpickle_custom_handlers')


class ROSMessageHandler(jsonpickle.handlers.BaseHandler):
    """
    JSONPickle handler to convert ROS messages to and from dictionaries.
    """

    def flatten(self, obj, data):
        data["ros/type"] = (
            "/".join(type(obj).__module__.split(".")[:-1]) + "/" + type(obj).__name__
        )
        data['ros/data'] = convert_ros_message_to_dictionary(obj)
        return data

    def restore(self, obj):
        message_type = obj['ros/type']
        dictionary = obj['ros/data']
        return convert_dictionary_to_ros_message(message_type, dictionary)


class BaseExceptionHandler(jsonpickle.handlers.BaseHandler):
    """
    JSONPickle handler to convert exceptions to and from dictionaries.
    """

    def flatten(self, obj, data):
        data['exception/type'] = type(obj).__name__
        data['exception/message'] = str(obj)
        data['exception/traceback'] = traceback.format_tb(obj.__traceback__)
        return data

    def restore(self, obj):
        exc_type = obj['exception/type']
        exc_message = obj['exception/message']

        # Reconstruct the exception object; does not include traceback
        exc_class = getattr(builtins, exc_type)
        exc_instance = exc_class(exc_message)

        return exc_instance


def register_custom_jsonpickle_handlers():
    """
    Register all custom JSONPickle handlers.
    """
    interface_classes = get_interface_classes()
    for cls in interface_classes:
        jsonpickle.handlers.register(cls, ROSMessageHandler, base=True)
    logger.info(f'Registered {len(interface_classes)} ROS interfaces')

    jsonpickle.handlers.register(BaseException, BaseExceptionHandler, base=True)


def get_interface_classes():
    """
    Return all ROS 2 interface classes (including custom interfaces).
    """
    interfaces = rosidl_runtime_py.get_interfaces()

    all_classes = []
    for package, interface_names in interfaces.items():
        for name in interface_names:
            _type = name.split("/")[0]  # msg, srv, action

            suffixes = {
                "msg": [""],
                "srv": ["", "_Event", "_Request", "_Response"],

                # TODO: Action messages don't seem to be working even though we are registering all possible classes
                "action": [
                    "",
                    "_GetResult_Event",
                    "_GetResult_Request",
                    "_GetResult_Response",
                    "_SendGoal_Event",
                    "_SendGoal_Request",
                    "_SendGoal_Response",
                ],
            }

            for suffix in suffixes[_type]:
                module = importlib.import_module(f"{package}.{_type}")
                cls = getattr(module, name.split("/")[-1] + suffix)
                all_classes.append(cls)

    return all_classes
