import builtins
import importlib
import traceback
from collections.abc import Generator
from typing import Any

import jsonpickle
import rosidl_runtime_py
from rclpy.logging import get_logger
from task_planning.message_conversion.ros_message_converter import (
    convert_dictionary_to_ros_message,
    convert_ros_message_to_dictionary,
)

logger = get_logger('jsonpickle_custom_handlers')

ACTION_CLASSES_TO_TYPES = {
    'Goal': lambda cls_base: cls_base.Goal,
    'Result': lambda cls_base: cls_base.Result,
    'Feedback': lambda cls_base: cls_base.Feedback,
}
ACTION_CLASSES_TO_TYPES_KEYS_TUPLE = tuple(ACTION_CLASSES_TO_TYPES.keys())


class ROSMessageHandler(jsonpickle.handlers.BaseHandler):
    """JSONPickle handler to convert ROS messages to and from dictionaries."""

    def flatten(self, obj : Any, data: dict) -> dict:  # noqa: ANN401
        """Flattens a ROS message object into a dictionary with relevant details."""
        data['ros/type'] = (
            '/'.join(type(obj).__module__.split('.')[:-1]) + '/' + type(obj).__name__
        )
        data['ros/data'] = convert_ros_message_to_dictionary(obj)
        return data

    def restore(self, obj: dict) -> Any:  # noqa: ANN401
        """Restores a ROS message from a dictionary representation."""
        message_type = obj['ros/type']
        dictionary = obj['ros/data']

        # Handle action messages that can't be directly imported
        package, interface_type, cls_name = message_type.split('/')
        if interface_type == 'action' and cls_name.endswith(ACTION_CLASSES_TO_TYPES_KEYS_TUPLE):
            module = importlib.import_module(f'{package}.{interface_type}')
            cls_base_name, cls_suffix = cls_name.rsplit('_', 1)
            cls_base = getattr(module, cls_base_name)
            message_type = ACTION_CLASSES_TO_TYPES[cls_suffix](cls_base)

        return convert_dictionary_to_ros_message(message_type, dictionary)


class BaseExceptionHandler(jsonpickle.handlers.BaseHandler):
    """JSONPickle handler to convert exceptions to and from dictionaries."""

    def flatten(self, obj: BaseException, data: dict[str, Any]) -> dict[str, Any]:
        """Flattens an exception object into a dictionary with relevant details."""
        data['exception/type'] = type(obj).__name__
        data['exception/message'] = str(obj)
        data['exception/traceback'] = traceback.format_tb(obj.__traceback__)
        return data

    def restore(self, obj: dict) -> Exception:
        """
        Restores an exception instance from serialized exception data.

        This method reconstructs an exception object using the provided dictionary,
        which contains the type and message of the exception. The traceback is not included
        in the reconstructed exception.

        Args:
            obj (dict): A dictionary containing serialized exception data with the following keys:
                - 'exception/type' (str): The name of the exception class (e.g., 'ValueError').
                - 'exception/message' (str): The message of the exception.

        Returns:
            Exception: The reconstructed exception instance based on the type and message provided.

        Raises:
            AttributeError: If the specified exception type is not found in the `builtins` module.
        """
        exc_type = obj['exception/type']
        exc_message = obj['exception/message']

        # Reconstruct the exception object; does not include traceback
        exc_class = getattr(builtins, exc_type)

        return exc_class(exc_message)


def register_custom_jsonpickle_handlers() -> None:
    """Register all custom JSONPickle handlers."""
    num_interface_classes = 0
    for cls in get_interface_classes():
        jsonpickle.handlers.register(cls, ROSMessageHandler, base=True)
        num_interface_classes += 1
    logger.info('Registered %d interface types', num_interface_classes)

    jsonpickle.handlers.register(BaseException, BaseExceptionHandler, base=True)


def get_interface_classes() -> Generator[Any, None, None]:
    """Generate ROS 2 interface classes (including custom interfaces)."""
    interfaces = rosidl_runtime_py.get_interfaces()

    for package, interface_names in interfaces.items():
        for name in interface_names:
            interface_type, cls_name = name.split('/')
            module = importlib.import_module(f'{package}.{interface_type}')

            suffixes = {
                'msg': [''],
                'srv': ['', '_Event', '_Request', '_Response'],
                'action': [
                    '',
                    '_GetResult_Event',
                    '_GetResult_Request',
                    '_GetResult_Response',
                    '_SendGoal_Event',
                    '_SendGoal_Request',
                    '_SendGoal_Response',
                ],
            }

            for suffix in suffixes[interface_type]:
                cls = getattr(module, cls_name + suffix)
                yield cls

            # Action messages have additional classes that are only accessible through the main action class
            if interface_type == 'action':
                action_cls = getattr(module, cls_name)
                for class_to_type_func in ACTION_CLASSES_TO_TYPES.values():
                    yield class_to_type_func(action_cls)
