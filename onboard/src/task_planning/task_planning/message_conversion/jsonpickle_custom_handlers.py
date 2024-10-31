import builtins
import jsonpickle
import traceback
import rosidl_runtime_py
from rclpy.logging import get_logger
from rosidl_runtime_py import get_message_interfaces


from task_planning.message_conversion.ros_message_converter import convert_ros_message_to_dictionary, \
    convert_dictionary_to_ros_message


logger = get_logger('jsonpickle_custom_handlers')

class ROSMessageHandler(jsonpickle.handlers.BaseHandler):
    """
    JSONPickle handler to convert ROS messages to and from dictionaries
    """
    def flatten(self, obj, data):
        data['ros/type'] = obj._type
        data['ros/data'] = convert_ros_message_to_dictionary(obj)
        return data

    def restore(self, obj):
        message_type = obj['ros/type']
        dictionary = obj['ros/data']
        return convert_dictionary_to_ros_message(message_type, dictionary)


class BaseExceptionHandler(jsonpickle.handlers.BaseHandler):
    """
    JSONPickle handler to convert exceptions to and from dictionaries
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
    Register all custom JSONPickle handlers
    """
    # Register ROS 2 message types (including custom messages)
    message_types = get_all_message_types()
    for message_type in message_types:
        jsonpickle.handlers.register(message_type, ROSMessageHandler, base=True)
    logger.info(f'Registered {len(message_types)} message types')

    jsonpickle.handlers.register(BaseException, BaseExceptionHandler, base=True)

def get_all_message_types():
    message_interfaces = rosidl_runtime_py.get_message_interfaces()

    all_message_types = []
    for package_name, message_names in message_interfaces.items():
        for message_name in message_names:
            module = __import__(f'{package_name}.msg', fromlist=[message_name])
            message_type = getattr(module, message_name.split('/')[-1])
            all_message_types.append(message_type)

    return all_message_types
