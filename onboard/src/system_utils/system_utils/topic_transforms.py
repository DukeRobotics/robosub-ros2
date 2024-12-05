#!/usr/bin/env python

import math

from geometry_msgs.msg import Pose, Twist, Vector3

from nav_msgs.msg import Odometry

from sensor_msgs.msg import Imu

import rclpy
from rclpy.node import Node

from transforms3d.euler import quat2euler


# Class to store data for topic transformations
class TopicTransformData:
    def __init__(self, input_topic, input_type, input_type_conversion, output_topic, output_type,
                 output_type_conversion, subscriber=None, publisher=None, publisher_queue_size=1, subscriber_queue_size=1):
        self.input_topic = input_topic
        self.input_type = input_type
        self.input_type_conversion = input_type_conversion
        self.output_topic = output_topic
        self.output_type = output_type
        self.output_type_conversion = output_type_conversion
        self.subscriber = subscriber
        self.publisher = publisher
        self.publisher_queue_size = publisher_queue_size
        self.subscriber_queue_size = subscriber_queue_size


# Class to store conversion functions
class Conversions:

    @staticmethod
    def quat_to_vector(quat_msg):
        # Convert quaternion to euler angles
        euler_angles = quat2euler([quat_msg.w, quat_msg.x, quat_msg.y, quat_msg.z])

        # Convert to degrees
        euler_angles = [math.degrees(angle) for angle in euler_angles]

        vector_msg = Vector3()
        vector_msg.x = euler_angles[0]
        vector_msg.y = euler_angles[1]
        vector_msg.z = euler_angles[2]

        return vector_msg

    @staticmethod
    def pose_to_twist(pose_msg):
        # Create a Twist message with identical linear position
        twist_msg = Twist()
        twist_msg.linear.x = pose_msg.position.x
        twist_msg.linear.y = pose_msg.position.y
        twist_msg.linear.z = pose_msg.position.z

        twist_msg.angular = Conversions.quat_to_vector(pose_msg.orientation)

        return twist_msg


# Class to perform topic transformations
class TopicTransforms(Node):
    NODE_NAME = 'topic_transforms'
    # List of topic transformation data
    TOPIC_TRANSFORM_DATA = [
        TopicTransformData('/state', Odometry, lambda x: x.pose.pose, '/transforms/state/pose', Twist,
                           Conversions.pose_to_twist),
        TopicTransformData('/vectornav/IMU', Imu, lambda x: x.orientation, '/transforms/vectornav/IMU/orientation',
                           Vector3, Conversions.quat_to_vector),
        TopicTransformData('/controls/desired_position', Pose, lambda x: x, '/transforms/controls/desired_position',
                           Twist, Conversions.pose_to_twist),
    ]

    def __init__(self):
        super().__init__(self.NODE_NAME)

        # Create subscribers and publishers for each topic transformation
        for data in self.TOPIC_TRANSFORM_DATA:
            data.subscriber = self.create_subscription(data.input_type, data.input_topic,
                                                       lambda msg, data=data: self.callback(data, msg),
                                                       data.subscriber_queue_size)
            data.publisher = self.create_publisher(data.output_type, data.output_topic, data.publisher_queue_size)

        self.get_logger().info("Topic transforms node started.")

    # Callback function to transform input message and publish output message
    # First, transform input message using input_type_conversion function, input the result to output_type_conversion,
    # and publish the final result
    def callback(self, data: TopicTransformData, msg):
        converted_input = data.input_type_conversion(msg)
        output_msg = data.output_type_conversion(converted_input)
        data.publisher.publish(output_msg)


def main(args=None):
    rclpy.init(args=args)
    topic_transforms = TopicTransforms()

    try:
        rclpy.spin(topic_transforms)
    except KeyboardInterrupt:
        pass
    finally:
        topic_transforms.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()