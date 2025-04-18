import os
from dataclasses import dataclass
from enum import Enum
from typing import ClassVar

from custom_msgs.srv import SonarSweepRequest
from rclpy.client import Client
from rclpy.logging import get_logger
from rclpy.node import Node
from rclpy.task import Future
from task_planning.utils.other_utils import singleton

logger = get_logger('sonar_interface')

@singleton
class Sonar:
    """Interface for sonar."""

    SONAR_SWEEP_REQUEST_SERVICE = '/sonar/request'

    def __init__(self, node: Node, bypass: bool = False) -> None:
        self.node = node
        self.bypass = bypass

        if not self.bypass:
            self._sonar_request = node.create_client(SonarSweepRequest, self.SONAR_SWEEP_REQUEST_SERVICE)
            while not self._sonar_request.wait_for_service(timeout_sec=1.0):
                logger.info(f'{self.SONAR_SWEEP_REQUEST_SERVICE} not ready, waiting...')

