"""
Robot Controller Library
A Python library for controlling robots using ROS2 services and actions.
"""

from .robot_controller import RobotController
from .trajectory_controller import TrajectoryController
from .mpc_controller import MPCController

__version__ = "1.0.0"
__author__ = "Robot Control Team"

__all__ = [
    "RobotController",
    "TrajectoryController",
    "MPCController"
]

