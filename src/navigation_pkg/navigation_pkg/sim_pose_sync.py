"""Align Gazebo Classic model pose with map-frame localization."""

from __future__ import annotations

import math
from typing import Tuple

from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import SetEntityState


def yaw_to_quaternion(yaw: float) -> Tuple[float, float, float, float]:
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


def make_set_entity_state_request(
    entity_name: str,
    x: float,
    y: float,
    yaw: float,
    z: float = 0.0,
    reference_frame: str = 'world',
) -> SetEntityState.Request:
    req = SetEntityState.Request()
    req.state = EntityState()
    req.state.name = entity_name
    req.state.reference_frame = reference_frame
    req.state.pose.position.x = x
    req.state.pose.position.y = y
    req.state.pose.position.z = z
    qx, qy, qz, qw = yaw_to_quaternion(yaw)
    req.state.pose.orientation.x = qx
    req.state.pose.orientation.y = qy
    req.state.pose.orientation.z = qz
    req.state.pose.orientation.w = qw
    return req
