"""Test angular velocity calculate."""

from turtle_brick.turtle_robot import calculate_angular_vel

def test_identity():
    assert calculate_angular_vel(10, 5) == 2
