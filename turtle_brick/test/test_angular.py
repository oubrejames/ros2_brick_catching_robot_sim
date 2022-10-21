"""Test angular velocity calculate."""

from turtle_brick.turtle_robot import calculate_angular_vel

def test_angular_vel():
    assert calculate_angular_vel(5, 10) == 2
