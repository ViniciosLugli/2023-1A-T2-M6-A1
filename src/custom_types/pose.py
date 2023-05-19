from turtlesim.msg import Pose

class PoseData(Pose):
    '''
        Represents a pose of a turtle. Inherits from turtlesim.msg.Pose
        * Add some methods to make it easier to work with the pose
    '''

    MAX_VARIANCE_DIFF = 0.1

    def __init__(self, x=0.0, y=0.0, theta=0.0):
        super().__init__(x=x, y=y, theta=theta)

    def __repr__(self):
        return f'PoseData(x={self.x}, y={self.y}, theta={self.theta})'

    def __add__(self, _pose_data):
        # Add two PoseData objects together

        self.x += _pose_data.x
        self.y += _pose_data.y
        return self

    def __sub__(self, _pose_data):
        # Subtract two PoseData objects

        self.x -= _pose_data.x
        self.y -= _pose_data.y
        return self

    def __mul__(self, _pose_data):
        # Multiply two PoseData objects

        self.x *= _pose_data.x
        self.y *= _pose_data.y
        return self

    def __eq__(self, _pose_data):
        # Check if two PoseData objects are equal

        return abs(self.x - _pose_data.x) <= self.MAX_VARIANCE_DIFF and abs(self.y - _pose_data.y) <= self.MAX_VARIANCE_DIFF