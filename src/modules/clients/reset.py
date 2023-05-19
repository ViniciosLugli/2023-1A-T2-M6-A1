from .base import Client
from rclpy.node import Node
from std_srvs.srv._empty import (
    Empty_Request as EmptyMsg,
)
from std_srvs.srv import Empty as EmptySrv


class Reset(Client):
    def __init__(self, node: Node):
        super().__init__("reset", node, "reset", EmptySrv)

    def execute(self):
        self.async_call(EmptyMsg())