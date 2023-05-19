from rclpy.node import Node
from typing import Any
from functools import partial


class Client:
    def __init__(self, name: str, node: Node, srv_name: str, srv_type: Any):
        self.name = name
        self.node = node
        self.srv_type = srv_type
        self.srv_name = srv_name
        self.client = None

        self.node.get_logger().info(f"Client {self.name} created.")

        self.connect()

    def connect(self) -> None:
        self.client = self.node.create_client(
            self.srv_type, self.srv_name
        )

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info(
                f"Client {self.name} waiting for service {self.srv_name}."
            )

        self.node.get_logger().info(
            f"Client {self.name} connected to service {self.srv_name}."
        )

    def disconnect(self) -> None:
        self.node.destroy_client(self.client)
        self.node.get_logger().info(f"Client {self.name} disconnected.")

    def callback(self, future: Any) -> None:
        try:
            response = future.result()
            self.node.get_logger().info(
                f"Client {self.name} received response: {response}"
            )
        except Exception as e:
            self.node.get_logger().error(
                f"Client {self.name} failed to receive response: {e}"
            )

    def async_call(self, request: Any, custom_callback: Any = None) -> None:
        self.node.get_logger().info(f"Client {self.name} sending request: {request}")
        future = self.client.call_async(request)
        future.add_done_callback(
            partial(custom_callback) if custom_callback else partial(self.callback)
        )

    def call(self, request: Any) -> Any:
        self.node.get_logger().info(f"Client {self.name} sending request: {request}")
        return self.client.call(request)