from rclpy.node import Node
from geometry_msgs.msg import Vector3
from custom_types import PoseData
from .path import PathController

from modules.clients import Reset
from modules.publishers import Velocity
from modules.subscribers import Pose as PoseM
from modules.modules_controller import ModulesController

class TurtleController(Node):
    __current_turtle1_pose = PoseData()
    __set_point_turtle1_pose = None

    __modules_controller = ModulesController()

    __target_path = None
    __inverse_target_path = None

    __finished = False


    def __update_current_turtle1_pose(self, _pose):
        self.__current_turtle1_pose = _pose
        if self.__set_point_turtle1_pose is None:
            self.__set_point_turtle1_pose =  PoseData(self.__target_path.current_position().x + 5.0, self.__target_path.current_position().y + 5.0)
            self.get_logger().info("Set point: {}".format(_pose))
            self.get_logger().info("Target: {}".format(self.__set_point_turtle1_pose))
            self.get_logger().info("Current: {}".format(self.__current_turtle1_pose))

    def __init__(self, path_controller: PathController):
        super().__init__("turtle_controller")

        self.__target_path = path_controller
        self.__inverse_target_path = path_controller.get_inverse_path()

        self.__modules_controller.add(PoseM(self, "turtle1", self.__update_current_turtle1_pose))
        self.__modules_controller.add(Velocity(self, "turtle1"))
        self.__modules_controller.add(Reset(self))

        self.__modules_controller.access("reset").execute()

        self.runtime_timer = self.create_timer(0.01, self.runtime)


    def runtime(self):



        if self.__set_point_turtle1_pose is None or self.__finished:
            return

        force = Vector3()

        x_diff = (self.__set_point_turtle1_pose.x - self.__current_turtle1_pose.x)
        y_diff = (self.__set_point_turtle1_pose.y - self.__current_turtle1_pose.y)

        if abs(y_diff) > PoseData.MAX_VARIANCE_DIFF:
            force.y = 0.5 if y_diff > 0 else -0.5
        else:
            force.y = 0.0

        if abs(x_diff) > PoseData.MAX_VARIANCE_DIFF:
            force.x = 0.5 if x_diff > 0 else -0.5
        else:
            force.x = 0.0

        force.z = 0.0

        is_close = abs(x_diff) < PoseData.MAX_VARIANCE_DIFF and abs(y_diff) < PoseData.MAX_VARIANCE_DIFF

        if is_close:
            self.get_logger().info("Close to target: {}".format(self.__target_path.current_position()))
            self.__target_path.dequeue()

            if len(self.__target_path) == 0:
                self.__finished = True
                return
            self.__set_point_turtle1_pose =  PoseData(self.__target_path.current_position().x + self.__current_turtle1_pose.x, self.__target_path.current_position().y + self.__current_turtle1_pose.y)

        self.__modules_controller.access("velocity").apply(force,  Vector3(x=0.0, y=0.0, z=0.0))

