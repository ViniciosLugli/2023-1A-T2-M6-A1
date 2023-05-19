import rclpy
from controllers import TurtleController, PathController


def main(args=None):
    rclpy.init()

    path_controller = PathController('positions')
    turtle_controller = TurtleController(path_controller)
    rclpy.spin(turtle_controller)

    turtle_controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
