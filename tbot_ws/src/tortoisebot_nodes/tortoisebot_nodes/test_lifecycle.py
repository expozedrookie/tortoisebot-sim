import rclpy
from rclpy.lifecycle import Node
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn


class SimpleLifecycleNode(Node):

    def __init__(self):
        super().__init__('simple_lifecycle_node')
        self.get_logger().info("Node created — currently in UNCONFIGURED state")

    def on_configure(self, state: State):
        self.get_logger().info("on_configure() called — setting up resources")
        # Imagine loading parameters, initializing publishers, etc.
        self.get_logger().info("Configuration done")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State):
        self.get_logger().info("on_activate() called — starting main functionality")
        # Here you might start publishers, timers, etc.
        self.get_logger().info("Node is now ACTIVE")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State):
        self.get_logger().info("on_deactivate() called — pausing activity")
        # Stop publishers/timers here
        self.get_logger().info("Node is now INACTIVE")
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State):
        self.get_logger().info("on_cleanup() called — releasing resources")
        # Cleanup anything you allocated in configure
        self.get_logger().info("Cleanup done — back to UNCONFIGURED")
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State):
        self.get_logger().info("on_shutdown() called — shutting down")
        return TransitionCallbackReturn.SUCCESS


def main(args=None):
    rclpy.init(args=args)
    node = SimpleLifecycleNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
