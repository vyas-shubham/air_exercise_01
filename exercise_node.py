#!/usr/bin/env python3
import threading
import time
import rclpy
from rclpy import Parameter
from interfaces.msg import QuadControlTarget
from interfaces.msg import QuadState
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node



class WalkingController(Node):
    def __init__(self):
        super().__init__('walking_controller')
        # Basic Interfaces to the Simulator and Controller
        # Get State from the Simulator
        self.quad_state_sub = self.create_subscription(QuadState, 'quad_state', self.quad_state_callback, 1)
        self.print_timer = self.create_timer(2.0, self.print_callback)
        self.quad_control_target_pub = self.create_publisher(QuadControlTarget, 'quad_control_target', 10)
        self.quad_state = None

        # Check if goal is reached
        self._goal_reached = False
        self.goal_check_timer = self.create_timer(0.1, self.goal_check_callback)

        # Command the Robot to Move
        self.param_client = self.create_client(SetParameters, '/mit_controller_node/set_parameters')

    def quad_state_callback(self, msg):
        self.quad_state = msg

    def print_callback(self):
        if self.quad_state is not None:
            # print(f"Quad state: {self.quad_state}")
            # print(f"Position: {self.quad_state.pose.pose.position}")
            # print(f"X Velocity: {self.quad_state.twist.twist.linear.x}")
            print(f"Y Velocity: {self.quad_state.twist.twist.linear.y}")
            pass

    def goal_check_callback(self):
        if self.quad_state is not None:
            if self.quad_state.pose.pose.position.x > 1:
                self._goal_reached = True
                print("Goal reached!!")
                cmdMsg = QuadControlTarget()
                cmdMsg.body_x_dot = 0.0
                cmdMsg.body_y_dot = 0.0
                cmdMsg.world_z = 0.30
                cmdMsg.hybrid_theta_dot = 0.0
                cmdMsg.pitch = 0.0
                cmdMsg.roll = 0.0
                self.quad_control_target_pub.publish(cmdMsg)
                param_req = SetParameters.Request()
                param_req.parameters = [
                        Parameter(name='simple_gait_sequencer.gait', value="STAND").to_parameter_msg()]
                self.param_client.call_async(param_req)
                rclpy.shutdown()

    def move_forward(self):
        while self.quad_state is None:
            time.sleep(0.1)
        if self.quad_state is not None:
            print("Moving forward")
            param_req = SetParameters.Request()
            param_req.parameters = [
                    Parameter(name='simple_gait_sequencer.gait', value="WALKING_TROT").to_parameter_msg()]
            self.param_client.call_async(param_req)
            time.sleep(1)
            cmdMsg = QuadControlTarget()
            cmdMsg.body_x_dot = 0.5
            cmdMsg.body_y_dot = 0.0
            cmdMsg.world_z = 0.30
            cmdMsg.hybrid_theta_dot = 0.0
            cmdMsg.pitch = 0.0
            cmdMsg.roll = 0.0
            self.quad_control_target_pub.publish(cmdMsg)


def main(args=None):
    rclpy.init(args=args)
    walking_controller = WalkingController()
    def run_func():
        executor = rclpy.executors.MultiThreadedExecutor(2)
        executor.add_node(walking_controller)
        while rclpy.ok():
            walking_controller.get_logger().info("Running")
            executor.spin()
            return

    runner = threading.Thread(target=run_func)
    runner.start()
    walking_controller.move_forward()
    while rclpy.ok():
        time.sleep(0.1)
    walking_controller.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
