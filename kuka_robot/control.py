import rclpy, math, time
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration

names_of_joints = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']

class ControlRobot(Node):
    def __init__(self):
        super().__init__("kuka_robot_controller")

        # Publisher for arm joint trajectories
        self.arm_publisher = self.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            10
        )
        # Action client for gripper commands
        # self.gripper_action_client = ActionClient(
        #     self, GripperCommand,
        #     '/gripper_controller/joint_trajectory'
        # )
        # self.get_logger().info("Waiting for gripper action server...")
        # self.gripper_action_client.wait_for_server()
        # self.get_logger().info("Gripper action server ready!")

        # Timer for periodic control
        self.timer_period = 1.0  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # Predefined joint positions
        self.positions = [1.57, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.gripper_open = 0.12
        self.gripper_closed = -0.63

        self.index = 0
        self.forward = True

    def timer_callback(self):
        # Publish arm joint trajectory
        traj_= JointTrajectory()
        traj_.joint_names = names_of_joints

        point= JointTrajectoryPoint()
        point.positions= self.positions

        traj_.points.append(point)
        self.arm_publisher.publish(traj_)
        self.get_logger().info(f"Published arm trajectory to positions: {self.positions}")

        # Send gripper command
        gripper_goal = GripperCommand.Goal()

def main(args=None):
    rclpy.init(args=args)
    control_robot = ControlRobot()
    rclpy.spin(control_robot)
    control_robot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()