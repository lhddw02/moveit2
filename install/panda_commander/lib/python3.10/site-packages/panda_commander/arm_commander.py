import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class ArmCommander(Node):
    def __init__(self):
        super().__init__('arm_commander')
        self.publisher = self.create_publisher(JointTrajectory, '/panda_arm_controller/joint_trajectory', 10)
        self.timer = self.create_timer(2.0, self.send_command)

    def send_command(self):
        traj = JointTrajectory()
        traj.joint_names = [
            'panda_joint1', 'panda_joint2', 'panda_joint3',
            'panda_joint4', 'panda_joint5', 'panda_joint6'
        ]

        point = JointTrajectoryPoint()
        point.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # target angles
        point.time_from_start.sec = 3  # reach target in 3 seconds

        traj.points.append(point)
        self.publisher.publish(traj)
        self.get_logger().info("Trajectory sent!")

def main(args=None):
    rclpy.init(args=args)
    node = ArmCommander()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
