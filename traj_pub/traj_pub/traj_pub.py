import rclpy
from rclpy.node import Node
from interface.msg import Traj
import math

class Trajectory_Publisher(Node):

    def __init__(self):
        super().__init__('trajectory_publisher_py')
        self.publisher_ = self.create_publisher(Traj, 'traj', 10)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        self.i += 1
        traj = Traj()
        # traj.x =  math.cos((self.i) * 0.1 * 0.05 * math.pi)
        # traj.y =  math.sin((self.i) * 0.1 * 0.05 * math.pi)
        traj.y = 2.0
        traj.x = self.i * 0.1 * 0.5 - 10
        self.publisher_.publish(traj)
        self.get_logger().info('coordiate: X = %f, Y = %f t = %i' % (traj.x,traj.y,self.i))

def main(args=None):
    rclpy.init(args=args)
    trajectory_publisher = Trajectory_Publisher()
    rclpy.spin(trajectory_publisher)
    rclpy.shutdown()

if __name__ == '__main__':
    main()