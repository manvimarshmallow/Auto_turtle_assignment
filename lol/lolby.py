import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import TeleportAbsolute
from std_srvs.srv import Empty
import random

class mover(Node):
    def __init__(self):
        super().__init__('moveline')
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.clean()
        self.x=round(random.uniform(3, 6),2)
        self.y=round(random.uniform(3, 6),2)
        self.teleport(self.x,self.y,0.000)
        self.clean()
        self.subscriber = self.create_subscription(Pose, '/turtle1/pose', self.callback, 100)
        self.timer_period = 0.005
        self.timer = self.create_timer(self.timer_period, self.publish_commands)
        self.pose=Pose()
        self.linear_velocity = round(random.uniform(0.5, 2),2)
        self.angular_velocity = round(random.uniform(1, 2),2)
        self.t=self.angular_velocity
      
        

    def callback(self, data):
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)
        self.pose.theta = round(self.pose.theta, 2)
        
    def publish_commands(self):
        msg = Twist()
        msg.linear.x = self.linear_velocity
        msg.angular.z= self.angular_velocity
        self.get_logger().info(f"pose x {self.pose}")
  
        if (self.pose.x)<self.x-0.1 and self.pose.x> self.x-0.2 and self.pose.y>self.y-0.1 and self.pose.y <self.y+0.1:
            self.angular_velocity=self.t
            self.teleport(self.x-0.04,self.y,0.0000)
        if abs(self.pose.x)>self.x+0.1:
            self.t=-self.angular_velocity
        self.publisher.publish(msg)
    
        

    def teleport(self, x, y, theta):
        teleport_client = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        while not teleport_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        request = TeleportAbsolute.Request()
        request.x = x
        request.y = y
        request.theta = theta
        future = teleport_client.call_async(request)
        #rclpy.spin_until_future_complete(self, future, timeout_sec=0.1)

    def clean(self):
        empty_client = self.create_client(Empty, '/clear')
        while not empty_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        future = empty_client.call_async(Empty.Request())
        #rclpy.spin_until_future_complete(self, future)

def main(args=None):
    rclpy.init(args=args)
    move_line = mover() 
    rclpy.spin(move_line)

    move_line.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()