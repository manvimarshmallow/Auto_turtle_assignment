import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
from std_srvs.srv import Empty


class mover(Node):
    def __init__(self):
        super().__init__('moveline')
        self.clean()
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.subscriber = self.create_subscription(Pose, '/turtle1/pose', self.callback, 100)
        self.timer_period = 0.2
        self.timer = self.create_timer(self.timer_period, self.publish_commands)
        self.pose=Pose()
        self.x = 0.0
        self.y = 0.0
        self.tol=0
   
   
    def callback(self, data):
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)
        self.pose.theta = round(self.pose.theta, 2)
        
    def publish_commands(self):
        msg = Twist()
        emp=Twist()
        self.get_logger().info(f"pose  {self.pose}")
        if self.dist()<self.tol:
            self.publisher.publish(emp)
        else:
            msg.linear.x = 0.5*self.dist()
            msg.angular.z= 2.5*(self.angle()-self.pose.theta)
            self.publisher.publish(msg)
            self.get_logger().info(f"Twist: Linear={msg.linear}, Angular={msg.angular}")
            self.get_logger().info(f"Angle: {self.angle()*180/math.pi}")


    def clean(self):
        empty_client = self.create_client(Empty, '/reset')
        while not empty_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        future = empty_client.call_async(Empty.Request())
        rclpy.spin_until_future_complete(self, future)       


   
    def dist(self):
        d=(self.pose.x-self.x)**2+ (self.pose.y-self.y)**2
        return math.sqrt(d)
    
    def angle(self):
        y=(self.y-self.pose.y)
        x=(self.x-self.pose.x)
        return math.atan2(y,x)
       
        
        


def main(args=None):
    rclpy.init(args=args)
    move_line = mover() 
    x = float(input("Enter x: "))
    y = float(input("Enter y: "))
    tol=float(input("Enter tolerance: "))

    move_line.x= x
    move_line.y = y
    move_line.tol=tol
    

    rclpy.spin(move_line)

    move_line.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    