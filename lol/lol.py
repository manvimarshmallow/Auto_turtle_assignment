import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import TeleportAbsolute
from std_srvs.srv import Empty

class mover(Node):
    def __init__(self):
        super().__init__('moveline')
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.subscriber = self.create_subscription(Pose, '/turtle1/pose', self.callback, 100)
        self.timer_period = 0.005
        self.timer = self.create_timer(self.timer_period, self.publish_commands)
        self.pose=Pose()
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.t=0
   
   
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
  
        if (self.pose.x)<5.45 and self.pose.x> 5.3 and self.pose.y>5.45 and self.pose.y <5.6:
            self.angular_velocity=self.t
            self.teleport(5.5,5.544445,0.0000)
        if abs(self.pose.x)>5.6:
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

def main(args=None):
    rclpy.init(args=args)
    move_line = mover() 
    linear_vel = float(input("Enter linear velocity: "))
    angular_vel = float(input("Enter angular velocity: "))

    move_line.linear_velocity = linear_vel
    move_line.angular_velocity = angular_vel
    move_line.t=angular_vel
    

    rclpy.spin(move_line)

    move_line.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()