import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import TeleportAbsolute
from std_srvs.srv import Empty
from rcl_interfaces.srv import SetParameters
from rclpy.parameter import Parameter
import math
import random


class SetExternalParam(Node):
    def __init__(self, server_name):
        super().__init__('ParamSetter')
        self.cli = self.create_client(SetParameters, '/' + server_name + '/set_parameters')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SetParameters.Request()

    def send_request(self, param_name, param_value):
        self.req.parameters = [Parameter(name=param_name, value=param_value).to_parameter_msg()]
        self.future = self.cli.call_async(self.req)

class mover(Node):
    def __init__(self):
        super().__init__('moveline')
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.clean()
        self.x=1.0
        self.y=1.0
        self.side=0.0
        self.teleport(self.x,self.y,0.000)
        self.clean()
        self.change_background()
        self.subscriber = self.create_subscription(Pose, '/turtle1/pose', self.callback, 100)
        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.publish_commands)
        self.pose=Pose()
        self.linear_velocity = 1.0
        self.angular_velocity = 0.0
        
      

    def callback(self, data):
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)
        self.pose.theta = round(self.pose.theta, 2)
        
    def publish_commands(self):
        self.get_logger().info(f"pose x {self.pose}")
        msg = Twist()
        emp=Twist()
        if self.pose.y==1.0 and self.pose.x<self.side+1.0:
            msg.linear.x = 1.0+ self.side-self.pose.x
            self.publisher.publish(msg)
            if self.pose.linear_velocity<0.001 and self.pose.linear_velocity>0.0:
                self.teleport(1.0+self.side,1.0, math.pi/2)
                self.publisher.publish(emp)
        elif self.pose.x==1.0+self.side and self.pose.y< self.side+1.0:
            msg.linear.x = 1.0+ self.side-self.pose.y
            self.publisher.publish(msg)
            if self.pose.linear_velocity<0.001 and self.pose.linear_velocity>0:
                self.teleport(1.0+self.side,1.0+self.side, math.pi)
                self.publisher.publish(emp)
        elif self.pose.x>1.0 and self.pose.y== self.side+1.0:
            msg.linear.x = self.pose.x-1.0
            self.publisher.publish(msg)
            if self.pose.linear_velocity<0.001 and self.pose.linear_velocity>0:
                self.teleport(1.0,1.0+self.side, math.pi*3/2)
                self.publisher.publish(emp)
        elif self.pose.x==1.0 and self.pose.y> 1.0:
            msg.linear.x = self.pose.y-1.0
            self.publisher.publish(msg)
            if self.pose.linear_velocity<0.001 and self.pose.linear_velocity>0:
                self.teleport(1.0, 1.0 , 0.0)
                self.publisher.publish(emp)



         
               
        
      
            
    def change_background(self):
        change_cli = SetExternalParam('turtlesim')
        change_cli.send_request('background_r', 255)
        change_cli.send_request('background_g', 0)
        change_cli.send_request('background_b', 0)
        change_cli.destroy_node()
  

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
        rclpy.spin_until_future_complete(self, future)

def main(args=None):
    rclpy.init(args=args)
    move_line = mover() 
    move_line.side=float(input("enter side length "))
    rclpy.spin(move_line)
    move_line.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()