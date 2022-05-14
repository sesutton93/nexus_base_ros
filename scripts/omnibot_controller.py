import rospy
from geometry_msgs.msg import Twist
import sys, select, os, time
import OmnibotController
import numpy as np
if os.name == 'nt':
  import msvcrt
else:
  import tty, termios

class OmnibotController:
    def __init__(self):

        if os.name != 'nt':
            settings = termios.tcgetattr(sys.stdin)

        rospy.init_node('nexus_teleop_key')
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.twist = Twist()

        self.twist.linear.y = 0.0
        self.twist.linear.x = 0.0
        self.twist.linear.z = 0.0
        self.twist.angular.z = 0.0

        self.pub.publish(self.twist)

        time.sleep(3)
        

    def turn(self, degrees):
        # Assuming a fixed turning time of 3 s, 1 degree is done with
        # an angluar speed set to 0.05
        
        speed = 0.05 * degrees
        
        #if dir == "left":
        #    speed = -speed
            
        self.twist.linear.y = 0.0
        self.twist.linear.x = 0.0
        self.twist.linear.z = 0.0
        self.twist.angular.z = speed

        self.pub.publish(self.twist)
        
        time.sleep(3)
    
    def move_step(self, dir):
        
        # A step is 10 cm and it is done in 2 s
        speed = 2.1
        
        self.twist.linear.y = 0.0
        self.twist.linear.x = 0.0
        self.twist.linear.z = 0.0
        self.twist.angular.z = 0.0
        
        if dir == "forward":
            self.twist.linear.y = speed
        elif dir == "backward":
            self.twist.linear.y = -speed
        elif dir == "left":
            self.twist.linear.x = -speed
        elif dir == "right":
            self.twist.linear.x = speed
        
        self.pub.publish(self.twist)    
        
        time.sleep(2)
        
    def move(self, length, dir):
        
        # Moves length cm in 2 seconds
        speed = length * 0.14
        
        self.twist.linear.y = 0.0
        self.twist.linear.x = 0.0
        self.twist.linear.z = 0.0
        self.twist.angular.z = 0.0
        
        if dir == "forward":
            self.twist.linear.y = speed
        elif dir == "backward":
            self.twist.linear.y = -speed
        elif dir == "left":
            self.twist.linear.x = -speed
        elif dir == "right":
            self.twist.linear.x = speed
        
        self.pub.publish(self.twist)    
        
        time.sleep(2)

    def stop(self):
        
        self.twist.linear.y = 0.0
        self.twist.linear.x = 0.0
        self.twist.linear.z = 0.0
        self.twist.angular.z = 0.0
        
        self.pub.publish(self.twist)
        
        time.sleep(3)
        
    def stop_clear(self):
        
        self.twist.linear.y = 0.0
        self.twist.linear.x = 0.0
        self.twist.linear.z = 0.0
        self.twist.angular.z = 0.0
        
        self.pub.publish(self.twist)
        
        time.sleep(3)

        if os.name != 'nt':
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)