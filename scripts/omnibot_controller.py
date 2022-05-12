#!/usr/bin/env python

# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the Willow Garage, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rospy
from geometry_msgs.msg import Twist
import sys, select, os, time
if os.name == 'nt':
  import msvcrt
else:
  import tty, termios

MAX_LIN_VEL = 100.00
MAX_ANG_VEL = 100.00

LIN_VEL_STEP_SIZE = 5.
ANG_VEL_STEP_SIZE = 5.

class OmnibotController:
    def __init__(self) -> None:
        
        if os.name != 'nt':
            self.settings = termios.tcgetattr(sys.stdin)

        rospy.init_node('omnibot_controller')
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.twist = Twist()
        
        self.twist.linear.y = 0.0
        self.twist.linear.x = 0.0
        self.twist.linear.z = 0.0
        self.twist.angular.z = 0.0

        self.pub.publish(self.twist)

        time.sleep(3)
        

    def turn(self, degrees, dir):
        # Assuming a fixed turning time of 3 s, 1 degree is done with
        # an angluar speed set to 0.05
        
        speed = 0.05 * degrees
        
        if dir == "left":
            speed = -speed
            
        self.twist.linear.y = 0.0
        self.twist.linear.x = 0.0
        self.twist.linear.z = 0.0
        self.twist.angular.z = speed

        self.pub.publish(self.twist)
        
        time.sleep(3)
    
    def move(self, length, dir):
        
        # Update this!
        speed = 5.0
        
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
        
        time.sleep(3)
        
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
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)