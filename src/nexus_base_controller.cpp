/*
 *
 * File: nexus_base_controller.cpp
 * Purpose: ros nexus base controller node.
 * Version: 1.0.0
 * File Date: 21-03-2020
 * Release Date: 21-03-2020
 * URL: https://github.com/MartinStokroos/nexus_base_ros
 * License: MIT License
 *
 *
 * Copyright (c) M.Stokroos 2020
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy,
 * modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *

*
				        Power Switch
				        Sonar0x11
				 -------------------------
				/                         \
			       /		           \
			      /			            \
			m2   /			             \ m1
		       INT0 /			              \INT1
			   /			               \
			  /			                \
			 /			                 \
			 \			                 /
			  \			                /
		  	   \			               /
			    \			              /
		  Sonar0x12  \		  	             / Sonar0x13
			      \		                    /
			       \	                   /
				--------------------------
					    m0

 */


#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <math.h>
#include "nexus_base_ros/Encoders.h"
#include "nexus_base_ros/Motors.h"
#include "pid_controller.h"


#define LOOP_RATE 10
#define QUEUE_SIZE 1 //subscriber buffer size
#define WHEEL_RADIUS 0.05 // [m]
#define ENC_CPR 12.0	// encoder counts/rev.
#define GEAR_REDUC 64.0 // gears reduction ratio
#define TS (1/20.0)	// loop period in wheel-base Arduino (via parameter sever?)
#define CPP2RADPS (2.0*M_PI/(TS*ENC_CPR*GEAR_REDUC))	// counts per loop period to rad/s conversion factor.
#define DEADBAND 10 // Stops actuating motors when: -DEADBAND < actuation < DEADBAND
		    // too large values will lead to instabillity 

using namespace std;



class NexusBaseController
{
public:
	NexusBaseController();

private:
	void cmdVelCallBack(const geometry_msgs::Twist::ConstPtr& twist_aux);
	void rawVelCallBack(const nexus_base_ros::Encoders::ConstPtr& rawvel_aux);

	ros::NodeHandle nh_;
	ros::Publisher cmd_motor_pub_;
	ros::Publisher odom_pub_;
	ros::Subscriber cmd_vel_sub_;
	ros::Subscriber raw_vel_sub_;
	ros::Time last_time;
	nav_msgs::Odometry odom;
	tf2::Quaternion odom_quat;
	geometry_msgs::TransformStamped odom_trans;

	const float Kp = 4.0; //controller coeff. (via parameter server?)
	const float Ki = 25.0;
	const float Kd = 0.75;
	const float minOutput = -150;
	const float maxOutput = 150;
	double cmd_wheel_m0_; // [rad/s]
	double cmd_wheel_m1_; // [rad/s]
	double cmd_wheel_m2_; // [rad/s]
	double prev_cmd_wheel_m0_;
	double prev_cmd_wheel_m1_;
	double prev_cmd_wheel_m2_;
	double vel_wheel_m0_; // [rad/s]
	double vel_wheel_m1_; // [rad/s]
	double vel_wheel_m2_; // [rad/s]
	double prev_vel_wheel_m0_;
	double prev_vel_wheel_m1_;
	double prev_vel_wheel_m2_;
	double prev_cmd_motor_m0_;
	double prev_cmd_motor_m1_;
	double prev_cmd_motor_m2_;
	double linear_velocity_x_;
	double linear_velocity_y_;
	double angular_velocity_z_;
	double dt_;
	double x_pos_;
	double y_pos_;
	double heading_;
	double ax_;
	double ay_;
	double omega_;
	double prev_ax_;
	double prev_ay_;
	double prev_omega_;


	//instantiate PIDs
	PIDControl myPID_wheel_m0_ = PIDControl(Kp, Ki, Kd, TS, minOutput, maxOutput, AUTOMATIC, DIRECT);
	PIDControl myPID_wheel_m1_ = PIDControl(Kp, Ki, Kd, TS, minOutput, maxOutput, AUTOMATIC, DIRECT);
	PIDControl myPID_wheel_m2_ = PIDControl(Kp, Ki, Kd, TS, minOutput, maxOutput, AUTOMATIC, DIRECT);
};





NexusBaseController::NexusBaseController():
	cmd_wheel_m0_(0),
	cmd_wheel_m1_(0),
	cmd_wheel_m2_(0),
	prev_cmd_wheel_m0_(0),
	prev_cmd_wheel_m1_(0),
	prev_cmd_wheel_m2_(0),
	vel_wheel_m0_(0),
	vel_wheel_m1_(0),
	vel_wheel_m2_(0),
	prev_vel_wheel_m0_(0),
	prev_vel_wheel_m1_(0),
	prev_vel_wheel_m2_(0),
	linear_velocity_x_(0),
	linear_velocity_y_(0),
	angular_velocity_z_(0),
	dt_(0),
	x_pos_(0),
	y_pos_(0),
	heading_(0),
	ax_(0),
	ay_(0),
	omega_(0),
        prev_ax_(0),
        prev_ay_(0),
        prev_omega_(0)
{
	cmd_motor_pub_ = nh_.advertise<nexus_base_ros::Motors>("cmd_motor", QUEUE_SIZE);
	odom_pub_ = nh_.advertise<nav_msgs::Odometry>("sensor_odom", QUEUE_SIZE);
	cmd_vel_sub_ = nh_.subscribe<geometry_msgs::Twist>("cmd_vel", QUEUE_SIZE, &NexusBaseController::cmdVelCallBack, this);
	raw_vel_sub_ = nh_.subscribe<nexus_base_ros::Encoders>("wheel_vel", QUEUE_SIZE, &NexusBaseController::rawVelCallBack, this);
}	




void NexusBaseController::cmdVelCallBack(const geometry_msgs::Twist::ConstPtr& twist_aux)
{
	// - Forward kinematics -
        // https://yainnoware.blogspot.com/2019/03/three-wheeled-holonomic-robot-theory.html

	ax_ = twist_aux->linear.x;
	ay_ = twist_aux->linear.y;
	omega_ = twist_aux->angular.z;
   
        ROS_INFO("\nax=%.2f\nay=%.2f\nomega=%.2f\n", 
		ax_, ay_, omega_);

        cmd_wheel_m0_ = ((-2./3.) * ax_) + ((1./3.) * omega_);
        cmd_wheel_m1_ = ((1./3.) * ax_) + ((1./sqrt(3.)) * ay_) + ((1./3.) * omega_);
        cmd_wheel_m2_ = -((1./3.) * ax_) - ((1./sqrt(3.)) * ay_) + ((1./3.) * omega_);
	        
       
	//store the old values
	prev_ax_ = ax_;
	prev_ay_ = ax_;
	prev_omega_ = omega_;


	
	//print to console for debugging purpose
	if( (cmd_wheel_m2_!=prev_cmd_wheel_m2_) || 
		(cmd_wheel_m1_!=prev_cmd_wheel_m1_) || 
		(cmd_wheel_m0_!=prev_cmd_wheel_m0_)) {
		ROS_INFO("\ncmd_m0=%.2f\ncmd_m1=%.2f\ncmd_m2=%.2f\n", 
			cmd_wheel_m0_, cmd_wheel_m1_, cmd_wheel_m2_);
	}

	//store the old values
	prev_cmd_wheel_m0_ = cmd_wheel_m0_;
	prev_cmd_wheel_m1_ = cmd_wheel_m1_;
	prev_cmd_wheel_m2_ = cmd_wheel_m2_;
}





void NexusBaseController::rawVelCallBack(const nexus_base_ros::Encoders::ConstPtr& rawvel_aux)
{
	nexus_base_ros::Motors cmd_motor;
	geometry_msgs::Twist twist;
        nav_msgs::Odometry odom;
	
	// timing
	ros::Time current_time = ros::Time::now();
	dt_ = (current_time - last_time).toSec();
	last_time = current_time;
	//ROS_INFO("dt=%.2f\n", dt_);	//check loop time

	// read encoders. Units are in encoder-ticks/loop-period

	vel_wheel_m0_ = rawvel_aux->enc0 * CPP2RADPS;
	vel_wheel_m1_ = rawvel_aux->enc1 * CPP2RADPS;
	vel_wheel_m2_ = rawvel_aux->enc2 * CPP2RADPS;

	// print to console for debugging purpose
	if( (vel_wheel_m2_ != prev_vel_wheel_m2_) || 
		(vel_wheel_m1_ != prev_vel_wheel_m1_) || 
		(vel_wheel_m0_ != prev_vel_wheel_m0_)) {
                ROS_INFO("\nset_m0=%.2f\nset_m1=%.2f\nset_m2=%.2f\n", 
			cmd_wheel_m0_, cmd_wheel_m1_, cmd_wheel_m2_);
		ROS_INFO("\nvel_m0=%.2f\nvel_m1=%.2f\nvel_m2=%.2f\n", 
			vel_wheel_m0_, vel_wheel_m1_, vel_wheel_m2_);
	}

	prev_vel_wheel_m0_ = vel_wheel_m0_; // store the old values
	prev_vel_wheel_m1_ = vel_wheel_m1_;
	prev_vel_wheel_m2_ = vel_wheel_m2_;

	// - Inverse Kinematics -
	// https://yainnoware.blogspot.com/2019/03/three-wheeled-holonomic-robot-theory.html

	double delta_heading = vel_wheel_m2_ + vel_wheel_m0_ + vel_wheel_m1_ * dt_; // [radians]
    	double delta_x = ((sqrt(3)/2) * vel_wheel_m2_) - ((sqrt(3)/2) * vel_wheel_m0_) * dt_; // [m]
    	double delta_y = -((1/2) * vel_wheel_m2_) - ((1/2) * vel_wheel_m0_) + vel_wheel_m1_ * dt_; // [m]

	//calculate current position of the robot
    	x_pos_ += delta_x;
    	y_pos_ += delta_y;
    	heading_ += delta_heading;
	//ROS_INFO("\nx=%.1f, y=%.1f, heading=%.2f\n", x_pos_, y_pos_, heading_);

	// uncomment for feed forward (open loop) testing.
/*	cmd_motor.motor0 = (short) lround(10*cmd_wheel_m0_);
	cmd_motor.motor1 = (short) lround(10*cmd_wheel_m1_);
	cmd_motor.motor2 = (short) lround(10*cmd_wheel_m2_);
*/
	// do PID control
	myPID_wheel_m0_.PIDSetpointSet(cmd_wheel_m0_);
	myPID_wheel_m0_.PIDInputSet(vel_wheel_m0_);
	myPID_wheel_m0_.PIDCompute();
	myPID_wheel_m1_.PIDSetpointSet(cmd_wheel_m1_);
	myPID_wheel_m1_.PIDInputSet(-vel_wheel_m1_);
	myPID_wheel_m1_.PIDCompute();
	myPID_wheel_m2_.PIDSetpointSet(cmd_wheel_m2_);
	myPID_wheel_m2_.PIDInputSet(vel_wheel_m2_);
	myPID_wheel_m2_.PIDCompute();

	// uncomment for closed loop feedback.
	cmd_motor.motor0 = (short) round(myPID_wheel_m0_.PIDOutputGet());
	cmd_motor.motor1 = (short) round(myPID_wheel_m1_.PIDOutputGet());
	cmd_motor.motor2 = (short) round(myPID_wheel_m2_.PIDOutputGet());

	//publish to motors when actuation is outside DEADBAND margin only.
/*	if( cmd_motor.motor0 <= -DEADBAND || cmd_motor.motor0 >= DEADBAND ||
		cmd_motor.motor1 <= -DEADBAND || cmd_motor.motor1 >= DEADBAND ||
		cmd_motor.motor2 <= -DEADBAND || cmd_motor.motor2 >= DEADBAND )
		{
	        cmd_motor_pub_.publish(cmd_motor);
		}
*/
	// publish to motors only when actuation value changes.
	if( (cmd_motor.motor0 != prev_cmd_motor_m0_) || 
		(cmd_motor.motor1 != prev_cmd_motor_m1_) || 
		(cmd_motor.motor2 != prev_cmd_motor_m2_) )
		{
		cmd_motor_pub_.publish(cmd_motor);
		}

	prev_cmd_motor_m0_ = cmd_motor.motor0; // store the old values//
	prev_cmd_motor_m1_ = cmd_motor.motor1;
	prev_cmd_motor_m2_ = cmd_motor.motor2;



	// The code below for odometry has been copied from the Linobot project, https://github.com/linorobot/linorobot
	// calculate robot's heading in quaternion angle
	// ROS has a function to calculate yaw in quaternion angle
	odom_quat.setRPY(0,0,heading_);
	odom_trans.header.frame_id = "odom";
	odom_trans.child_frame_id = "base_footprint";
	// robot's position in x,y, and z
	odom_trans.transform.translation.x = x_pos_;
	odom_trans.transform.translation.y = y_pos_;
	odom_trans.transform.translation.z = 0.0;
	// robot's heading in quaternion
	odom_trans.transform.rotation.x = odom_quat.x();
	odom_trans.transform.rotation.y = odom_quat.y();
	odom_trans.transform.rotation.z = odom_quat.z();
	odom_trans.transform.rotation.w = odom_quat.w();
	odom_trans.header.stamp = current_time;
	// publish robot's tf using odom_trans object
	// odom_broadcaster_.sendTransform(odom_trans);
	odom.header.stamp = current_time;
	odom.header.frame_id = "odom";
	odom.child_frame_id = "base_footprint";
	// robot's position in x,y, and z
	odom.pose.pose.position.x = x_pos_;
	odom.pose.pose.position.y = y_pos_;
	odom.pose.pose.position.z = 0.0;
	// robot's heading in quaternion
	odom.pose.pose.orientation.x = odom_quat.x();
	odom.pose.pose.orientation.y = odom_quat.y();
	odom.pose.pose.orientation.z = odom_quat.z();
	odom.pose.pose.orientation.w = odom_quat.w();
	odom.pose.covariance[0] = 0.001;
	odom.pose.covariance[7] = 0.001;
	odom.pose.covariance[35] = 0.001;
	// linear speed from encoders
	odom.twist.twist.linear.x = linear_velocity_x_;
	odom.twist.twist.linear.y = linear_velocity_y_;
	odom.twist.twist.linear.z = 0.0;
	odom.twist.twist.angular.x = 0.0;
	odom.twist.twist.angular.y = 0.0;
	// angular speed from encoders
	odom.twist.twist.angular.z = angular_velocity_z_;
	odom.twist.covariance[0] = 0.0001;
	odom.twist.covariance[7] = 0.0001;
	odom.twist.covariance[35] = 0.0001;
	odom_pub_.publish(odom);

	// check execution time. Make sure there is no overrun.
	//current_time = ros::Time::now();
	//double exect = 1000*(current_time - last_time).toSec();
	//ROS_INFO("exec. time(ms)=%.2f\n", exect);
}





int main(int argc, char** argv)
{
	ros::init(argc, argv, "nexus_base_controller");
	NexusBaseController nexus_base_controller;

	ros::spin();
}
