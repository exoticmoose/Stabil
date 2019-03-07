#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>





class TeleopBot {
	public:
		TeleopBot();

		ros::Publisher vel_pub_;
		geometry_msgs::Twist twist;
	private:
		void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
		void imuCallback(const sensor_msgs::Imu::ConstPtr& imu);
		ros::NodeHandle nh_;

		ros::Subscriber joy_sub_;

		std_msgs::Float64 body_tilt_x;
		std_msgs::Float64 body_tilt_y;

};

TeleopBot::TeleopBot() {
	ROS_INFO("Created teleop_bot");

	vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopBot::joyCallback, this);

}


void TeleopBot::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
	//ROS_INFO("Joy callback");

	twist.linear.x = joy->axes[0]; // TODO: angular naming convention
	twist.linear.y = joy->axes[1];

	twist.angular.x = joy->axes[3];
	twist.angular.y = joy->axes[4];

	twist.linear.z = joy->axes[2];
	twist.angular.z = joy->axes[5];
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_bot");
  ROS_INFO("Starting node: teleop_bot");

  TeleopBot teleop_bot;

  ros::Rate loop_rate(50);

  while (ros::ok()) {

	  teleop_bot.vel_pub_.publish(teleop_bot.twist);
	  //ROS_INFO("Publishing %f \t %f \t %f", teleop_bot.twist.linear.x, teleop_bot.twist.linear.y, teleop_bot.twist.linear.z);

	  ros::spinOnce();
	  loop_rate.sleep();
  }
}


