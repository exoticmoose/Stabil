#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>





class TeleopBot {
	public:
		TeleopBot();
	private:
		void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
		ros::NodeHandle nh_;

		int linear_, angular_;
		double l_scale_, a_scale_;

		ros::Publisher vel_pub_;
		ros::Subscriber joy_sub_;
};

TeleopBot::TeleopBot(): linear_(1), angular_(2) {
	ROS_INFO("Created teleop_bot");

	linear_ = 0;
	angular_ = 1;
	a_scale_ = 1;
	l_scale_ = 1;
//	nh_.param("axis_linear", linear_, linear_);
//	nh_.param("axis_angular", angular_, angular_);
//	nh_.param("scale_angular", a_scale_, a_scale_);
//	nh_.param("scale_linear", l_scale_, l_scale_);

	vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopBot::joyCallback, this);
}


void TeleopBot::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
	ROS_INFO("Joy callback");
  geometry_msgs::Twist twist;
  twist.linear.x = l_scale_*joy->axes[0]; // TODO: angular naming convention
  twist.linear.y = l_scale_*joy->axes[1];

  twist.angular.x = a_scale_*joy->axes[3];
  twist.angular.y = a_scale_*joy->axes[4];

  twist.linear.z = joy->axes[2];
  twist.angular.z = joy->axes[5];
  vel_pub_.publish(twist);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_bot");
  TeleopBot teleop_bot;

  ros::spin();
}


