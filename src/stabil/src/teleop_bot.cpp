#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>





class TeleopBot {
	public:
		TeleopBot();
	private:
		void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
		void imuCallback(const sensor_msgs::Imu::ConstPtr& imu);
		ros::NodeHandle nh_;

		int linear_, angular_;
		double l_scale_, a_scale_;

		ros::Publisher vel_pub_;
		ros::Subscriber joy_sub_;

		ros::Publisher latt_pub_;
		ros::Publisher long_pub_;
		ros::Subscriber imu_sub_;
};

TeleopBot::TeleopBot(): linear_(1), angular_(2) {
	ROS_INFO("Created teleop_bot");

	linear_ = 0;
	angular_ = 1;
	a_scale_ = 1;
	l_scale_ = 1;

	vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopBot::joyCallback, this);

	imu_sub_ = nh_.subscribe<sensor_msgs::Imu>("imu", 100, &TeleopBot::imuCallback, this);
	latt_pub_ = nh_.advertise<std_msgs::Float64>("latitude_tilt", 1);
	long_pub_ = nh_.advertise<std_msgs::Float64>("longitude_tilt", 1);
}


void TeleopBot::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
//ROS_INFO("Joy callback");
  geometry_msgs::Twist twist;
  twist.linear.x = l_scale_*joy->axes[0]; // TODO: angular naming convention
  twist.linear.y = l_scale_*joy->axes[1];

  twist.angular.x = a_scale_*joy->axes[3];
  twist.angular.y = a_scale_*joy->axes[4];

  twist.linear.z = joy->axes[2];
  twist.angular.z = joy->axes[5];
  vel_pub_.publish(twist);
}

void TeleopBot::imuCallback(const sensor_msgs::Imu::ConstPtr& imu) {
	std_msgs::Float64 latititude_tilt;
	std_msgs::Float64 longitude_tilt;

	ROS_INFO("IMU Callback");
	latititude_tilt.data = 200 * imu->orientation.x;
	longitude_tilt.data = 200 * imu->orientation.y;

	latt_pub_.publish(latititude_tilt);
	long_pub_.publish(longitude_tilt);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_bot");
  TeleopBot teleop_bot;

  ros::spin();
}


