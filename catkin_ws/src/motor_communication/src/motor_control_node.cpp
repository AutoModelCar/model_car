#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/UInt32MultiArray.h>

class motor_control {
private:
	ros::NodeHandle nh_;
	ros::Subscriber sub_speed_;
	ros::Subscriber sub_speed_feedback_;
	ros::Subscriber sub_steering_;
	ros::Subscriber sub_stop_;
	ros::Subscriber sub_led_front_;
	ros::Subscriber sub_led_back_;
	ros::Publisher pub_bldc_speed_;
	ros::Publisher pub_bldc_steering_;
	ros::Publisher pub_bldc_stop_start_;
	ros::Publisher pub_velocity_;

	int steering_min, steering_max;

public:
	motor_control(ros::NodeHandle nh) :
			nh_(nh) {
		nh_.param<int>("steering_min", steering_min, 900);
		nh_.param<int>("steering_max", steering_max, 2000);
		sub_speed_ = nh_.subscribe("motor_control/speed", 1, &motor_control::motorSpeedCallback, this);
		sub_speed_feedback_ = nh_.subscribe("bldc_board_communication/current_speed_feedback", 1,
				&motor_control::publishMotorTwist, this);
		sub_steering_ = nh.subscribe("manual_control/steering", 1, &motor_control::steeringCallback, this);
		sub_stop_ = nh_.subscribe("motor_control/stop_start", 1, &motor_control::motorStopStartCallback, this);

		pub_bldc_speed_ = nh_.advertise<std_msgs::Float64>(nh_.resolveName("bldc_board_communication/speed"), 1);
		pub_bldc_steering_ = nh_.advertise<std_msgs::Int16>(nh_.resolveName("bldc_board_communication/steering"), 1);
		pub_bldc_stop_start_ = nh_.advertise<std_msgs::Int16>(nh_.resolveName("bldc_board_communication/stop_start"),
				1);
		pub_velocity_ = nh_.advertise<geometry_msgs::Twist>(nh_.resolveName("motor_control/twist"), 1);
	}
	~motor_control() {
	}
	void motorSpeedCallback(const std_msgs::Int16 speed_value);
	void steeringCallback(const std_msgs::Int16 steering_value);
	void motorStopStartCallback(const std_msgs::Int16 stop_value);
	void ledFrontCallback(const std_msgs::UInt32MultiArray msg);
	void ledBackCallback(const std_msgs::UInt32MultiArray msg);
	void publishMotorTwist(const std_msgs::Float64 msg);
};

void motor_control::motorSpeedCallback(const std_msgs::Int16 speed_value) {
	// convert rpm to 0.5rotation/s
	std_msgs::Float64 msg;
	msg.data = (double) speed_value.data / 30.0;

	pub_bldc_speed_.publish(msg);
}

void motor_control::steeringCallback(const std_msgs::Int16 steering_value) {
	if (not (steering_min >= 0 && steering_min < steering_max)) {
		ROS_ERROR("param steering_min must be positive and < steering_max: Value = %i", steering_min);
		return;
	}
	if (not (steering_max >= 0 && steering_min < steering_max)) {
		ROS_ERROR("param steering_max must be positive and > steering_min: Value = %i", steering_max);
		return;
	}

	if (steering_value.data >= 0 && steering_value.data <= 180) {
		int angle = steering_value.data;
		int pwm = ((steering_max - steering_min) / 180) * angle + steering_min;
		pub_bldc_steering_.publish(pwm);
	} else {
		ROS_ERROR("Steering value out of bounds! Allowed min = 0, Allowed max = 180, Value = %i", steering_value.data);
	}
}

void motor_control::motorStopStartCallback(const std_msgs::Int16 stop_value) {
	pub_bldc_stop_start_.publish(stop_value);
}

void motor_control::publishMotorTwist(const std_msgs::Float64 speed_feedback) {
	// convert 0.5rotation/s to rpm with threshold of at least >=2rotation/s, 0 rpm otherwise
	double rpm = (abs(speed_feedback.data) >= 4.0) ? 30.0 * speed_feedback.data : 0.0;

	geometry_msgs::Twist currentTwist;
	currentTwist.linear.x = rpm;
	currentTwist.linear.y = 0.0;
	currentTwist.linear.z = 0.0;

	pub_velocity_.publish(currentTwist);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "motor_control_node");
	ros::NodeHandle nh;
	motor_control MC1(nh);

	std_msgs::Int16 stop_value;
	stop_value.data = 1;
	MC1.motorStopStartCallback(stop_value);

	ros::spin();

	return 0;
}
