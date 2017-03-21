#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/UInt32MultiArray.h>

class motor_control
{
  private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_speed_;
    ros::Subscriber sub_steering_;
    ros::Subscriber sub_stop_;
    ros::Subscriber sub_led_front_;
    ros::Subscriber sub_led_back_;
    ros::Publisher pub_bldc_speed_;
    ros::Publisher pub_bldc_steering_;
    ros::Publisher pub_bldc_stop_start_;
    ros::Publisher pub_velocity_;

  public:
    motor_control(ros::NodeHandle nh) : nh_(nh)
    {
      sub_speed_ = nh_.subscribe( "motor_control/speed", 1, &motor_control::motorSpeedCallback,this);
      sub_steering_ = nh.subscribe( "manual_control/steering", 1, &motor_control::steeringCallback,this);
      sub_stop_ = nh_.subscribe( "motor_control/stop_start", 1,  &motor_control::motorStopStartCallback,this);

      pub_bldc_speed_ = nh_.advertise<std_msgs::Int32>(nh_.resolveName("bldc_board_communication/speed"), 1);
      pub_bldc_steering_ = nh_.advertise<std_msgs::Int16>(nh_.resolveName("bldc_board_communication/steering"), 1);
      pub_bldc_stop_start_ = nh_.advertise<std_msgs::Int16>(nh_.resolveName("bldc_board_communication/stop_start"), 1);
      pub_velocity_ = nh_.advertise<geometry_msgs::Twist>(nh_.resolveName("motor_control/twist"), 1);
    }
    ~motor_control(){}
    void motorSpeedCallback(const std_msgs::Int32 speed_value);
    void steeringCallback(const std_msgs::Int16 steering_value);
    void motorStopStartCallback(const std_msgs::Int16 stop_value);
    void ledFrontCallback(const std_msgs::UInt32MultiArray msg);
    void ledBackCallback(const std_msgs::UInt32MultiArray msg);
    void publishMotorTwist();
};

void motor_control::motorSpeedCallback(const std_msgs::Int32 speed_value)
{
  pub_bldc_speed_.publish(speed_value);
}

void motor_control::steeringCallback(const std_msgs::Int16 steering_value)
{ 
  pub_bldc_steering_.publish(steering_value);
}

void motor_control::motorStopStartCallback(const std_msgs::Int16 stop_value) {
  pub_bldc_stop_start_.publish(stop_value);
}

void motor_control::publishMotorTwist()
{
  double currentSpeed = 0.0;
  geometry_msgs::Twist currentTwist;
  currentTwist.linear.x = currentSpeed;
  currentTwist.linear.y = 0.0;
  currentTwist.linear.z = 0.0;
  pub_velocity_.publish(currentTwist);
  
}

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "motor_control_node");
  ros::NodeHandle nh;
  motor_control MC1(nh);
  ros::Rate loop_rate(30);
   while(ros::ok())
  {
    MC1.publishMotorTwist();
    ros::spinOnce();
    loop_rate.sleep();
  }
  std_msgs::Int16 stop_value;
  stop_value.data=1;
  MC1.motorStopStartCallback(stop_value);
  return 0;
}
