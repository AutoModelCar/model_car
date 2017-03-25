#include <bldc_board_communication/bldc_board_communication.h>
#include <std_msgs/UInt32MultiArray.h>
#include <std_msgs/Float64.h>
#include <iostream>

class bldc_board_communication_node
{
  private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_speed_;
    ros::Subscriber sub_steering_;
    ros::Subscriber sub_stop_;
    ros::Subscriber sub_led_front_;
    ros::Subscriber sub_led_back_;
    ros::Publisher pub_current_speed_;

  public:
    bldc_board_communication_node(ros::NodeHandle nh) : nh_(nh)
    {
      sub_speed_ = nh_.subscribe( "bldc_board_communication/speed", 1, &bldc_board_communication_node::motorSpeedCallback,this);
      sub_steering_ = nh.subscribe( "bldc_board_communication/steering", 1, &bldc_board_communication_node::steeringCallback,this);
      sub_stop_ = nh_.subscribe( "bldc_board_communication/stop_start", 1,  &bldc_board_communication_node::motorStopStartCallback,this);
      sub_led_front_ = nh_.subscribe( "bldc_board_communication/led_front", 1,  &bldc_board_communication_node::ledFrontCallback,this);
      sub_led_back_ = nh_.subscribe( "bldc_board_communication/led_back", 1,  &bldc_board_communication_node::ledBackCallback,this);
      pub_current_speed_ = nh_.advertise<std_msgs::Float64>(nh_.resolveName("bldc_board_communication/current_speed_feedback"), 1);
      std::cout << "bldc_board_communication_node initialized" << std::endl;
    }
    ~bldc_board_communication_node(){}
    void motorSpeedCallback(const std_msgs::Float64 speed_value);
    void steeringCallback(const std_msgs::Int16 steering_value);
    void motorStopStartCallback(const std_msgs::Int16 stop_value);
    void ledFrontCallback(const std_msgs::UInt32MultiArray msg);
    void ledBackCallback(const std_msgs::UInt32MultiArray msg);
    void publishCurrentSpeedFeedback();
    bldc_board_communication bldc;
};

void bldc_board_communication_node::motorSpeedCallback(const std_msgs::Float64 speed_value)
{
  bldc.run(float(speed_value.data));
}

void bldc_board_communication_node::steeringCallback(const std_msgs::Int16 steering_value)
{ 
  int steering=steering_value.data;
  // std::cout << "steering: " << steering << std::endl;
  bldc.steer(steering);
}

void bldc_board_communication_node::motorStopStartCallback(const std_msgs::Int16 stop_value) {
  if (stop_value.data==1) {
    bldc.stop();
  }
  else {
    bldc.start();
  }
}

void bldc_board_communication_node::ledFrontCallback(const std_msgs::UInt32MultiArray msg) {
  std::array<uint32_t, 10> led_values;
  if(msg.data.size() >= led_values.size()) {
     for(int i=0; i<led_values.size(); ++i) {
       led_values[i] = msg.data[i];
     }
  }
  bldc.setLedFront(led_values);
}

void bldc_board_communication_node::ledBackCallback(const std_msgs::UInt32MultiArray msg) {
  std::array<uint32_t, 11> led_values;
  if(msg.data.size() >= led_values.size()) {
     for(int i=0; i<led_values.size(); ++i) {
       led_values[i] = msg.data[i];
     }
  }
  bldc.setLedBack(led_values);
}

void bldc_board_communication_node::publishCurrentSpeedFeedback() {
  std_msgs::Float64 msg;
  msg.data = bldc.getSpeed();
  // std::cout << "speed = " << msg.data << std::endl;
  pub_current_speed_.publish(msg);
}

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "bldc_board_communication_node");
  ros::NodeHandle nh;

  bldc_board_communication_node MC1(nh);
  std_msgs::Int16 stop_value;
  stop_value.data=1;
  MC1.motorStopStartCallback(stop_value);

  ros::Rate loop_rate(10);
  while (ros::ok()) {
    MC1.publishCurrentSpeedFeedback();

    ros::spinOnce();
    loop_rate.sleep();
  }


  return 0;
}
