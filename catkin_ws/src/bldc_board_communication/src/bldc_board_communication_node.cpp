#include <bldc_board_communication/bldc_board_communication.h>
#include <std_msgs/UInt32MultiArray.h>
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

  public:
    bldc_board_communication_node(ros::NodeHandle nh) : nh_(nh)
    {
      sub_speed_ = nh_.subscribe( "bldc_board_communication/speed", 1, &bldc_board_communication_node::motorSpeedCallback,this);
      sub_steering_ = nh.subscribe( "bldc_board_communication/steering", 1, &bldc_board_communication_node::steeringCallback,this);
      sub_stop_ = nh_.subscribe( "bldc_board_communication/stop_start", 1,  &bldc_board_communication_node::motorStopStartCallback,this);
      sub_led_front_ = nh_.subscribe( "bldc_board_communication/led_front", 1,  &bldc_board_communication_node::ledFrontCallback,this);
      sub_led_back_ = nh_.subscribe( "bldc_board_communication/led_back", 1,  &bldc_board_communication_node::ledBackCallback,this);
      std::cout << "bldc_board_communication_node initialized" << std::endl;
    }
    ~bldc_board_communication_node(){}
    void motorSpeedCallback(const std_msgs::Int32 speed_value);
    void steeringCallback(const std_msgs::Int16 steering_value);
    void motorStopStartCallback(const std_msgs::Int16 stop_value);
    void ledFrontCallback(const std_msgs::UInt32MultiArray msg);
    void ledBackCallback(const std_msgs::UInt32MultiArray msg);
    bldc_board_communication bldc;
};

void bldc_board_communication_node::motorSpeedCallback(const std_msgs::Int32 speed_value)
{
  int speed;
  speed = speed_value.data;
  // std::cout << "speed: " << speed << std::endl;
  bldc.run(float(speed));
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

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "bldc_board_communication_node");
  ros::NodeHandle nh;

  bldc_board_communication_node MC1(nh);
  std_msgs::Int16 stop_value;
  stop_value.data=1;
  MC1.motorStopStartCallback(stop_value);

  ros::spin();

  return 0;
}
