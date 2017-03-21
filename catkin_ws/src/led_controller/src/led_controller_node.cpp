#include <ros/ros.h>
#include <std_msgs/UInt32MultiArray.h>
#include <array>

using namespace std;

ros::Publisher pub_led_front;
ros::Publisher pub_led_back;

void publish(const std::array<uint32_t, 10> & led_front_values, const std::array<uint32_t, 11> & led_back_values) {
  std_msgs::UInt32MultiArray msg_front;
  for(int i=0; i < led_front_values.size(); ++i) {
    msg_front.data.push_back(led_front_values[i]);
  }

  std_msgs::UInt32MultiArray msg_back;
  for(int i=0; i < led_back_values.size(); ++i) {
    msg_back.data.push_back(led_back_values[i]);
  }

  pub_led_front.publish(msg_front);
  pub_led_back.publish(msg_back);
}

void swinging() {
  std::array<uint32_t, 10> led_front_values {0};
  std::array<uint32_t, 11> led_back_values  {0};

  int id = 0;

  ros::Rate loop_rate(10);
   while(ros::ok())
  {

    led_front_values[(id + 0) % led_front_values.size()] = 0;
    led_front_values[(id + 1) % led_front_values.size()] = 0;

    led_back_values[(id + 0) % led_back_values.size()] = 0;
    led_back_values[(id + 1) % led_back_values.size()] = 0;
    led_back_values[(id + 2) % led_back_values.size()] = 0;

    ++id;
    if(id == led_front_values.size()) {
        id = 0;
    }

    led_front_values[(id + 0) % led_front_values.size()] = 0x111111;
    led_front_values[(id + 1) % led_front_values.size()] = 0x111111;

    led_back_values[(id + 0) % led_back_values.size()] = 0x001100; // green red blue
    led_back_values[(id + 1) % led_back_values.size()] = 0x001100;
    led_back_values[(id + 2) % led_back_values.size()] = 0x00ff00;

    publish(led_front_values, led_back_values);

    ros::spinOnce();
    loop_rate.sleep();
  }
}

void knightRider() {
  std::array<uint32_t, 10> led_front_values {0};
  std::array<uint32_t, 11> led_back_values  {0};

  led_front_values[0] = 0xffffff;
  led_front_values[1] = 0xffffff;
  led_front_values[2] = 0xffffff;
  led_front_values[led_front_values.size() - 1] = 0xffffff;
  led_front_values[led_front_values.size() - 2] = 0xffffff;
  led_front_values[led_front_values.size() - 3] = 0xffffff;

  led_back_values[0] = 0x00ff00;
  led_back_values[1] = 0x00ff00;
  led_back_values[2] = 0x00ff00;
  led_back_values[led_back_values.size() - 1] = 0x00ff00;
  led_back_values[led_back_values.size() - 2] = 0x00ff00;
  led_back_values[led_back_values.size() - 3] = 0x00ff00;

  int id = 0;
  bool increment = true;

  ros::Rate loop_rate(15);
   while(ros::ok()) {

    led_front_values[max(3, min((int)led_front_values.size() - 4, id + 0))] = 0;
    led_front_values[max(3, min((int)led_front_values.size() - 4, id + 1))] = 0;
    led_front_values[max(3, min((int)led_front_values.size() - 4, id + 2))] = 0;

    if(increment && id == led_front_values.size()-4) {
        increment = false;
    } else if(!increment && id == 0) {
        increment = true;
    }

    if(increment) {
        ++id;
    } else {
        --id;
    }

    led_front_values[std::max(3,std::min((int)led_front_values.size() - 4, id + 0))] = 0x00ff00;
    led_front_values[std::max(3,std::min((int)led_front_values.size() - 4, id + 1))] = 0x00ff00;
    led_front_values[std::max(3,std::min((int)led_front_values.size() - 4, id + 2))] = 0x00ff00;

    publish(led_front_values, led_back_values);

    ros::spinOnce();
    loop_rate.sleep();
  }
}


int main(int argc, char **argv) {

  ros::init(argc, argv, "led_controller_node");
  ros::NodeHandle nh;

  pub_led_front = nh.advertise<std_msgs::UInt32MultiArray>(nh.resolveName("bldc_board_communication/led_front"), 1);
  pub_led_back = nh.advertise<std_msgs::UInt32MultiArray>(nh.resolveName("bldc_board_communication/led_back"), 1);

  ROS_INFO("led_controller_node started");

  swinging();
  //knightRider();


  return 0;
}
