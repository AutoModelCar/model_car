#include <termios.h>
#include <ros/ros.h>
#include <std_msgs/Int16.h>

static struct termios t_old;
static struct termios t_new;

int get_char() {
  // capture term mode and set to raw
  tcgetattr(STDIN_FILENO, &t_old);

  t_new = t_old;
  t_new.c_lflag &= ~(ICANON | ECHO);

  tcsetattr(STDIN_FILENO, TCSANOW, &t_new);

  // capture keypress
  int c = std::getchar();

  // reset term mode
  tcsetattr(STDIN_FILENO, TCSANOW, &t_old);

  return c;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "control_wasd");
  ros::NodeHandle nh; 

  ros::Publisher pub_motor    = nh.advertise<std_msgs::Int16>("/manual_control/stop_start", 1);
  ros::Publisher pub_speed    = nh.advertise<std_msgs::Int16>("/manual_control/speed", 1);
  ros::Publisher pub_steering = nh.advertise<std_msgs::Int16>("/manual_control/steering", 1);

  std_msgs::Int16 msg_speed;
  std_msgs::Int16 msg_steering;
  std_msgs::Int16 msg_stop;
  std_msgs::Int16 msg_start;

  msg_speed.data    = 0;
  msg_steering.data = 90;
  msg_stop.data     = 1;
  msg_start.data    = 0;

  int c;

  bool running = true;

  ROS_INFO("WASD drive: Use wasd or arrow-keys to drive. Hit q to quit. Any other key will set speed=0.");
  
  while(running && ros::ok() ) {
    ros::spinOnce();

    c = get_char();

    // catch arrow-keys
    if(c == 27 && get_char() == 91)
      c = get_char();

    switch(c) {
    case 113: // q
      pub_motor.publish(msg_stop);
      running = false;
      break;

    case 65:  // arrow up
    case 119: // w
      msg_speed.data -= 100;
      pub_motor.publish(msg_start);
      pub_speed.publish(msg_speed);
      ROS_INFO("speed=%d", -msg_speed.data);
      break;

    case 68:  // arrow left
    case 97:  // a
      msg_steering.data = std::min(170, msg_steering.data + 10);
      pub_steering.publish(msg_steering);
      ROS_INFO("steering=%d", msg_steering.data - 90);
      break;

    case 66:  // arrow down
    case 115: // s
      msg_speed.data += 100;
      pub_motor.publish(msg_start);
      pub_speed.publish(msg_speed);
      ROS_INFO("speed=%d", -msg_speed.data);
      break;

    case 67:  // arrow right
    case 100: // d
      msg_steering.data = std::max(10, msg_steering.data - 10);
      pub_steering.publish(msg_steering);
      ROS_INFO("steering=%d", msg_steering.data - 90);
      break;

    default:
      msg_speed.data = 0;
      pub_speed.publish(msg_speed);
      pub_motor.publish(msg_stop);
      ROS_INFO("stopped.");
      break;
    }
  }

  return 0;
}
