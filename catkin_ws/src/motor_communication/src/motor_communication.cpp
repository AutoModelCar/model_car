#include <motor_communication/motor_communication.h>

motor_communication::motor_communication(): priv_nh_("~"), my_serial("/dev/ttyUSB1",9600, serial::Timeout::simpleTimeout(1000))
{
  result="";
  priv_nh_.param<std::string>("motor_serial_port", serial_port_, "/dev/ttyUSB1");
  priv_nh_.param("motor_baud_rate", baud_rate_,115200);
  my_serial.close();
  my_serial.setPort(serial_port_);
  my_serial.setBaudrate(baud_rate_);
  my_serial.open();
  //my_serial.setTimeout(1000);
  //my_serial.Timeout.simpleTimeout(1000)
  init();

}

    //! Empty stub
motor_communication::~motor_communication() {}

void motor_communication::my_sleep(unsigned long milliseconds) {
    usleep(milliseconds*1000); // 100 ms
}

void motor_communication::init()
{
  	try
    {
      ROS_INFO("motor_communication::Is the serial port %s open?",serial_port_.c_str());
      //cout << my_serial.getBaudrate() << endl;
	  if(my_serial.isOpen())
	    ROS_INFO("motor_communication::Yes.");
	  else
	    ROS_ERROR("motor_communication::No.");

	  bytes_wrote =my_serial.write("en\r\n");
    // read until new line terminates result string
    result = "";
    uint8_t c = '\0';
    while (c  != '\n') {
            my_serial.read(&c, 1);
            if(c != '\n' and c != '\r') {
                    result += char(c);
            }
    }
	  ROS_INFO("motor_communication:: start:%s \n",result.c_str());
    }
    catch(const std::exception& e)
    {
      	ROS_ERROR("motor_communication::could not find serial port");
    }
}
void motor_communication::start()
{
    try
    {
      bytes_wrote =my_serial.write("en\r\n");
      // read until new line terminates result string
      result = "";
      uint8_t c = '\0';
      while (c  != '\n') {
              my_serial.read(&c, 1);
              if(c != '\n' and c != '\r') {
                      result += char(c);
              }
      }
      ROS_INFO("motor_communication::start:%s \n",result.c_str());
    }
    catch(const std::exception& e)
    {
        ROS_ERROR("motor_communication::could not find serial port");
    }
}
void motor_communication::run(int speed)
{
    try
    {
      std::string speed_string = std::to_string(speed);;
          string test_string="v"+ speed_string +"\r\n";
          bytes_wrote =my_serial.write(test_string);
                // TODO: find out if result string is actually needed.
                result = "";
                uint8_t c = '\0';
                // read until new line terminates result string
                while (c  != '\n') {
                        my_serial.read(&c, 1);
                        if(c != '\n' and c != '\r') {
                                result += char(c);
                        }
                }
      //ROS_INFO("read speed:%s \n",result.c_str());
    }
    catch(const std::exception& e)
    {
      	ROS_ERROR("motor_communication::could not find serial port");
    }
}
void motor_communication::stop()
{
	try
	{
		bytes_wrote =my_serial.write("di\r\n");
    // read until new line terminates result string
    result = "";
    uint8_t c = '\0';
    while (c  != '\n') {
            my_serial.read(&c, 1);
            if(c != '\n' and c != '\r') {
                    result += char(c);
            }
    }
		ROS_INFO("motor_communication::read di:%s \n",result.c_str());
	}
	catch(const std::exception& e)
	{
		ROS_ERROR("motor_communication::could not find serial port");
	}
}
double motor_communication::getSpeed()
{
	try
	{
		bytes_wrote = my_serial.write("gn\r\n");
    // read until new line terminates result string
    result = "";
    uint8_t c = '\0';
    while (c  != '\n') {
            my_serial.read(&c, 1);
            if(c != '\n' and c != '\r') {
                    result += char(c);
            }
    }
		//ROS_INFO("speed:%s \n",result.c_str());
    std::stringstream ss(result);
    double velocity;
    ss >> velocity; //string to double
		return velocity;
	}
	catch(const std::exception& e)
	{
		ROS_ERROR("motor_communication::could not find serial port");
	}
}
