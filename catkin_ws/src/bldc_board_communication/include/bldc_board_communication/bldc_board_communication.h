#include <ros/ros.h>
#include <ros/message_operations.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <iostream>
#include <unistd.h>
#include <array>
#include <bldc_board_communication/bldc-interface/BLDCInterface.h>

using namespace std;

typedef int16_t speed_MMpS_t;


class bldc_board_communication
{
  private:
    //! Node handle in the private namespace
    ros::NodeHandle priv_nh_;
    bldcInterface::BLDCInterface iface;
    bldcInterface::BLDCInterface::ConfigurationHandle<uint8_t> enMotorHandle;
    bldcInterface::BLDCInterface::ConfigurationHandle<float> speedHandle;
    bldcInterface::BLDCInterface::ConfigurationHandle<std::array<uint16_t, 2> > steeringHandle;
    bldcInterface::BLDCInterface::ConfigurationHandle<std::array<uint32_t, 10> > ledFrontHandle;
    bldcInterface::BLDCInterface::ConfigurationHandle<std::array<uint32_t, 11> > ledBackHandle;


  public:
    

    bldc_board_communication();
    ~bldc_board_communication();
    void init();
    void run(float speed);
    void steer(int steering);
    void stop();
    void start();
    double getSpeed();
    void setLedFront(std::array<uint32_t, 10> values);
    void setLedBack(std::array<uint32_t, 11> values);

};
