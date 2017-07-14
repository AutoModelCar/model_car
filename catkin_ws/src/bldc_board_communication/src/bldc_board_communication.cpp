#include <bldc_board_communication/bldc_board_communication.h>

bldc_board_communication::bldc_board_communication()
	: priv_nh_("~")
	, iface()
    , enMotorHandle		(iface.getHandle<uint8_t>("motor.enable", true))
    , speedHandle		(iface.getHandle<float>("motor.target_frequency", true))
    , speedFeedbackHandle	(iface.getHandle<float>("motor.speed", true))
    , steeringHandle		(iface.getHandle<std::array<uint16_t, 2> >("servo.pwm_values", true))
    , ledFrontHandle		(iface.getHandle<std::array<uint32_t, 10> >("led.values.front", true))
    , ledBackHandle		(iface.getHandle<std::array<uint32_t, 11> >("led.values.back", true))
{
  init();
}

bldc_board_communication::~bldc_board_communication() {
	enMotorHandle = 0;
}

void bldc_board_communication::init() {
	std::cout << "motor_communication STARTED" << std::endl;
}

void bldc_board_communication::start() {
	enMotorHandle = 1;
}

void bldc_board_communication::run(float speed) {
	speedHandle  = speed;
}

void bldc_board_communication::steer(int steering) {
	steeringHandle = {(short unsigned int)steering, (short unsigned int)steering};
}

void bldc_board_communication::stop() {
	speedHandle   = 0;
	enMotorHandle = 0;
}

double bldc_board_communication::getSpeed() {
	double velocity = (double) speedFeedbackHandle;
	return velocity;
}

void bldc_board_communication::setLedFront(std::array<uint32_t, 10> values) {
	ledFrontHandle = values;
}

void bldc_board_communication::setLedBack(std::array<uint32_t, 11> values) {
        ledBackHandle = values;
}

