
#include "BLDCInterface.h"
#include <unistd.h>


int main(void) {
	bldcInterface::BLDCInterface iface;

	auto enMotorHandle  = iface.getHandle<uint8_t>("enable_motor");
	auto enBldcHandle   = iface.getHandle<uint8_t>("bldc_enable");
	auto speedHandle    = iface.getHandle<int32_t>("bldc_target_frequency");
	auto steeringHandle = iface.getHandle<std::array<uint16_t, 2>>("servo_pwm_values");

	enMotorHandle.setAutopush(true);
	enBldcHandle.setAutopush(true);
	speedHandle.setAutopush(true);
	steeringHandle.setAutopush(false);

	steeringHandle = {1, 2};
	steeringHandle.push();

	enMotorHandle = 1;
	enBldcHandle  = 1;

	speedHandle  = 200000;
	usleep(2000000);
	speedHandle  = -200000;
	usleep(2000000);

	enMotorHandle = 0;
	enBldcHandle  = 0;

	return 0;
}
