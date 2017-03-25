#include <bldc_board_communication/bldc-interface/BLDCInterface.h>

#include <array>
#include <vector>
#include <string>
#include <string.h>
#include <algorithm>

namespace bldcInterface
{

BLDCInterface::BLDCInterface()
{
	// flush the remote device
	std::array<char, 64> mFlushBuffer;
	while (0 < mInterface.receive(mFlushBuffer.data(), mFlushBuffer.size()));

	// fetch the config from the remote device
	std::array<uint8_t, 1> getCommand = {1};
	mInterface.send(getCommand.data(), getCommand.size());

	std::vector<char> payload(64);
	int received = mInterface.receive(payload.data(), payload.size());
	uint16_t actualPayloadLen = 0;
	actualPayloadLen |= (payload[1] << 0) & 0x00ff;
	actualPayloadLen |= (payload[2] << 8) & 0xff00;
	payload.resize(actualPayloadLen+3);

	// receive the rest
	while (received < actualPayloadLen + 3) {
		received += mInterface.receive(payload.data()+received, std::min(64U, payload.size()-received));
	}
	auto iter = payload.begin() + 3;
	int idx = 0;
	while (iter != payload.end()) {
		ConfigurationHandleBase handle;
		handle.index = idx++;
		handle.size = *((uint16_t const*)&(*iter));
		iter += sizeof(handle.size);
		std::string name(&(*iter));
		iter += name.size()+1;
		std::string format = &(*iter);
		iter += format.size()+1;
		mConfigurations[name] = handle;
	}

}

BLDCInterface::~BLDCInterface()
{
}

void BLDCInterface::setConfig(ConfigurationHandleBase const* handle, void const* values)
{
	if (nullptr != handle) {
		std::vector<uint8_t> packet(handle->size+2);
		packet[0] = 1;
		packet[1] = handle->index;
		memcpy(packet.data()+2, values, handle->size);
		mInterface.send(packet.data(), packet.size());
	}
}

void BLDCInterface::getConfig(ConfigurationHandleBase const* handle, void* values)
{
	if (nullptr != handle) {
		std::array<uint8_t, 2> getCommand = {1, handle->index};
		mInterface.send(getCommand.data(), getCommand.size());
		std::vector<uint8_t> packet(handle->size+3);
		mInterface.receive(packet.data(), packet.size());
		memcpy(values, packet.data()+3, handle->size);
	}
}

}
