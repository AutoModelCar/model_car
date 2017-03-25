#pragma once

#include "usb/USBInterface.h"
#include <string.h>
#include <iostream>
#include <istream>
#include <map>
#include <stdexcept>
#include <unistd.h>
#include "util/demangle.h"

namespace bldcInterface
{

namespace serialProtocolHelpers
{
enum class InstructionType : uint8_t {
	QUERRY           = 0, // Request a description of a publishable
	READ             = 1, // pubish all publishables to which there is a subscription
	SUBSCRIBE        = 2, // subscribe to a publishable
	SET              = 3, // set the value of (one or more) publishables
	QUERRY_RESPONSE  = 4, // a response to a query
	READ_RESPONSE    = 5, // the read response
	INVALID          = 0xff, // marked as invalid
};
struct PacketHeader {
	uint8_t targetID;
	InstructionType instruction;
} __attribute__((packed));
struct QuerryReply {
	uint8_t index;
	uint8_t dataSize;
};
struct QueryResponsePacket {
	PacketHeader header;
	QuerryReply queryReply;
} __attribute__((packed));


struct QuerryInfo {
	uint8_t targetID;
	std::array<char, 60> descriptor;
} __attribute__((packed));
struct SetInfo {
	uint8_t targetID;
	uint8_t targetIdx;
	uint8_t payloadLen;
	std::array<char, 59> descriptor;
} __attribute__((packed));

}

class BLDCInterface final
{
	struct ConfigurationHandleBase {
		uint8_t index;
		uint16_t size;
		std::string name;
		std::string format;
	};

	USBInterface mInterface;
	std::map<std::string, ConfigurationHandleBase> mConfigurations;

public:
	template<typename T>
	class ConfigurationHandle {
	public:
		ConfigurationHandle(ConfigurationHandleBase* handle, BLDCInterface *iface, bool autoSync) : mHandle(handle), mIface(iface), mAutoSync(autoSync) {
			pull();
		}

		void push() {
			mIface->setConfig(mHandle, &mValue);
		}

		void pull() {
			mIface->getConfig(mHandle, &mValue);
		}

		operator T() {
			if (mAutoSync) {
				pull();
			}
			return mValue;
		}

		T* operator->() {
			if (mAutoSync) {
				pull();
			}
			return &mValue;
		}

		T& operator=(T const& rhs) {
			mValue = rhs;
			if (mAutoSync) {
				push();
			}
			return mValue;
		}

		void setAutopush(bool autopush) {
			mAutoSync = autopush;
		}
	private:
		ConfigurationHandleBase* mHandle;
		BLDCInterface *mIface;
		bool mAutoSync {false};
		T mValue;
	};

	template<typename T>
	class RemoteSettableConfigurationHandle {
	public:
		RemoteSettableConfigurationHandle(ConfigurationHandle<serialProtocolHelpers::SetInfo> wrapper, uint8_t targetID, uint8_t targetIdx) :
			mWrapper(wrapper),
			mTargetID(targetID),
			mTargetIdx(targetIdx){}

		void push() {
			serialProtocolHelpers::SetInfo info;
			info.targetID = mTargetID;
			info.targetIdx  = mTargetIdx;
			info.payloadLen = sizeof(T);
			memcpy(info.descriptor.data(), &mValue, sizeof(mValue));
			mWrapper = info;
		}

		T& operator=(T const& rhs) {
			mValue = rhs;
			push();
			return mValue;
		}
	private:
		ConfigurationHandle<serialProtocolHelpers::SetInfo> mWrapper;
		uint8_t mTargetID;
		uint8_t mTargetIdx;
		T mValue;
	};

public:
	BLDCInterface();
	virtual ~BLDCInterface();

	void setConfig(ConfigurationHandleBase const* handle, void const* values);
	void getConfig(ConfigurationHandleBase const* handle, void* values);

//	void printConfigurations();

	template<typename T>
	ConfigurationHandle<T> getHandle(std::string const& name, bool autoSync=true) {
		ConfigurationHandleBase* handle = &(mConfigurations.at(name));
		if (handle->size != sizeof(T)) {
			std::string demangledName = demangle<T>();
			std::cerr << "requesting a mapping of " << demangledName << " to " << handle->name << "of incompatible size! sizeof(" << demangledName << ")=" << sizeof(T) << " vs. " << handle->size << std::endl;
			throw std::runtime_error("Invalid Configuration Requested");
		}
		return ConfigurationHandle<T>(handle, this, autoSync);
	}

	template<typename T>
	RemoteSettableConfigurationHandle<T> getRemoteSettableHandle(uint8_t deviceID, std::string const& name, int queryRetryCount = 10) {

		serialProtocolHelpers::QuerryInfo info;
		if (name.size()+1 > info.descriptor.size()) {
			throw std::runtime_error("Cannot adress a remote configuration with a string descriptor of more than 60 characters (including terminating 0)");
		}

		auto serialQuerryHandle = getHandle<serialProtocolHelpers::QuerryInfo>("serial.interface.query", true);
		auto serialResponseHandle = getHandle<serialProtocolHelpers::QueryResponsePacket>("serial.interface.query.reply", true);
		info.targetID = deviceID;
		memcpy(info.descriptor.data(), name.c_str(), name.size()+1);
		int tries = 0;
		while (tries < queryRetryCount) {
			serialQuerryHandle = info;

			int counter = 0;
			// wait for the response
			while (serialResponseHandle->header.instruction == serialProtocolHelpers::InstructionType::INVALID and
					counter < 10) {
				usleep(100000);
				++counter;
			}
			if (serialResponseHandle->header.instruction == serialProtocolHelpers::InstructionType::QUERRY_RESPONSE) {
				break;
			}
			++tries;
		}

		if (serialResponseHandle->header.instruction != serialProtocolHelpers::InstructionType::QUERRY_RESPONSE) {
			throw std::runtime_error("got invalid response from remote device!");
		}

		if (serialResponseHandle->queryReply.dataSize != sizeof(T)) {
			throw std::runtime_error("The remote handle is of different size than the resquested type!");
		}
		uint8_t targetIdx = serialResponseHandle->queryReply.index;
		return RemoteSettableConfigurationHandle<T>(
				getHandle<serialProtocolHelpers::SetInfo>("serial.interface.set", true),
				deviceID,
				targetIdx);
	}
};

}

