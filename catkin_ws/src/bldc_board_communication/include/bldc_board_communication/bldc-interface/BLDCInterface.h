#pragma once

#include <bldc_board_communication/bldc-interface/usb/USBInterface.h>
#include <iostream>
#include <map>

namespace bldcInterface
{

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
		ConfigurationHandle(ConfigurationHandleBase* handle, BLDCInterface *iface, bool autopush) : mHandle(handle), mIface(iface), mAutoPush(autopush) {
			pull();
		}
		ConfigurationHandle(ConfigurationHandleBase* handle, BLDCInterface *iface, T const& value, bool autopush) : mHandle(handle), mIface(iface), mAutoPush(autopush), mValue(value) {
			if (mAutoPush) {
				push();
			} else {
				pull();
			}
		}

		void push() {
			mIface->setConfig(mHandle, &mValue);
		}

		void pull() {
			mIface->getConfig(mHandle, &mValue);
		}

		T* operator->() {
			return &mValue;
		}

		T& operator=(T const& rhs) {
			mValue = rhs;
			if (mAutoPush) {
				push();
			}
			return mValue;
		}

		void setAutopush(bool autopush) {
			mAutoPush = autopush;
		}
	private:
		ConfigurationHandleBase* mHandle;
		BLDCInterface *mIface;
		bool mAutoPush {false};
		T mValue;
	};

public:
	BLDCInterface();
	virtual ~BLDCInterface();

	void setConfig(ConfigurationHandleBase const* handle, void const* values);
	void getConfig(ConfigurationHandleBase const* handle, void* values);

	template<typename T>
	ConfigurationHandle<T> const getHandle(std::string const& name, bool autopush=false) {
		return ConfigurationHandle<T>(&(mConfigurations.at(name)), this, autopush);
	}
	template<typename T>
	ConfigurationHandle<T> const getHandle(std::string const& name, T const& initval, bool autopush=false) {
		return ConfigurationHandle<T>(&(mConfigurations.at(name)), this, initval, autopush);
	}
};

}
