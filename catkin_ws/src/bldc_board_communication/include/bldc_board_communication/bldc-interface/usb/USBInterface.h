#pragma once

#include <libusb-1.0/libusb.h>
#include <array>

class USBInterface final {
public:
	USBInterface();
	~USBInterface();

	USBInterface(USBInterface const& _other) = delete;
	USBInterface& operator=(USBInterface const& _other) = delete;


	void send(void const* data, std::size_t len);
	int receive(void* data, std::size_t len);

	bool isOpen() const;

private:
	/** Open the USB connection
	 **
	 ** @param silent   If set, error messages will be inhibited. This is useful
	 **                 when calling this function in a polling mode.
	 ** @return true if connection was established
	 */
	void open();
	void close();

private:
	libusb_context*       m_usbContext;
	libusb_device_handle* m_deviceHandle;
};
