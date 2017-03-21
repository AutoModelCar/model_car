#include <bldc_board_communication/bldc-interface/usb/USBInterface.h>

#include <iostream>
#include <stdexcept>

USBInterface::USBInterface()
	: m_usbContext(nullptr)
	, m_deviceHandle(nullptr)
{
	open();
}


USBInterface::~USBInterface() {
	close();
}


void USBInterface::open() {
	int error = libusb_init(&m_usbContext);
	libusb_set_debug(m_usbContext, LIBUSB_LOG_LEVEL_ERROR); //set verbosity level to 3, as suggested in the documentation
	if (error != 0) {
		std::string libusberror = libusb_strerror( (libusb_error) error);
		std::string error = std::string("Cannot establish libusb session: ") + libusberror;
		throw std::runtime_error(error);
	}

	m_deviceHandle = libusb_open_device_with_vid_pid(m_usbContext, 0xcafe, 0xcafe);
	if (m_deviceHandle == nullptr) {
		throw std::runtime_error("No bldc-controller attached");
	}

	// find out if kernel driver is attached
	if (libusb_kernel_driver_active(m_deviceHandle, 0)) {
		// detach it
		if (libusb_detach_kernel_driver(m_deviceHandle, 0)) {
			throw std::runtime_error("Cannot detach usb driver from kernel");
		}
	}

	error = libusb_set_configuration(m_deviceHandle, 1);
	if (error != 0) {
		throw std::runtime_error("Cannot set bldc-controller configuration!");
	}

	error = libusb_claim_interface(m_deviceHandle, 0);
	if (error != 0) {
		throw std::runtime_error("Cannot cleam bldc-controller interface!");
	}
}


/*------------------------------------------------------------------------------------------------*/

bool USBInterface::isOpen() const {
	return m_usbContext != nullptr and m_deviceHandle != nullptr;
}


void USBInterface::close() {
	libusb_release_interface(m_deviceHandle, 0); //release the claimed interface
	libusb_close(m_deviceHandle);

	m_deviceHandle = nullptr;

	//close the session
	libusb_exit(m_usbContext);
	m_usbContext = nullptr;
	std::cout << "closed bldc-usb-interface" << std::endl;
}


void USBInterface::send(void const* data, std::size_t len)
{
	int transfered = 0;
	int error = 0;
	uint8_t * txBuf = (uint8_t *) data;
	do {
		error = libusb_bulk_transfer(m_deviceHandle, 0x01, txBuf, std::min(64U, len), &transfered, 100);
		len -= transfered;
		txBuf += transfered;
	} while (len and error == 0);
}

int USBInterface::receive(void* data, std::size_t len)
{
	int transfered = 0;
	int error = 0;
	uint8_t *recBuf = (uint8_t*)data;
	do {
		error = libusb_bulk_transfer(m_deviceHandle, 0x81, recBuf, std::min(64U, len), &transfered, 100);
		len -= transfered;
		recBuf += transfered;
	} while (len and error == 0);
	return transfered;
}
