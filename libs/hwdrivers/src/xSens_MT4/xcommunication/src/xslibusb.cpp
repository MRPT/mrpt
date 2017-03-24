/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef _WIN32  // patch for MRPT

#include "xslibusb.h"
#include <xsens/xslibraryloader.h>

/*! \class XsLibUsb
	\brief Class for dynamic loading of winusb
*/
XsLibUsb::XsLibUsb(void)
{
	m_libraryLoader = new XsLibraryLoader();
	initLibrary();
}

XsLibUsb::~XsLibUsb(void)
{
	delete m_libraryLoader;
}

void XsLibUsb::initLibrary()
{
	if (!m_libraryLoader->isLoaded())
		m_libraryLoader->load("libusb-1.0.so");

	m_libUsb.init = NULL;
	m_libUsb.exit = NULL;
	m_libUsb.open = NULL;
	m_libUsb.close = NULL;
	m_libUsb.kernel_driver_active = NULL;
	m_libUsb.attach_kernel_driver = NULL;
	m_libUsb.detach_kernel_driver = NULL;
	m_libUsb.ref_device = NULL;
	m_libUsb.unref_device = NULL;
	m_libUsb.claim_interface = NULL;
	m_libUsb.release_interface = NULL;
	m_libUsb.get_active_config_descriptor = NULL;
	m_libUsb.free_config_descriptor = NULL;
	m_libUsb.get_bus_number = NULL;
	m_libUsb.get_device = NULL;
	m_libUsb.get_device_address = NULL;
	m_libUsb.get_device_descriptor = NULL;
	m_libUsb.get_device_list = NULL;
	m_libUsb.free_device_list = NULL;
	m_libUsb.get_string_descriptor_ascii = NULL;
	m_libUsb.bulk_transfer = NULL;
	m_libUsb.set_debug = NULL;

	if (m_libraryLoader->isLoaded())
	{
		m_libUsb.init = (libUSB_init*)m_libraryLoader->resolve("libusb_init");
		m_libUsb.exit = (libUSB_exit*)m_libraryLoader->resolve("libusb_exit");
		m_libUsb.open = (libUSB_open*)m_libraryLoader->resolve("libusb_open");
		m_libUsb.close = (libUSB_close*)m_libraryLoader->resolve("libusb_close");
		m_libUsb.kernel_driver_active = (libUSB_kernel_driver_active*)m_libraryLoader->resolve("libusb_kernel_driver_active");
		m_libUsb.attach_kernel_driver = (libUSB_attach_kernel_driver*)m_libraryLoader->resolve("libusb_attach_kernel_driver");
		m_libUsb.detach_kernel_driver = (libUSB_detach_kernel_driver*)m_libraryLoader->resolve("libusb_detach_kernel_driver");
		m_libUsb.ref_device = (libUSB_ref_device*)m_libraryLoader->resolve("libusb_ref_device");
		m_libUsb.unref_device = (libUSB_unref_device*)m_libraryLoader->resolve("libusb_unref_device");
		m_libUsb.claim_interface = (libUSB_claim_interface*)m_libraryLoader->resolve("libusb_claim_interface");
		m_libUsb.release_interface = (libUSB_release_interface*)m_libraryLoader->resolve("libusb_release_interface");
		m_libUsb.get_active_config_descriptor = (libUSB_get_active_config_descriptor*)m_libraryLoader->resolve("libusb_get_active_config_descriptor");
		m_libUsb.free_config_descriptor = (libUSB_free_config_descriptor*)m_libraryLoader->resolve("libusb_free_config_descriptor");
		m_libUsb.get_bus_number = (libUSB_get_bus_number*)m_libraryLoader->resolve("libusb_get_bus_number");
		m_libUsb.get_device = (libUSB_get_device*)m_libraryLoader->resolve("libusb_get_device");
		m_libUsb.get_device_address = (libUSB_get_device_address*)m_libraryLoader->resolve("libusb_get_device_address");
		m_libUsb.get_device_descriptor = (libUSB_get_device_descriptor*)m_libraryLoader->resolve("libusb_get_device_descriptor");
		m_libUsb.get_device_list = (libUSB_get_device_list*)m_libraryLoader->resolve("libusb_get_device_list");
		m_libUsb.free_device_list = (libUSB_free_device_list*)m_libraryLoader->resolve("libusb_free_device_list");
		m_libUsb.get_string_descriptor_ascii = (libUSB_get_string_descriptor_ascii*)m_libraryLoader->resolve("libusb_get_string_descriptor_ascii");
		m_libUsb.bulk_transfer = (libUSB_bulk_transfer*)m_libraryLoader->resolve("libusb_bulk_transfer");
		m_libUsb.set_debug = (libUSB_set_debug*)m_libraryLoader->resolve("libusb_set_debug");
	}
}

/*! \brief Initialize libusb. This function must be called before calling any other libusb function.

	If you do not provide an output location for a context pointer, a default
	context will be created. If there was already a default context, it will
	be reused (and nothing will be initialized/reinitialized).

	\param context Optional output location for context pointer.
				   Only valid on return code 0.
	\returns 0 on success, or a LIBUSB_ERROR code on failure
*/
int XsLibUsb::init(libusb_context **ctx)
{
	if (m_libUsb.init)
		return m_libUsb.init(ctx);
	else
		return LIBUSB_ERROR_NOT_SUPPORTED;
}

/*! \brief Deinitialize libusb. Should be called after closing all open devices and before your application terminates.
	\param ctx the context to deinitialize, or NULL for the default context
*/
void XsLibUsb::exit(libusb_context *ctx)
{
	if (m_libUsb.exit)
		m_libUsb.exit(ctx);
}

/*! \brief Open a device and obtain a device handle. A handle allows you to perform I/O on the device in question.

	Internally, this function adds a reference to the device and makes it available to you through libusb_get_device().
	This reference is removed during libusb_close().

	This is a non-blocking function; no requests are sent over the bus.

	\param dev the device to open
	\param handle output location for the returned device handle pointer. Only populated when the return code is 0.

	\returns 0 on success
	\returns LIBUSB_ERROR_NO_MEM on memory allocation failure
	\returns LIBUSB_ERROR_ACCESS if the user has insufficient permissions
	\returns LIBUSB_ERROR_NO_DEVICE if the device has been disconnected
	\returns another LIBUSB_ERROR code on other failure
*/
int XsLibUsb::open(libusb_device *dev, libusb_device_handle **handle)
{
	if (m_libUsb.open)
		return m_libUsb.open(dev, handle);
	else
		return LIBUSB_ERROR_NOT_SUPPORTED;
}

/*! \brief Close a device handle. Should be called on all open handles before your application exits.
	Internally, this function destroys the reference that was added by libusb_open() on the given device.

	This is a non-blocking function; no requests are sent over the bus.
	\param dev_handle the handle to close
*/
void XsLibUsb::close(libusb_device_handle *dev_handle)
{
	if (m_libUsb.close)
		m_libUsb.close(dev_handle);
}

/*! \brief Determine if a kernel driver is active on an interface.
	If a kernel driver is active, you cannot claim the interface, and libusb will be unable to perform I/O.

	This functionality is not available on Windows.

	\param dev a device handle
	\param interface_number the interface to check

	\returns 0 if no kernel driver is active
	\returns 1 if a kernel driver is active
	\returns LIBUSB_ERROR_NO_DEVICE if the device has been disconnected
	\returns LIBUSB_ERROR_NOT_SUPPORTED on platforms where the functionality is not available
	\returns another LIBUSB_ERROR code on other failure
	\see libusb_detach_kernel_driver()
*/
int XsLibUsb::kernel_driver_active(libusb_device_handle *dev,int interface_number)
{
	if (m_libUsb.kernel_driver_active)
		return m_libUsb.kernel_driver_active(dev, interface_number);
	else
		return LIBUSB_ERROR_NOT_SUPPORTED;
}

/** \brief Re-attach an interface's kernel driver, which was previously detached using libusb_detach_kernel_driver().
	This call is only effective on Linux and returns LIBUSB_ERROR_NOT_SUPPORTED on all other platforms.

	This functionality is not available on Darwin or Windows.

	\param dev a device handle
	\param interface_number the interface to attach the driver from

	\returns 0 on success
	\returns LIBUSB_ERROR_NOT_FOUND if no kernel driver was active
	\returns LIBUSB_ERROR_INVALID_PARAM if the interface does not exist
	\returns LIBUSB_ERROR_NO_DEVICE if the device has been disconnected
	\returns LIBUSB_ERROR_NOT_SUPPORTED on platforms where the functionality is not available
	\returns LIBUSB_ERROR_BUSY if the driver cannot be attached because the interface is claimed by a program or driver
	\returns another LIBUSB_ERROR code on other failure
	\see libusb_kernel_driver_active()
*/
int XsLibUsb::attach_kernel_driver(libusb_device_handle *dev,int interface_number)
{
	if (m_libUsb.attach_kernel_driver)
		return m_libUsb.attach_kernel_driver(dev, interface_number);
	else
		return LIBUSB_ERROR_NOT_SUPPORTED;
}

/*! \brief Detach a kernel driver from an interface. If successful, you will then be able to claim the interface and perform I/O.

	This functionality is not available on Darwin or Windows.

	\param dev a device handle
	\param interface_number the interface to detach the driver from

	\returns 0 on success
	\returns LIBUSB_ERROR_NOT_FOUND if no kernel driver was active
	\returns LIBUSB_ERROR_INVALID_PARAM if the interface does not exist
	\returns LIBUSB_ERROR_NO_DEVICE if the device has been disconnected
	\returns LIBUSB_ERROR_NOT_SUPPORTED on platforms where the functionality is not available
	\returns another LIBUSB_ERROR code on other failure
	\see libusb_kernel_driver_active()
*/
int XsLibUsb::detach_kernel_driver(libusb_device_handle *dev,int interface_number)
{
	if (m_libUsb.detach_kernel_driver)
		return m_libUsb.detach_kernel_driver(dev, interface_number);
	else
		return LIBUSB_ERROR_NOT_SUPPORTED;
}

/*! \brief Increment the reference count of a device.
	\param dev the device to reference
	\returns the same device
*/
libusb_device * XsLibUsb::ref_device(libusb_device *dev)
{
	if (m_libUsb.ref_device)
		return m_libUsb.ref_device(dev);
	else
		return NULL;
}

/*! \brief Decrement the reference count of a device.
	If the decrement operation causes the reference count to reach zero, the device shall be destroyed.
	\param dev the device to unreference
*/
void XsLibUsb::unref_device(libusb_device *dev)
{
	if (m_libUsb.unref_device)
		m_libUsb.unref_device(dev);
}

/*! \brief Claim an interface on a given device handle.
	You must claim the interface you wish to use before you can perform I/O on any of its endpoints.

	It is legal to attempt to claim an already-claimed interface, in which
	case libusb just returns 0 without doing anything.

	Claiming of interfaces is a purely logical operation; it does not cause
	any requests to be sent over the bus. Interface claiming is used to
	instruct the underlying operating system that your application wishes
	to take ownership of the interface.

	This is a non-blocking function.

	\param dev a device handle
	\param interface_number the \a bInterfaceNumber of the interface you wish to claim
	\returns 0 on success
	\returns LIBUSB_ERROR_NOT_FOUND if the requested interface does not exist
	\returns LIBUSB_ERROR_BUSY if another program or driver has claimed the interface
	\returns LIBUSB_ERROR_NO_DEVICE if the device has been disconnected
	\returns a LIBUSB_ERROR code on other failure
*/
int XsLibUsb::claim_interface(libusb_device_handle *dev,int interface_number)
{
	if (m_libUsb.claim_interface)
		return m_libUsb.claim_interface(dev, interface_number);
	else
		return LIBUSB_ERROR_NOT_SUPPORTED;
}

/*! \brief Release an interface previously claimed with libusb_claim_interface().
	You should release all claimed interfaces before closing a device handle.

	This is a blocking function. A SET_INTERFACE control request will be sent
	to the device, resetting interface state to the first alternate setting.

	\param dev a device handle
	\param interface_number the \a bInterfaceNumber of the previously-claimed interface
	\returns 0 on success
	\returns LIBUSB_ERROR_NOT_FOUND if the interface was not claimed
	\returns LIBUSB_ERROR_NO_DEVICE if the device has been disconnected
	\returns another LIBUSB_ERROR code on other failure
*/
int XsLibUsb::release_interface(libusb_device_handle *dev,	int interface_number)
{
	if (m_libUsb.release_interface)
		return m_libUsb.release_interface(dev, interface_number);
	else
		return LIBUSB_ERROR_NOT_SUPPORTED;
}

/*! \brief Get the USB configuration descriptor for the currently active configuration.

	This is a non-blocking function which does not involve any requests being
	sent to the device.

	\param dev a device
	\param config output location for the USB configuration descriptor.
	Only valid if 0 was returned. Must be freed with libusb_free_config_descriptor() after use.
	\returns 0 on success
	\returns LIBUSB_ERROR_NOT_FOUND if the device is in unconfigured state
	\returns another LIBUSB_ERROR code on error
	\see libusb_get_config_descriptor
*/
int XsLibUsb::get_active_config_descriptor(libusb_device *dev,	struct libusb_config_descriptor **config)
{
	if (m_libUsb.get_active_config_descriptor)
		return m_libUsb.get_active_config_descriptor(dev, config);
	else
		return LIBUSB_ERROR_NOT_SUPPORTED;
}

/*! \brief Free a configuration descriptor obtained from libusb_get_active_config_descriptor() or libusb_get_config_descriptor().

	It is safe to call this function with a NULL config parameter, in which case the function simply returns.

	\param config the configuration descriptor to free
*/
void XsLibUsb::free_config_descriptor(struct libusb_config_descriptor *config)
{
	if (m_libUsb.free_config_descriptor)
		m_libUsb.free_config_descriptor(config);
}

/*! \brief Get the number of the bus that a device is connected to.
	\param dev a device
	\returns the bus number
*/
uint8_t XsLibUsb::get_bus_number(libusb_device *dev)
{
	if (m_libUsb.get_bus_number)
		return m_libUsb.get_bus_number(dev);
	else
		return 0;
}

/*! \brief Get the underlying device for a handle.

	This function does not modify the reference count of the returned device,
	so do not feel compelled to unreference it when you are done.
	\param dev_handle a device handle
	\returns the underlying device
*/
libusb_device * XsLibUsb::get_device(libusb_device_handle *dev_handle)
{
	if (m_libUsb.get_device)
		return m_libUsb.get_device(dev_handle);
	else
		return NULL;
}

/*! \brief Get the address of the device on the bus it is connected to.
	\param dev a device
	\returns the device address
*/
uint8_t XsLibUsb::get_device_address(libusb_device *dev)
{
	if (m_libUsb.get_device_address)
		return m_libUsb.get_device_address(dev);
	else
		return 0;
}

/*! \brief Get the USB device descriptor for a given device.

	This is a non-blocking function; the device descriptor is cached in memory.

	\param dev the device
	\param desc output location for the descriptor data
	\returns 0 on success or a LIBUSB_ERROR code on failure
*/
int XsLibUsb::get_device_descriptor(libusb_device *dev, struct libusb_device_descriptor *desc)
{
	if (m_libUsb.get_device_descriptor)
		return m_libUsb.get_device_descriptor(dev, desc);
	else
		return LIBUSB_ERROR_NOT_SUPPORTED;
}

/*! \brief Returns a list of USB devices currently attached to the system.
	This is your entry point into finding a USB device to operate.

	You are expected to unreference all the devices when you are done with
	them, and then free the list with libusb_free_device_list(). Note that
	libusb_free_device_list() can unref all the devices for you. Be careful
	not to unreference a device you are about to open until after you have
	opened it.

	This return value of this function indicates the number of devices in
	the resultant list. The list is actually one element larger, as it is
	NULL-terminated.

	\param ctx the context to operate on, or NULL for the default context
	\param list output location for a list of devices. Must be later freed with libusb_free_device_list().
	\returns The number of devices in the outputted list, or any LIBUSB_ERROR code to errors encountered by the backend.
*/
ssize_t XsLibUsb::get_device_list(libusb_context *ctx,	libusb_device ***list)
{
	if (m_libUsb.get_device_list)
		return m_libUsb.get_device_list(ctx, list);
	else
		return LIBUSB_ERROR_NOT_SUPPORTED;
}

/*! \brief Frees a list of devices previously discovered using libusb_get_device_list().
	If the unref_devices parameter is set, the reference count of each device in the list is decremented by 1.
	\param list the list to free
	\param unref_devices whether to unref the devices in the list
*/
void XsLibUsb::free_device_list(libusb_device **list,	int unref_devices)
{
	if (m_libUsb.free_device_list)
		m_libUsb.free_device_list(list, unref_devices);
}

/*! \brief Retrieve a string descriptor in C style ASCII.

	Wrapper around libusb_get_string_descriptor(). Uses the first language supported by the device.

	\param dev a device handle
	\param desc_index the index of the descriptor to retrieve
	\param data output buffer for ASCII string descriptor
	\param length size of data buffer
	\returns number of bytes returned in data, or LIBUSB_ERROR code on failure
*/
int XsLibUsb::get_string_descriptor_ascii(libusb_device_handle *dev, uint8_t desc_index, unsigned char *data, int length)
{
	if (m_libUsb.get_string_descriptor_ascii)
		return m_libUsb.get_string_descriptor_ascii(dev, desc_index, data, length);
	else
		return LIBUSB_ERROR_NOT_SUPPORTED;
}

/*! \brief Perform a USB bulk transfer. The direction of the transfer is inferred from the direction bits of the endpoint address.

	For bulk reads, the \a length field indicates the maximum length of
	data you are expecting to receive. If less data arrives than expected,
	this function will return that data, so be sure to check the
	\a transferred output parameter.

	You should also check the \a transferred parameter for bulk writes.
	Not all of the data may have been written.

	Also check \a transferred when dealing with a timeout error code.
	libusb may have to split your transfer into a number of chunks to satisfy
	underlying O/S requirements, meaning that the timeout may expire after
	the first few chunks have completed. libusb is careful not to lose any data
	that may have been transferred; do not assume that timeout conditions
	indicate a complete lack of I/O.

	\param dev_handle a handle for the device to communicate with
	\param endpoint the address of a valid endpoint to communicate with
	\param data a suitably-sized data buffer for either input or output (depending on endpoint)
	\param length for bulk writes, the number of bytes from data to be sent. for bulk reads, the maximum number of bytes to receive into the data buffer.
	\param transferred output location for the number of bytes actually transferred.
	\param timeout timeout (in millseconds) that this function should wait before giving up due to no response being received. For an unlimited timeout, use value 0.

	\returns 0 on success (and populates \a transferred)
	\returns LIBUSB_ERROR_TIMEOUT if the transfer timed out (and populates \a transferred)
	\returns LIBUSB_ERROR_PIPE if the endpoint halted
	\returns LIBUSB_ERROR_OVERFLOW if the device offered more data
	\returns LIBUSB_ERROR_NO_DEVICE if the device has been disconnected
	\returns another LIBUSB_ERROR code on other failures
*/
int XsLibUsb::bulk_transfer(libusb_device_handle *dev_handle,	unsigned char endpoint, unsigned char *data, int length, int *actual_length, unsigned int timeout)
{
	if (m_libUsb.bulk_transfer)
		return m_libUsb.bulk_transfer(dev_handle, endpoint, data, length, actual_length, timeout);
	else
		return LIBUSB_ERROR_NOT_SUPPORTED;
}

/*! \brief Set message verbosity.
	 - Level 0: no messages ever printed by the library (default)
	 - Level 1: error messages are printed to stderr
	 - Level 2: warning and error messages are printed to stderr
	 - Level 3: informational messages are printed to stdout, warning and error messages are printed to stderr

	The default level is 0, which means no messages are ever printed. If you
	choose to increase the message verbosity level, ensure that your
	application does not close the stdout/stderr file descriptors.

	You are advised to set level 3. libusb is conservative with its message
	logging and most of the time, will only log messages that explain error
	conditions and other oddities. This will help you debug your software.

	If the LIBUSB_DEBUG environment variable was set when libusb was
	initialized, this function does nothing: the message verbosity is fixed
	to the value in the environment variable.

	If libusb was compiled without any message logging, this function does
	nothing: you'll never get any messages.

	If libusb was compiled with verbose debug message logging, this function
	does nothing: you'll always get messages from all levels.

	\param ctx the context to operate on, or NULL for the default context
	\param level debug level to set
*/
void XsLibUsb::set_debug(libusb_context *ctx, int level)
{
	if (m_libUsb.set_debug)
		m_libUsb.set_debug(ctx, level);
}

#endif // patch for MRPT
