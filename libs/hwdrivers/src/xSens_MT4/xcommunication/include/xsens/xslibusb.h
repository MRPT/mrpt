/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef XSLIBUSB
#define XSLIBUSB

#include <libusb-1.0/libusb.h>

struct XsLibraryLoader;

typedef int libUSB_init(libusb_context **ctx);
typedef void libUSB_exit(libusb_context *ctx);
typedef int libUSB_open(libusb_device *dev, libusb_device_handle **handle);
typedef void libUSB_close(libusb_device_handle *dev_handle);
typedef int libUSB_kernel_driver_active(libusb_device_handle *dev,int interface_number);
typedef int libUSB_attach_kernel_driver(libusb_device_handle *dev,int interface_number);
typedef int libUSB_detach_kernel_driver(libusb_device_handle *dev,int interface_number);
typedef libusb_device * libUSB_ref_device(libusb_device *dev);
typedef void libUSB_unref_device(libusb_device *dev);
typedef int libUSB_claim_interface(libusb_device_handle *dev,int interface_number);
typedef int libUSB_release_interface(libusb_device_handle *dev,	int interface_number);
typedef int libUSB_get_active_config_descriptor(libusb_device *dev,	struct libusb_config_descriptor **config);
typedef void libUSB_free_config_descriptor(struct libusb_config_descriptor *config);
typedef uint8_t libUSB_get_bus_number(libusb_device *dev);
typedef libusb_device * libUSB_get_device(libusb_device_handle *dev_handle);
typedef uint8_t libUSB_get_device_address(libusb_device *dev);
typedef int libUSB_get_device_descriptor(libusb_device *dev, struct libusb_device_descriptor *desc);
typedef ssize_t libUSB_get_device_list(libusb_context *ctx,	libusb_device ***list);
typedef void libUSB_free_device_list(libusb_device **list,	int unref_devices);
typedef int libUSB_get_string_descriptor_ascii(libusb_device_handle *dev, uint8_t desc_index, unsigned char *data, int length);
typedef int libUSB_bulk_transfer(libusb_device_handle *dev_handle,	unsigned char endpoint, unsigned char *data, int length, int *actual_length, unsigned int timeout);
typedef void libUSB_set_debug(libusb_context *ctx, int level);

class XsLibUsb
{
public:
	XsLibUsb(void);
	~XsLibUsb(void);

	libUSB_init init;
	libUSB_exit exit;
	libUSB_open open;
	libUSB_close close;
	libUSB_kernel_driver_active kernel_driver_active;
	libUSB_attach_kernel_driver attach_kernel_driver;
	libUSB_detach_kernel_driver detach_kernel_driver;
	libUSB_ref_device ref_device;
	libUSB_unref_device unref_device;
	libUSB_claim_interface claim_interface;
	libUSB_release_interface release_interface;
	libUSB_get_active_config_descriptor get_active_config_descriptor;
	libUSB_free_config_descriptor free_config_descriptor;
	libUSB_get_bus_number get_bus_number;
	libUSB_get_device get_device;
	libUSB_get_device_address get_device_address;
	libUSB_get_device_descriptor get_device_descriptor;
	libUSB_get_device_list get_device_list;
	libUSB_free_device_list free_device_list;
	libUSB_get_string_descriptor_ascii get_string_descriptor_ascii;
	libUSB_bulk_transfer bulk_transfer;
	libUSB_set_debug set_debug;
private:

	typedef struct _LIBUSB_API
	{
		libUSB_init (* init);
		libUSB_exit (* exit);
		libUSB_open (* open);
		libUSB_close (* close);
		libUSB_kernel_driver_active (* kernel_driver_active);
		libUSB_attach_kernel_driver (* attach_kernel_driver);
		libUSB_detach_kernel_driver (* detach_kernel_driver);
		libUSB_ref_device (* ref_device);
		libUSB_unref_device (* unref_device);
		libUSB_claim_interface (* claim_interface);
		libUSB_release_interface (* release_interface);
		libUSB_get_active_config_descriptor (* get_active_config_descriptor);
		libUSB_free_config_descriptor (* free_config_descriptor);
		libUSB_get_bus_number (* get_bus_number);
		libUSB_get_device (* get_device);
		libUSB_get_device_address (* get_device_address);
		libUSB_get_device_descriptor (* get_device_descriptor);
		libUSB_get_device_list (* get_device_list);
		libUSB_free_device_list (* free_device_list);
		libUSB_get_string_descriptor_ascii (* get_string_descriptor_ascii);
		libUSB_bulk_transfer (* bulk_transfer);
		libUSB_set_debug (* set_debug);
	} LIBUSB_API;

	LIBUSB_API m_libUsb;
	XsLibraryLoader* m_libraryLoader;

	void initLibrary();
};

#endif // file guard

