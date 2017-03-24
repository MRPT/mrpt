/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef XSUDEV
#define XSUDEV

#include <libudev.h>

struct XsLibraryLoader;

typedef struct udev *uDEV_new(void);
typedef struct udev *uDEV_unref(struct udev *udev);
typedef struct udev_device *uDEV_device_unref(struct udev_device *udev_device);
typedef struct udev_enumerate *uDEV_enumerate_new(struct udev *udev);
typedef int uDEV_enumerate_add_match_subsystem(struct udev_enumerate *udev_enumerate, const char *subsystem);
typedef int uDEV_enumerate_scan_devices(struct udev_enumerate *udev_enumerate);
typedef struct udev_list_entry *uDEV_enumerate_get_list_entry(struct udev_enumerate *udev_enumerate);
typedef struct udev_enumerate *uDEV_enumerate_unref(struct udev_enumerate *udev_enumerate);
typedef struct udev_list_entry *uDEV_list_entry_get_next(struct udev_list_entry *list_entry);
typedef const char *uDEV_list_entry_get_name(struct udev_list_entry *list_entry);
typedef struct udev_device *uDEV_device_new_from_syspath(struct udev *udev, const char *syspath);
typedef struct udev_device *uDEV_device_get_parent(struct udev_device *udev_device);
typedef const char *uDEV_device_get_devnode(struct udev_device *udev_device);
typedef struct udev_device *uDEV_device_get_parent_with_subsystem_devtype(struct udev_device *udev_device, const char *subsystem, const char *devtype);
typedef const char *uDEV_device_get_sysattr_value(struct udev_device *udev_device, const char *sysattr);

class XsUdev
{
public:
	XsUdev(void);
	~XsUdev(void);

	uDEV_new unew;
	uDEV_unref unref;
	uDEV_device_unref device_unref;
	uDEV_enumerate_new enumerate_new;
	uDEV_enumerate_add_match_subsystem enumerate_add_match_subsystem;
	uDEV_enumerate_scan_devices enumerate_scan_devices;
	uDEV_enumerate_get_list_entry enumerate_get_list_entry;
	uDEV_enumerate_unref enumerate_unref;
	uDEV_list_entry_get_next list_entry_get_next;
	uDEV_list_entry_get_name list_entry_get_name;
	uDEV_device_new_from_syspath device_new_from_syspath;
	uDEV_device_get_parent device_get_parent;
	uDEV_device_get_devnode device_get_devnode;
	uDEV_device_get_parent_with_subsystem_devtype device_get_parent_with_subsystem_devtype;
	uDEV_device_get_sysattr_value device_get_sysattr_value;

private:

	typedef struct _UDEV_API
	{
		uDEV_new (* unew);
		uDEV_unref (* unref);
		uDEV_device_unref (* device_unref);
		uDEV_enumerate_new (* enumerate_new);
		uDEV_enumerate_add_match_subsystem (* enumerate_add_match_subsystem);
		uDEV_enumerate_scan_devices (* enumerate_scan_devices);
		uDEV_enumerate_get_list_entry (* enumerate_get_list_entry);
		uDEV_enumerate_unref (* enumerate_unref);
		uDEV_list_entry_get_next (* list_entry_get_next);
		uDEV_list_entry_get_name (* list_entry_get_name);
		uDEV_device_new_from_syspath (* device_new_from_syspath);
		uDEV_device_get_parent (* device_get_parent);
		uDEV_device_get_devnode (* device_get_devnode);
		uDEV_device_get_parent_with_subsystem_devtype (* device_get_parent_with_subsystem_devtype);
		uDEV_device_get_sysattr_value (* device_get_sysattr_value);
	} UDEV_API;

	UDEV_API m_uDev;
	XsLibraryLoader* m_libraryLoader;

	void initLibrary();
};

#endif // file guard

