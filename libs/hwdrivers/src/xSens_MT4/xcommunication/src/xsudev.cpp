/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef _WIN32  // patch for MRPT

#include "xsudev.h"
#include <xsens/xslibraryloader.h>

/*! \class XsUdev
	\brief Class for dynamic loading of winusb
*/
XsUdev::XsUdev(void)
{
	m_libraryLoader = new XsLibraryLoader();
	initLibrary();
}

XsUdev::~XsUdev(void)
{
	delete m_libraryLoader;
}

void XsUdev::initLibrary()
{
	if (!m_libraryLoader->isLoaded())
		m_libraryLoader->load("libudev.so");

	m_uDev.unew = NULL;
	m_uDev.unref = NULL;
	m_uDev.device_unref = NULL;
	m_uDev.enumerate_new = NULL;
	m_uDev.enumerate_add_match_subsystem = NULL;
	m_uDev.enumerate_scan_devices = NULL;
	m_uDev.enumerate_get_list_entry = NULL;
	m_uDev.enumerate_unref = NULL;
	m_uDev.list_entry_get_next = NULL;
	m_uDev.list_entry_get_name = NULL;
	m_uDev.device_new_from_syspath = NULL;
	m_uDev.device_get_parent = NULL;
	m_uDev.device_get_devnode = NULL;
	m_uDev.device_get_parent_with_subsystem_devtype = NULL;
	m_uDev.device_get_sysattr_value = NULL;

	if (m_libraryLoader->isLoaded())
	{
		m_uDev.unew = (uDEV_new*)m_libraryLoader->resolve("udev_new");
		m_uDev.unref = (uDEV_unref*)m_libraryLoader->resolve("udev_unref");
		m_uDev.device_unref = (uDEV_device_unref*)m_libraryLoader->resolve("udev_device_unref");
		m_uDev.enumerate_new = (uDEV_enumerate_new*)m_libraryLoader->resolve("udev_enumerate_new");
		m_uDev.enumerate_add_match_subsystem = (uDEV_enumerate_add_match_subsystem*)m_libraryLoader->resolve("udev_enumerate_add_match_subsystem");
		m_uDev.enumerate_scan_devices = (uDEV_enumerate_scan_devices*)m_libraryLoader->resolve("udev_enumerate_scan_devices");
		m_uDev.enumerate_get_list_entry = (uDEV_enumerate_get_list_entry*)m_libraryLoader->resolve("udev_enumerate_get_list_entry");
		m_uDev.enumerate_unref = (uDEV_enumerate_unref*)m_libraryLoader->resolve("udev_enumerate_unref");
		m_uDev.list_entry_get_next = (uDEV_list_entry_get_next*)m_libraryLoader->resolve("udev_list_entry_get_next");
		m_uDev.list_entry_get_name = (uDEV_list_entry_get_name*)m_libraryLoader->resolve("udev_list_entry_get_name");
		m_uDev.device_new_from_syspath = (uDEV_device_new_from_syspath*)m_libraryLoader->resolve("udev_device_new_from_syspath");
		m_uDev.device_get_parent = (uDEV_device_get_parent*)m_libraryLoader->resolve("udev_device_get_parent");
		m_uDev.device_get_devnode = (uDEV_device_get_devnode*)m_libraryLoader->resolve("udev_device_get_devnode");
		m_uDev.device_get_parent_with_subsystem_devtype = (uDEV_device_get_parent_with_subsystem_devtype*)m_libraryLoader->resolve("udev_device_get_parent_with_subsystem_devtype");
		m_uDev.device_get_sysattr_value = (uDEV_device_get_sysattr_value*)m_libraryLoader->resolve("udev_device_get_sysattr_value");
	}
}

/*! \brief Create udev library context.

	This reads the udev configuration file, and fills in the default values.

	The initial refcount is 1, and needs to be decremented to release the resources of the udev library context.

	\returns a new udev library context
*/
udev *XsUdev::unew(void)
{
	if (m_uDev.unew)
		return m_uDev.unew();
	else
		return NULL;
}

/*! \brief Drop a reference of the udev library context.

	\param udev udev library context

	If the refcount reaches zero, the resources of the context will be released.
*/
udev *XsUdev::unref(struct udev *udev)
{
	if (m_uDev.unref)
		return m_uDev.unref(udev);
	else
		return NULL;
}

/*! \brief Drop a reference of a udev device.

	If the refcount reaches zero, the resources of the device will be released.

	\param udev_device udev device
	\return NULL
*/
udev_device *XsUdev::device_unref(struct udev_device *udev_device)
{
	if (m_uDev.device_unref)
		return m_uDev.device_unref(udev_device);
	else
		return NULL;
}

/*! \brief Create an enumeration context to scan.
	\param udev udev library context

	\return an enumeration context.
*/
udev_enumerate *XsUdev::enumerate_new(struct udev *udev)
{
	if (m_uDev.enumerate_new)
		return m_uDev.enumerate_new(udev);
	else
		return NULL;
}

/*! \brief Match only devices belonging to a certain kernel subsystem.
	\param udev_enumerate context
	\param subsystem  filter for a subsystem of the device to include in the list
	\return: 0 on success, otherwise a negative error value.
*/
int XsUdev::enumerate_add_match_subsystem(struct udev_enumerate *udev_enumerate, const char *subsystem)
{
	if (m_uDev.enumerate_add_match_subsystem)
		return m_uDev.enumerate_add_match_subsystem(udev_enumerate, subsystem);
	else
		return -1;
}

/*! \brief Scan /sys for all devices which match the given filters. No matches will return all currently available devices.
	\param udev_enumerate udev enumeration context
	\return 0 on success, otherwise a negative error value.
*/
int XsUdev::enumerate_scan_devices(struct udev_enumerate *udev_enumerate)
{
	if (m_uDev.enumerate_scan_devices)
		return m_uDev.enumerate_scan_devices(udev_enumerate);
	else
		return -1;
}

/*! \brief Get the next entry from the list.

	\param list_entry current entry
	\return udev_list_entry, NULL if no more entries are available.
*/
udev_list_entry *XsUdev::list_entry_get_next(struct udev_list_entry *list_entry)
{
	if (m_uDev.list_entry_get_next)
		return m_uDev.list_entry_get_next(list_entry);
	else
		return NULL;
}

/*! \brief Get the first entry of the sorted list of device paths.
	\param udev_enumerate context
	\return a udev_list_entry.
*/
udev_list_entry *XsUdev::enumerate_get_list_entry(struct udev_enumerate *udev_enumerate)
{
	if (m_uDev.enumerate_get_list_entry)
		return m_uDev.enumerate_get_list_entry(udev_enumerate);
	else
		return NULL;
}

/*! \brief Drop a reference of an enumeration context.

	If the refcount reaches zero, all resources of the enumeration context will be released.

	\param udev_enumerate context

	\return: NULL
*/
udev_enumerate *XsUdev::enumerate_unref(struct udev_enumerate *udev_enumerate)
{
	if (m_uDev.enumerate_unref)
		return m_uDev.enumerate_unref(udev_enumerate);
	else
		return NULL;
}

/*! \brief Get the name of a list entry.
	\param list_entry: current entry
	\return the name string of this entry.
*/
const char *XsUdev::list_entry_get_name(struct udev_list_entry *list_entry)
{
	if (m_uDev.list_entry_get_name)
		return m_uDev.list_entry_get_name(list_entry);
	else
		return "";
}

/*! \brief Create new udev device, and fill in information from the sys device and the udev database entry.

	The syspath is the absolute path to the device, including the sys mount point.

	\param udev udev library context
	\param syspath sys device path including sys directory
	\return a new udev device, or NULL, if it does not exist
*/
udev_device *XsUdev::device_new_from_syspath(struct udev *udev, const char *syspath)
{
	if (m_uDev.device_new_from_syspath)
		return m_uDev.device_new_from_syspath(udev, syspath);
	else
		return NULL;
}

/*! \brief Find the next parent device, and fill in information from the sys device and the udev database entry.

	Returned device is not referenced. It is attached to the child device, and will be cleaned up when the child device is cleaned up.
	It is not necessarily just the upper level directory, empty or not recognized sys directories are ignored.

	It can be called as many times as needed, without caring about references.

	\param udev_device: the device to start searching from
	\return a new udev device, or NULL, if it no parent exist.
*/
udev_device *XsUdev::device_get_parent(struct udev_device *udev_device)
{
	if (m_uDev.device_get_parent)
		return m_uDev.device_get_parent(udev_device);
	else
		return NULL;
}

/*! \brief Retrieve the device node file name belonging to the udev device.

	The path is an absolute path, and starts with the device directory.

	\param udev_device udev device
	\return the device node file name of the udev device, or NULL if no device node exists
*/
const char *XsUdev::device_get_devnode(struct udev_device *udev_device)
{
	if (m_uDev.device_get_devnode)
		return m_uDev.device_get_devnode(udev_device);
	else
		return "";
}

/*! \brief Find the next parent device, with a matching subsystem and devtypevalue, and fill in information from the sys device and the udev database entry.

	If devtype is NULL, only subsystem is checked, and any devtype will match.

	Returned device is not referenced. It is attached to the child device, and will be cleaned up when the child device is cleaned up.

	It can be called as many times as needed, without caring about references.

	\param udev_device udev device to start searching from
	\param subsystem the subsystem of the device
	\param devtype the type (DEVTYPE) of the device
	\return a new udev device, or NULL if no matching parent exists.
*/
udev_device *XsUdev::device_get_parent_with_subsystem_devtype(struct udev_device *udev_device, const char *subsystem, const char *devtype)
{
	if (m_uDev.device_get_parent_with_subsystem_devtype)
		return m_uDev.device_get_parent_with_subsystem_devtype(udev_device, subsystem, devtype);
	else
		return NULL;
}

/*! \brief Get a sys attribute value

	The retrieved value is cached in the device. Repeated calls will return the same value and not open the attribute again.

	\param udev_device udev device
	\param sysattr attribute name

	\return the content of a sys attribute file, or NULL if there is no sys attribute value.
*/
const char *XsUdev::device_get_sysattr_value(struct udev_device *udev_device, const char *sysattr)
{
	if (m_uDev.device_get_sysattr_value)
		return m_uDev.device_get_sysattr_value(udev_device, sysattr);
	else
		return "";
}

#endif // patch for MRPT
