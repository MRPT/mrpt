
//  Copyright (c) 2003-2019 Xsens Technologies B.V. or subsidiaries worldwide.
//  All rights reserved.
//  
//  Redistribution and use in source and binary forms, with or without modification,
//  are permitted provided that the following conditions are met:
//  
//  1.	Redistributions of source code must retain the above copyright notice,
//  	this list of conditions, and the following disclaimer.
//  
//  2.	Redistributions in binary form must reproduce the above copyright notice,
//  	this list of conditions, and the following disclaimer in the documentation
//  	and/or other materials provided with the distribution.
//  
//  3.	Neither the names of the copyright holders nor the names of their contributors
//  	may be used to endorse or promote products derived from this software without
//  	specific prior written permission.
//  
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
//  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
//  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
//  THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
//  OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
//  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY OR
//  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.THE LAWS OF THE NETHERLANDS 
//  SHALL BE EXCLUSIVELY APPLICABLE AND ANY DISPUTES SHALL BE FINALLY SETTLED UNDER THE RULES 
//  OF ARBITRATION OF THE INTERNATIONAL CHAMBER OF COMMERCE IN THE HAGUE BY ONE OR MORE 
//  ARBITRATORS APPOINTED IN ACCORDANCE WITH SAID RULES.
//  

#ifndef UDEV
#define UDEV

struct udev;
struct udev_device;
struct udev_enumerate;
struct udev_list_entry;

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

#ifdef __cplusplus
class Udev
{
public:
	Udev(void);
	~Udev(void);

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

	struct UDEV_API
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
	} m_uDev;

	XsLibraryLoader* m_libraryLoader;

	void initLibrary();
};
#endif

#endif // file guard

