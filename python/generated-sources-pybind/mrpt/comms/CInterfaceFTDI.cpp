#include <ios>
#include <locale>
#include <mrpt/comms/CInterfaceFTDI.h>
#include <ostream>
#include <sstream> // __str__
#include <streambuf>
#include <string>

#include <functional>
#include <pybind11/pybind11.h>
#include <string>
#include <pybind11/stl.h>


#ifndef BINDER_PYBIND11_TYPE_CASTER
	#define BINDER_PYBIND11_TYPE_CASTER
	PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>)
	PYBIND11_DECLARE_HOLDER_TYPE(T, T*)
	PYBIND11_MAKE_OPAQUE(std::shared_ptr<void>)
#endif

void bind_mrpt_comms_CInterfaceFTDI(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::comms::TFTDIDevice file:mrpt/comms/CInterfaceFTDI.h line:23
		pybind11::class_<mrpt::comms::TFTDIDevice, std::shared_ptr<mrpt::comms::TFTDIDevice>> cl(M("mrpt::comms"), "TFTDIDevice", "A list of FTDI devices and their descriptors.\n \n\n CInterfaceFTDI::ListAllDevices\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::comms::TFTDIDevice(); } ) );
		cl.def( pybind11::init( [](mrpt::comms::TFTDIDevice const &o){ return new mrpt::comms::TFTDIDevice(o); } ) );
		cl.def_readwrite("ftdi_manufacturer", &mrpt::comms::TFTDIDevice::ftdi_manufacturer);
		cl.def_readwrite("ftdi_description", &mrpt::comms::TFTDIDevice::ftdi_description);
		cl.def_readwrite("ftdi_serial", &mrpt::comms::TFTDIDevice::ftdi_serial);
		cl.def_readwrite("usb_idVendor", &mrpt::comms::TFTDIDevice::usb_idVendor);
		cl.def_readwrite("usb_idProduct", &mrpt::comms::TFTDIDevice::usb_idProduct);
		cl.def_readwrite("usb_serialNumber", &mrpt::comms::TFTDIDevice::usb_serialNumber);
		cl.def("assign", (struct mrpt::comms::TFTDIDevice & (mrpt::comms::TFTDIDevice::*)(const struct mrpt::comms::TFTDIDevice &)) &mrpt::comms::TFTDIDevice::operator=, "C++: mrpt::comms::TFTDIDevice::operator=(const struct mrpt::comms::TFTDIDevice &) --> struct mrpt::comms::TFTDIDevice &", pybind11::return_value_policy::automatic, pybind11::arg(""));

		cl.def("__str__", [](mrpt::comms::TFTDIDevice const &o) -> std::string { std::ostringstream s; using namespace mrpt::comms; s << o; return s.str(); } );
	}
}
