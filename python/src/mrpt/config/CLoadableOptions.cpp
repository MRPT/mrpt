#include <ios>
#include <iterator>
#include <memory>
#include <mrpt/bayes/CKalmanFilterCapable.h>
#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/config/CLoadableOptions.h>
#include <mrpt/system/COutputLogger.h>
#include <ostream>
#include <sstream> // __str__
#include <streambuf>
#include <string>
#include <vector>

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

// mrpt::config::CLoadableOptions file:mrpt/config/CLoadableOptions.h line:24
struct PyCallBack_mrpt_config_CLoadableOptions : public mrpt::config::CLoadableOptions {
	using mrpt::config::CLoadableOptions::CLoadableOptions;

	void loadFromConfigFile(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::config::CLoadableOptions *>(this), "loadFromConfigFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CLoadableOptions::loadFromConfigFile\"");
	}
	void saveToConfigFile(class mrpt::config::CConfigFileBase & a0, const std::string & a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::config::CLoadableOptions *>(this), "saveToConfigFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CLoadableOptions::saveToConfigFile(a0, a1);
	}
};

void bind_mrpt_config_CLoadableOptions(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::config::CLoadableOptions file:mrpt/config/CLoadableOptions.h line:24
		pybind11::class_<mrpt::config::CLoadableOptions, std::shared_ptr<mrpt::config::CLoadableOptions>, PyCallBack_mrpt_config_CLoadableOptions> cl(M("mrpt::config"), "CLoadableOptions", "This is a virtual base class for sets of options than can be loaded from\n and/or saved to configuration plain-text files.\n \n\n\n ");
		cl.def(pybind11::init<PyCallBack_mrpt_config_CLoadableOptions const &>());
		cl.def( pybind11::init( [](){ return new PyCallBack_mrpt_config_CLoadableOptions(); } ) );
		cl.def("loadFromConfigFile", (void (mrpt::config::CLoadableOptions::*)(const class mrpt::config::CConfigFileBase &, const std::string &)) &mrpt::config::CLoadableOptions::loadFromConfigFile, "This method load the options from a \".ini\"-like file or memory-stored\n string list.\n   Only those parameters found in the given \"section\" and having\n   the same name that the variable are loaded. Those not found in\n   the file will stay with their previous values (usually the default\n   values loaded at initialization). An example of an \".ini\" file:\n  \n\n\n\n\n\n \n loadFromConfigFileName, saveToConfigFile\n\nC++: mrpt::config::CLoadableOptions::loadFromConfigFile(const class mrpt::config::CConfigFileBase &, const std::string &) --> void", pybind11::arg("source"), pybind11::arg("section"));
		cl.def("loadFromConfigFileName", (void (mrpt::config::CLoadableOptions::*)(const std::string &, const std::string &)) &mrpt::config::CLoadableOptions::loadFromConfigFileName, "Behaves like loadFromConfigFile, but you can pass directly a file name\n and a temporary CConfigFile object will be created automatically to load\n the file.\n \n\n loadFromConfigFile\n\nC++: mrpt::config::CLoadableOptions::loadFromConfigFileName(const std::string &, const std::string &) --> void", pybind11::arg("config_file"), pybind11::arg("section"));
		cl.def("saveToConfigFile", (void (mrpt::config::CLoadableOptions::*)(class mrpt::config::CConfigFileBase &, const std::string &) const) &mrpt::config::CLoadableOptions::saveToConfigFile, "This method saves the options to a \".ini\"-like file or memory-stored\n string list.\n \n\n loadFromConfigFile, saveToConfigFileName\n\nC++: mrpt::config::CLoadableOptions::saveToConfigFile(class mrpt::config::CConfigFileBase &, const std::string &) const --> void", pybind11::arg("target"), pybind11::arg("section"));
		cl.def("saveToConfigFileName", (void (mrpt::config::CLoadableOptions::*)(const std::string &, const std::string &) const) &mrpt::config::CLoadableOptions::saveToConfigFileName, "Behaves like saveToConfigFile, but you can pass directly a file name and\n a temporary CConfigFile object will be created automatically to save the\n file.\n \n\n saveToConfigFile, loadFromConfigFileName\n\nC++: mrpt::config::CLoadableOptions::saveToConfigFileName(const std::string &, const std::string &) const --> void", pybind11::arg("config_file"), pybind11::arg("section"));
		cl.def("dumpToConsole", (void (mrpt::config::CLoadableOptions::*)() const) &mrpt::config::CLoadableOptions::dumpToConsole, "Just like  but sending the text to the console\n (std::cout) \n\nC++: mrpt::config::CLoadableOptions::dumpToConsole() const --> void");
		cl.def("assign", (class mrpt::config::CLoadableOptions & (mrpt::config::CLoadableOptions::*)(const class mrpt::config::CLoadableOptions &)) &mrpt::config::CLoadableOptions::operator=, "C++: mrpt::config::CLoadableOptions::operator=(const class mrpt::config::CLoadableOptions &) --> class mrpt::config::CLoadableOptions &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
