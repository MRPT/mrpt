#include <iterator>
#include <memory>
#include <mrpt/bayes/CKalmanFilterCapable.h>
#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/config/CConfigFileMemory.h>
#include <mrpt/system/COutputLogger.h>
#include <sstream> // __str__
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

// mrpt::config::CConfigFileBase file:mrpt/config/CConfigFileBase.h line:45
struct PyCallBack_mrpt_config_CConfigFileBase : public mrpt::config::CConfigFileBase {
	using mrpt::config::CConfigFileBase::CConfigFileBase;

	void writeString(const std::string & a0, const std::string & a1, const std::string & a2) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::config::CConfigFileBase *>(this), "writeString");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CConfigFileBase::writeString\"");
	}
	std::string readString(const std::string & a0, const std::string & a1, const std::string & a2, bool a3) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::config::CConfigFileBase *>(this), "readString");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2, a3);
			if (pybind11::detail::cast_is_temporary_value_reference<std::string>::value) {
				static pybind11::detail::override_caster_t<std::string> caster;
				return pybind11::detail::cast_ref<std::string>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<std::string>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CConfigFileBase::readString\"");
	}
	void getAllSections(class std::vector<std::string > & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::config::CConfigFileBase *>(this), "getAllSections");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CConfigFileBase::getAllSections\"");
	}
	void getAllKeys(const std::string & a0, class std::vector<std::string > & a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::config::CConfigFileBase *>(this), "getAllKeys");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CConfigFileBase::getAllKeys\"");
	}
	void clear() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::config::CConfigFileBase *>(this), "clear");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CConfigFileBase::clear\"");
	}
};

// mrpt::config::CConfigFileMemory file:mrpt/config/CConfigFileMemory.h line:37
struct PyCallBack_mrpt_config_CConfigFileMemory : public mrpt::config::CConfigFileMemory {
	using mrpt::config::CConfigFileMemory::CConfigFileMemory;

	void clear() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::config::CConfigFileMemory *>(this), "clear");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CConfigFileMemory::clear();
	}
	void getAllSections(class std::vector<std::string > & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::config::CConfigFileMemory *>(this), "getAllSections");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CConfigFileMemory::getAllSections(a0);
	}
	void getAllKeys(const std::string & a0, class std::vector<std::string > & a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::config::CConfigFileMemory *>(this), "getAllKeys");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CConfigFileMemory::getAllKeys(a0, a1);
	}
	void writeString(const std::string & a0, const std::string & a1, const std::string & a2) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::config::CConfigFileMemory *>(this), "writeString");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CConfigFileMemory::writeString(a0, a1, a2);
	}
	std::string readString(const std::string & a0, const std::string & a1, const std::string & a2, bool a3) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::config::CConfigFileMemory *>(this), "readString");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2, a3);
			if (pybind11::detail::cast_is_temporary_value_reference<std::string>::value) {
				static pybind11::detail::override_caster_t<std::string> caster;
				return pybind11::detail::cast_ref<std::string>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<std::string>(std::move(o));
		}
		return CConfigFileMemory::readString(a0, a1, a2, a3);
	}
};

void bind_mrpt_config_CConfigFileBase(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	// mrpt::config::MRPT_SAVE_NAME_PADDING() file:mrpt/config/CConfigFileBase.h line:33
	M("mrpt::config").def("MRPT_SAVE_NAME_PADDING", (int (*)()) &mrpt::config::MRPT_SAVE_NAME_PADDING, "Default padding sizes for macros MRPT_SAVE_CONFIG_VAR_COMMENT(), etc. \n\nC++: mrpt::config::MRPT_SAVE_NAME_PADDING() --> int");

	// mrpt::config::MRPT_SAVE_VALUE_PADDING() file:mrpt/config/CConfigFileBase.h line:34
	M("mrpt::config").def("MRPT_SAVE_VALUE_PADDING", (int (*)()) &mrpt::config::MRPT_SAVE_VALUE_PADDING, "C++: mrpt::config::MRPT_SAVE_VALUE_PADDING() --> int");

	{ // mrpt::config::CConfigFileBase file:mrpt/config/CConfigFileBase.h line:45
		pybind11::class_<mrpt::config::CConfigFileBase, std::shared_ptr<mrpt::config::CConfigFileBase>, PyCallBack_mrpt_config_CConfigFileBase> cl(M("mrpt::config"), "CConfigFileBase", "This class allows loading and storing values and vectors of different types\n from a configuration text, which can be implemented as a \".ini\" file, a\n memory-stored string, etc...\n   This is a virtual class, use only as a pointer to an implementation of one\n of the derived classes.\n\n See: \n \n\n\n ");
		cl.def(pybind11::init<PyCallBack_mrpt_config_CConfigFileBase const &>());
		cl.def( pybind11::init( [](){ return new PyCallBack_mrpt_config_CConfigFileBase(); } ) );
		cl.def("read_enum", [](mrpt::config::CConfigFileBase const &o, const std::string & a0, const std::string & a1, const enum mrpt::bayes::TKFMethod & a2) -> mrpt::bayes::TKFMethod { return o.read_enum(a0, a1, a2); }, "", pybind11::arg("section"), pybind11::arg("name"), pybind11::arg("defaultValue"));
		cl.def("read_enum", (enum mrpt::bayes::TKFMethod (mrpt::config::CConfigFileBase::*)(const std::string &, const std::string &, const enum mrpt::bayes::TKFMethod &, bool) const) &mrpt::config::CConfigFileBase::read_enum<mrpt::bayes::TKFMethod>, "C++: mrpt::config::CConfigFileBase::read_enum(const std::string &, const std::string &, const enum mrpt::bayes::TKFMethod &, bool) const --> enum mrpt::bayes::TKFMethod", pybind11::arg("section"), pybind11::arg("name"), pybind11::arg("defaultValue"), pybind11::arg("failIfNotFound"));
		cl.def("read_enum", [](mrpt::config::CConfigFileBase const &o, const std::string & a0, const std::string & a1, const enum mrpt::system::VerbosityLevel & a2) -> mrpt::system::VerbosityLevel { return o.read_enum(a0, a1, a2); }, "", pybind11::arg("section"), pybind11::arg("name"), pybind11::arg("defaultValue"));
		cl.def("read_enum", (enum mrpt::system::VerbosityLevel (mrpt::config::CConfigFileBase::*)(const std::string &, const std::string &, const enum mrpt::system::VerbosityLevel &, bool) const) &mrpt::config::CConfigFileBase::read_enum<mrpt::system::VerbosityLevel>, "C++: mrpt::config::CConfigFileBase::read_enum(const std::string &, const std::string &, const enum mrpt::system::VerbosityLevel &, bool) const --> enum mrpt::system::VerbosityLevel", pybind11::arg("section"), pybind11::arg("name"), pybind11::arg("defaultValue"), pybind11::arg("failIfNotFound"));
		cl.def("getAllSections", (void (mrpt::config::CConfigFileBase::*)(class std::vector<std::string > &) const) &mrpt::config::CConfigFileBase::getAllSections, "Returns a list with all the section names. \n\nC++: mrpt::config::CConfigFileBase::getAllSections(class std::vector<std::string > &) const --> void", pybind11::arg("sections"));
		cl.def("sections", (class std::vector<std::string > (mrpt::config::CConfigFileBase::*)() const) &mrpt::config::CConfigFileBase::sections, "Returns, by value, a list with all the section names. \n\nC++: mrpt::config::CConfigFileBase::sections() const --> class std::vector<std::string >");
		cl.def("getAllKeys", (void (mrpt::config::CConfigFileBase::*)(const std::string &, class std::vector<std::string > &) const) &mrpt::config::CConfigFileBase::getAllKeys, "Returs a list with all the keys into a section \n\nC++: mrpt::config::CConfigFileBase::getAllKeys(const std::string &, class std::vector<std::string > &) const --> void", pybind11::arg("section"), pybind11::arg("keys"));
		cl.def("keys", (class std::vector<std::string > (mrpt::config::CConfigFileBase::*)(const std::string &) const) &mrpt::config::CConfigFileBase::keys, "Returs, by value, a list with all the keys into a section \n\nC++: mrpt::config::CConfigFileBase::keys(const std::string &) const --> class std::vector<std::string >", pybind11::arg("section"));
		cl.def("sectionExists", (bool (mrpt::config::CConfigFileBase::*)(const std::string &) const) &mrpt::config::CConfigFileBase::sectionExists, "Checks if a given section exists (name is case insensitive)\n \n\n keyExists() \n\nC++: mrpt::config::CConfigFileBase::sectionExists(const std::string &) const --> bool", pybind11::arg("section_name"));
		cl.def("keyExists", (bool (mrpt::config::CConfigFileBase::*)(const std::string &, const std::string &) const) &mrpt::config::CConfigFileBase::keyExists, "Checks if a given key exists inside a section (case insensitive)\n \n\n sectionExists() \n\nC++: mrpt::config::CConfigFileBase::keyExists(const std::string &, const std::string &) const --> bool", pybind11::arg("section"), pybind11::arg("key"));
		cl.def("setContentFromYAML", (void (mrpt::config::CConfigFileBase::*)(const std::string &)) &mrpt::config::CConfigFileBase::setContentFromYAML, "Changes the contents of the virtual \"config file\" from a text block\n containing a YAML configuration text. Refer to unit test\n yaml2config_unittest.cpp for examples of use.\n \n\n getContentAsYAML()\n\nC++: mrpt::config::CConfigFileBase::setContentFromYAML(const std::string &) --> void", pybind11::arg("yaml_block"));
		cl.def("getContentAsYAML", (std::string (mrpt::config::CConfigFileBase::*)() const) &mrpt::config::CConfigFileBase::getContentAsYAML, "Returns a text block representing the contents of the config file in\n YAML format.\n \n\n setContentFromYAML()\n\nC++: mrpt::config::CConfigFileBase::getContentAsYAML() const --> std::string");
		cl.def("clear", (void (mrpt::config::CConfigFileBase::*)()) &mrpt::config::CConfigFileBase::clear, "Empties the \"config file\" \n\nC++: mrpt::config::CConfigFileBase::clear() --> void");
		cl.def("write", [](mrpt::config::CConfigFileBase &o, const std::string & a0, const std::string & a1, double const & a2) -> void { return o.write(a0, a1, a2); }, "", pybind11::arg("section"), pybind11::arg("name"), pybind11::arg("value"));
		cl.def("write", [](mrpt::config::CConfigFileBase &o, const std::string & a0, const std::string & a1, double const & a2, const int & a3) -> void { return o.write(a0, a1, a2, a3); }, "", pybind11::arg("section"), pybind11::arg("name"), pybind11::arg("value"), pybind11::arg("name_padding_width"));
		cl.def("write", [](mrpt::config::CConfigFileBase &o, const std::string & a0, const std::string & a1, double const & a2, const int & a3, const int & a4) -> void { return o.write(a0, a1, a2, a3, a4); }, "", pybind11::arg("section"), pybind11::arg("name"), pybind11::arg("value"), pybind11::arg("name_padding_width"), pybind11::arg("value_padding_width"));
		cl.def("write", (void (mrpt::config::CConfigFileBase::*)(const std::string &, const std::string &, double, const int, const int, const std::string &)) &mrpt::config::CConfigFileBase::write, "C++: mrpt::config::CConfigFileBase::write(const std::string &, const std::string &, double, const int, const int, const std::string &) --> void", pybind11::arg("section"), pybind11::arg("name"), pybind11::arg("value"), pybind11::arg("name_padding_width"), pybind11::arg("value_padding_width"), pybind11::arg("comment"));
		cl.def("write", [](mrpt::config::CConfigFileBase &o, const std::string & a0, const std::string & a1, float const & a2) -> void { return o.write(a0, a1, a2); }, "", pybind11::arg("section"), pybind11::arg("name"), pybind11::arg("value"));
		cl.def("write", [](mrpt::config::CConfigFileBase &o, const std::string & a0, const std::string & a1, float const & a2, const int & a3) -> void { return o.write(a0, a1, a2, a3); }, "", pybind11::arg("section"), pybind11::arg("name"), pybind11::arg("value"), pybind11::arg("name_padding_width"));
		cl.def("write", [](mrpt::config::CConfigFileBase &o, const std::string & a0, const std::string & a1, float const & a2, const int & a3, const int & a4) -> void { return o.write(a0, a1, a2, a3, a4); }, "", pybind11::arg("section"), pybind11::arg("name"), pybind11::arg("value"), pybind11::arg("name_padding_width"), pybind11::arg("value_padding_width"));
		cl.def("write", (void (mrpt::config::CConfigFileBase::*)(const std::string &, const std::string &, float, const int, const int, const std::string &)) &mrpt::config::CConfigFileBase::write, "C++: mrpt::config::CConfigFileBase::write(const std::string &, const std::string &, float, const int, const int, const std::string &) --> void", pybind11::arg("section"), pybind11::arg("name"), pybind11::arg("value"), pybind11::arg("name_padding_width"), pybind11::arg("value_padding_width"), pybind11::arg("comment"));
		cl.def("read_double", [](mrpt::config::CConfigFileBase const &o, const std::string & a0, const std::string & a1, double const & a2) -> double { return o.read_double(a0, a1, a2); }, "", pybind11::arg("section"), pybind11::arg("name"), pybind11::arg("defaultValue"));
		cl.def("read_double", (double (mrpt::config::CConfigFileBase::*)(const std::string &, const std::string &, double, bool) const) &mrpt::config::CConfigFileBase::read_double, "not found and `failIfNotFound`=true\n @{ \n\nC++: mrpt::config::CConfigFileBase::read_double(const std::string &, const std::string &, double, bool) const --> double", pybind11::arg("section"), pybind11::arg("name"), pybind11::arg("defaultValue"), pybind11::arg("failIfNotFound"));
		cl.def("read_float", [](mrpt::config::CConfigFileBase const &o, const std::string & a0, const std::string & a1, float const & a2) -> float { return o.read_float(a0, a1, a2); }, "", pybind11::arg("section"), pybind11::arg("name"), pybind11::arg("defaultValue"));
		cl.def("read_float", (float (mrpt::config::CConfigFileBase::*)(const std::string &, const std::string &, float, bool) const) &mrpt::config::CConfigFileBase::read_float, "C++: mrpt::config::CConfigFileBase::read_float(const std::string &, const std::string &, float, bool) const --> float", pybind11::arg("section"), pybind11::arg("name"), pybind11::arg("defaultValue"), pybind11::arg("failIfNotFound"));
		cl.def("read_bool", [](mrpt::config::CConfigFileBase const &o, const std::string & a0, const std::string & a1, bool const & a2) -> bool { return o.read_bool(a0, a1, a2); }, "", pybind11::arg("section"), pybind11::arg("name"), pybind11::arg("defaultValue"));
		cl.def("read_bool", (bool (mrpt::config::CConfigFileBase::*)(const std::string &, const std::string &, bool, bool) const) &mrpt::config::CConfigFileBase::read_bool, "C++: mrpt::config::CConfigFileBase::read_bool(const std::string &, const std::string &, bool, bool) const --> bool", pybind11::arg("section"), pybind11::arg("name"), pybind11::arg("defaultValue"), pybind11::arg("failIfNotFound"));
		cl.def("read_int", [](mrpt::config::CConfigFileBase const &o, const std::string & a0, const std::string & a1, int const & a2) -> int { return o.read_int(a0, a1, a2); }, "", pybind11::arg("section"), pybind11::arg("name"), pybind11::arg("defaultValue"));
		cl.def("read_int", (int (mrpt::config::CConfigFileBase::*)(const std::string &, const std::string &, int, bool) const) &mrpt::config::CConfigFileBase::read_int, "C++: mrpt::config::CConfigFileBase::read_int(const std::string &, const std::string &, int, bool) const --> int", pybind11::arg("section"), pybind11::arg("name"), pybind11::arg("defaultValue"), pybind11::arg("failIfNotFound"));
		cl.def("read_uint64_t", [](mrpt::config::CConfigFileBase const &o, const std::string & a0, const std::string & a1, uint64_t const & a2) -> uint64_t { return o.read_uint64_t(a0, a1, a2); }, "", pybind11::arg("section"), pybind11::arg("name"), pybind11::arg("defaultValue"));
		cl.def("read_uint64_t", (uint64_t (mrpt::config::CConfigFileBase::*)(const std::string &, const std::string &, uint64_t, bool) const) &mrpt::config::CConfigFileBase::read_uint64_t, "C++: mrpt::config::CConfigFileBase::read_uint64_t(const std::string &, const std::string &, uint64_t, bool) const --> uint64_t", pybind11::arg("section"), pybind11::arg("name"), pybind11::arg("defaultValue"), pybind11::arg("failIfNotFound"));
		cl.def("read_string", [](mrpt::config::CConfigFileBase const &o, const std::string & a0, const std::string & a1, const std::string & a2) -> std::string { return o.read_string(a0, a1, a2); }, "", pybind11::arg("section"), pybind11::arg("name"), pybind11::arg("defaultValue"));
		cl.def("read_string", (std::string (mrpt::config::CConfigFileBase::*)(const std::string &, const std::string &, const std::string &, bool) const) &mrpt::config::CConfigFileBase::read_string, "C++: mrpt::config::CConfigFileBase::read_string(const std::string &, const std::string &, const std::string &, bool) const --> std::string", pybind11::arg("section"), pybind11::arg("name"), pybind11::arg("defaultValue"), pybind11::arg("failIfNotFound"));
		cl.def("read_string_first_word", [](mrpt::config::CConfigFileBase const &o, const std::string & a0, const std::string & a1, const std::string & a2) -> std::string { return o.read_string_first_word(a0, a1, a2); }, "", pybind11::arg("section"), pybind11::arg("name"), pybind11::arg("defaultValue"));
		cl.def("read_string_first_word", (std::string (mrpt::config::CConfigFileBase::*)(const std::string &, const std::string &, const std::string &, bool) const) &mrpt::config::CConfigFileBase::read_string_first_word, "Reads a configuration parameter of type \"string\", and keeps only the\n first word (this can be used to eliminate possible comments at the end of\n the line) \n\nC++: mrpt::config::CConfigFileBase::read_string_first_word(const std::string &, const std::string &, const std::string &, bool) const --> std::string", pybind11::arg("section"), pybind11::arg("name"), pybind11::arg("defaultValue"), pybind11::arg("failIfNotFound"));
		cl.def("assign", (class mrpt::config::CConfigFileBase & (mrpt::config::CConfigFileBase::*)(const class mrpt::config::CConfigFileBase &)) &mrpt::config::CConfigFileBase::operator=, "C++: mrpt::config::CConfigFileBase::operator=(const class mrpt::config::CConfigFileBase &) --> class mrpt::config::CConfigFileBase &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::config::CConfigFileMemory file:mrpt/config/CConfigFileMemory.h line:37
		pybind11::class_<mrpt::config::CConfigFileMemory, std::shared_ptr<mrpt::config::CConfigFileMemory>, PyCallBack_mrpt_config_CConfigFileMemory, mrpt::config::CConfigFileBase> cl(M("mrpt::config"), "CConfigFileMemory", "This class implements a config file-like interface over a memory-stored\n string list.\n\n Use base class `CConfigFileBase`'s methods\n `read_{int,float,double,string,...}()` and `write()` to actually read and\n write values.\n\n It can also parse a YAML text block and expose its fields (up to the first\n level of hierarchy, as allowed by INI-like files). This can be used to port\n MRPT classes relying on INI files to using YAML files transparently.\n This feature required building MRPT with yaml-cpp, and is provided by\n CConfigFileMemory::setContentFromYAML().\n\n See: \n\n \n\n \n YAML support was introduced in MRPT 1.9.9");
		cl.def( pybind11::init( [](){ return new mrpt::config::CConfigFileMemory(); }, [](){ return new PyCallBack_mrpt_config_CConfigFileMemory(); } ) );
		cl.def( pybind11::init<const class std::vector<std::string > &>(), pybind11::arg("stringList") );

		cl.def( pybind11::init<const std::string &>(), pybind11::arg("str") );

		cl.def( pybind11::init( [](PyCallBack_mrpt_config_CConfigFileMemory const &o){ return new PyCallBack_mrpt_config_CConfigFileMemory(o); } ) );
		cl.def( pybind11::init( [](mrpt::config::CConfigFileMemory const &o){ return new mrpt::config::CConfigFileMemory(o); } ) );
		cl.def("setContent", (void (mrpt::config::CConfigFileMemory::*)(const class std::vector<std::string > &)) &mrpt::config::CConfigFileMemory::setContent, "Changes the contents of the virtual \"config file\" \n\nC++: mrpt::config::CConfigFileMemory::setContent(const class std::vector<std::string > &) --> void", pybind11::arg("stringList"));
		cl.def("setContent", (void (mrpt::config::CConfigFileMemory::*)(const std::string &)) &mrpt::config::CConfigFileMemory::setContent, "Changes the contents of the virtual \"config file\" \n\nC++: mrpt::config::CConfigFileMemory::setContent(const std::string &) --> void", pybind11::arg("str"));
		cl.def("getContent", (void (mrpt::config::CConfigFileMemory::*)(std::string &) const) &mrpt::config::CConfigFileMemory::getContent, "Return the current contents of the virtual \"config file\" \n\nC++: mrpt::config::CConfigFileMemory::getContent(std::string &) const --> void", pybind11::arg("str"));
		cl.def("getContent", (std::string (mrpt::config::CConfigFileMemory::*)() const) &mrpt::config::CConfigFileMemory::getContent, "C++: mrpt::config::CConfigFileMemory::getContent() const --> std::string");
		cl.def("clear", (void (mrpt::config::CConfigFileMemory::*)()) &mrpt::config::CConfigFileMemory::clear, "Empties the virtual \"config file\" \n\nC++: mrpt::config::CConfigFileMemory::clear() --> void");
		cl.def("getAllSections", (void (mrpt::config::CConfigFileMemory::*)(class std::vector<std::string > &) const) &mrpt::config::CConfigFileMemory::getAllSections, "Returns a list with all the section names \n\nC++: mrpt::config::CConfigFileMemory::getAllSections(class std::vector<std::string > &) const --> void", pybind11::arg("sections"));
		cl.def("getAllKeys", (void (mrpt::config::CConfigFileMemory::*)(const std::string &, class std::vector<std::string > &) const) &mrpt::config::CConfigFileMemory::getAllKeys, "Returs a list with all the keys into a section \n\nC++: mrpt::config::CConfigFileMemory::getAllKeys(const std::string &, class std::vector<std::string > &) const --> void", pybind11::arg("section"), pybind11::arg("keys"));
		cl.def("assign", (class mrpt::config::CConfigFileMemory & (mrpt::config::CConfigFileMemory::*)(const class mrpt::config::CConfigFileMemory &)) &mrpt::config::CConfigFileMemory::operator=, "C++: mrpt::config::CConfigFileMemory::operator=(const class mrpt::config::CConfigFileMemory &) --> class mrpt::config::CConfigFileMemory &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
