#include <iterator>
#include <memory>
#include <mrpt/bayes/CKalmanFilterCapable.h>
#include <mrpt/config/CConfigFile.h>
#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/config/CConfigFilePrefixer.h>
#include <mrpt/config/config_parser.h>
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

// mrpt::config::CConfigFile file:mrpt/config/CConfigFile.h line:31
struct PyCallBack_mrpt_config_CConfigFile : public mrpt::config::CConfigFile {
	using mrpt::config::CConfigFile::CConfigFile;

	void writeString(const std::string & a0, const std::string & a1, const std::string & a2) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::config::CConfigFile *>(this), "writeString");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CConfigFile::writeString(a0, a1, a2);
	}
	std::string readString(const std::string & a0, const std::string & a1, const std::string & a2, bool a3) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::config::CConfigFile *>(this), "readString");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2, a3);
			if (pybind11::detail::cast_is_temporary_value_reference<std::string>::value) {
				static pybind11::detail::override_caster_t<std::string> caster;
				return pybind11::detail::cast_ref<std::string>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<std::string>(std::move(o));
		}
		return CConfigFile::readString(a0, a1, a2, a3);
	}
	void getAllSections(class std::vector<std::string > & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::config::CConfigFile *>(this), "getAllSections");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CConfigFile::getAllSections(a0);
	}
	void clear() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::config::CConfigFile *>(this), "clear");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CConfigFile::clear();
	}
	void getAllKeys(const std::string & a0, class std::vector<std::string > & a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::config::CConfigFile *>(this), "getAllKeys");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CConfigFile::getAllKeys(a0, a1);
	}
};

// mrpt::config::CConfigFilePrefixer file:mrpt/config/CConfigFilePrefixer.h line:37
struct PyCallBack_mrpt_config_CConfigFilePrefixer : public mrpt::config::CConfigFilePrefixer {
	using mrpt::config::CConfigFilePrefixer::CConfigFilePrefixer;

	void writeString(const std::string & a0, const std::string & a1, const std::string & a2) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::config::CConfigFilePrefixer *>(this), "writeString");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CConfigFilePrefixer::writeString(a0, a1, a2);
	}
	std::string readString(const std::string & a0, const std::string & a1, const std::string & a2, bool a3) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::config::CConfigFilePrefixer *>(this), "readString");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2, a3);
			if (pybind11::detail::cast_is_temporary_value_reference<std::string>::value) {
				static pybind11::detail::override_caster_t<std::string> caster;
				return pybind11::detail::cast_ref<std::string>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<std::string>(std::move(o));
		}
		return CConfigFilePrefixer::readString(a0, a1, a2, a3);
	}
	void getAllSections(class std::vector<std::string > & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::config::CConfigFilePrefixer *>(this), "getAllSections");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CConfigFilePrefixer::getAllSections(a0);
	}
	void getAllKeys(const std::string & a0, class std::vector<std::string > & a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::config::CConfigFilePrefixer *>(this), "getAllKeys");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CConfigFilePrefixer::getAllKeys(a0, a1);
	}
	void clear() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::config::CConfigFilePrefixer *>(this), "clear");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CConfigFilePrefixer::clear();
	}
};

void bind_mrpt_config_CConfigFile(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::config::CConfigFile file:mrpt/config/CConfigFile.h line:31
		pybind11::class_<mrpt::config::CConfigFile, std::shared_ptr<mrpt::config::CConfigFile>, PyCallBack_mrpt_config_CConfigFile, mrpt::config::CConfigFileBase> cl(M("mrpt::config"), "CConfigFile", "This class allows loading and storing values and vectors of different types\n from \".ini\" files easily.\n  The contents of the file will be modified by \"write\" operations in memory,\n and will be saved back\n   to the file at the destructor, and only if at least one write operation\n has been applied.\n\n Use base class `CConfigFileBase`'s methods\n `read_{int,float,double,string,...}()` and `write()` to actually read and\n write values.\n\n See: \n\n \n\n ");
		cl.def( pybind11::init<const std::string &>(), pybind11::arg("fileName") );

		cl.def( pybind11::init( [](){ return new mrpt::config::CConfigFile(); }, [](){ return new PyCallBack_mrpt_config_CConfigFile(); } ) );
		cl.def( pybind11::init( [](PyCallBack_mrpt_config_CConfigFile const &o){ return new PyCallBack_mrpt_config_CConfigFile(o); } ) );
		cl.def( pybind11::init( [](mrpt::config::CConfigFile const &o){ return new mrpt::config::CConfigFile(o); } ) );
		cl.def("setFileName", (void (mrpt::config::CConfigFile::*)(const std::string &)) &mrpt::config::CConfigFile::setFileName, "Associate this object with the given file, reading its contents right\n now. Upon destruction, the updated contents will be written to that file.\n\nC++: mrpt::config::CConfigFile::setFileName(const std::string &) --> void", pybind11::arg("fil_path"));
		cl.def("writeNow", (void (mrpt::config::CConfigFile::*)()) &mrpt::config::CConfigFile::writeNow, "Dumps the changes to the physical configuration file now, not waiting\n until destruction.\n \n\n std::runtime_error Upon error writing.\n\nC++: mrpt::config::CConfigFile::writeNow() --> void");
		cl.def("discardSavingChanges", (void (mrpt::config::CConfigFile::*)()) &mrpt::config::CConfigFile::discardSavingChanges, "Discard saving (current) changes to physical file upon destruction \n\nC++: mrpt::config::CConfigFile::discardSavingChanges() --> void");
		cl.def("getAssociatedFile", (std::string (mrpt::config::CConfigFile::*)() const) &mrpt::config::CConfigFile::getAssociatedFile, "Returns the file currently open by this object. \n\nC++: mrpt::config::CConfigFile::getAssociatedFile() const --> std::string");
		cl.def("getAllSections", (void (mrpt::config::CConfigFile::*)(class std::vector<std::string > &) const) &mrpt::config::CConfigFile::getAllSections, "Returns a list with all the section names. \n\nC++: mrpt::config::CConfigFile::getAllSections(class std::vector<std::string > &) const --> void", pybind11::arg("sections"));
		cl.def("clear", (void (mrpt::config::CConfigFile::*)()) &mrpt::config::CConfigFile::clear, "Empties the \"config file\" \n\nC++: mrpt::config::CConfigFile::clear() --> void");
		cl.def("getAllKeys", (void (mrpt::config::CConfigFile::*)(const std::string &, class std::vector<std::string > &) const) &mrpt::config::CConfigFile::getAllKeys, "Returs a list with all the keys into a section. \n\nC++: mrpt::config::CConfigFile::getAllKeys(const std::string &, class std::vector<std::string > &) const --> void", pybind11::arg("section"), pybind11::arg("keys"));
		cl.def("assign", (class mrpt::config::CConfigFile & (mrpt::config::CConfigFile::*)(const class mrpt::config::CConfigFile &)) &mrpt::config::CConfigFile::operator=, "C++: mrpt::config::CConfigFile::operator=(const class mrpt::config::CConfigFile &) --> class mrpt::config::CConfigFile &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::config::CConfigFilePrefixer file:mrpt/config/CConfigFilePrefixer.h line:37
		pybind11::class_<mrpt::config::CConfigFilePrefixer, std::shared_ptr<mrpt::config::CConfigFilePrefixer>, PyCallBack_mrpt_config_CConfigFilePrefixer, mrpt::config::CConfigFileBase> cl(M("mrpt::config"), "CConfigFilePrefixer", "A wrapper for other CConfigFileBase-based objects that prefixes a given\n token to every key and/or section.\n  If, for example, your code expect:\n   \n\n\n\n\n\n  Using this class with key entries prefix `s1_` will enable the same\n existing code to transparently parse this file content:\n\n   \n\n\n\n\n See: \n \n\n CConfigFileBase\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::config::CConfigFilePrefixer(); }, [](){ return new PyCallBack_mrpt_config_CConfigFilePrefixer(); } ) );
		cl.def( pybind11::init<const class mrpt::config::CConfigFileBase &, const std::string &, const std::string &>(), pybind11::arg("o"), pybind11::arg("prefix_sections"), pybind11::arg("prefix_keys") );

		cl.def( pybind11::init( [](PyCallBack_mrpt_config_CConfigFilePrefixer const &o){ return new PyCallBack_mrpt_config_CConfigFilePrefixer(o); } ) );
		cl.def( pybind11::init( [](mrpt::config::CConfigFilePrefixer const &o){ return new mrpt::config::CConfigFilePrefixer(o); } ) );
		cl.def("bind", (void (mrpt::config::CConfigFilePrefixer::*)(const class mrpt::config::CConfigFileBase &)) &mrpt::config::CConfigFilePrefixer::bind, "Make this object to wrap the given existing CConfigFileBase object. Can\n be changed at any moment after construction \n\nC++: mrpt::config::CConfigFilePrefixer::bind(const class mrpt::config::CConfigFileBase &) --> void", pybind11::arg("o"));
		cl.def("setPrefixes", (void (mrpt::config::CConfigFilePrefixer::*)(const std::string &, const std::string &)) &mrpt::config::CConfigFilePrefixer::setPrefixes, "Change the prefix for sections and keys. Can be called at any moment. \n\nC++: mrpt::config::CConfigFilePrefixer::setPrefixes(const std::string &, const std::string &) --> void", pybind11::arg("prefix_sections"), pybind11::arg("prefix_keys"));
		cl.def("getSectionPrefix", (std::string (mrpt::config::CConfigFilePrefixer::*)() const) &mrpt::config::CConfigFilePrefixer::getSectionPrefix, "C++: mrpt::config::CConfigFilePrefixer::getSectionPrefix() const --> std::string");
		cl.def("getKeyPrefix", (std::string (mrpt::config::CConfigFilePrefixer::*)() const) &mrpt::config::CConfigFilePrefixer::getKeyPrefix, "C++: mrpt::config::CConfigFilePrefixer::getKeyPrefix() const --> std::string");
		cl.def("getBoundConfigFileBase", (class mrpt::config::CConfigFileBase * (mrpt::config::CConfigFilePrefixer::*)() const) &mrpt::config::CConfigFilePrefixer::getBoundConfigFileBase, "Returns the currently-bounded config source, or nullptr if none. \n\nC++: mrpt::config::CConfigFilePrefixer::getBoundConfigFileBase() const --> class mrpt::config::CConfigFileBase *", pybind11::return_value_policy::automatic);
		cl.def("getAllSections", (void (mrpt::config::CConfigFilePrefixer::*)(class std::vector<std::string > &) const) &mrpt::config::CConfigFilePrefixer::getAllSections, "C++: mrpt::config::CConfigFilePrefixer::getAllSections(class std::vector<std::string > &) const --> void", pybind11::arg("sections"));
		cl.def("getAllKeys", (void (mrpt::config::CConfigFilePrefixer::*)(const std::string &, class std::vector<std::string > &) const) &mrpt::config::CConfigFilePrefixer::getAllKeys, "C++: mrpt::config::CConfigFilePrefixer::getAllKeys(const std::string &, class std::vector<std::string > &) const --> void", pybind11::arg("section"), pybind11::arg("keys"));
		cl.def("clear", (void (mrpt::config::CConfigFilePrefixer::*)()) &mrpt::config::CConfigFilePrefixer::clear, "C++: mrpt::config::CConfigFilePrefixer::clear() --> void");
		cl.def("assign", (class mrpt::config::CConfigFilePrefixer & (mrpt::config::CConfigFilePrefixer::*)(const class mrpt::config::CConfigFilePrefixer &)) &mrpt::config::CConfigFilePrefixer::operator=, "C++: mrpt::config::CConfigFilePrefixer::operator=(const class mrpt::config::CConfigFilePrefixer &) --> class mrpt::config::CConfigFilePrefixer &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	// mrpt::config::config_parser(const std::string &) file:mrpt/config/config_parser.h line:20
	M("mrpt::config").def("config_parser", (std::string (*)(const std::string &)) &mrpt::config::config_parser, "Parses a document and replaces all formulas, variables, etc. as defined\n in \n\n \n\n \n\nC++: mrpt::config::config_parser(const std::string &) --> std::string", pybind11::arg("input"));

}
