#include <iterator>
#include <memory>
#include <mrpt/img/TColor.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CQuaternion.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/opengl/PLY_import_export.h>
#include <optional>
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

// mrpt::opengl::PLY_Importer file:mrpt/opengl/PLY_import_export.h line:25
struct PyCallBack_mrpt_opengl_PLY_Importer : public mrpt::opengl::PLY_Importer {
	using mrpt::opengl::PLY_Importer::PLY_Importer;

	void PLY_import_set_vertex_count(size_t a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::PLY_Importer *>(this), "PLY_import_set_vertex_count");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"PLY_Importer::PLY_import_set_vertex_count\"");
	}
	void PLY_import_set_face_count(size_t a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::PLY_Importer *>(this), "PLY_import_set_face_count");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"PLY_Importer::PLY_import_set_face_count\"");
	}
	void PLY_import_set_vertex(size_t a0, const struct mrpt::math::TPoint3D_<float> & a1, const struct mrpt::img::TColorf * a2) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::PLY_Importer *>(this), "PLY_import_set_vertex");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"PLY_Importer::PLY_import_set_vertex\"");
	}
	void PLY_import_set_vertex_timestamp(size_t a0, const double a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::PLY_Importer *>(this), "PLY_import_set_vertex_timestamp");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"PLY_Importer::PLY_import_set_vertex_timestamp\"");
	}
};

// mrpt::opengl::PLY_Exporter file:mrpt/opengl/PLY_import_export.h line:84
struct PyCallBack_mrpt_opengl_PLY_Exporter : public mrpt::opengl::PLY_Exporter {
	using mrpt::opengl::PLY_Exporter::PLY_Exporter;

	size_t PLY_export_get_vertex_count() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::PLY_Exporter *>(this), "PLY_export_get_vertex_count");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<size_t>::value) {
				static pybind11::detail::override_caster_t<size_t> caster;
				return pybind11::detail::cast_ref<size_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<size_t>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"PLY_Exporter::PLY_export_get_vertex_count\"");
	}
	size_t PLY_export_get_face_count() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::PLY_Exporter *>(this), "PLY_export_get_face_count");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<size_t>::value) {
				static pybind11::detail::override_caster_t<size_t> caster;
				return pybind11::detail::cast_ref<size_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<size_t>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"PLY_Exporter::PLY_export_get_face_count\"");
	}
	void PLY_export_get_vertex(size_t a0, struct mrpt::math::TPoint3D_<float> & a1, bool & a2, struct mrpt::img::TColorf & a3) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::PLY_Exporter *>(this), "PLY_export_get_vertex");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2, a3);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"PLY_Exporter::PLY_export_get_vertex\"");
	}
};

void bind_mrpt_opengl_PLY_import_export(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::opengl::PLY_Importer file:mrpt/opengl/PLY_import_export.h line:25
		pybind11::class_<mrpt::opengl::PLY_Importer, std::shared_ptr<mrpt::opengl::PLY_Importer>, PyCallBack_mrpt_opengl_PLY_Importer> cl(M("mrpt::opengl"), "PLY_Importer", "A virtual base class that implements the capability of importing 3D point\n clouds and faces from a file in the Stanford PLY format.\n \n\n https://www.mrpt.org/Support_for_the_Stanford_3D_models_file_format_PLY\n \n\n PLY_Exporter\n \n\n\n ");
		cl.def(pybind11::init<PyCallBack_mrpt_opengl_PLY_Importer const &>());
		cl.def( pybind11::init( [](){ return new PyCallBack_mrpt_opengl_PLY_Importer(); } ) );
		cl.def("loadFromPlyFile", [](mrpt::opengl::PLY_Importer &o, const std::string & a0) -> bool { return o.loadFromPlyFile(a0); }, "", pybind11::arg("filename"));
		cl.def("loadFromPlyFile", [](mrpt::opengl::PLY_Importer &o, const std::string & a0, class std::vector<std::string > * a1) -> bool { return o.loadFromPlyFile(a0, a1); }, "", pybind11::arg("filename"), pybind11::arg("file_comments"));
		cl.def("loadFromPlyFile", (bool (mrpt::opengl::PLY_Importer::*)(const std::string &, class std::vector<std::string > *, class std::vector<std::string > *)) &mrpt::opengl::PLY_Importer::loadFromPlyFile, "Loads from a PLY file.\n \n\n The filename to open. It can be either in binary or\n text format.\n \n\n If provided (!=nullptr) the list of comment\n strings stored in the file will be returned.\n \n\n If provided (!=nullptr) the list of \"object\n info\" strings stored in the file will be returned.\n \n\n false on any error in the file format or reading it. To obtain\n more details on the error you can call getLoadPLYErrorString()\n\nC++: mrpt::opengl::PLY_Importer::loadFromPlyFile(const std::string &, class std::vector<std::string > *, class std::vector<std::string > *) --> bool", pybind11::arg("filename"), pybind11::arg("file_comments"), pybind11::arg("file_obj_info"));
		cl.def("getLoadPLYErrorString", (std::string (mrpt::opengl::PLY_Importer::*)() const) &mrpt::opengl::PLY_Importer::getLoadPLYErrorString, "Return a description of the error if loadFromPlyFile() returned false,\n or an empty string if the file was loaded without problems. \n\nC++: mrpt::opengl::PLY_Importer::getLoadPLYErrorString() const --> std::string");
		cl.def("assign", (class mrpt::opengl::PLY_Importer & (mrpt::opengl::PLY_Importer::*)(const class mrpt::opengl::PLY_Importer &)) &mrpt::opengl::PLY_Importer::operator=, "C++: mrpt::opengl::PLY_Importer::operator=(const class mrpt::opengl::PLY_Importer &) --> class mrpt::opengl::PLY_Importer &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::opengl::PLY_Exporter file:mrpt/opengl/PLY_import_export.h line:84
		pybind11::class_<mrpt::opengl::PLY_Exporter, std::shared_ptr<mrpt::opengl::PLY_Exporter>, PyCallBack_mrpt_opengl_PLY_Exporter> cl(M("mrpt::opengl"), "PLY_Exporter", "A virtual base class that implements the capability of exporting 3D point\n clouds and faces to a file in the Stanford PLY format.\n \n\n https://www.mrpt.org/Support_for_the_Stanford_3D_models_file_format_PLY\n \n\n PLY_Importer\n \n\n\n ");
		cl.def(pybind11::init<PyCallBack_mrpt_opengl_PLY_Exporter const &>());
		cl.def( pybind11::init( [](){ return new PyCallBack_mrpt_opengl_PLY_Exporter(); } ) );
		cl.def("saveToPlyFile", [](mrpt::opengl::PLY_Exporter const &o, const std::string & a0) -> bool { return o.saveToPlyFile(a0); }, "", pybind11::arg("filename"));
		cl.def("saveToPlyFile", [](mrpt::opengl::PLY_Exporter const &o, const std::string & a0, bool const & a1) -> bool { return o.saveToPlyFile(a0, a1); }, "", pybind11::arg("filename"), pybind11::arg("save_in_binary"));
		cl.def("saveToPlyFile", [](mrpt::opengl::PLY_Exporter const &o, const std::string & a0, bool const & a1, const class std::vector<std::string > & a2) -> bool { return o.saveToPlyFile(a0, a1, a2); }, "", pybind11::arg("filename"), pybind11::arg("save_in_binary"), pybind11::arg("file_comments"));
		cl.def("saveToPlyFile", (bool (mrpt::opengl::PLY_Exporter::*)(const std::string &, bool, const class std::vector<std::string > &, const class std::vector<std::string > &) const) &mrpt::opengl::PLY_Exporter::saveToPlyFile, "Saves to a PLY file.\n \n\n The filename to be saved.\n \n\n If provided (!=nullptr) the list of comment\n strings stored in the file will be returned.\n \n\n If provided (!=nullptr) the list of \"object\n info\" strings stored in the file will be returned.\n \n\n false on any error writing the file. To obtain more details on\n the error you can call getSavePLYErrorString()\n\nC++: mrpt::opengl::PLY_Exporter::saveToPlyFile(const std::string &, bool, const class std::vector<std::string > &, const class std::vector<std::string > &) const --> bool", pybind11::arg("filename"), pybind11::arg("save_in_binary"), pybind11::arg("file_comments"), pybind11::arg("file_obj_info"));
		cl.def("getSavePLYErrorString", (std::string (mrpt::opengl::PLY_Exporter::*)() const) &mrpt::opengl::PLY_Exporter::getSavePLYErrorString, "Return a description of the error if loadFromPlyFile() returned false,\n or an empty string if the file was loaded without problems. \n\nC++: mrpt::opengl::PLY_Exporter::getSavePLYErrorString() const --> std::string");
		cl.def("assign", (class mrpt::opengl::PLY_Exporter & (mrpt::opengl::PLY_Exporter::*)(const class mrpt::opengl::PLY_Exporter &)) &mrpt::opengl::PLY_Exporter::operator=, "C++: mrpt::opengl::PLY_Exporter::operator=(const class mrpt::opengl::PLY_Exporter &) --> class mrpt::opengl::PLY_Exporter &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
