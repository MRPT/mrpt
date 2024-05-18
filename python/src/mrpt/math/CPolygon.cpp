#include <iterator>
#include <memory>
#include <mrpt/math/CPolygon.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/CSerializable.h>
#include <mrpt/typemeta/static_string.h>
#include <sstream> // __str__
#include <string>
#include <variant>
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

// mrpt::math::CPolygon file:mrpt/math/CPolygon.h line:19
struct PyCallBack_mrpt_math_CPolygon : public mrpt::math::CPolygon {
	using mrpt::math::CPolygon::CPolygon;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::math::CPolygon *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CPolygon::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::math::CPolygon *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CPolygon::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::math::CPolygon *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CPolygon::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::math::CPolygon *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPolygon::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::math::CPolygon *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPolygon::serializeFrom(a0, a1);
	}
};

void bind_mrpt_math_CPolygon(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::math::CPolygon file:mrpt/math/CPolygon.h line:19
		pybind11::class_<mrpt::math::CPolygon, std::shared_ptr<mrpt::math::CPolygon>, PyCallBack_mrpt_math_CPolygon, mrpt::serialization::CSerializable, mrpt::math::TPolygon2D> cl(M("mrpt::math"), "CPolygon", "A wrapper of a TPolygon2D class, implementing CSerializable.\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::math::CPolygon(); }, [](){ return new PyCallBack_mrpt_math_CPolygon(); } ) );
		cl.def( pybind11::init( [](PyCallBack_mrpt_math_CPolygon const &o){ return new PyCallBack_mrpt_math_CPolygon(o); } ) );
		cl.def( pybind11::init( [](mrpt::math::CPolygon const &o){ return new mrpt::math::CPolygon(o); } ) );
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::math::CPolygon::GetRuntimeClassIdStatic, "C++: mrpt::math::CPolygon::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::math::CPolygon::*)() const) &mrpt::math::CPolygon::GetRuntimeClass, "C++: mrpt::math::CPolygon::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::math::CPolygon::*)() const) &mrpt::math::CPolygon::clone, "C++: mrpt::math::CPolygon::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::math::CPolygon::CreateObject, "C++: mrpt::math::CPolygon::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("AddVertex", (void (mrpt::math::CPolygon::*)(double, double)) &mrpt::math::CPolygon::AddVertex, "Add a new vertex to polygon \n\nC++: mrpt::math::CPolygon::AddVertex(double, double) --> void", pybind11::arg("x"), pybind11::arg("y"));
		cl.def("GetVertex_x", (double (mrpt::math::CPolygon::*)(size_t) const) &mrpt::math::CPolygon::GetVertex_x, "Methods for accessing the vertices \n verticesCount \n\nC++: mrpt::math::CPolygon::GetVertex_x(size_t) const --> double", pybind11::arg("i"));
		cl.def("GetVertex_y", (double (mrpt::math::CPolygon::*)(size_t) const) &mrpt::math::CPolygon::GetVertex_y, "C++: mrpt::math::CPolygon::GetVertex_y(size_t) const --> double", pybind11::arg("i"));
		cl.def("verticesCount", (size_t (mrpt::math::CPolygon::*)() const) &mrpt::math::CPolygon::verticesCount, "Returns the vertices count in the polygon: \n\nC++: mrpt::math::CPolygon::verticesCount() const --> size_t");
		cl.def("setAllVertices", (void (mrpt::math::CPolygon::*)(size_t, const double *, const double *)) &mrpt::math::CPolygon::setAllVertices, "Set all vertices at once. Please use the std::vector version whenever\n possible unless efficiency is really an issue \n\nC++: mrpt::math::CPolygon::setAllVertices(size_t, const double *, const double *) --> void", pybind11::arg("nVertices"), pybind11::arg("xs"), pybind11::arg("ys"));
		cl.def("setAllVertices", (void (mrpt::math::CPolygon::*)(size_t, const float *, const float *)) &mrpt::math::CPolygon::setAllVertices, "Set all vertices at once. Please use the std::vector version whenever\n possible unless efficiency is really an issue \n\nC++: mrpt::math::CPolygon::setAllVertices(size_t, const float *, const float *) --> void", pybind11::arg("nVertices"), pybind11::arg("xs"), pybind11::arg("ys"));
		cl.def("Clear", (void (mrpt::math::CPolygon::*)()) &mrpt::math::CPolygon::Clear, "Clear the polygon, erasing all vertices \n\nC++: mrpt::math::CPolygon::Clear() --> void");
		cl.def("PointIntoPolygon", (bool (mrpt::math::CPolygon::*)(double, double) const) &mrpt::math::CPolygon::PointIntoPolygon, "	Check if a point is inside the polygon \n\nC++: mrpt::math::CPolygon::PointIntoPolygon(double, double) const --> bool", pybind11::arg("x"), pybind11::arg("y"));
		cl.def("assign", (class mrpt::math::CPolygon & (mrpt::math::CPolygon::*)(const class mrpt::math::CPolygon &)) &mrpt::math::CPolygon::operator=, "C++: mrpt::math::CPolygon::operator=(const class mrpt::math::CPolygon &) --> class mrpt::math::CPolygon &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
