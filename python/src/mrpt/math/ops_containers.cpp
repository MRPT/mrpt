#include <ios>
#include <iterator>
#include <locale>
#include <memory>
#include <mrpt/containers/yaml.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CSplineInterpolator1D.h>
#include <mrpt/math/CVectorDynamic.h>
#include <mrpt/math/MatrixVectorBase.h>
#include <mrpt/math/TLine2D.h>
#include <mrpt/math/TLine3D.h>
#include <mrpt/math/TObject2D.h>
#include <mrpt/math/TObject3D.h>
#include <mrpt/math/TPlane.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPolygon2D.h>
#include <mrpt/math/TPolygon3D.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/math/TSegment2D.h>
#include <mrpt/math/TSegment3D.h>
#include <mrpt/math/math_frwds.h>
#include <mrpt/math/matrix_size_t.h>
#include <mrpt/math/ops_containers.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/CSerializable.h>
#include <mrpt/typemeta/static_string.h>
#include <ostream>
#include <sstream> // __str__
#include <streambuf>
#include <string>
#include <tuple>
#include <type_traits>
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

// mrpt::math::CSplineInterpolator1D file:mrpt/math/CSplineInterpolator1D.h line:25
struct PyCallBack_mrpt_math_CSplineInterpolator1D : public mrpt::math::CSplineInterpolator1D {
	using mrpt::math::CSplineInterpolator1D::CSplineInterpolator1D;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::math::CSplineInterpolator1D *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CSplineInterpolator1D::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::math::CSplineInterpolator1D *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CSplineInterpolator1D::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::math::CSplineInterpolator1D *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CSplineInterpolator1D::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::math::CSplineInterpolator1D *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CSplineInterpolator1D::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::math::CSplineInterpolator1D *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CSplineInterpolator1D::serializeFrom(a0, a1);
	}
};

void bind_mrpt_math_ops_containers(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	// mrpt::math::maximum(const class mrpt::math::CVectorDynamic<double> &) file:mrpt/math/ops_containers.h line:139
	M("mrpt::math").def("maximum", (double (*)(const class mrpt::math::CVectorDynamic<double> &)) &mrpt::math::maximum<mrpt::math::CVectorDynamic<double>,1>, "C++: mrpt::math::maximum(const class mrpt::math::CVectorDynamic<double> &) --> double", pybind11::arg("v"));

	// mrpt::math::minimum(const class mrpt::math::CVectorDynamic<double> &) file:mrpt/math/ops_containers.h line:144
	M("mrpt::math").def("minimum", (double (*)(const class mrpt::math::CVectorDynamic<double> &)) &mrpt::math::minimum<mrpt::math::CVectorDynamic<double>,1>, "C++: mrpt::math::minimum(const class mrpt::math::CVectorDynamic<double> &) --> double", pybind11::arg("v"));

	// mrpt::math::mean(const class mrpt::math::CVectorDynamic<double> &) file:mrpt/math/ops_containers.h line:244
	M("mrpt::math").def("mean", (double (*)(const class mrpt::math::CVectorDynamic<double> &)) &mrpt::math::mean<mrpt::math::CVectorDynamic<double>>, "C++: mrpt::math::mean(const class mrpt::math::CVectorDynamic<double> &) --> double", pybind11::arg("v"));

	{ // mrpt::math::CSplineInterpolator1D file:mrpt/math/CSplineInterpolator1D.h line:25
		pybind11::class_<mrpt::math::CSplineInterpolator1D, std::shared_ptr<mrpt::math::CSplineInterpolator1D>, PyCallBack_mrpt_math_CSplineInterpolator1D, mrpt::serialization::CSerializable> cl(M("mrpt::math"), "CSplineInterpolator1D", "A (persistent) sequence of (x,y) coordinates, allowing queries of\n intermediate points through spline interpolation, where possible.\n  This class internally relies on mrpt::math::spline. Optionally the y\n coordinate can be set as wrapped in ]-pi,pi].\n  For querying interpolated points, see\n \\ sa mrpt::math::spline, mrpt::poses::CPose3DInterpolator\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::math::CSplineInterpolator1D(); }, [](){ return new PyCallBack_mrpt_math_CSplineInterpolator1D(); } ), "doc");
		cl.def( pybind11::init<bool>(), pybind11::arg("wrap2pi") );

		cl.def( pybind11::init( [](PyCallBack_mrpt_math_CSplineInterpolator1D const &o){ return new PyCallBack_mrpt_math_CSplineInterpolator1D(o); } ) );
		cl.def( pybind11::init( [](mrpt::math::CSplineInterpolator1D const &o){ return new mrpt::math::CSplineInterpolator1D(o); } ) );
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::math::CSplineInterpolator1D::GetRuntimeClassIdStatic, "C++: mrpt::math::CSplineInterpolator1D::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::math::CSplineInterpolator1D::*)() const) &mrpt::math::CSplineInterpolator1D::GetRuntimeClass, "C++: mrpt::math::CSplineInterpolator1D::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::math::CSplineInterpolator1D::*)() const) &mrpt::math::CSplineInterpolator1D::clone, "C++: mrpt::math::CSplineInterpolator1D::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::math::CSplineInterpolator1D::CreateObject, "C++: mrpt::math::CSplineInterpolator1D::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("setWrap2pi", (void (mrpt::math::CSplineInterpolator1D::*)(bool)) &mrpt::math::CSplineInterpolator1D::setWrap2pi, "If set to true, the interpolated data will be wrapped to ]-pi,pi] \n\nC++: mrpt::math::CSplineInterpolator1D::setWrap2pi(bool) --> void", pybind11::arg("wrap"));
		cl.def("getWrap2pi", (bool (mrpt::math::CSplineInterpolator1D::*)()) &mrpt::math::CSplineInterpolator1D::getWrap2pi, "Return the wrap property \n\nC++: mrpt::math::CSplineInterpolator1D::getWrap2pi() --> bool");
		cl.def("appendXY", (void (mrpt::math::CSplineInterpolator1D::*)(double, double)) &mrpt::math::CSplineInterpolator1D::appendXY, "Append a new point: \n\nC++: mrpt::math::CSplineInterpolator1D::appendXY(double, double) --> void", pybind11::arg("x"), pybind11::arg("y"));
		cl.def("clear", (void (mrpt::math::CSplineInterpolator1D::*)()) &mrpt::math::CSplineInterpolator1D::clear, "Clears all stored points \n\nC++: mrpt::math::CSplineInterpolator1D::clear() --> void");
		cl.def("query", (double & (mrpt::math::CSplineInterpolator1D::*)(double, double &, bool &) const) &mrpt::math::CSplineInterpolator1D::query, "Query an interpolation of the curve at some \"x\".\n   The result is stored in \"y\". If the \"x\" point is out of range,\n \"valid_out\" is set to false.\n  \n\n A reference to \"y\"\n \n\n queryVector\n\nC++: mrpt::math::CSplineInterpolator1D::query(double, double &, bool &) const --> double &", pybind11::return_value_policy::automatic, pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("out_valid"));
		cl.def("assign", (class mrpt::math::CSplineInterpolator1D & (mrpt::math::CSplineInterpolator1D::*)(const class mrpt::math::CSplineInterpolator1D &)) &mrpt::math::CSplineInterpolator1D::operator=, "C++: mrpt::math::CSplineInterpolator1D::operator=(const class mrpt::math::CSplineInterpolator1D &) --> class mrpt::math::CSplineInterpolator1D &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::math::TObject2D file:mrpt/math/TObject2D.h line:29
		pybind11::class_<mrpt::math::TObject2D, std::shared_ptr<mrpt::math::TObject2D>> cl(M("mrpt::math"), "TObject2D", "A variant type for any lightweight 2D type: point, segment, line, polygon.\n Use provided helper method, or directly access the variant `data`.\n\n \n TPoint2D,TSegment2D,TLine2D,TPolygon2D");
		cl.def( pybind11::init( [](){ return new mrpt::math::TObject2D(); } ) );
		cl.def( pybind11::init( [](mrpt::math::TObject2D const &o){ return new mrpt::math::TObject2D(o); } ) );
		cl.def_readwrite("data", &mrpt::math::TObject2D::data);
		cl.def("getAs", (const struct mrpt::math::TPoint2D_<double> & (mrpt::math::TObject2D::*)() const) &mrpt::math::TObject2D::getAs<mrpt::math::TPoint2D_<double>>, "C++: mrpt::math::TObject2D::getAs() const --> const struct mrpt::math::TPoint2D_<double> &", pybind11::return_value_policy::automatic);
		cl.def("getAs", (const struct mrpt::math::TSegment2D & (mrpt::math::TObject2D::*)() const) &mrpt::math::TObject2D::getAs<mrpt::math::TSegment2D>, "C++: mrpt::math::TObject2D::getAs() const --> const struct mrpt::math::TSegment2D &", pybind11::return_value_policy::automatic);
		cl.def("getAs", (const struct mrpt::math::TLine2D & (mrpt::math::TObject2D::*)() const) &mrpt::math::TObject2D::getAs<mrpt::math::TLine2D>, "C++: mrpt::math::TObject2D::getAs() const --> const struct mrpt::math::TLine2D &", pybind11::return_value_policy::automatic);
		cl.def("getAs", (const class mrpt::math::TPolygon2D & (mrpt::math::TObject2D::*)() const) &mrpt::math::TObject2D::getAs<mrpt::math::TPolygon2D>, "C++: mrpt::math::TObject2D::getAs() const --> const class mrpt::math::TPolygon2D &", pybind11::return_value_policy::automatic);
		cl.def("isPoint", (bool (mrpt::math::TObject2D::*)() const) &mrpt::math::TObject2D::isPoint, "Checks whether content is a point. \n\nC++: mrpt::math::TObject2D::isPoint() const --> bool");
		cl.def("isSegment", (bool (mrpt::math::TObject2D::*)() const) &mrpt::math::TObject2D::isSegment, "Checks whether content is a segment. \n\nC++: mrpt::math::TObject2D::isSegment() const --> bool");
		cl.def("isLine", (bool (mrpt::math::TObject2D::*)() const) &mrpt::math::TObject2D::isLine, "Checks whether content is a line.\n\nC++: mrpt::math::TObject2D::isLine() const --> bool");
		cl.def("isPolygon", (bool (mrpt::math::TObject2D::*)() const) &mrpt::math::TObject2D::isPolygon, "Checks whether content is a polygon.\n\nC++: mrpt::math::TObject2D::isPolygon() const --> bool");
		cl.def("empty", (bool (mrpt::math::TObject2D::*)() const) &mrpt::math::TObject2D::empty, "C++: mrpt::math::TObject2D::empty() const --> bool");
		cl.def("getPoint", (bool (mrpt::math::TObject2D::*)(struct mrpt::math::TPoint2D_<double> &) const) &mrpt::math::TObject2D::getPoint, "returns true if the objects is a point, and retrieves its value in\n `out`. Prefer getAs(). This method was left in mrpt 2.3.0 for backwards\n compatibility.\n\nC++: mrpt::math::TObject2D::getPoint(struct mrpt::math::TPoint2D_<double> &) const --> bool", pybind11::arg("out"));
		cl.def("getSegment", (bool (mrpt::math::TObject2D::*)(struct mrpt::math::TSegment2D &) const) &mrpt::math::TObject2D::getSegment, "returns true if the objects is a segment, and retrieves its value in\n `out`. Prefer getAs(). This method was left in mrpt 2.3.0 for backwards\n compatibility.\n\nC++: mrpt::math::TObject2D::getSegment(struct mrpt::math::TSegment2D &) const --> bool", pybind11::arg("out"));
		cl.def("getLine", (bool (mrpt::math::TObject2D::*)(struct mrpt::math::TLine2D &) const) &mrpt::math::TObject2D::getLine, "returns true if the objects is a line, and retrieves its value in\n `out`. Prefer getAs(). This method was left in mrpt 2.3.0 for backwards\n compatibility.\n\nC++: mrpt::math::TObject2D::getLine(struct mrpt::math::TLine2D &) const --> bool", pybind11::arg("out"));
		cl.def("getPolygon", (bool (mrpt::math::TObject2D::*)(class mrpt::math::TPolygon2D &) const) &mrpt::math::TObject2D::getPolygon, "returns true if the objects is a TPolygon2D, and retrieves its value in\n `out`. Prefer getAs(). This method was left in mrpt 2.3.0 for backwards\n compatibility.\n\nC++: mrpt::math::TObject2D::getPolygon(class mrpt::math::TPolygon2D &) const --> bool", pybind11::arg("out"));
		cl.def("asString", (std::string (mrpt::math::TObject2D::*)() const) &mrpt::math::TObject2D::asString, "Gets a string with the type and the parameters of the object. `empty` if\n not defined. \n\n New in MRPT 2.3.0 \n\nC++: mrpt::math::TObject2D::asString() const --> std::string");
		cl.def("generate3DObject", (struct mrpt::math::TObject3D (mrpt::math::TObject2D::*)() const) &mrpt::math::TObject2D::generate3DObject, "Cast into 3D space. \n\nC++: mrpt::math::TObject2D::generate3DObject() const --> struct mrpt::math::TObject3D");
		cl.def("assign", (struct mrpt::math::TObject2D & (mrpt::math::TObject2D::*)(const struct mrpt::math::TObject2D &)) &mrpt::math::TObject2D::operator=, "C++: mrpt::math::TObject2D::operator=(const struct mrpt::math::TObject2D &) --> struct mrpt::math::TObject2D &", pybind11::return_value_policy::automatic, pybind11::arg(""));

		cl.def("__str__", [](mrpt::math::TObject2D const &o) -> std::string { std::ostringstream s; using namespace mrpt::math; s << o; return s.str(); } );
	}
}
