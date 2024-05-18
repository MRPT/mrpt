#include <ios>
#include <iterator>
#include <locale>
#include <memory>
#include <mrpt/math/CMatrixB.h>
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/TLine2D.h>
#include <mrpt/math/TLine3D.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/math/TSegment2D.h>
#include <mrpt/math/TSegment3D.h>
#include <mrpt/math/matrix_size_t.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/CSerializable.h>
#include <mrpt/typemeta/static_string.h>
#include <optional>
#include <ostream>
#include <sstream> // __str__
#include <streambuf>
#include <string>
#include <variant>

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

// mrpt::math::CMatrixB file:mrpt/math/CMatrixB.h line:21
struct PyCallBack_mrpt_math_CMatrixB : public mrpt::math::CMatrixB {
	using mrpt::math::CMatrixB::CMatrixB;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::math::CMatrixB *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CMatrixB::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::math::CMatrixB *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CMatrixB::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::math::CMatrixB *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CMatrixB::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::math::CMatrixB *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CMatrixB::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::math::CMatrixB *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CMatrixB::serializeFrom(a0, a1);
	}
};

void bind_mrpt_math_CMatrixB(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::math::CMatrixB file:mrpt/math/CMatrixB.h line:21
		pybind11::class_<mrpt::math::CMatrixB, std::shared_ptr<mrpt::math::CMatrixB>, PyCallBack_mrpt_math_CMatrixB, mrpt::serialization::CSerializable> cl(M("mrpt::math"), "CMatrixB", "This class is a \"CSerializable\" wrapper for \"CMatrixBool\".\n \n\n For a complete introduction to Matrices and vectors in MRPT, see:\n https://www.mrpt.org/Matrices_vectors_arrays_and_Linear_Algebra_MRPT_and_Eigen_classes\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::math::CMatrixB(); }, [](){ return new PyCallBack_mrpt_math_CMatrixB(); } ), "doc");
		cl.def( pybind11::init( [](size_t const & a0){ return new mrpt::math::CMatrixB(a0); }, [](size_t const & a0){ return new PyCallBack_mrpt_math_CMatrixB(a0); } ), "doc");
		cl.def( pybind11::init<size_t, size_t>(), pybind11::arg("row"), pybind11::arg("col") );

		cl.def( pybind11::init( [](PyCallBack_mrpt_math_CMatrixB const &o){ return new PyCallBack_mrpt_math_CMatrixB(o); } ) );
		cl.def( pybind11::init( [](mrpt::math::CMatrixB const &o){ return new mrpt::math::CMatrixB(o); } ) );
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::math::CMatrixB::GetRuntimeClassIdStatic, "C++: mrpt::math::CMatrixB::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::math::CMatrixB::*)() const) &mrpt::math::CMatrixB::GetRuntimeClass, "C++: mrpt::math::CMatrixB::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::math::CMatrixB::*)() const) &mrpt::math::CMatrixB::clone, "C++: mrpt::math::CMatrixB::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::math::CMatrixB::CreateObject, "C++: mrpt::math::CMatrixB::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("assign", (class mrpt::math::CMatrixB & (mrpt::math::CMatrixB::*)(const class mrpt::math::CMatrixB &)) &mrpt::math::CMatrixB::operator=, "C++: mrpt::math::CMatrixB::operator=(const class mrpt::math::CMatrixB &) --> class mrpt::math::CMatrixB &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::math::TLine2D file:mrpt/math/TLine2D.h line:23
		pybind11::class_<mrpt::math::TLine2D, std::shared_ptr<mrpt::math::TLine2D>> cl(M("mrpt::math"), "TLine2D", "2D line without bounds, represented by its equation \n.\n \n\n TLine3D,TSegment2D,TPolygon2D,TPoint2D");
		cl.def( pybind11::init<const struct mrpt::math::TPoint2D_<double> &, const struct mrpt::math::TPoint2D_<double> &>(), pybind11::arg("p1"), pybind11::arg("p2") );

		cl.def( pybind11::init<const struct mrpt::math::TSegment2D &>(), pybind11::arg("s") );

		cl.def( pybind11::init( [](){ return new mrpt::math::TLine2D(); } ) );
		cl.def( pybind11::init<double, double, double>(), pybind11::arg("A"), pybind11::arg("B"), pybind11::arg("C") );

		cl.def( pybind11::init<const struct mrpt::math::TLine3D &>(), pybind11::arg("l") );

		cl.def( pybind11::init( [](mrpt::math::TLine2D const &o){ return new mrpt::math::TLine2D(o); } ) );
		cl.def_readwrite("coefs", &mrpt::math::TLine2D::coefs);
		cl.def_static("FromCoefficientsABC", (struct mrpt::math::TLine2D (*)(double, double, double)) &mrpt::math::TLine2D::FromCoefficientsABC, "Static constructor from Ax+By+C=0 coefficients.\n \n\n [New in MRPT 2.0.4]\n\nC++: mrpt::math::TLine2D::FromCoefficientsABC(double, double, double) --> struct mrpt::math::TLine2D", pybind11::arg("A"), pybind11::arg("B"), pybind11::arg("C"));
		cl.def_static("FromTwoPoints", (struct mrpt::math::TLine2D (*)(const struct mrpt::math::TPoint2D_<double> &, const struct mrpt::math::TPoint2D_<double> &)) &mrpt::math::TLine2D::FromTwoPoints, "Static constructor from two points.\n \n\n [New in MRPT 2.0.4]\n\nC++: mrpt::math::TLine2D::FromTwoPoints(const struct mrpt::math::TPoint2D_<double> &, const struct mrpt::math::TPoint2D_<double> &) --> struct mrpt::math::TLine2D", pybind11::arg("p1"), pybind11::arg("p2"));
		cl.def("evaluatePoint", (double (mrpt::math::TLine2D::*)(const struct mrpt::math::TPoint2D_<double> &) const) &mrpt::math::TLine2D::evaluatePoint, "Evaluate point in the line's equation.\n\nC++: mrpt::math::TLine2D::evaluatePoint(const struct mrpt::math::TPoint2D_<double> &) const --> double", pybind11::arg("point"));
		cl.def("contains", (bool (mrpt::math::TLine2D::*)(const struct mrpt::math::TPoint2D_<double> &) const) &mrpt::math::TLine2D::contains, "Check whether a point is inside the line.\n\nC++: mrpt::math::TLine2D::contains(const struct mrpt::math::TPoint2D_<double> &) const --> bool", pybind11::arg("point"));
		cl.def("distance", (double (mrpt::math::TLine2D::*)(const struct mrpt::math::TPoint2D_<double> &) const) &mrpt::math::TLine2D::distance, "Absolute distance from a given point.\n\nC++: mrpt::math::TLine2D::distance(const struct mrpt::math::TPoint2D_<double> &) const --> double", pybind11::arg("point"));
		cl.def("signedDistance", (double (mrpt::math::TLine2D::*)(const struct mrpt::math::TPoint2D_<double> &) const) &mrpt::math::TLine2D::signedDistance, "Distance with sign from a given point (sign indicates side).\n\nC++: mrpt::math::TLine2D::signedDistance(const struct mrpt::math::TPoint2D_<double> &) const --> double", pybind11::arg("point"));
		cl.def("unitarize", (void (mrpt::math::TLine2D::*)()) &mrpt::math::TLine2D::unitarize, "Unitarize line's normal vector.\n\nC++: mrpt::math::TLine2D::unitarize() --> void");
		cl.def("generate3DObject", (void (mrpt::math::TLine2D::*)(struct mrpt::math::TLine3D &) const) &mrpt::math::TLine2D::generate3DObject, "Project into 3D space, setting the z to 0.\n\nC++: mrpt::math::TLine2D::generate3DObject(struct mrpt::math::TLine3D &) const --> void", pybind11::arg("l"));
		cl.def("getAsPose2D", (void (mrpt::math::TLine2D::*)(struct mrpt::math::TPose2D &) const) &mrpt::math::TLine2D::getAsPose2D, "C++: mrpt::math::TLine2D::getAsPose2D(struct mrpt::math::TPose2D &) const --> void", pybind11::arg("outPose"));
		cl.def("getAsPose2DForcingOrigin", (void (mrpt::math::TLine2D::*)(const struct mrpt::math::TPoint2D_<double> &, struct mrpt::math::TPose2D &) const) &mrpt::math::TLine2D::getAsPose2DForcingOrigin, "C++: mrpt::math::TLine2D::getAsPose2DForcingOrigin(const struct mrpt::math::TPoint2D_<double> &, struct mrpt::math::TPose2D &) const --> void", pybind11::arg("origin"), pybind11::arg("outPose"));
		cl.def("asString", (std::string (mrpt::math::TLine2D::*)() const) &mrpt::math::TLine2D::asString, "Returns \"[A, B, C]\"\n \n\n [New in MRPT 2.1.0]\n\n \n Do not inherit from mrpt::Stringifyable to avoid virtual class\n table and keeping the class trivially-copiable.\n\nC++: mrpt::math::TLine2D::asString() const --> std::string");
		cl.def("assign", (struct mrpt::math::TLine2D & (mrpt::math::TLine2D::*)(const struct mrpt::math::TLine2D &)) &mrpt::math::TLine2D::operator=, "C++: mrpt::math::TLine2D::operator=(const struct mrpt::math::TLine2D &) --> struct mrpt::math::TLine2D &", pybind11::return_value_policy::automatic, pybind11::arg(""));

		cl.def("__str__", [](mrpt::math::TLine2D const &o) -> std::string { std::ostringstream s; using namespace mrpt::math; s << o; return s.str(); } );
	}
}
