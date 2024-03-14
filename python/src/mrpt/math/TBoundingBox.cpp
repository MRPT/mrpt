#include <iterator>
#include <memory>
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CQuaternion.h>
#include <mrpt/math/TBoundingBox.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/poses/CPoint2D.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DQuat.h>
#include <mrpt/poses/CPoseOrPoint.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/typemeta/static_string.h>
#include <optional>
#include <sstream> // __str__
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

void bind_mrpt_math_TBoundingBox(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::math::TBoundingBox_ file:mrpt/math/TBoundingBox.h line:27
		pybind11::class_<mrpt::math::TBoundingBox_<double>, std::shared_ptr<mrpt::math::TBoundingBox_<double>>> cl(M("mrpt::math"), "TBoundingBox_double_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::math::TBoundingBox_<double>(); } ) );
		cl.def( pybind11::init( [](const struct mrpt::math::TPoint3D_<double> & a0, const struct mrpt::math::TPoint3D_<double> & a1){ return new mrpt::math::TBoundingBox_<double>(a0, a1); } ), "doc" , pybind11::arg("Min"), pybind11::arg("Max"));
		cl.def( pybind11::init<const struct mrpt::math::TPoint3D_<double> &, const struct mrpt::math::TPoint3D_<double> &, const enum mrpt::math::TBoundingBox_<double>::CTOR_FLAGS>(), pybind11::arg("Min"), pybind11::arg("Max"), pybind11::arg("f") );

		cl.def( pybind11::init( [](mrpt::math::TBoundingBox_<double> const &o){ return new mrpt::math::TBoundingBox_<double>(o); } ) );

		pybind11::enum_<mrpt::math::TBoundingBox_<double>::CTOR_FLAGS>(cl, "CTOR_FLAGS", "")
			.value("None", mrpt::math::TBoundingBox_<double>::CTOR_FLAGS::None)
			.value("AllowUnordered", mrpt::math::TBoundingBox_<double>::CTOR_FLAGS::AllowUnordered);

		cl.def_readwrite("min", &mrpt::math::TBoundingBox_<double>::min);
		cl.def_readwrite("max", &mrpt::math::TBoundingBox_<double>::max);
		cl.def("compose", (struct mrpt::math::TBoundingBox_<double> (mrpt::math::TBoundingBox_<double>::*)(const class mrpt::poses::CPose3D &) const) &mrpt::math::TBoundingBox_<double>::compose<mrpt::poses::CPose3D>, "C++: mrpt::math::TBoundingBox_<double>::compose(const class mrpt::poses::CPose3D &) const --> struct mrpt::math::TBoundingBox_<double>", pybind11::arg("pose"));
		cl.def_static("PlusMinusInfinity", (struct mrpt::math::TBoundingBox_<double> (*)()) &mrpt::math::TBoundingBox_<double>::PlusMinusInfinity, "C++: mrpt::math::TBoundingBox_<double>::PlusMinusInfinity() --> struct mrpt::math::TBoundingBox_<double>");
		cl.def_static("FromUnsortedPoints", (struct mrpt::math::TBoundingBox_<double> (*)(const struct mrpt::math::TPoint3D_<double> &, const struct mrpt::math::TPoint3D_<double> &)) &mrpt::math::TBoundingBox_<double>::FromUnsortedPoints, "C++: mrpt::math::TBoundingBox_<double>::FromUnsortedPoints(const struct mrpt::math::TPoint3D_<double> &, const struct mrpt::math::TPoint3D_<double> &) --> struct mrpt::math::TBoundingBox_<double>", pybind11::arg("pt1"), pybind11::arg("pt2"));
		cl.def("volume", (double (mrpt::math::TBoundingBox_<double>::*)() const) &mrpt::math::TBoundingBox_<double>::volume, "C++: mrpt::math::TBoundingBox_<double>::volume() const --> double");
		cl.def("unionWith", (struct mrpt::math::TBoundingBox_<double> (mrpt::math::TBoundingBox_<double>::*)(const struct mrpt::math::TBoundingBox_<double> &) const) &mrpt::math::TBoundingBox_<double>::unionWith, "C++: mrpt::math::TBoundingBox_<double>::unionWith(const struct mrpt::math::TBoundingBox_<double> &) const --> struct mrpt::math::TBoundingBox_<double>", pybind11::arg("b"));
		cl.def("updateWithPoint", (void (mrpt::math::TBoundingBox_<double>::*)(const struct mrpt::math::TPoint3D_<double> &)) &mrpt::math::TBoundingBox_<double>::updateWithPoint, "C++: mrpt::math::TBoundingBox_<double>::updateWithPoint(const struct mrpt::math::TPoint3D_<double> &) --> void", pybind11::arg("p"));
		cl.def("containsPoint", (bool (mrpt::math::TBoundingBox_<double>::*)(const struct mrpt::math::TPoint3D_<double> &) const) &mrpt::math::TBoundingBox_<double>::containsPoint, "C++: mrpt::math::TBoundingBox_<double>::containsPoint(const struct mrpt::math::TPoint3D_<double> &) const --> bool", pybind11::arg("p"));
		cl.def("asString", (std::string (mrpt::math::TBoundingBox_<double>::*)() const) &mrpt::math::TBoundingBox_<double>::asString, "C++: mrpt::math::TBoundingBox_<double>::asString() const --> std::string");
		cl.def("__eq__", (bool (mrpt::math::TBoundingBox_<double>::*)(const struct mrpt::math::TBoundingBox_<double> &) const) &mrpt::math::TBoundingBox_<double>::operator==, "C++: mrpt::math::TBoundingBox_<double>::operator==(const struct mrpt::math::TBoundingBox_<double> &) const --> bool", pybind11::arg("o"));
		cl.def("__ne__", (bool (mrpt::math::TBoundingBox_<double>::*)(const struct mrpt::math::TBoundingBox_<double> &) const) &mrpt::math::TBoundingBox_<double>::operator!=, "C++: mrpt::math::TBoundingBox_<double>::operator!=(const struct mrpt::math::TBoundingBox_<double> &) const --> bool", pybind11::arg("o"));
		cl.def("assign", (struct mrpt::math::TBoundingBox_<double> & (mrpt::math::TBoundingBox_<double>::*)(const struct mrpt::math::TBoundingBox_<double> &)) &mrpt::math::TBoundingBox_<double>::operator=, "C++: mrpt::math::TBoundingBox_<double>::operator=(const struct mrpt::math::TBoundingBox_<double> &) --> struct mrpt::math::TBoundingBox_<double> &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::math::TBoundingBox_ file:mrpt/math/TBoundingBox.h line:27
		pybind11::class_<mrpt::math::TBoundingBox_<float>, std::shared_ptr<mrpt::math::TBoundingBox_<float>>> cl(M("mrpt::math"), "TBoundingBox_float_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::math::TBoundingBox_<float>(); } ) );
		cl.def( pybind11::init( [](const struct mrpt::math::TPoint3D_<float> & a0, const struct mrpt::math::TPoint3D_<float> & a1){ return new mrpt::math::TBoundingBox_<float>(a0, a1); } ), "doc" , pybind11::arg("Min"), pybind11::arg("Max"));
		cl.def( pybind11::init<const struct mrpt::math::TPoint3D_<float> &, const struct mrpt::math::TPoint3D_<float> &, const enum mrpt::math::TBoundingBox_<float>::CTOR_FLAGS>(), pybind11::arg("Min"), pybind11::arg("Max"), pybind11::arg("f") );

		cl.def( pybind11::init( [](mrpt::math::TBoundingBox_<float> const &o){ return new mrpt::math::TBoundingBox_<float>(o); } ) );

		pybind11::enum_<mrpt::math::TBoundingBox_<float>::CTOR_FLAGS>(cl, "CTOR_FLAGS", "")
			.value("None", mrpt::math::TBoundingBox_<float>::CTOR_FLAGS::None)
			.value("AllowUnordered", mrpt::math::TBoundingBox_<float>::CTOR_FLAGS::AllowUnordered);

		cl.def_readwrite("min", &mrpt::math::TBoundingBox_<float>::min);
		cl.def_readwrite("max", &mrpt::math::TBoundingBox_<float>::max);
		cl.def_static("PlusMinusInfinity", (struct mrpt::math::TBoundingBox_<float> (*)()) &mrpt::math::TBoundingBox_<float>::PlusMinusInfinity, "C++: mrpt::math::TBoundingBox_<float>::PlusMinusInfinity() --> struct mrpt::math::TBoundingBox_<float>");
		cl.def_static("FromUnsortedPoints", (struct mrpt::math::TBoundingBox_<float> (*)(const struct mrpt::math::TPoint3D_<float> &, const struct mrpt::math::TPoint3D_<float> &)) &mrpt::math::TBoundingBox_<float>::FromUnsortedPoints, "C++: mrpt::math::TBoundingBox_<float>::FromUnsortedPoints(const struct mrpt::math::TPoint3D_<float> &, const struct mrpt::math::TPoint3D_<float> &) --> struct mrpt::math::TBoundingBox_<float>", pybind11::arg("pt1"), pybind11::arg("pt2"));
		cl.def("volume", (float (mrpt::math::TBoundingBox_<float>::*)() const) &mrpt::math::TBoundingBox_<float>::volume, "C++: mrpt::math::TBoundingBox_<float>::volume() const --> float");
		cl.def("unionWith", (struct mrpt::math::TBoundingBox_<float> (mrpt::math::TBoundingBox_<float>::*)(const struct mrpt::math::TBoundingBox_<float> &) const) &mrpt::math::TBoundingBox_<float>::unionWith, "C++: mrpt::math::TBoundingBox_<float>::unionWith(const struct mrpt::math::TBoundingBox_<float> &) const --> struct mrpt::math::TBoundingBox_<float>", pybind11::arg("b"));
		cl.def("updateWithPoint", (void (mrpt::math::TBoundingBox_<float>::*)(const struct mrpt::math::TPoint3D_<float> &)) &mrpt::math::TBoundingBox_<float>::updateWithPoint, "C++: mrpt::math::TBoundingBox_<float>::updateWithPoint(const struct mrpt::math::TPoint3D_<float> &) --> void", pybind11::arg("p"));
		cl.def("containsPoint", (bool (mrpt::math::TBoundingBox_<float>::*)(const struct mrpt::math::TPoint3D_<float> &) const) &mrpt::math::TBoundingBox_<float>::containsPoint, "C++: mrpt::math::TBoundingBox_<float>::containsPoint(const struct mrpt::math::TPoint3D_<float> &) const --> bool", pybind11::arg("p"));
		cl.def("asString", (std::string (mrpt::math::TBoundingBox_<float>::*)() const) &mrpt::math::TBoundingBox_<float>::asString, "C++: mrpt::math::TBoundingBox_<float>::asString() const --> std::string");
		cl.def("__eq__", (bool (mrpt::math::TBoundingBox_<float>::*)(const struct mrpt::math::TBoundingBox_<float> &) const) &mrpt::math::TBoundingBox_<float>::operator==, "C++: mrpt::math::TBoundingBox_<float>::operator==(const struct mrpt::math::TBoundingBox_<float> &) const --> bool", pybind11::arg("o"));
		cl.def("__ne__", (bool (mrpt::math::TBoundingBox_<float>::*)(const struct mrpt::math::TBoundingBox_<float> &) const) &mrpt::math::TBoundingBox_<float>::operator!=, "C++: mrpt::math::TBoundingBox_<float>::operator!=(const struct mrpt::math::TBoundingBox_<float> &) const --> bool", pybind11::arg("o"));
		cl.def("assign", (struct mrpt::math::TBoundingBox_<float> & (mrpt::math::TBoundingBox_<float>::*)(const struct mrpt::math::TBoundingBox_<float> &)) &mrpt::math::TBoundingBox_<float>::operator=, "C++: mrpt::math::TBoundingBox_<float>::operator=(const struct mrpt::math::TBoundingBox_<float> &) --> struct mrpt::math::TBoundingBox_<float> &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
