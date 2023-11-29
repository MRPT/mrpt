#include <chrono>
#include <iterator>
#include <memory>
#include <mrpt/core/Clock.h>
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CQuaternion.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/poses/CPoint2D.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DQuat.h>
#include <mrpt/poses/CPoseInterpolatorBase.h>
#include <mrpt/poses/CPoseOrPoint.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/typemeta/static_string.h>
#include <optional>
#include <ratio>
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

void bind_mrpt_poses_CPoseInterpolatorBase(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	// mrpt::poses::TInterpolatorMethod file:mrpt/poses/CPoseInterpolatorBase.h line:35
	pybind11::enum_<mrpt::poses::TInterpolatorMethod>(M("mrpt::poses"), "TInterpolatorMethod", pybind11::arithmetic(), "Type to select the interpolation method in CPoseInterpolatorBase derived\n classes.\n  - imSpline: Spline interpolation using 4 points (2 before + 2 after the\n query point).\n  - imLinear2Neig: Linear interpolation between the previous and next\n neightbour.\n  - imLinear4Neig: Linear interpolation using the linear fit of the 4 closer\n points (2 before + 2 after the query point).\n  - imSSLLLL : Use Spline for X and Y, and Linear Least squares for Z, yaw,\n pitch and roll.\n  - imSSLSLL : Use Spline for X, Y and yaw, and Linear Lesat squares for Z,\n pitch and roll.\n  - imLinearSlerp: Linear for X,Y,Z, Slerp for 3D angles.\n  - imSplineSlerp: Spline for X,Y,Z, Slerp for 3D angles.\n \n\n\n ")
		.value("imSpline", mrpt::poses::imSpline)
		.value("imLinear2Neig", mrpt::poses::imLinear2Neig)
		.value("imLinear4Neig", mrpt::poses::imLinear4Neig)
		.value("imSSLLLL", mrpt::poses::imSSLLLL)
		.value("imSSLSLL", mrpt::poses::imSSLSLL)
		.value("imLinearSlerp", mrpt::poses::imLinearSlerp)
		.value("imSplineSlerp", mrpt::poses::imSplineSlerp)
		.export_values();

;

	{ // mrpt::poses::CPoseInterpolatorBase file:mrpt/poses/CPoseInterpolatorBase.h line:50
		pybind11::class_<mrpt::poses::CPoseInterpolatorBase<2>, std::shared_ptr<mrpt::poses::CPoseInterpolatorBase<2>>> cl(M("mrpt::poses"), "CPoseInterpolatorBase_2_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::poses::CPoseInterpolatorBase<2>(); } ) );
		cl.def( pybind11::init( [](mrpt::poses::CPoseInterpolatorBase<2> const &o){ return new mrpt::poses::CPoseInterpolatorBase<2>(o); } ) );
		cl.def("size", (size_t (mrpt::poses::CPoseInterpolatorBase<2>::*)() const) &mrpt::poses::CPoseInterpolatorBase<2>::size, "C++: mrpt::poses::CPoseInterpolatorBase<2>::size() const --> size_t");
		cl.def("empty", (bool (mrpt::poses::CPoseInterpolatorBase<2>::*)() const) &mrpt::poses::CPoseInterpolatorBase<2>::empty, "C++: mrpt::poses::CPoseInterpolatorBase<2>::empty() const --> bool");
		cl.def("at", (struct mrpt::math::TPose2D & (mrpt::poses::CPoseInterpolatorBase<2>::*)(const mrpt::Clock::time_point &)) &mrpt::poses::CPoseInterpolatorBase<2>::at, "C++: mrpt::poses::CPoseInterpolatorBase<2>::at(const mrpt::Clock::time_point &) --> struct mrpt::math::TPose2D &", pybind11::return_value_policy::automatic, pybind11::arg("t"));
		cl.def("insert", (void (mrpt::poses::CPoseInterpolatorBase<2>::*)(const mrpt::Clock::time_point &, const struct mrpt::math::TPose2D &)) &mrpt::poses::CPoseInterpolatorBase<2>::insert, "C++: mrpt::poses::CPoseInterpolatorBase<2>::insert(const mrpt::Clock::time_point &, const struct mrpt::math::TPose2D &) --> void", pybind11::arg("t"), pybind11::arg("p"));
		cl.def("insert", (void (mrpt::poses::CPoseInterpolatorBase<2>::*)(const mrpt::Clock::time_point &, const class mrpt::poses::CPose2D &)) &mrpt::poses::CPoseInterpolatorBase<2>::insert, "C++: mrpt::poses::CPoseInterpolatorBase<2>::insert(const mrpt::Clock::time_point &, const class mrpt::poses::CPose2D &) --> void", pybind11::arg("t"), pybind11::arg("p"));
		cl.def("interpolate", (struct mrpt::math::TPose2D & (mrpt::poses::CPoseInterpolatorBase<2>::*)(const mrpt::Clock::time_point &, struct mrpt::math::TPose2D &, bool &) const) &mrpt::poses::CPoseInterpolatorBase<2>::interpolate, "C++: mrpt::poses::CPoseInterpolatorBase<2>::interpolate(const mrpt::Clock::time_point &, struct mrpt::math::TPose2D &, bool &) const --> struct mrpt::math::TPose2D &", pybind11::return_value_policy::automatic, pybind11::arg("t"), pybind11::arg("out_interp"), pybind11::arg("out_valid_interp"));
		cl.def("interpolate", (class mrpt::poses::CPose2D & (mrpt::poses::CPoseInterpolatorBase<2>::*)(const mrpt::Clock::time_point &, class mrpt::poses::CPose2D &, bool &) const) &mrpt::poses::CPoseInterpolatorBase<2>::interpolate, "C++: mrpt::poses::CPoseInterpolatorBase<2>::interpolate(const mrpt::Clock::time_point &, class mrpt::poses::CPose2D &, bool &) const --> class mrpt::poses::CPose2D &", pybind11::return_value_policy::automatic, pybind11::arg("t"), pybind11::arg("out_interp"), pybind11::arg("out_valid_interp"));
		cl.def("clear", (void (mrpt::poses::CPoseInterpolatorBase<2>::*)()) &mrpt::poses::CPoseInterpolatorBase<2>::clear, "C++: mrpt::poses::CPoseInterpolatorBase<2>::clear() --> void");
		cl.def("setMaxTimeInterpolation", (void (mrpt::poses::CPoseInterpolatorBase<2>::*)(const struct std::chrono::duration<int64_t,struct std::ratio<1,10000000>> &)) &mrpt::poses::CPoseInterpolatorBase<2>::setMaxTimeInterpolation, "C++: mrpt::poses::CPoseInterpolatorBase<2>::setMaxTimeInterpolation(const struct std::chrono::duration<int64_t,struct std::ratio<1,10000000>> &) --> void", pybind11::arg("time"));
		cl.def("getMaxTimeInterpolation", (struct std::chrono::duration<int64_t,struct std::ratio<1,10000000>> (mrpt::poses::CPoseInterpolatorBase<2>::*)()) &mrpt::poses::CPoseInterpolatorBase<2>::getMaxTimeInterpolation, "C++: mrpt::poses::CPoseInterpolatorBase<2>::getMaxTimeInterpolation() --> struct std::chrono::duration<int64_t,struct std::ratio<1,10000000>>");
		cl.def("getPreviousPoseWithMinDistance", (bool (mrpt::poses::CPoseInterpolatorBase<2>::*)(const mrpt::Clock::time_point &, double, struct mrpt::math::TPose2D &)) &mrpt::poses::CPoseInterpolatorBase<2>::getPreviousPoseWithMinDistance, "C++: mrpt::poses::CPoseInterpolatorBase<2>::getPreviousPoseWithMinDistance(const mrpt::Clock::time_point &, double, struct mrpt::math::TPose2D &) --> bool", pybind11::arg("t"), pybind11::arg("distance"), pybind11::arg("out_pose"));
		cl.def("getPreviousPoseWithMinDistance", (bool (mrpt::poses::CPoseInterpolatorBase<2>::*)(const mrpt::Clock::time_point &, double, class mrpt::poses::CPose2D &)) &mrpt::poses::CPoseInterpolatorBase<2>::getPreviousPoseWithMinDistance, "C++: mrpt::poses::CPoseInterpolatorBase<2>::getPreviousPoseWithMinDistance(const mrpt::Clock::time_point &, double, class mrpt::poses::CPose2D &) --> bool", pybind11::arg("t"), pybind11::arg("distance"), pybind11::arg("out_pose"));
		cl.def("saveToTextFile", (bool (mrpt::poses::CPoseInterpolatorBase<2>::*)(const std::string &) const) &mrpt::poses::CPoseInterpolatorBase<2>::saveToTextFile, "C++: mrpt::poses::CPoseInterpolatorBase<2>::saveToTextFile(const std::string &) const --> bool", pybind11::arg("s"));
		cl.def("saveToTextFile_TUM", (bool (mrpt::poses::CPoseInterpolatorBase<2>::*)(const std::string &) const) &mrpt::poses::CPoseInterpolatorBase<2>::saveToTextFile_TUM, "C++: mrpt::poses::CPoseInterpolatorBase<2>::saveToTextFile_TUM(const std::string &) const --> bool", pybind11::arg("s"));
		cl.def("saveInterpolatedToTextFile", (bool (mrpt::poses::CPoseInterpolatorBase<2>::*)(const std::string &, const struct std::chrono::duration<int64_t,struct std::ratio<1,10000000>> &) const) &mrpt::poses::CPoseInterpolatorBase<2>::saveInterpolatedToTextFile, "C++: mrpt::poses::CPoseInterpolatorBase<2>::saveInterpolatedToTextFile(const std::string &, const struct std::chrono::duration<int64_t,struct std::ratio<1,10000000>> &) const --> bool", pybind11::arg("s"), pybind11::arg("period"));
		cl.def("loadFromTextFile", (bool (mrpt::poses::CPoseInterpolatorBase<2>::*)(const std::string &)) &mrpt::poses::CPoseInterpolatorBase<2>::loadFromTextFile, "C++: mrpt::poses::CPoseInterpolatorBase<2>::loadFromTextFile(const std::string &) --> bool", pybind11::arg("s"));
		cl.def("loadFromTextFile_TUM", (bool (mrpt::poses::CPoseInterpolatorBase<2>::*)(const std::string &)) &mrpt::poses::CPoseInterpolatorBase<2>::loadFromTextFile_TUM, "C++: mrpt::poses::CPoseInterpolatorBase<2>::loadFromTextFile_TUM(const std::string &) --> bool", pybind11::arg("s"));
		cl.def("getBoundingBox", (void (mrpt::poses::CPoseInterpolatorBase<2>::*)(struct mrpt::math::TPoint2D_<double> &, struct mrpt::math::TPoint2D_<double> &) const) &mrpt::poses::CPoseInterpolatorBase<2>::getBoundingBox, "C++: mrpt::poses::CPoseInterpolatorBase<2>::getBoundingBox(struct mrpt::math::TPoint2D_<double> &, struct mrpt::math::TPoint2D_<double> &) const --> void", pybind11::arg("minCorner"), pybind11::arg("maxCorner"));
		cl.def("setInterpolationMethod", (void (mrpt::poses::CPoseInterpolatorBase<2>::*)(enum mrpt::poses::TInterpolatorMethod)) &mrpt::poses::CPoseInterpolatorBase<2>::setInterpolationMethod, "C++: mrpt::poses::CPoseInterpolatorBase<2>::setInterpolationMethod(enum mrpt::poses::TInterpolatorMethod) --> void", pybind11::arg("method"));
		cl.def("getInterpolationMethod", (enum mrpt::poses::TInterpolatorMethod (mrpt::poses::CPoseInterpolatorBase<2>::*)() const) &mrpt::poses::CPoseInterpolatorBase<2>::getInterpolationMethod, "C++: mrpt::poses::CPoseInterpolatorBase<2>::getInterpolationMethod() const --> enum mrpt::poses::TInterpolatorMethod");
		cl.def("filter", (void (mrpt::poses::CPoseInterpolatorBase<2>::*)(unsigned int, unsigned int)) &mrpt::poses::CPoseInterpolatorBase<2>::filter, "C++: mrpt::poses::CPoseInterpolatorBase<2>::filter(unsigned int, unsigned int) --> void", pybind11::arg("component"), pybind11::arg("samples"));
		cl.def("assign", (class mrpt::poses::CPoseInterpolatorBase<2> & (mrpt::poses::CPoseInterpolatorBase<2>::*)(const class mrpt::poses::CPoseInterpolatorBase<2> &)) &mrpt::poses::CPoseInterpolatorBase<2>::operator=, "C++: mrpt::poses::CPoseInterpolatorBase<2>::operator=(const class mrpt::poses::CPoseInterpolatorBase<2> &) --> class mrpt::poses::CPoseInterpolatorBase<2> &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::poses::CPoseInterpolatorBase file:mrpt/poses/CPoseInterpolatorBase.h line:50
		pybind11::class_<mrpt::poses::CPoseInterpolatorBase<3>, std::shared_ptr<mrpt::poses::CPoseInterpolatorBase<3>>> cl(M("mrpt::poses"), "CPoseInterpolatorBase_3_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::poses::CPoseInterpolatorBase<3>(); } ) );
		cl.def( pybind11::init( [](mrpt::poses::CPoseInterpolatorBase<3> const &o){ return new mrpt::poses::CPoseInterpolatorBase<3>(o); } ) );
		cl.def("size", (size_t (mrpt::poses::CPoseInterpolatorBase<3>::*)() const) &mrpt::poses::CPoseInterpolatorBase<3>::size, "C++: mrpt::poses::CPoseInterpolatorBase<3>::size() const --> size_t");
		cl.def("empty", (bool (mrpt::poses::CPoseInterpolatorBase<3>::*)() const) &mrpt::poses::CPoseInterpolatorBase<3>::empty, "C++: mrpt::poses::CPoseInterpolatorBase<3>::empty() const --> bool");
		cl.def("at", (struct mrpt::math::TPose3D & (mrpt::poses::CPoseInterpolatorBase<3>::*)(const mrpt::Clock::time_point &)) &mrpt::poses::CPoseInterpolatorBase<3>::at, "C++: mrpt::poses::CPoseInterpolatorBase<3>::at(const mrpt::Clock::time_point &) --> struct mrpt::math::TPose3D &", pybind11::return_value_policy::automatic, pybind11::arg("t"));
		cl.def("insert", (void (mrpt::poses::CPoseInterpolatorBase<3>::*)(const mrpt::Clock::time_point &, const struct mrpt::math::TPose3D &)) &mrpt::poses::CPoseInterpolatorBase<3>::insert, "C++: mrpt::poses::CPoseInterpolatorBase<3>::insert(const mrpt::Clock::time_point &, const struct mrpt::math::TPose3D &) --> void", pybind11::arg("t"), pybind11::arg("p"));
		cl.def("insert", (void (mrpt::poses::CPoseInterpolatorBase<3>::*)(const mrpt::Clock::time_point &, const class mrpt::poses::CPose3D &)) &mrpt::poses::CPoseInterpolatorBase<3>::insert, "C++: mrpt::poses::CPoseInterpolatorBase<3>::insert(const mrpt::Clock::time_point &, const class mrpt::poses::CPose3D &) --> void", pybind11::arg("t"), pybind11::arg("p"));
		cl.def("interpolate", (struct mrpt::math::TPose3D & (mrpt::poses::CPoseInterpolatorBase<3>::*)(const mrpt::Clock::time_point &, struct mrpt::math::TPose3D &, bool &) const) &mrpt::poses::CPoseInterpolatorBase<3>::interpolate, "C++: mrpt::poses::CPoseInterpolatorBase<3>::interpolate(const mrpt::Clock::time_point &, struct mrpt::math::TPose3D &, bool &) const --> struct mrpt::math::TPose3D &", pybind11::return_value_policy::automatic, pybind11::arg("t"), pybind11::arg("out_interp"), pybind11::arg("out_valid_interp"));
		cl.def("interpolate", (class mrpt::poses::CPose3D & (mrpt::poses::CPoseInterpolatorBase<3>::*)(const mrpt::Clock::time_point &, class mrpt::poses::CPose3D &, bool &) const) &mrpt::poses::CPoseInterpolatorBase<3>::interpolate, "C++: mrpt::poses::CPoseInterpolatorBase<3>::interpolate(const mrpt::Clock::time_point &, class mrpt::poses::CPose3D &, bool &) const --> class mrpt::poses::CPose3D &", pybind11::return_value_policy::automatic, pybind11::arg("t"), pybind11::arg("out_interp"), pybind11::arg("out_valid_interp"));
		cl.def("clear", (void (mrpt::poses::CPoseInterpolatorBase<3>::*)()) &mrpt::poses::CPoseInterpolatorBase<3>::clear, "C++: mrpt::poses::CPoseInterpolatorBase<3>::clear() --> void");
		cl.def("setMaxTimeInterpolation", (void (mrpt::poses::CPoseInterpolatorBase<3>::*)(const struct std::chrono::duration<int64_t,struct std::ratio<1,10000000>> &)) &mrpt::poses::CPoseInterpolatorBase<3>::setMaxTimeInterpolation, "C++: mrpt::poses::CPoseInterpolatorBase<3>::setMaxTimeInterpolation(const struct std::chrono::duration<int64_t,struct std::ratio<1,10000000>> &) --> void", pybind11::arg("time"));
		cl.def("getMaxTimeInterpolation", (struct std::chrono::duration<int64_t,struct std::ratio<1,10000000>> (mrpt::poses::CPoseInterpolatorBase<3>::*)()) &mrpt::poses::CPoseInterpolatorBase<3>::getMaxTimeInterpolation, "C++: mrpt::poses::CPoseInterpolatorBase<3>::getMaxTimeInterpolation() --> struct std::chrono::duration<int64_t,struct std::ratio<1,10000000>>");
		cl.def("getPreviousPoseWithMinDistance", (bool (mrpt::poses::CPoseInterpolatorBase<3>::*)(const mrpt::Clock::time_point &, double, struct mrpt::math::TPose3D &)) &mrpt::poses::CPoseInterpolatorBase<3>::getPreviousPoseWithMinDistance, "C++: mrpt::poses::CPoseInterpolatorBase<3>::getPreviousPoseWithMinDistance(const mrpt::Clock::time_point &, double, struct mrpt::math::TPose3D &) --> bool", pybind11::arg("t"), pybind11::arg("distance"), pybind11::arg("out_pose"));
		cl.def("getPreviousPoseWithMinDistance", (bool (mrpt::poses::CPoseInterpolatorBase<3>::*)(const mrpt::Clock::time_point &, double, class mrpt::poses::CPose3D &)) &mrpt::poses::CPoseInterpolatorBase<3>::getPreviousPoseWithMinDistance, "C++: mrpt::poses::CPoseInterpolatorBase<3>::getPreviousPoseWithMinDistance(const mrpt::Clock::time_point &, double, class mrpt::poses::CPose3D &) --> bool", pybind11::arg("t"), pybind11::arg("distance"), pybind11::arg("out_pose"));
		cl.def("saveToTextFile", (bool (mrpt::poses::CPoseInterpolatorBase<3>::*)(const std::string &) const) &mrpt::poses::CPoseInterpolatorBase<3>::saveToTextFile, "C++: mrpt::poses::CPoseInterpolatorBase<3>::saveToTextFile(const std::string &) const --> bool", pybind11::arg("s"));
		cl.def("saveToTextFile_TUM", (bool (mrpt::poses::CPoseInterpolatorBase<3>::*)(const std::string &) const) &mrpt::poses::CPoseInterpolatorBase<3>::saveToTextFile_TUM, "C++: mrpt::poses::CPoseInterpolatorBase<3>::saveToTextFile_TUM(const std::string &) const --> bool", pybind11::arg("s"));
		cl.def("saveInterpolatedToTextFile", (bool (mrpt::poses::CPoseInterpolatorBase<3>::*)(const std::string &, const struct std::chrono::duration<int64_t,struct std::ratio<1,10000000>> &) const) &mrpt::poses::CPoseInterpolatorBase<3>::saveInterpolatedToTextFile, "C++: mrpt::poses::CPoseInterpolatorBase<3>::saveInterpolatedToTextFile(const std::string &, const struct std::chrono::duration<int64_t,struct std::ratio<1,10000000>> &) const --> bool", pybind11::arg("s"), pybind11::arg("period"));
		cl.def("loadFromTextFile", (bool (mrpt::poses::CPoseInterpolatorBase<3>::*)(const std::string &)) &mrpt::poses::CPoseInterpolatorBase<3>::loadFromTextFile, "C++: mrpt::poses::CPoseInterpolatorBase<3>::loadFromTextFile(const std::string &) --> bool", pybind11::arg("s"));
		cl.def("loadFromTextFile_TUM", (bool (mrpt::poses::CPoseInterpolatorBase<3>::*)(const std::string &)) &mrpt::poses::CPoseInterpolatorBase<3>::loadFromTextFile_TUM, "C++: mrpt::poses::CPoseInterpolatorBase<3>::loadFromTextFile_TUM(const std::string &) --> bool", pybind11::arg("s"));
		cl.def("getBoundingBox", (void (mrpt::poses::CPoseInterpolatorBase<3>::*)(struct mrpt::math::TPoint3D_<double> &, struct mrpt::math::TPoint3D_<double> &) const) &mrpt::poses::CPoseInterpolatorBase<3>::getBoundingBox, "C++: mrpt::poses::CPoseInterpolatorBase<3>::getBoundingBox(struct mrpt::math::TPoint3D_<double> &, struct mrpt::math::TPoint3D_<double> &) const --> void", pybind11::arg("minCorner"), pybind11::arg("maxCorner"));
		cl.def("setInterpolationMethod", (void (mrpt::poses::CPoseInterpolatorBase<3>::*)(enum mrpt::poses::TInterpolatorMethod)) &mrpt::poses::CPoseInterpolatorBase<3>::setInterpolationMethod, "C++: mrpt::poses::CPoseInterpolatorBase<3>::setInterpolationMethod(enum mrpt::poses::TInterpolatorMethod) --> void", pybind11::arg("method"));
		cl.def("getInterpolationMethod", (enum mrpt::poses::TInterpolatorMethod (mrpt::poses::CPoseInterpolatorBase<3>::*)() const) &mrpt::poses::CPoseInterpolatorBase<3>::getInterpolationMethod, "C++: mrpt::poses::CPoseInterpolatorBase<3>::getInterpolationMethod() const --> enum mrpt::poses::TInterpolatorMethod");
		cl.def("filter", (void (mrpt::poses::CPoseInterpolatorBase<3>::*)(unsigned int, unsigned int)) &mrpt::poses::CPoseInterpolatorBase<3>::filter, "C++: mrpt::poses::CPoseInterpolatorBase<3>::filter(unsigned int, unsigned int) --> void", pybind11::arg("component"), pybind11::arg("samples"));
		cl.def("assign", (class mrpt::poses::CPoseInterpolatorBase<3> & (mrpt::poses::CPoseInterpolatorBase<3>::*)(const class mrpt::poses::CPoseInterpolatorBase<3> &)) &mrpt::poses::CPoseInterpolatorBase<3>::operator=, "C++: mrpt::poses::CPoseInterpolatorBase<3>::operator=(const class mrpt::poses::CPoseInterpolatorBase<3> &) --> class mrpt::poses::CPoseInterpolatorBase<3> &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
