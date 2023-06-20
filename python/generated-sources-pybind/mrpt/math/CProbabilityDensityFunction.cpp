#include <iterator>
#include <memory>
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CProbabilityDensityFunction.h>
#include <mrpt/math/CQuaternion.h>
#include <mrpt/math/CVectorDynamic.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/math/TPose3DQuat.h>
#include <mrpt/math/math_frwds.h>
#include <mrpt/math/matrix_size_t.h>
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
#include <tuple>
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

// mrpt::math::CProbabilityDensityFunction file:mrpt/math/CProbabilityDensityFunction.h line:30
struct PyCallBack_mrpt_math_CProbabilityDensityFunction_mrpt_poses_CPose2D_3UL_t : public mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPose2D,3UL> {
	using mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPose2D,3UL>::CProbabilityDensityFunction;

	void getMean(class mrpt::poses::CPose2D & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPose2D,3UL> *>(this), "getMean");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CProbabilityDensityFunction::getMean\"");
	}
	using _binder_ret_0 = class std::tuple<class mrpt::math::CMatrixFixed<double, 3, 3>, class mrpt::poses::CPose2D>;
	_binder_ret_0 getCovarianceAndMean() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPose2D,3UL> *>(this), "getCovarianceAndMean");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<_binder_ret_0>::value) {
				static pybind11::detail::override_caster_t<_binder_ret_0> caster;
				return pybind11::detail::cast_ref<_binder_ret_0>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<_binder_ret_0>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CProbabilityDensityFunction::getCovarianceAndMean\"");
	}
	bool isInfType() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPose2D,3UL> *>(this), "isInfType");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CProbabilityDensityFunction::isInfType();
	}
	void getInformationMatrix(class mrpt::math::CMatrixFixed<double, 3, 3> & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPose2D,3UL> *>(this), "getInformationMatrix");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CProbabilityDensityFunction::getInformationMatrix(a0);
	}
	bool saveToTextFile(const std::string & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPose2D,3UL> *>(this), "saveToTextFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CProbabilityDensityFunction::saveToTextFile\"");
	}
	void drawSingleSample(class mrpt::poses::CPose2D & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPose2D,3UL> *>(this), "drawSingleSample");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CProbabilityDensityFunction::drawSingleSample\"");
	}
};

// mrpt::math::CProbabilityDensityFunction file:mrpt/math/CProbabilityDensityFunction.h line:30
struct PyCallBack_mrpt_math_CProbabilityDensityFunction_mrpt_poses_CPose3D_6UL_t : public mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPose3D,6UL> {
	using mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPose3D,6UL>::CProbabilityDensityFunction;

	void getMean(class mrpt::poses::CPose3D & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPose3D,6UL> *>(this), "getMean");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CProbabilityDensityFunction::getMean\"");
	}
	using _binder_ret_0 = class std::tuple<class mrpt::math::CMatrixFixed<double, 6, 6>, class mrpt::poses::CPose3D>;
	_binder_ret_0 getCovarianceAndMean() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPose3D,6UL> *>(this), "getCovarianceAndMean");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<_binder_ret_0>::value) {
				static pybind11::detail::override_caster_t<_binder_ret_0> caster;
				return pybind11::detail::cast_ref<_binder_ret_0>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<_binder_ret_0>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CProbabilityDensityFunction::getCovarianceAndMean\"");
	}
	bool isInfType() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPose3D,6UL> *>(this), "isInfType");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CProbabilityDensityFunction::isInfType();
	}
	void getInformationMatrix(class mrpt::math::CMatrixFixed<double, 6, 6> & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPose3D,6UL> *>(this), "getInformationMatrix");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CProbabilityDensityFunction::getInformationMatrix(a0);
	}
	bool saveToTextFile(const std::string & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPose3D,6UL> *>(this), "saveToTextFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CProbabilityDensityFunction::saveToTextFile\"");
	}
	void drawSingleSample(class mrpt::poses::CPose3D & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPose3D,6UL> *>(this), "drawSingleSample");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CProbabilityDensityFunction::drawSingleSample\"");
	}
};

// mrpt::math::CProbabilityDensityFunction file:mrpt/math/CProbabilityDensityFunction.h line:30
struct PyCallBack_mrpt_math_CProbabilityDensityFunction_mrpt_poses_CPoint3D_3UL_t : public mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPoint3D,3UL> {
	using mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPoint3D,3UL>::CProbabilityDensityFunction;

	void getMean(class mrpt::poses::CPoint3D & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPoint3D,3UL> *>(this), "getMean");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CProbabilityDensityFunction::getMean\"");
	}
	using _binder_ret_0 = class std::tuple<class mrpt::math::CMatrixFixed<double, 3, 3>, class mrpt::poses::CPoint3D>;
	_binder_ret_0 getCovarianceAndMean() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPoint3D,3UL> *>(this), "getCovarianceAndMean");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<_binder_ret_0>::value) {
				static pybind11::detail::override_caster_t<_binder_ret_0> caster;
				return pybind11::detail::cast_ref<_binder_ret_0>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<_binder_ret_0>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CProbabilityDensityFunction::getCovarianceAndMean\"");
	}
	bool isInfType() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPoint3D,3UL> *>(this), "isInfType");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CProbabilityDensityFunction::isInfType();
	}
	void getInformationMatrix(class mrpt::math::CMatrixFixed<double, 3, 3> & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPoint3D,3UL> *>(this), "getInformationMatrix");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CProbabilityDensityFunction::getInformationMatrix(a0);
	}
	bool saveToTextFile(const std::string & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPoint3D,3UL> *>(this), "saveToTextFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CProbabilityDensityFunction::saveToTextFile\"");
	}
	void drawSingleSample(class mrpt::poses::CPoint3D & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPoint3D,3UL> *>(this), "drawSingleSample");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CProbabilityDensityFunction::drawSingleSample\"");
	}
};

void bind_mrpt_math_CProbabilityDensityFunction(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::math::CProbabilityDensityFunction file:mrpt/math/CProbabilityDensityFunction.h line:30
		pybind11::class_<mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPose2D,3UL>, std::shared_ptr<mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPose2D,3UL>>, PyCallBack_mrpt_math_CProbabilityDensityFunction_mrpt_poses_CPose2D_3UL_t> cl(M("mrpt::math"), "CProbabilityDensityFunction_mrpt_poses_CPose2D_3UL_t", "");
		cl.def(pybind11::init<PyCallBack_mrpt_math_CProbabilityDensityFunction_mrpt_poses_CPose2D_3UL_t const &>());
		cl.def( pybind11::init( [](){ return new PyCallBack_mrpt_math_CProbabilityDensityFunction_mrpt_poses_CPose2D_3UL_t(); } ) );
		cl.def("getMean", (void (mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPose2D,3UL>::*)(class mrpt::poses::CPose2D &) const) &mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPose2D, 3>::getMean, "C++: mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPose2D, 3>::getMean(class mrpt::poses::CPose2D &) const --> void", pybind11::arg("mean_point"));
		cl.def("getCovarianceAndMean", (class std::tuple<class mrpt::math::CMatrixFixed<double, 3, 3>, class mrpt::poses::CPose2D> (mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPose2D,3UL>::*)() const) &mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPose2D, 3>::getCovarianceAndMean, "C++: mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPose2D, 3>::getCovarianceAndMean() const --> class std::tuple<class mrpt::math::CMatrixFixed<double, 3, 3>, class mrpt::poses::CPose2D>");
		cl.def("getCovarianceAndMean", (void (mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPose2D,3UL>::*)(class mrpt::math::CMatrixFixed<double, 3, 3> &, class mrpt::poses::CPose2D &) const) &mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPose2D, 3>::getCovarianceAndMean, "C++: mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPose2D, 3>::getCovarianceAndMean(class mrpt::math::CMatrixFixed<double, 3, 3> &, class mrpt::poses::CPose2D &) const --> void", pybind11::arg("c"), pybind11::arg("mean"));
		cl.def("getCovarianceDynAndMean", (void (mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPose2D,3UL>::*)(class mrpt::math::CMatrixDynamic<double> &, class mrpt::poses::CPose2D &) const) &mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPose2D, 3>::getCovarianceDynAndMean, "C++: mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPose2D, 3>::getCovarianceDynAndMean(class mrpt::math::CMatrixDynamic<double> &, class mrpt::poses::CPose2D &) const --> void", pybind11::arg("cov"), pybind11::arg("mean_point"));
		cl.def("getMeanVal", (class mrpt::poses::CPose2D (mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPose2D,3UL>::*)() const) &mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPose2D, 3>::getMeanVal, "C++: mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPose2D, 3>::getMeanVal() const --> class mrpt::poses::CPose2D");
		cl.def("getCovariance", (void (mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPose2D,3UL>::*)(class mrpt::math::CMatrixDynamic<double> &) const) &mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPose2D, 3>::getCovariance, "C++: mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPose2D, 3>::getCovariance(class mrpt::math::CMatrixDynamic<double> &) const --> void", pybind11::arg("cov"));
		cl.def("getCovariance", (void (mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPose2D,3UL>::*)(class mrpt::math::CMatrixFixed<double, 3, 3> &) const) &mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPose2D, 3>::getCovariance, "C++: mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPose2D, 3>::getCovariance(class mrpt::math::CMatrixFixed<double, 3, 3> &) const --> void", pybind11::arg("cov"));
		cl.def("getCovariance", (class mrpt::math::CMatrixFixed<double, 3, 3> (mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPose2D,3UL>::*)() const) &mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPose2D, 3>::getCovariance, "C++: mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPose2D, 3>::getCovariance() const --> class mrpt::math::CMatrixFixed<double, 3, 3>");
		cl.def("isInfType", (bool (mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPose2D,3UL>::*)() const) &mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPose2D, 3>::isInfType, "C++: mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPose2D, 3>::isInfType() const --> bool");
		cl.def("getInformationMatrix", (void (mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPose2D,3UL>::*)(class mrpt::math::CMatrixFixed<double, 3, 3> &) const) &mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPose2D, 3>::getInformationMatrix, "C++: mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPose2D, 3>::getInformationMatrix(class mrpt::math::CMatrixFixed<double, 3, 3> &) const --> void", pybind11::arg("inf"));
		cl.def("saveToTextFile", (bool (mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPose2D,3UL>::*)(const std::string &) const) &mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPose2D, 3>::saveToTextFile, "C++: mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPose2D, 3>::saveToTextFile(const std::string &) const --> bool", pybind11::arg("file"));
		cl.def("drawSingleSample", (void (mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPose2D,3UL>::*)(class mrpt::poses::CPose2D &) const) &mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPose2D, 3>::drawSingleSample, "C++: mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPose2D, 3>::drawSingleSample(class mrpt::poses::CPose2D &) const --> void", pybind11::arg("outPart"));
		cl.def("getCovarianceEntropy", (double (mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPose2D,3UL>::*)() const) &mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPose2D, 3>::getCovarianceEntropy, "C++: mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPose2D, 3>::getCovarianceEntropy() const --> double");
		cl.def("assign", (class mrpt::math::CProbabilityDensityFunction<class mrpt::poses::CPose2D, 3> & (mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPose2D,3UL>::*)(const class mrpt::math::CProbabilityDensityFunction<class mrpt::poses::CPose2D, 3> &)) &mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPose2D, 3>::operator=, "C++: mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPose2D, 3>::operator=(const class mrpt::math::CProbabilityDensityFunction<class mrpt::poses::CPose2D, 3> &) --> class mrpt::math::CProbabilityDensityFunction<class mrpt::poses::CPose2D, 3> &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::math::CProbabilityDensityFunction file:mrpt/math/CProbabilityDensityFunction.h line:30
		pybind11::class_<mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPose3D,6UL>, std::shared_ptr<mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPose3D,6UL>>, PyCallBack_mrpt_math_CProbabilityDensityFunction_mrpt_poses_CPose3D_6UL_t> cl(M("mrpt::math"), "CProbabilityDensityFunction_mrpt_poses_CPose3D_6UL_t", "");
		cl.def(pybind11::init<PyCallBack_mrpt_math_CProbabilityDensityFunction_mrpt_poses_CPose3D_6UL_t const &>());
		cl.def( pybind11::init( [](){ return new PyCallBack_mrpt_math_CProbabilityDensityFunction_mrpt_poses_CPose3D_6UL_t(); } ) );
		cl.def("getMean", (void (mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPose3D,6UL>::*)(class mrpt::poses::CPose3D &) const) &mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPose3D, 6>::getMean, "C++: mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPose3D, 6>::getMean(class mrpt::poses::CPose3D &) const --> void", pybind11::arg("mean_point"));
		cl.def("getCovarianceAndMean", (class std::tuple<class mrpt::math::CMatrixFixed<double, 6, 6>, class mrpt::poses::CPose3D> (mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPose3D,6UL>::*)() const) &mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPose3D, 6>::getCovarianceAndMean, "C++: mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPose3D, 6>::getCovarianceAndMean() const --> class std::tuple<class mrpt::math::CMatrixFixed<double, 6, 6>, class mrpt::poses::CPose3D>");
		cl.def("getCovarianceAndMean", (void (mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPose3D,6UL>::*)(class mrpt::math::CMatrixFixed<double, 6, 6> &, class mrpt::poses::CPose3D &) const) &mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPose3D, 6>::getCovarianceAndMean, "C++: mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPose3D, 6>::getCovarianceAndMean(class mrpt::math::CMatrixFixed<double, 6, 6> &, class mrpt::poses::CPose3D &) const --> void", pybind11::arg("c"), pybind11::arg("mean"));
		cl.def("getCovarianceDynAndMean", (void (mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPose3D,6UL>::*)(class mrpt::math::CMatrixDynamic<double> &, class mrpt::poses::CPose3D &) const) &mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPose3D, 6>::getCovarianceDynAndMean, "C++: mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPose3D, 6>::getCovarianceDynAndMean(class mrpt::math::CMatrixDynamic<double> &, class mrpt::poses::CPose3D &) const --> void", pybind11::arg("cov"), pybind11::arg("mean_point"));
		cl.def("getMeanVal", (class mrpt::poses::CPose3D (mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPose3D,6UL>::*)() const) &mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPose3D, 6>::getMeanVal, "C++: mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPose3D, 6>::getMeanVal() const --> class mrpt::poses::CPose3D");
		cl.def("getCovariance", (void (mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPose3D,6UL>::*)(class mrpt::math::CMatrixDynamic<double> &) const) &mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPose3D, 6>::getCovariance, "C++: mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPose3D, 6>::getCovariance(class mrpt::math::CMatrixDynamic<double> &) const --> void", pybind11::arg("cov"));
		cl.def("getCovariance", (void (mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPose3D,6UL>::*)(class mrpt::math::CMatrixFixed<double, 6, 6> &) const) &mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPose3D, 6>::getCovariance, "C++: mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPose3D, 6>::getCovariance(class mrpt::math::CMatrixFixed<double, 6, 6> &) const --> void", pybind11::arg("cov"));
		cl.def("getCovariance", (class mrpt::math::CMatrixFixed<double, 6, 6> (mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPose3D,6UL>::*)() const) &mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPose3D, 6>::getCovariance, "C++: mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPose3D, 6>::getCovariance() const --> class mrpt::math::CMatrixFixed<double, 6, 6>");
		cl.def("isInfType", (bool (mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPose3D,6UL>::*)() const) &mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPose3D, 6>::isInfType, "C++: mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPose3D, 6>::isInfType() const --> bool");
		cl.def("getInformationMatrix", (void (mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPose3D,6UL>::*)(class mrpt::math::CMatrixFixed<double, 6, 6> &) const) &mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPose3D, 6>::getInformationMatrix, "C++: mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPose3D, 6>::getInformationMatrix(class mrpt::math::CMatrixFixed<double, 6, 6> &) const --> void", pybind11::arg("inf"));
		cl.def("saveToTextFile", (bool (mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPose3D,6UL>::*)(const std::string &) const) &mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPose3D, 6>::saveToTextFile, "C++: mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPose3D, 6>::saveToTextFile(const std::string &) const --> bool", pybind11::arg("file"));
		cl.def("drawSingleSample", (void (mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPose3D,6UL>::*)(class mrpt::poses::CPose3D &) const) &mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPose3D, 6>::drawSingleSample, "C++: mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPose3D, 6>::drawSingleSample(class mrpt::poses::CPose3D &) const --> void", pybind11::arg("outPart"));
		cl.def("getCovarianceEntropy", (double (mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPose3D,6UL>::*)() const) &mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPose3D, 6>::getCovarianceEntropy, "C++: mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPose3D, 6>::getCovarianceEntropy() const --> double");
		cl.def("assign", (class mrpt::math::CProbabilityDensityFunction<class mrpt::poses::CPose3D, 6> & (mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPose3D,6UL>::*)(const class mrpt::math::CProbabilityDensityFunction<class mrpt::poses::CPose3D, 6> &)) &mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPose3D, 6>::operator=, "C++: mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPose3D, 6>::operator=(const class mrpt::math::CProbabilityDensityFunction<class mrpt::poses::CPose3D, 6> &) --> class mrpt::math::CProbabilityDensityFunction<class mrpt::poses::CPose3D, 6> &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::math::CProbabilityDensityFunction file:mrpt/math/CProbabilityDensityFunction.h line:30
		pybind11::class_<mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPoint3D,3UL>, std::shared_ptr<mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPoint3D,3UL>>, PyCallBack_mrpt_math_CProbabilityDensityFunction_mrpt_poses_CPoint3D_3UL_t> cl(M("mrpt::math"), "CProbabilityDensityFunction_mrpt_poses_CPoint3D_3UL_t", "");
		cl.def(pybind11::init<PyCallBack_mrpt_math_CProbabilityDensityFunction_mrpt_poses_CPoint3D_3UL_t const &>());
		cl.def( pybind11::init( [](){ return new PyCallBack_mrpt_math_CProbabilityDensityFunction_mrpt_poses_CPoint3D_3UL_t(); } ) );
		cl.def("getMean", (void (mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPoint3D,3UL>::*)(class mrpt::poses::CPoint3D &) const) &mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPoint3D, 3>::getMean, "C++: mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPoint3D, 3>::getMean(class mrpt::poses::CPoint3D &) const --> void", pybind11::arg("mean_point"));
		cl.def("getCovarianceAndMean", (class std::tuple<class mrpt::math::CMatrixFixed<double, 3, 3>, class mrpt::poses::CPoint3D> (mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPoint3D,3UL>::*)() const) &mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPoint3D, 3>::getCovarianceAndMean, "C++: mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPoint3D, 3>::getCovarianceAndMean() const --> class std::tuple<class mrpt::math::CMatrixFixed<double, 3, 3>, class mrpt::poses::CPoint3D>");
		cl.def("getCovarianceAndMean", (void (mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPoint3D,3UL>::*)(class mrpt::math::CMatrixFixed<double, 3, 3> &, class mrpt::poses::CPoint3D &) const) &mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPoint3D, 3>::getCovarianceAndMean, "C++: mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPoint3D, 3>::getCovarianceAndMean(class mrpt::math::CMatrixFixed<double, 3, 3> &, class mrpt::poses::CPoint3D &) const --> void", pybind11::arg("c"), pybind11::arg("mean"));
		cl.def("getCovarianceDynAndMean", (void (mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPoint3D,3UL>::*)(class mrpt::math::CMatrixDynamic<double> &, class mrpt::poses::CPoint3D &) const) &mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPoint3D, 3>::getCovarianceDynAndMean, "C++: mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPoint3D, 3>::getCovarianceDynAndMean(class mrpt::math::CMatrixDynamic<double> &, class mrpt::poses::CPoint3D &) const --> void", pybind11::arg("cov"), pybind11::arg("mean_point"));
		cl.def("getMeanVal", (class mrpt::poses::CPoint3D (mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPoint3D,3UL>::*)() const) &mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPoint3D, 3>::getMeanVal, "C++: mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPoint3D, 3>::getMeanVal() const --> class mrpt::poses::CPoint3D");
		cl.def("getCovariance", (void (mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPoint3D,3UL>::*)(class mrpt::math::CMatrixDynamic<double> &) const) &mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPoint3D, 3>::getCovariance, "C++: mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPoint3D, 3>::getCovariance(class mrpt::math::CMatrixDynamic<double> &) const --> void", pybind11::arg("cov"));
		cl.def("getCovariance", (void (mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPoint3D,3UL>::*)(class mrpt::math::CMatrixFixed<double, 3, 3> &) const) &mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPoint3D, 3>::getCovariance, "C++: mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPoint3D, 3>::getCovariance(class mrpt::math::CMatrixFixed<double, 3, 3> &) const --> void", pybind11::arg("cov"));
		cl.def("getCovariance", (class mrpt::math::CMatrixFixed<double, 3, 3> (mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPoint3D,3UL>::*)() const) &mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPoint3D, 3>::getCovariance, "C++: mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPoint3D, 3>::getCovariance() const --> class mrpt::math::CMatrixFixed<double, 3, 3>");
		cl.def("isInfType", (bool (mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPoint3D,3UL>::*)() const) &mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPoint3D, 3>::isInfType, "C++: mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPoint3D, 3>::isInfType() const --> bool");
		cl.def("getInformationMatrix", (void (mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPoint3D,3UL>::*)(class mrpt::math::CMatrixFixed<double, 3, 3> &) const) &mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPoint3D, 3>::getInformationMatrix, "C++: mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPoint3D, 3>::getInformationMatrix(class mrpt::math::CMatrixFixed<double, 3, 3> &) const --> void", pybind11::arg("inf"));
		cl.def("saveToTextFile", (bool (mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPoint3D,3UL>::*)(const std::string &) const) &mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPoint3D, 3>::saveToTextFile, "C++: mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPoint3D, 3>::saveToTextFile(const std::string &) const --> bool", pybind11::arg("file"));
		cl.def("drawSingleSample", (void (mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPoint3D,3UL>::*)(class mrpt::poses::CPoint3D &) const) &mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPoint3D, 3>::drawSingleSample, "C++: mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPoint3D, 3>::drawSingleSample(class mrpt::poses::CPoint3D &) const --> void", pybind11::arg("outPart"));
		cl.def("getCovarianceEntropy", (double (mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPoint3D,3UL>::*)() const) &mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPoint3D, 3>::getCovarianceEntropy, "C++: mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPoint3D, 3>::getCovarianceEntropy() const --> double");
		cl.def("assign", (class mrpt::math::CProbabilityDensityFunction<class mrpt::poses::CPoint3D, 3> & (mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPoint3D,3UL>::*)(const class mrpt::math::CProbabilityDensityFunction<class mrpt::poses::CPoint3D, 3> &)) &mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPoint3D, 3>::operator=, "C++: mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPoint3D, 3>::operator=(const class mrpt::math::CProbabilityDensityFunction<class mrpt::poses::CPoint3D, 3> &) --> class mrpt::math::CProbabilityDensityFunction<class mrpt::poses::CPoint3D, 3> &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
