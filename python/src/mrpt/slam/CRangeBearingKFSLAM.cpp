#include <functional>
#include <ios>
#include <iterator>
#include <map>
#include <memory>
#include <mrpt/bayes/CKalmanFilterCapable.h>
#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CQuaternion.h>
#include <mrpt/math/CVectorDynamic.h>
#include <mrpt/math/MatrixVectorBase.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPose3DQuat.h>
#include <mrpt/math/math_frwds.h>
#include <mrpt/math/matrix_size_t.h>
#include <mrpt/obs/CActionCollection.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/poses/CPoint2DPDFGaussian.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DPDF.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/poses/CPose3DQuat.h>
#include <mrpt/poses/CPose3DQuatPDF.h>
#include <mrpt/poses/CPose3DQuatPDFGaussian.h>
#include <mrpt/poses/CPoseOrPoint.h>
#include <mrpt/poses/CPosePDF.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/slam/CIncrementalMapPartitioner.h>
#include <mrpt/slam/CRangeBearingKFSLAM.h>
#include <mrpt/slam/CRangeBearingKFSLAM2D.h>
#include <mrpt/system/COutputLogger.h>
#include <mrpt/typemeta/static_string.h>
#include <ostream>
#include <sstream> // __str__
#include <streambuf>
#include <string>
#include <tuple>
#include <utility>
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

// mrpt::slam::CRangeBearingKFSLAM file:mrpt/slam/CRangeBearingKFSLAM.h line:48
struct PyCallBack_mrpt_slam_CRangeBearingKFSLAM : public mrpt::slam::CRangeBearingKFSLAM {
	using mrpt::slam::CRangeBearingKFSLAM::CRangeBearingKFSLAM;

	void OnGetAction(class mrpt::math::CMatrixFixed<double, 7, 1> & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CRangeBearingKFSLAM *>(this), "OnGetAction");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CRangeBearingKFSLAM::OnGetAction(a0);
	}
	void OnTransitionModel(const class mrpt::math::CMatrixFixed<double, 7, 1> & a0, class mrpt::math::CMatrixFixed<double, 7, 1> & a1, bool & a2) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CRangeBearingKFSLAM *>(this), "OnTransitionModel");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CRangeBearingKFSLAM::OnTransitionModel(a0, a1, a2);
	}
	void OnTransitionJacobian(class mrpt::math::CMatrixFixed<double, 7, 7> & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CRangeBearingKFSLAM *>(this), "OnTransitionJacobian");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CRangeBearingKFSLAM::OnTransitionJacobian(a0);
	}
	void OnTransitionNoise(class mrpt::math::CMatrixFixed<double, 7, 7> & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CRangeBearingKFSLAM *>(this), "OnTransitionNoise");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CRangeBearingKFSLAM::OnTransitionNoise(a0);
	}
	void OnSubstractObservationVectors(class mrpt::math::CMatrixFixed<double, 3, 1> & a0, const class mrpt::math::CMatrixFixed<double, 3, 1> & a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CRangeBearingKFSLAM *>(this), "OnSubstractObservationVectors");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CRangeBearingKFSLAM::OnSubstractObservationVectors(a0, a1);
	}
	void OnGetObservationNoise(class mrpt::math::CMatrixFixed<double, 3, 3> & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CRangeBearingKFSLAM *>(this), "OnGetObservationNoise");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CRangeBearingKFSLAM::OnGetObservationNoise(a0);
	}
	void OnNewLandmarkAddedToMap(size_t a0, size_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CRangeBearingKFSLAM *>(this), "OnNewLandmarkAddedToMap");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CRangeBearingKFSLAM::OnNewLandmarkAddedToMap(a0, a1);
	}
	void OnNormalizeStateVector() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CRangeBearingKFSLAM *>(this), "OnNormalizeStateVector");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CRangeBearingKFSLAM::OnNormalizeStateVector();
	}
	void OnTransitionJacobianNumericGetIncrements(class mrpt::math::CMatrixFixed<double, 7, 1> & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CRangeBearingKFSLAM *>(this), "OnTransitionJacobianNumericGetIncrements");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CKalmanFilterCapable::OnTransitionJacobianNumericGetIncrements(a0);
	}
	void OnObservationJacobiansNumericGetIncrements(class mrpt::math::CMatrixFixed<double, 7, 1> & a0, class mrpt::math::CMatrixFixed<double, 3, 1> & a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CRangeBearingKFSLAM *>(this), "OnObservationJacobiansNumericGetIncrements");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CKalmanFilterCapable::OnObservationJacobiansNumericGetIncrements(a0, a1);
	}
	void OnPostIteration() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CRangeBearingKFSLAM *>(this), "OnPostIteration");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CKalmanFilterCapable::OnPostIteration();
	}
};

// mrpt::slam::CRangeBearingKFSLAM::TOptions file:mrpt/slam/CRangeBearingKFSLAM.h line:158
struct PyCallBack_mrpt_slam_CRangeBearingKFSLAM_TOptions : public mrpt::slam::CRangeBearingKFSLAM::TOptions {
	using mrpt::slam::CRangeBearingKFSLAM::TOptions::TOptions;

	void loadFromConfigFile(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CRangeBearingKFSLAM::TOptions *>(this), "loadFromConfigFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return TOptions::loadFromConfigFile(a0, a1);
	}
	void saveToConfigFile(class mrpt::config::CConfigFileBase & a0, const std::string & a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CRangeBearingKFSLAM::TOptions *>(this), "saveToConfigFile");
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

// mrpt::slam::CRangeBearingKFSLAM2D file:mrpt/slam/CRangeBearingKFSLAM2D.h line:40
struct PyCallBack_mrpt_slam_CRangeBearingKFSLAM2D : public mrpt::slam::CRangeBearingKFSLAM2D {
	using mrpt::slam::CRangeBearingKFSLAM2D::CRangeBearingKFSLAM2D;

	void OnGetAction(class mrpt::math::CMatrixFixed<double, 3, 1> & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CRangeBearingKFSLAM2D *>(this), "OnGetAction");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CRangeBearingKFSLAM2D::OnGetAction(a0);
	}
	void OnTransitionModel(const class mrpt::math::CMatrixFixed<double, 3, 1> & a0, class mrpt::math::CMatrixFixed<double, 3, 1> & a1, bool & a2) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CRangeBearingKFSLAM2D *>(this), "OnTransitionModel");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CRangeBearingKFSLAM2D::OnTransitionModel(a0, a1, a2);
	}
	void OnTransitionJacobian(class mrpt::math::CMatrixFixed<double, 3, 3> & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CRangeBearingKFSLAM2D *>(this), "OnTransitionJacobian");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CRangeBearingKFSLAM2D::OnTransitionJacobian(a0);
	}
	void OnTransitionJacobianNumericGetIncrements(class mrpt::math::CMatrixFixed<double, 3, 1> & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CRangeBearingKFSLAM2D *>(this), "OnTransitionJacobianNumericGetIncrements");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CRangeBearingKFSLAM2D::OnTransitionJacobianNumericGetIncrements(a0);
	}
	void OnTransitionNoise(class mrpt::math::CMatrixFixed<double, 3, 3> & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CRangeBearingKFSLAM2D *>(this), "OnTransitionNoise");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CRangeBearingKFSLAM2D::OnTransitionNoise(a0);
	}
	void OnObservationJacobiansNumericGetIncrements(class mrpt::math::CMatrixFixed<double, 3, 1> & a0, class mrpt::math::CMatrixFixed<double, 2, 1> & a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CRangeBearingKFSLAM2D *>(this), "OnObservationJacobiansNumericGetIncrements");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CRangeBearingKFSLAM2D::OnObservationJacobiansNumericGetIncrements(a0, a1);
	}
	void OnSubstractObservationVectors(class mrpt::math::CMatrixFixed<double, 2, 1> & a0, const class mrpt::math::CMatrixFixed<double, 2, 1> & a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CRangeBearingKFSLAM2D *>(this), "OnSubstractObservationVectors");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CRangeBearingKFSLAM2D::OnSubstractObservationVectors(a0, a1);
	}
	void OnGetObservationNoise(class mrpt::math::CMatrixFixed<double, 2, 2> & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CRangeBearingKFSLAM2D *>(this), "OnGetObservationNoise");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CRangeBearingKFSLAM2D::OnGetObservationNoise(a0);
	}
	void OnNewLandmarkAddedToMap(size_t a0, size_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CRangeBearingKFSLAM2D *>(this), "OnNewLandmarkAddedToMap");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CRangeBearingKFSLAM2D::OnNewLandmarkAddedToMap(a0, a1);
	}
	void OnNormalizeStateVector() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CRangeBearingKFSLAM2D *>(this), "OnNormalizeStateVector");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CRangeBearingKFSLAM2D::OnNormalizeStateVector();
	}
	void OnPostIteration() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CRangeBearingKFSLAM2D *>(this), "OnPostIteration");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CKalmanFilterCapable::OnPostIteration();
	}
};

// mrpt::slam::CRangeBearingKFSLAM2D::TOptions file:mrpt/slam/CRangeBearingKFSLAM2D.h line:103
struct PyCallBack_mrpt_slam_CRangeBearingKFSLAM2D_TOptions : public mrpt::slam::CRangeBearingKFSLAM2D::TOptions {
	using mrpt::slam::CRangeBearingKFSLAM2D::TOptions::TOptions;

	void loadFromConfigFile(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CRangeBearingKFSLAM2D::TOptions *>(this), "loadFromConfigFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return TOptions::loadFromConfigFile(a0, a1);
	}
	void saveToConfigFile(class mrpt::config::CConfigFileBase & a0, const std::string & a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CRangeBearingKFSLAM2D::TOptions *>(this), "saveToConfigFile");
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

void bind_mrpt_slam_CRangeBearingKFSLAM(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::slam::CRangeBearingKFSLAM file:mrpt/slam/CRangeBearingKFSLAM.h line:48
		pybind11::class_<mrpt::slam::CRangeBearingKFSLAM, std::shared_ptr<mrpt::slam::CRangeBearingKFSLAM>, PyCallBack_mrpt_slam_CRangeBearingKFSLAM, mrpt::bayes::CKalmanFilterCapable<7UL,3UL,3UL,7UL,double>> cl(M("mrpt::slam"), "CRangeBearingKFSLAM", "An implementation of EKF-based SLAM with range-bearing sensors, odometry, a\n SE(3) robot pose, and 3D landmarks.\n The main method is processActionObservation() which processes pairs of\n actions/observations.\n\n The state vector comprises: 3D robot position, a quaternion for its\n attitude, and the 3D landmarks in the map.\n\n The front-end application [kf-slam](page_app_kf-slam.html) is based on this\n class.\n\n For the theory behind this implementation, see the technical report:\n \n\n \n An implementation for 2D and SE(2) is in CRangeBearingKFSLAM2D\n\n \n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::slam::CRangeBearingKFSLAM(); }, [](){ return new PyCallBack_mrpt_slam_CRangeBearingKFSLAM(); } ) );
		cl.def( pybind11::init( [](PyCallBack_mrpt_slam_CRangeBearingKFSLAM const &o){ return new PyCallBack_mrpt_slam_CRangeBearingKFSLAM(o); } ) );
		cl.def( pybind11::init( [](mrpt::slam::CRangeBearingKFSLAM const &o){ return new mrpt::slam::CRangeBearingKFSLAM(o); } ) );
		cl.def_readwrite("options", &mrpt::slam::CRangeBearingKFSLAM::options);
		cl.def("reset", (void (mrpt::slam::CRangeBearingKFSLAM::*)()) &mrpt::slam::CRangeBearingKFSLAM::reset, "Reset the state of the SLAM filter: The map is emptied and the robot put\n back to (0,0,0). \n\nC++: mrpt::slam::CRangeBearingKFSLAM::reset() --> void");
		cl.def("processActionObservation", (void (mrpt::slam::CRangeBearingKFSLAM::*)(class std::shared_ptr<class mrpt::obs::CActionCollection> &, class std::shared_ptr<class mrpt::obs::CSensoryFrame> &)) &mrpt::slam::CRangeBearingKFSLAM::processActionObservation, "Process one new action and observations to update the map and robot pose\nestimate. See the description of the class at the top of this page.\n  \n\n May contain odometry\n	\n\n The set of observations, must contain at least one\nCObservationBearingRange\n\nC++: mrpt::slam::CRangeBearingKFSLAM::processActionObservation(class std::shared_ptr<class mrpt::obs::CActionCollection> &, class std::shared_ptr<class mrpt::obs::CSensoryFrame> &) --> void", pybind11::arg("action"), pybind11::arg("SF"));
		cl.def("getCurrentRobotPose", (void (mrpt::slam::CRangeBearingKFSLAM::*)(class mrpt::poses::CPose3DQuatPDFGaussian &) const) &mrpt::slam::CRangeBearingKFSLAM::getCurrentRobotPose, "Returns the mean & the 7x7 covariance matrix of the robot 6D pose (with\n rotation as a quaternion).\n \n\n getCurrentState, getCurrentRobotPoseMean\n\nC++: mrpt::slam::CRangeBearingKFSLAM::getCurrentRobotPose(class mrpt::poses::CPose3DQuatPDFGaussian &) const --> void", pybind11::arg("out_robotPose"));
		cl.def("getCurrentRobotPoseMean", (class mrpt::poses::CPose3DQuat (mrpt::slam::CRangeBearingKFSLAM::*)() const) &mrpt::slam::CRangeBearingKFSLAM::getCurrentRobotPoseMean, "Get the current robot pose mean, as a 3D+quaternion pose.\n \n\n getCurrentRobotPose\n\nC++: mrpt::slam::CRangeBearingKFSLAM::getCurrentRobotPoseMean() const --> class mrpt::poses::CPose3DQuat");
		cl.def("getCurrentRobotPose", (void (mrpt::slam::CRangeBearingKFSLAM::*)(class mrpt::poses::CPose3DPDFGaussian &) const) &mrpt::slam::CRangeBearingKFSLAM::getCurrentRobotPose, "Returns the mean & the 6x6 covariance matrix of the robot 6D pose (with\n rotation as 3 angles).\n \n\n getCurrentState\n\nC++: mrpt::slam::CRangeBearingKFSLAM::getCurrentRobotPose(class mrpt::poses::CPose3DPDFGaussian &) const --> void", pybind11::arg("out_robotPose"));
		cl.def("getAs3DObject", (void (mrpt::slam::CRangeBearingKFSLAM::*)(class std::shared_ptr<class mrpt::opengl::CSetOfObjects> &) const) &mrpt::slam::CRangeBearingKFSLAM::getAs3DObject, "Returns a 3D representation of the landmarks in the map and the robot 3D\n position according to the current filter state.\n  \n\n\n   \n\nC++: mrpt::slam::CRangeBearingKFSLAM::getAs3DObject(class std::shared_ptr<class mrpt::opengl::CSetOfObjects> &) const --> void", pybind11::arg("outObj"));
		cl.def("loadOptions", (void (mrpt::slam::CRangeBearingKFSLAM::*)(const class mrpt::config::CConfigFileBase &)) &mrpt::slam::CRangeBearingKFSLAM::loadOptions, "Load options from a ini-like file/text\n\nC++: mrpt::slam::CRangeBearingKFSLAM::loadOptions(const class mrpt::config::CConfigFileBase &) --> void", pybind11::arg("ini"));
		cl.def("getLastDataAssociation", (const struct mrpt::slam::CRangeBearingKFSLAM::TDataAssocInfo & (mrpt::slam::CRangeBearingKFSLAM::*)() const) &mrpt::slam::CRangeBearingKFSLAM::getLastDataAssociation, "Returns a read-only reference to the information on the last\n data-association \n\nC++: mrpt::slam::CRangeBearingKFSLAM::getLastDataAssociation() const --> const struct mrpt::slam::CRangeBearingKFSLAM::TDataAssocInfo &", pybind11::return_value_policy::automatic);
		cl.def("reconsiderPartitionsNow", (void (mrpt::slam::CRangeBearingKFSLAM::*)()) &mrpt::slam::CRangeBearingKFSLAM::reconsiderPartitionsNow, "The partitioning of the entire map is recomputed again.\n  Only when options.doPartitioningExperiment = true.\n  This can be used after changing the parameters of the partitioning\n method.\n  After this method, you can call getLastPartitionLandmarks.\n \n\n getLastPartitionLandmarks\n\nC++: mrpt::slam::CRangeBearingKFSLAM::reconsiderPartitionsNow() --> void");
		cl.def("mapPartitionOptions", (struct mrpt::slam::CIncrementalMapPartitioner::TOptions * (mrpt::slam::CRangeBearingKFSLAM::*)()) &mrpt::slam::CRangeBearingKFSLAM::mapPartitionOptions, "Provides access to the parameters of the map partitioning algorithm.\n\nC++: mrpt::slam::CRangeBearingKFSLAM::mapPartitionOptions() --> struct mrpt::slam::CIncrementalMapPartitioner::TOptions *", pybind11::return_value_policy::automatic);
		cl.def("saveMapAndPath2DRepresentationAsMATLABFile", [](mrpt::slam::CRangeBearingKFSLAM const &o, const std::string & a0) -> void { return o.saveMapAndPath2DRepresentationAsMATLABFile(a0); }, "", pybind11::arg("fil"));
		cl.def("saveMapAndPath2DRepresentationAsMATLABFile", [](mrpt::slam::CRangeBearingKFSLAM const &o, const std::string & a0, float const & a1) -> void { return o.saveMapAndPath2DRepresentationAsMATLABFile(a0, a1); }, "", pybind11::arg("fil"), pybind11::arg("stdCount"));
		cl.def("saveMapAndPath2DRepresentationAsMATLABFile", [](mrpt::slam::CRangeBearingKFSLAM const &o, const std::string & a0, float const & a1, const std::string & a2) -> void { return o.saveMapAndPath2DRepresentationAsMATLABFile(a0, a1, a2); }, "", pybind11::arg("fil"), pybind11::arg("stdCount"), pybind11::arg("styleLandmarks"));
		cl.def("saveMapAndPath2DRepresentationAsMATLABFile", [](mrpt::slam::CRangeBearingKFSLAM const &o, const std::string & a0, float const & a1, const std::string & a2, const std::string & a3) -> void { return o.saveMapAndPath2DRepresentationAsMATLABFile(a0, a1, a2, a3); }, "", pybind11::arg("fil"), pybind11::arg("stdCount"), pybind11::arg("styleLandmarks"), pybind11::arg("stylePath"));
		cl.def("saveMapAndPath2DRepresentationAsMATLABFile", (void (mrpt::slam::CRangeBearingKFSLAM::*)(const std::string &, float, const std::string &, const std::string &, const std::string &) const) &mrpt::slam::CRangeBearingKFSLAM::saveMapAndPath2DRepresentationAsMATLABFile, "Save the current state of the filter (robot pose & map) to a MATLAB\n script which displays all the elements in 2D\n\nC++: mrpt::slam::CRangeBearingKFSLAM::saveMapAndPath2DRepresentationAsMATLABFile(const std::string &, float, const std::string &, const std::string &, const std::string &) const --> void", pybind11::arg("fil"), pybind11::arg("stdCount"), pybind11::arg("styleLandmarks"), pybind11::arg("stylePath"), pybind11::arg("styleRobot"));

		{ // mrpt::slam::CRangeBearingKFSLAM::TOptions file:mrpt/slam/CRangeBearingKFSLAM.h line:158
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::slam::CRangeBearingKFSLAM::TOptions, std::shared_ptr<mrpt::slam::CRangeBearingKFSLAM::TOptions>, PyCallBack_mrpt_slam_CRangeBearingKFSLAM_TOptions, mrpt::config::CLoadableOptions> cl(enclosing_class, "TOptions", "The options for the algorithm");
			cl.def( pybind11::init( [](){ return new mrpt::slam::CRangeBearingKFSLAM::TOptions(); }, [](){ return new PyCallBack_mrpt_slam_CRangeBearingKFSLAM_TOptions(); } ) );
			cl.def( pybind11::init( [](PyCallBack_mrpt_slam_CRangeBearingKFSLAM_TOptions const &o){ return new PyCallBack_mrpt_slam_CRangeBearingKFSLAM_TOptions(o); } ) );
			cl.def( pybind11::init( [](mrpt::slam::CRangeBearingKFSLAM::TOptions const &o){ return new mrpt::slam::CRangeBearingKFSLAM::TOptions(o); } ) );
			cl.def_readwrite("stds_Q_no_odo", &mrpt::slam::CRangeBearingKFSLAM::TOptions::stds_Q_no_odo);
			cl.def_readwrite("std_sensor_range", &mrpt::slam::CRangeBearingKFSLAM::TOptions::std_sensor_range);
			cl.def_readwrite("std_sensor_yaw", &mrpt::slam::CRangeBearingKFSLAM::TOptions::std_sensor_yaw);
			cl.def_readwrite("std_sensor_pitch", &mrpt::slam::CRangeBearingKFSLAM::TOptions::std_sensor_pitch);
			cl.def_readwrite("std_odo_z_additional", &mrpt::slam::CRangeBearingKFSLAM::TOptions::std_odo_z_additional);
			cl.def_readwrite("doPartitioningExperiment", &mrpt::slam::CRangeBearingKFSLAM::TOptions::doPartitioningExperiment);
			cl.def_readwrite("quantiles_3D_representation", &mrpt::slam::CRangeBearingKFSLAM::TOptions::quantiles_3D_representation);
			cl.def_readwrite("partitioningMethod", &mrpt::slam::CRangeBearingKFSLAM::TOptions::partitioningMethod);
			cl.def_readwrite("data_assoc_method", &mrpt::slam::CRangeBearingKFSLAM::TOptions::data_assoc_method);
			cl.def_readwrite("data_assoc_metric", &mrpt::slam::CRangeBearingKFSLAM::TOptions::data_assoc_metric);
			cl.def_readwrite("data_assoc_IC_chi2_thres", &mrpt::slam::CRangeBearingKFSLAM::TOptions::data_assoc_IC_chi2_thres);
			cl.def_readwrite("data_assoc_IC_metric", &mrpt::slam::CRangeBearingKFSLAM::TOptions::data_assoc_IC_metric);
			cl.def_readwrite("data_assoc_IC_ml_threshold", &mrpt::slam::CRangeBearingKFSLAM::TOptions::data_assoc_IC_ml_threshold);
			cl.def_readwrite("create_simplemap", &mrpt::slam::CRangeBearingKFSLAM::TOptions::create_simplemap);
			cl.def_readwrite("force_ignore_odometry", &mrpt::slam::CRangeBearingKFSLAM::TOptions::force_ignore_odometry);
			cl.def("loadFromConfigFile", (void (mrpt::slam::CRangeBearingKFSLAM::TOptions::*)(const class mrpt::config::CConfigFileBase &, const std::string &)) &mrpt::slam::CRangeBearingKFSLAM::TOptions::loadFromConfigFile, "C++: mrpt::slam::CRangeBearingKFSLAM::TOptions::loadFromConfigFile(const class mrpt::config::CConfigFileBase &, const std::string &) --> void", pybind11::arg("source"), pybind11::arg("section"));
			cl.def("assign", (struct mrpt::slam::CRangeBearingKFSLAM::TOptions & (mrpt::slam::CRangeBearingKFSLAM::TOptions::*)(const struct mrpt::slam::CRangeBearingKFSLAM::TOptions &)) &mrpt::slam::CRangeBearingKFSLAM::TOptions::operator=, "C++: mrpt::slam::CRangeBearingKFSLAM::TOptions::operator=(const struct mrpt::slam::CRangeBearingKFSLAM::TOptions &) --> struct mrpt::slam::CRangeBearingKFSLAM::TOptions &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

		{ // mrpt::slam::CRangeBearingKFSLAM::TDataAssocInfo file:mrpt/slam/CRangeBearingKFSLAM.h line:220
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::slam::CRangeBearingKFSLAM::TDataAssocInfo, std::shared_ptr<mrpt::slam::CRangeBearingKFSLAM::TDataAssocInfo>> cl(enclosing_class, "TDataAssocInfo", "Information for data-association:\n \n\n getLastDataAssociation");
			cl.def( pybind11::init( [](){ return new mrpt::slam::CRangeBearingKFSLAM::TDataAssocInfo(); } ) );
			cl.def( pybind11::init( [](mrpt::slam::CRangeBearingKFSLAM::TDataAssocInfo const &o){ return new mrpt::slam::CRangeBearingKFSLAM::TDataAssocInfo(o); } ) );
			cl.def_readwrite("Y_pred_means", &mrpt::slam::CRangeBearingKFSLAM::TDataAssocInfo::Y_pred_means);
			cl.def_readwrite("Y_pred_covs", &mrpt::slam::CRangeBearingKFSLAM::TDataAssocInfo::Y_pred_covs);
			cl.def_readwrite("predictions_IDs", &mrpt::slam::CRangeBearingKFSLAM::TDataAssocInfo::predictions_IDs);
			cl.def_readwrite("newly_inserted_landmarks", &mrpt::slam::CRangeBearingKFSLAM::TDataAssocInfo::newly_inserted_landmarks);
			cl.def_readwrite("results", &mrpt::slam::CRangeBearingKFSLAM::TDataAssocInfo::results);
			cl.def("clear", (void (mrpt::slam::CRangeBearingKFSLAM::TDataAssocInfo::*)()) &mrpt::slam::CRangeBearingKFSLAM::TDataAssocInfo::clear, "C++: mrpt::slam::CRangeBearingKFSLAM::TDataAssocInfo::clear() --> void");
			cl.def("assign", (struct mrpt::slam::CRangeBearingKFSLAM::TDataAssocInfo & (mrpt::slam::CRangeBearingKFSLAM::TDataAssocInfo::*)(const struct mrpt::slam::CRangeBearingKFSLAM::TDataAssocInfo &)) &mrpt::slam::CRangeBearingKFSLAM::TDataAssocInfo::operator=, "C++: mrpt::slam::CRangeBearingKFSLAM::TDataAssocInfo::operator=(const struct mrpt::slam::CRangeBearingKFSLAM::TDataAssocInfo &) --> struct mrpt::slam::CRangeBearingKFSLAM::TDataAssocInfo &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

	}
	{ // mrpt::slam::CRangeBearingKFSLAM2D file:mrpt/slam/CRangeBearingKFSLAM2D.h line:40
		pybind11::class_<mrpt::slam::CRangeBearingKFSLAM2D, std::shared_ptr<mrpt::slam::CRangeBearingKFSLAM2D>, PyCallBack_mrpt_slam_CRangeBearingKFSLAM2D, mrpt::bayes::CKalmanFilterCapable<3UL,2UL,2UL,3UL,double>> cl(M("mrpt::slam"), "CRangeBearingKFSLAM2D", "An implementation of EKF-based SLAM with range-bearing sensors, odometry,\n SE(2) robot pose, and 2D landmarks.\n The main method is processActionObservation() which processes pairs of\n actions/observations.\n\n The following front-end applications are based on this class:\n	- [2d-slam-demo](app_2d-slam-demo.html)\n	- [kf-slam](page_app_kf-slam.html)\n\n \n CRangeBearingKFSLAM  \n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::slam::CRangeBearingKFSLAM2D(); }, [](){ return new PyCallBack_mrpt_slam_CRangeBearingKFSLAM2D(); } ) );
		cl.def( pybind11::init( [](PyCallBack_mrpt_slam_CRangeBearingKFSLAM2D const &o){ return new PyCallBack_mrpt_slam_CRangeBearingKFSLAM2D(o); } ) );
		cl.def( pybind11::init( [](mrpt::slam::CRangeBearingKFSLAM2D const &o){ return new mrpt::slam::CRangeBearingKFSLAM2D(o); } ) );
		cl.def_readwrite("options", &mrpt::slam::CRangeBearingKFSLAM2D::options);
		cl.def("reset", (void (mrpt::slam::CRangeBearingKFSLAM2D::*)()) &mrpt::slam::CRangeBearingKFSLAM2D::reset, "Reset the state of the SLAM filter: The map is emptied and the robot put\n back to (0,0,0). \n\nC++: mrpt::slam::CRangeBearingKFSLAM2D::reset() --> void");
		cl.def("processActionObservation", (void (mrpt::slam::CRangeBearingKFSLAM2D::*)(class std::shared_ptr<class mrpt::obs::CActionCollection> &, class std::shared_ptr<class mrpt::obs::CSensoryFrame> &)) &mrpt::slam::CRangeBearingKFSLAM2D::processActionObservation, "Process one new action and observations to update the map and robot pose\nestimate. See the description of the class at the top of this page.\n  \n\n May contain odometry\n	\n\n The set of observations, must contain at least one\nCObservationBearingRange\n\nC++: mrpt::slam::CRangeBearingKFSLAM2D::processActionObservation(class std::shared_ptr<class mrpt::obs::CActionCollection> &, class std::shared_ptr<class mrpt::obs::CSensoryFrame> &) --> void", pybind11::arg("action"), pybind11::arg("SF"));
		cl.def("getCurrentRobotPose", (void (mrpt::slam::CRangeBearingKFSLAM2D::*)(class mrpt::poses::CPosePDFGaussian &) const) &mrpt::slam::CRangeBearingKFSLAM2D::getCurrentRobotPose, "Returns the mean & 3x3 covariance matrix of the robot 2D pose.\n \n\n getCurrentState\n\nC++: mrpt::slam::CRangeBearingKFSLAM2D::getCurrentRobotPose(class mrpt::poses::CPosePDFGaussian &) const --> void", pybind11::arg("out_robotPose"));
		cl.def("getAs3DObject", (void (mrpt::slam::CRangeBearingKFSLAM2D::*)(class std::shared_ptr<class mrpt::opengl::CSetOfObjects> &) const) &mrpt::slam::CRangeBearingKFSLAM2D::getAs3DObject, "Returns a 3D representation of the landmarks in the map and the robot 3D\n position according to the current filter state.\n  \n\n\n   \n\nC++: mrpt::slam::CRangeBearingKFSLAM2D::getAs3DObject(class std::shared_ptr<class mrpt::opengl::CSetOfObjects> &) const --> void", pybind11::arg("outObj"));
		cl.def("loadOptions", (void (mrpt::slam::CRangeBearingKFSLAM2D::*)(const class mrpt::config::CConfigFileBase &)) &mrpt::slam::CRangeBearingKFSLAM2D::loadOptions, "Load options from a ini-like file/text\n\nC++: mrpt::slam::CRangeBearingKFSLAM2D::loadOptions(const class mrpt::config::CConfigFileBase &) --> void", pybind11::arg("ini"));
		cl.def("saveMapAndPath2DRepresentationAsMATLABFile", [](mrpt::slam::CRangeBearingKFSLAM2D const &o, const std::string & a0) -> void { return o.saveMapAndPath2DRepresentationAsMATLABFile(a0); }, "", pybind11::arg("fil"));
		cl.def("saveMapAndPath2DRepresentationAsMATLABFile", [](mrpt::slam::CRangeBearingKFSLAM2D const &o, const std::string & a0, float const & a1) -> void { return o.saveMapAndPath2DRepresentationAsMATLABFile(a0, a1); }, "", pybind11::arg("fil"), pybind11::arg("stdCount"));
		cl.def("saveMapAndPath2DRepresentationAsMATLABFile", [](mrpt::slam::CRangeBearingKFSLAM2D const &o, const std::string & a0, float const & a1, const std::string & a2) -> void { return o.saveMapAndPath2DRepresentationAsMATLABFile(a0, a1, a2); }, "", pybind11::arg("fil"), pybind11::arg("stdCount"), pybind11::arg("styleLandmarks"));
		cl.def("saveMapAndPath2DRepresentationAsMATLABFile", [](mrpt::slam::CRangeBearingKFSLAM2D const &o, const std::string & a0, float const & a1, const std::string & a2, const std::string & a3) -> void { return o.saveMapAndPath2DRepresentationAsMATLABFile(a0, a1, a2, a3); }, "", pybind11::arg("fil"), pybind11::arg("stdCount"), pybind11::arg("styleLandmarks"), pybind11::arg("stylePath"));
		cl.def("saveMapAndPath2DRepresentationAsMATLABFile", (void (mrpt::slam::CRangeBearingKFSLAM2D::*)(const std::string &, float, const std::string &, const std::string &, const std::string &) const) &mrpt::slam::CRangeBearingKFSLAM2D::saveMapAndPath2DRepresentationAsMATLABFile, "Save the current state of the filter (robot pose & map) to a MATLAB\n script which displays all the elements in 2D\n\nC++: mrpt::slam::CRangeBearingKFSLAM2D::saveMapAndPath2DRepresentationAsMATLABFile(const std::string &, float, const std::string &, const std::string &, const std::string &) const --> void", pybind11::arg("fil"), pybind11::arg("stdCount"), pybind11::arg("styleLandmarks"), pybind11::arg("stylePath"), pybind11::arg("styleRobot"));
		cl.def("getLastDataAssociation", (const struct mrpt::slam::CRangeBearingKFSLAM2D::TDataAssocInfo & (mrpt::slam::CRangeBearingKFSLAM2D::*)() const) &mrpt::slam::CRangeBearingKFSLAM2D::getLastDataAssociation, "Returns a read-only reference to the information on the last\n data-association \n\nC++: mrpt::slam::CRangeBearingKFSLAM2D::getLastDataAssociation() const --> const struct mrpt::slam::CRangeBearingKFSLAM2D::TDataAssocInfo &", pybind11::return_value_policy::automatic);

		{ // mrpt::slam::CRangeBearingKFSLAM2D::TOptions file:mrpt/slam/CRangeBearingKFSLAM2D.h line:103
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::slam::CRangeBearingKFSLAM2D::TOptions, std::shared_ptr<mrpt::slam::CRangeBearingKFSLAM2D::TOptions>, PyCallBack_mrpt_slam_CRangeBearingKFSLAM2D_TOptions, mrpt::config::CLoadableOptions> cl(enclosing_class, "TOptions", "The options for the algorithm");
			cl.def( pybind11::init( [](){ return new mrpt::slam::CRangeBearingKFSLAM2D::TOptions(); }, [](){ return new PyCallBack_mrpt_slam_CRangeBearingKFSLAM2D_TOptions(); } ) );
			cl.def( pybind11::init( [](PyCallBack_mrpt_slam_CRangeBearingKFSLAM2D_TOptions const &o){ return new PyCallBack_mrpt_slam_CRangeBearingKFSLAM2D_TOptions(o); } ) );
			cl.def( pybind11::init( [](mrpt::slam::CRangeBearingKFSLAM2D::TOptions const &o){ return new mrpt::slam::CRangeBearingKFSLAM2D::TOptions(o); } ) );
			cl.def_readwrite("stds_Q_no_odo", &mrpt::slam::CRangeBearingKFSLAM2D::TOptions::stds_Q_no_odo);
			cl.def_readwrite("std_sensor_range", &mrpt::slam::CRangeBearingKFSLAM2D::TOptions::std_sensor_range);
			cl.def_readwrite("std_sensor_yaw", &mrpt::slam::CRangeBearingKFSLAM2D::TOptions::std_sensor_yaw);
			cl.def_readwrite("quantiles_3D_representation", &mrpt::slam::CRangeBearingKFSLAM2D::TOptions::quantiles_3D_representation);
			cl.def_readwrite("create_simplemap", &mrpt::slam::CRangeBearingKFSLAM2D::TOptions::create_simplemap);
			cl.def_readwrite("data_assoc_method", &mrpt::slam::CRangeBearingKFSLAM2D::TOptions::data_assoc_method);
			cl.def_readwrite("data_assoc_metric", &mrpt::slam::CRangeBearingKFSLAM2D::TOptions::data_assoc_metric);
			cl.def_readwrite("data_assoc_IC_chi2_thres", &mrpt::slam::CRangeBearingKFSLAM2D::TOptions::data_assoc_IC_chi2_thres);
			cl.def_readwrite("data_assoc_IC_metric", &mrpt::slam::CRangeBearingKFSLAM2D::TOptions::data_assoc_IC_metric);
			cl.def_readwrite("data_assoc_IC_ml_threshold", &mrpt::slam::CRangeBearingKFSLAM2D::TOptions::data_assoc_IC_ml_threshold);
			cl.def("loadFromConfigFile", (void (mrpt::slam::CRangeBearingKFSLAM2D::TOptions::*)(const class mrpt::config::CConfigFileBase &, const std::string &)) &mrpt::slam::CRangeBearingKFSLAM2D::TOptions::loadFromConfigFile, "C++: mrpt::slam::CRangeBearingKFSLAM2D::TOptions::loadFromConfigFile(const class mrpt::config::CConfigFileBase &, const std::string &) --> void", pybind11::arg("source"), pybind11::arg("section"));
			cl.def("assign", (struct mrpt::slam::CRangeBearingKFSLAM2D::TOptions & (mrpt::slam::CRangeBearingKFSLAM2D::TOptions::*)(const struct mrpt::slam::CRangeBearingKFSLAM2D::TOptions &)) &mrpt::slam::CRangeBearingKFSLAM2D::TOptions::operator=, "C++: mrpt::slam::CRangeBearingKFSLAM2D::TOptions::operator=(const struct mrpt::slam::CRangeBearingKFSLAM2D::TOptions &) --> struct mrpt::slam::CRangeBearingKFSLAM2D::TOptions &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

		{ // mrpt::slam::CRangeBearingKFSLAM2D::TDataAssocInfo file:mrpt/slam/CRangeBearingKFSLAM2D.h line:157
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::slam::CRangeBearingKFSLAM2D::TDataAssocInfo, std::shared_ptr<mrpt::slam::CRangeBearingKFSLAM2D::TDataAssocInfo>> cl(enclosing_class, "TDataAssocInfo", "Information for data-association:\n \n\n getLastDataAssociation");
			cl.def( pybind11::init( [](){ return new mrpt::slam::CRangeBearingKFSLAM2D::TDataAssocInfo(); } ) );
			cl.def( pybind11::init( [](mrpt::slam::CRangeBearingKFSLAM2D::TDataAssocInfo const &o){ return new mrpt::slam::CRangeBearingKFSLAM2D::TDataAssocInfo(o); } ) );
			cl.def_readwrite("Y_pred_means", &mrpt::slam::CRangeBearingKFSLAM2D::TDataAssocInfo::Y_pred_means);
			cl.def_readwrite("Y_pred_covs", &mrpt::slam::CRangeBearingKFSLAM2D::TDataAssocInfo::Y_pred_covs);
			cl.def_readwrite("predictions_IDs", &mrpt::slam::CRangeBearingKFSLAM2D::TDataAssocInfo::predictions_IDs);
			cl.def_readwrite("newly_inserted_landmarks", &mrpt::slam::CRangeBearingKFSLAM2D::TDataAssocInfo::newly_inserted_landmarks);
			cl.def_readwrite("results", &mrpt::slam::CRangeBearingKFSLAM2D::TDataAssocInfo::results);
			cl.def("clear", (void (mrpt::slam::CRangeBearingKFSLAM2D::TDataAssocInfo::*)()) &mrpt::slam::CRangeBearingKFSLAM2D::TDataAssocInfo::clear, "C++: mrpt::slam::CRangeBearingKFSLAM2D::TDataAssocInfo::clear() --> void");
			cl.def("assign", (struct mrpt::slam::CRangeBearingKFSLAM2D::TDataAssocInfo & (mrpt::slam::CRangeBearingKFSLAM2D::TDataAssocInfo::*)(const struct mrpt::slam::CRangeBearingKFSLAM2D::TDataAssocInfo &)) &mrpt::slam::CRangeBearingKFSLAM2D::TDataAssocInfo::operator=, "C++: mrpt::slam::CRangeBearingKFSLAM2D::TDataAssocInfo::operator=(const struct mrpt::slam::CRangeBearingKFSLAM2D::TDataAssocInfo &) --> struct mrpt::slam::CRangeBearingKFSLAM2D::TDataAssocInfo &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

	}
}
