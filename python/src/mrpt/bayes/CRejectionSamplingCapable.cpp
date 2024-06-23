#include <iterator>
#include <memory>
#include <mrpt/bayes/CProbabilityParticle.h>
#include <mrpt/bayes/CRejectionSamplingCapable.h>
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CQuaternion.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/math/TPose3D.h>
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

// mrpt::bayes::CRejectionSamplingCapable file:mrpt/bayes/CRejectionSamplingCapable.h line:30
struct PyCallBack_mrpt_bayes_CRejectionSamplingCapable_mrpt_poses_CPose2D_mrpt_bayes_particle_storage_mode_POINTER_t : public mrpt::bayes::CRejectionSamplingCapable<mrpt::poses::CPose2D,mrpt::bayes::particle_storage_mode::POINTER> {
	using mrpt::bayes::CRejectionSamplingCapable<mrpt::poses::CPose2D,mrpt::bayes::particle_storage_mode::POINTER>::CRejectionSamplingCapable;

	void RS_drawFromProposal(class mrpt::poses::CPose2D & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::bayes::CRejectionSamplingCapable<mrpt::poses::CPose2D,mrpt::bayes::particle_storage_mode::POINTER> *>(this), "RS_drawFromProposal");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CRejectionSamplingCapable::RS_drawFromProposal\"");
	}
	double RS_observationLikelihood(const class mrpt::poses::CPose2D & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::bayes::CRejectionSamplingCapable<mrpt::poses::CPose2D,mrpt::bayes::particle_storage_mode::POINTER> *>(this), "RS_observationLikelihood");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<double>::value) {
				static pybind11::detail::override_caster_t<double> caster;
				return pybind11::detail::cast_ref<double>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<double>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CRejectionSamplingCapable::RS_observationLikelihood\"");
	}
};

void bind_mrpt_bayes_CRejectionSamplingCapable(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::bayes::CRejectionSamplingCapable file:mrpt/bayes/CRejectionSamplingCapable.h line:30
		pybind11::class_<mrpt::bayes::CRejectionSamplingCapable<mrpt::poses::CPose2D,mrpt::bayes::particle_storage_mode::POINTER>, std::shared_ptr<mrpt::bayes::CRejectionSamplingCapable<mrpt::poses::CPose2D,mrpt::bayes::particle_storage_mode::POINTER>>, PyCallBack_mrpt_bayes_CRejectionSamplingCapable_mrpt_poses_CPose2D_mrpt_bayes_particle_storage_mode_POINTER_t> cl(M("mrpt::bayes"), "CRejectionSamplingCapable_mrpt_poses_CPose2D_mrpt_bayes_particle_storage_mode_POINTER_t", "");
		cl.def( pybind11::init( [](){ return new PyCallBack_mrpt_bayes_CRejectionSamplingCapable_mrpt_poses_CPose2D_mrpt_bayes_particle_storage_mode_POINTER_t(); } ) );
		cl.def(pybind11::init<PyCallBack_mrpt_bayes_CRejectionSamplingCapable_mrpt_poses_CPose2D_mrpt_bayes_particle_storage_mode_POINTER_t const &>());
		cl.def("assign", (class mrpt::bayes::CRejectionSamplingCapable<class mrpt::poses::CPose2D, mrpt::bayes::particle_storage_mode::POINTER> & (mrpt::bayes::CRejectionSamplingCapable<mrpt::poses::CPose2D,mrpt::bayes::particle_storage_mode::POINTER>::*)(const class mrpt::bayes::CRejectionSamplingCapable<class mrpt::poses::CPose2D, mrpt::bayes::particle_storage_mode::POINTER> &)) &mrpt::bayes::CRejectionSamplingCapable<mrpt::poses::CPose2D, mrpt::bayes::particle_storage_mode::POINTER>::operator=, "C++: mrpt::bayes::CRejectionSamplingCapable<mrpt::poses::CPose2D, mrpt::bayes::particle_storage_mode::POINTER>::operator=(const class mrpt::bayes::CRejectionSamplingCapable<class mrpt::poses::CPose2D, mrpt::bayes::particle_storage_mode::POINTER> &) --> class mrpt::bayes::CRejectionSamplingCapable<class mrpt::poses::CPose2D, mrpt::bayes::particle_storage_mode::POINTER> &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
