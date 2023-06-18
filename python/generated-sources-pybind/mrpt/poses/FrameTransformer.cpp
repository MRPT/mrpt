#include <mrpt/poses/FrameTransformer.h>

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

void bind_mrpt_poses_FrameTransformer(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	// mrpt::poses::FrameLookUpStatus file:mrpt/poses/FrameTransformer.h line:19
	pybind11::enum_<mrpt::poses::FrameLookUpStatus>(M("mrpt::poses"), "FrameLookUpStatus", pybind11::arithmetic(), "")
		.value("LKUP_GOOD", mrpt::poses::LKUP_GOOD)
		.value("LKUP_UNKNOWN_FRAME", mrpt::poses::LKUP_UNKNOWN_FRAME)
		.value("LKUP_NO_CONNECTIVITY", mrpt::poses::LKUP_NO_CONNECTIVITY)
		.value("LKUP_EXTRAPOLATION_ERROR", mrpt::poses::LKUP_EXTRAPOLATION_ERROR)
		.export_values();

;

}
