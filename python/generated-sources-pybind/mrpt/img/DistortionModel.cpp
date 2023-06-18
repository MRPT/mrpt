#include <mrpt/img/DistortionModel.h>

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

void bind_mrpt_img_DistortionModel(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	// mrpt::img::DistortionModel file:mrpt/img/DistortionModel.h line:22
	pybind11::enum_<mrpt::img::DistortionModel>(M("mrpt::img"), "DistortionModel", "Enum for different camera distortion models.\n\n \n TCamera\n \n\n\n ")
		.value("none", mrpt::img::DistortionModel::none)
		.value("plumb_bob", mrpt::img::DistortionModel::plumb_bob)
		.value("kannala_brandt", mrpt::img::DistortionModel::kannala_brandt);

;

}
