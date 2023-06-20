#include <chrono>
#include <ios>
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
#include <mrpt/obs/CObservation.h>
#include <mrpt/obs/format_externals_filename.h>
#include <mrpt/poses/CPoint2D.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DQuat.h>
#include <mrpt/poses/CPoseOrPoint.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/typemeta/static_string.h>
#include <optional>
#include <ostream>
#include <ratio>
#include <streambuf>
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

void bind_mrpt_obs_format_externals_filename(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	// mrpt::obs::format_externals_filename(const class mrpt::obs::CObservation &, const std::string &) file:mrpt/obs/format_externals_filename.h line:32
	M("mrpt::obs").def("format_externals_filename", (std::string (*)(const class mrpt::obs::CObservation &, const std::string &)) &mrpt::obs::format_externals_filename, "Replaces format placeholders in a string according to an observation:\n  - `${type}` is replaced by: `img`, `stereo`, `3dcam` for single images,\n     stereo images and depth camera observations, respectively, or `other`\n     otherwise.\n  - `${label}` is replaced by the observation `sensorLabel` field.\n  - `%f` with any standard `printf()` format modifiers will be replaced by\n    UNIX timestamp (with fractions of seconds) of the observation.\n  - Anything else will be left unmodified.\n\n  For example, the default format string used in\n  `rawlog-edit --rename-externals` is `\"${type}_${label}_%.06%f\"`.\n\n \n\n \n (new in MRPT 2.4.1)\n\nC++: mrpt::obs::format_externals_filename(const class mrpt::obs::CObservation &, const std::string &) --> std::string", pybind11::arg("obs"), pybind11::arg("fmt"));

}
