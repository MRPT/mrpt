#include <iterator>
#include <memory>
#include <mrpt/containers/yaml.h>
#include <mrpt/math/TPlane.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPolygon2D.h>
#include <mrpt/math/TPolygon3D.h>
#include <mrpt/math/TPolygonWithPlane.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/math/TSegment3D.h>
#include <sstream> // __str__
#include <string>
#include <type_traits>
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

void bind_mrpt_math_TPolygonWithPlane(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::math::TPolygonWithPlane file:mrpt/math/TPolygonWithPlane.h line:26
		pybind11::class_<mrpt::math::TPolygonWithPlane, std::shared_ptr<mrpt::math::TPolygonWithPlane>> cl(M("mrpt::math"), "TPolygonWithPlane", "Slightly heavyweight type to speed-up calculations with polygons in 3D\n \n\n TPolygon3D,TPlane");
		cl.def( pybind11::init<const class mrpt::math::TPolygon3D &>(), pybind11::arg("p") );

		cl.def( pybind11::init( [](){ return new mrpt::math::TPolygonWithPlane(); } ) );
		cl.def_readwrite("poly", &mrpt::math::TPolygonWithPlane::poly);
		cl.def_readwrite("plane", &mrpt::math::TPolygonWithPlane::plane);
		cl.def_readwrite("pose", &mrpt::math::TPolygonWithPlane::pose);
		cl.def_readwrite("inversePose", &mrpt::math::TPolygonWithPlane::inversePose);
		cl.def_readwrite("poly2D", &mrpt::math::TPolygonWithPlane::poly2D);
	}
}
