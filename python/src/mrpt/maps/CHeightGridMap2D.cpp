#include <mrpt/maps/CHeightGridMap2D.h>

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

void bind_mrpt_maps_CHeightGridMap2D(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	// mrpt::global_settings::HEIGHTGRIDMAP_EXPORT3D_AS_MESH(bool) file:mrpt/maps/CHeightGridMap2D.h line:190
	M("mrpt::global_settings").def("HEIGHTGRIDMAP_EXPORT3D_AS_MESH", (void (*)(bool)) &mrpt::global_settings::HEIGHTGRIDMAP_EXPORT3D_AS_MESH, "If set to true (default), mrpt::maps::CHeightGridMap2D will be exported as a\nopengl::CMesh, otherwise, as a opengl::CPointCloudColoured\n Affects to:\n		- CHeightGridMap2D::getAs3DObject\n\nC++: mrpt::global_settings::HEIGHTGRIDMAP_EXPORT3D_AS_MESH(bool) --> void", pybind11::arg("value"));

	// mrpt::global_settings::HEIGHTGRIDMAP_EXPORT3D_AS_MESH() file:mrpt/maps/CHeightGridMap2D.h line:191
	M("mrpt::global_settings").def("HEIGHTGRIDMAP_EXPORT3D_AS_MESH", (bool (*)()) &mrpt::global_settings::HEIGHTGRIDMAP_EXPORT3D_AS_MESH, "C++: mrpt::global_settings::HEIGHTGRIDMAP_EXPORT3D_AS_MESH() --> bool");

}
