#include <mrpt/opengl/COctreePointRenderer.h>

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

void bind_mrpt_opengl_COctreePointRenderer(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	// mrpt::global_settings::OCTREE_RENDER_MAX_DENSITY_POINTS_PER_SQPIXEL(float) file:mrpt/opengl/COctreePointRenderer.h line:31
	M("mrpt::global_settings").def("OCTREE_RENDER_MAX_DENSITY_POINTS_PER_SQPIXEL", (void (*)(float)) &mrpt::global_settings::OCTREE_RENDER_MAX_DENSITY_POINTS_PER_SQPIXEL, "Default value = 0.01 points/px^2. Affects to these classes (read their docs\nfor further details):\n		- mrpt::opengl::CPointCloud\n		- mrpt::opengl::CPointCloudColoured\n \n\n\n \n\nC++: mrpt::global_settings::OCTREE_RENDER_MAX_DENSITY_POINTS_PER_SQPIXEL(float) --> void", pybind11::arg("value"));

	// mrpt::global_settings::OCTREE_RENDER_MAX_DENSITY_POINTS_PER_SQPIXEL() file:mrpt/opengl/COctreePointRenderer.h line:32
	M("mrpt::global_settings").def("OCTREE_RENDER_MAX_DENSITY_POINTS_PER_SQPIXEL", (float (*)()) &mrpt::global_settings::OCTREE_RENDER_MAX_DENSITY_POINTS_PER_SQPIXEL, "C++: mrpt::global_settings::OCTREE_RENDER_MAX_DENSITY_POINTS_PER_SQPIXEL() --> float");

	// mrpt::global_settings::OCTREE_RENDER_MAX_POINTS_PER_NODE() file:mrpt/opengl/COctreePointRenderer.h line:40
	M("mrpt::global_settings").def("OCTREE_RENDER_MAX_POINTS_PER_NODE", (size_t (*)()) &mrpt::global_settings::OCTREE_RENDER_MAX_POINTS_PER_NODE, "Default value = 1e5. Maximum number of elements in each octree node before\nspliting. Affects to these classes (read their docs for further details):\n		- mrpt::opengl::CPointCloud\n		- mrpt::opengl::CPointCloudColoured\n \n\n\n \n\nC++: mrpt::global_settings::OCTREE_RENDER_MAX_POINTS_PER_NODE() --> size_t");

	// mrpt::global_settings::OCTREE_RENDER_MAX_POINTS_PER_NODE(size_t) file:mrpt/opengl/COctreePointRenderer.h line:41
	M("mrpt::global_settings").def("OCTREE_RENDER_MAX_POINTS_PER_NODE", (void (*)(size_t)) &mrpt::global_settings::OCTREE_RENDER_MAX_POINTS_PER_NODE, "C++: mrpt::global_settings::OCTREE_RENDER_MAX_POINTS_PER_NODE(size_t) --> void", pybind11::arg("value"));

}
