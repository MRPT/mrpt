#include <any>
#include <ios>
#include <istream>
#include <memory>
#include <mrpt/containers/CommentPosition.h>
#include <mrpt/containers/YamlEmitOptions.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/opengl/CRenderizable.h>
#include <mrpt/opengl/RenderQueue.h>
#include <mrpt/opengl/TRenderMatrices.h>
#include <ostream>
#include <streambuf>
#include <string>
#include <string_view>
#include <typeinfo>

#include <functional>
#include <pybind11/pybind11.h>
#include <string>
#include <stl_binders.hpp>


#ifndef BINDER_PYBIND11_TYPE_CASTER
	#define BINDER_PYBIND11_TYPE_CASTER
	PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>)
	PYBIND11_DECLARE_HOLDER_TYPE(T, T*)
	PYBIND11_MAKE_OPAQUE(std::shared_ptr<void>)
#endif

void bind_mrpt_opengl_CRenderizable(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	// mrpt::opengl::enqueueForRendering(const int &, const struct mrpt::opengl::TRenderMatrices &, int &, const bool, const bool, struct mrpt::opengl::RenderQueueStats *) file:mrpt/opengl/CRenderizable.h line:535
	M("mrpt::opengl").def("enqueueForRendering", [](const int & a0, const struct mrpt::opengl::TRenderMatrices & a1, int & a2, const bool & a3, const bool & a4) -> void { return mrpt::opengl::enqueueForRendering(a0, a1, a2, a3, a4); }, "", pybind11::arg("objs"), pybind11::arg("state"), pybind11::arg("rq"), pybind11::arg("skipCullChecks"), pybind11::arg("is1stShadowMapPass"));
	M("mrpt::opengl").def("enqueueForRendering", (void (*)(const int &, const struct mrpt::opengl::TRenderMatrices &, int &, const bool, const bool, struct mrpt::opengl::RenderQueueStats *)) &mrpt::opengl::enqueueForRendering, "Processes, recursively, all objects in the list, classifying them by shader\n programs into a list suitable to be used within processPendingRendering()\n\n For each object in the list:\n   - checks visibility of each object\n   - update the MODELVIEW matrix according to its coordinates\n   - call its ::render()\n   - shows its name (if enabled).\n\n \n Will be true if the render engine already checked that\n        the object lies within the viewport area, so it is pointless to waste\n        more time checking.\n\n \n Used by CSetOfObjects and Viewport\n\n \n processPendingRendering\n\nC++: mrpt::opengl::enqueueForRendering(const int &, const struct mrpt::opengl::TRenderMatrices &, int &, const bool, const bool, struct mrpt::opengl::RenderQueueStats *) --> void", pybind11::arg("objs"), pybind11::arg("state"), pybind11::arg("rq"), pybind11::arg("skipCullChecks"), pybind11::arg("is1stShadowMapPass"), pybind11::arg("stats"));

}
