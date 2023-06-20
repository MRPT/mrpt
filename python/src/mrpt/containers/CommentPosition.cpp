#include <mrpt/containers/CommentPosition.h>
#include <mrpt/containers/YamlEmitOptions.h>
#include <sstream> // __str__

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

void bind_mrpt_containers_CommentPosition(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	// mrpt::containers::CommentPosition file:mrpt/containers/CommentPosition.h line:21
	pybind11::enum_<mrpt::containers::CommentPosition>(M("mrpt::containers"), "CommentPosition", "Defines possible positions for a comment in a document (INI file, YAML).\n Valid positions are: Top, right.\n\n \n\n \n [New in MRPT 2.1.0]")
		.value("TOP", mrpt::containers::CommentPosition::TOP)
		.value("RIGHT", mrpt::containers::CommentPosition::RIGHT)
		.value("MAX", mrpt::containers::CommentPosition::MAX);

;

	{ // mrpt::containers::YamlEmitOptions file:mrpt/containers/YamlEmitOptions.h line:18
		pybind11::class_<mrpt::containers::YamlEmitOptions, std::shared_ptr<mrpt::containers::YamlEmitOptions>> cl(M("mrpt::containers"), "YamlEmitOptions", "See mrpt::containers::yaml::PrintAsYaml\n\n \n\n \n [New in MRPT 2.1.0]");
		cl.def( pybind11::init( [](){ return new mrpt::containers::YamlEmitOptions(); } ) );
		cl.def_readwrite("emitHeader", &mrpt::containers::YamlEmitOptions::emitHeader);
		cl.def_readwrite("emitComments", &mrpt::containers::YamlEmitOptions::emitComments);
		cl.def_readwrite("endWithNewLine", &mrpt::containers::YamlEmitOptions::endWithNewLine);
		cl.def_readwrite("indentSequences", &mrpt::containers::YamlEmitOptions::indentSequences);
	}
}
