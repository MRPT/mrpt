#include <iterator>
#include <memory>
#include <mrpt/img/DistortionModel.h>
#include <mrpt/img/color_maps.h>
#include <mrpt/opengl/CRenderizable.h>
#include <mrpt/poses/CPoseInterpolatorBase.h>
#include <mrpt/slam/CICP.h>
#include <mrpt/typemeta/TEnumType.h>
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

void bind_mrpt_typemeta_TEnumType_1(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::typemeta::internal::bimap file:mrpt/typemeta/TEnumType.h line:22
		pybind11::class_<mrpt::typemeta::internal::bimap<mrpt::poses::TInterpolatorMethod,std::string>, std::shared_ptr<mrpt::typemeta::internal::bimap<mrpt::poses::TInterpolatorMethod,std::string>>> cl(M("mrpt::typemeta::internal"), "bimap_mrpt_poses_TInterpolatorMethod_std_string_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::typemeta::internal::bimap<mrpt::poses::TInterpolatorMethod,std::string>(); } ) );
		cl.def( pybind11::init( [](mrpt::typemeta::internal::bimap<mrpt::poses::TInterpolatorMethod,std::string> const &o){ return new mrpt::typemeta::internal::bimap<mrpt::poses::TInterpolatorMethod,std::string>(o); } ) );
		cl.def_readwrite("m_k2v", &mrpt::typemeta::internal::bimap<mrpt::poses::TInterpolatorMethod,std::string>::m_k2v);
		cl.def_readwrite("m_v2k", &mrpt::typemeta::internal::bimap<mrpt::poses::TInterpolatorMethod,std::string>::m_v2k);
		cl.def("direct", (bool (mrpt::typemeta::internal::bimap<mrpt::poses::TInterpolatorMethod,std::string>::*)(const enum mrpt::poses::TInterpolatorMethod &, std::string &) const) &mrpt::typemeta::internal::bimap<mrpt::poses::TInterpolatorMethod, std::string>::direct, "C++: mrpt::typemeta::internal::bimap<mrpt::poses::TInterpolatorMethod, std::string>::direct(const enum mrpt::poses::TInterpolatorMethod &, std::string &) const --> bool", pybind11::arg("k"), pybind11::arg("out_v"));
		cl.def("inverse", (bool (mrpt::typemeta::internal::bimap<mrpt::poses::TInterpolatorMethod,std::string>::*)(const std::string &, enum mrpt::poses::TInterpolatorMethod &) const) &mrpt::typemeta::internal::bimap<mrpt::poses::TInterpolatorMethod, std::string>::inverse, "C++: mrpt::typemeta::internal::bimap<mrpt::poses::TInterpolatorMethod, std::string>::inverse(const std::string &, enum mrpt::poses::TInterpolatorMethod &) const --> bool", pybind11::arg("v"), pybind11::arg("out_k"));
		cl.def("insert", (void (mrpt::typemeta::internal::bimap<mrpt::poses::TInterpolatorMethod,std::string>::*)(const enum mrpt::poses::TInterpolatorMethod &, const std::string &)) &mrpt::typemeta::internal::bimap<mrpt::poses::TInterpolatorMethod, std::string>::insert, "C++: mrpt::typemeta::internal::bimap<mrpt::poses::TInterpolatorMethod, std::string>::insert(const enum mrpt::poses::TInterpolatorMethod &, const std::string &) --> void", pybind11::arg("k"), pybind11::arg("v"));
		cl.def("assign", (struct mrpt::typemeta::internal::bimap<enum mrpt::poses::TInterpolatorMethod, std::string > & (mrpt::typemeta::internal::bimap<mrpt::poses::TInterpolatorMethod,std::string>::*)(const struct mrpt::typemeta::internal::bimap<enum mrpt::poses::TInterpolatorMethod, std::string > &)) &mrpt::typemeta::internal::bimap<mrpt::poses::TInterpolatorMethod, std::string>::operator=, "C++: mrpt::typemeta::internal::bimap<mrpt::poses::TInterpolatorMethod, std::string>::operator=(const struct mrpt::typemeta::internal::bimap<enum mrpt::poses::TInterpolatorMethod, std::string > &) --> struct mrpt::typemeta::internal::bimap<enum mrpt::poses::TInterpolatorMethod, std::string > &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::typemeta::internal::bimap file:mrpt/typemeta/TEnumType.h line:22
		pybind11::class_<mrpt::typemeta::internal::bimap<mrpt::img::TColormap,std::string>, std::shared_ptr<mrpt::typemeta::internal::bimap<mrpt::img::TColormap,std::string>>> cl(M("mrpt::typemeta::internal"), "bimap_mrpt_img_TColormap_std_string_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::typemeta::internal::bimap<mrpt::img::TColormap,std::string>(); } ) );
		cl.def( pybind11::init( [](mrpt::typemeta::internal::bimap<mrpt::img::TColormap,std::string> const &o){ return new mrpt::typemeta::internal::bimap<mrpt::img::TColormap,std::string>(o); } ) );
		cl.def_readwrite("m_k2v", &mrpt::typemeta::internal::bimap<mrpt::img::TColormap,std::string>::m_k2v);
		cl.def_readwrite("m_v2k", &mrpt::typemeta::internal::bimap<mrpt::img::TColormap,std::string>::m_v2k);
		cl.def("direct", (bool (mrpt::typemeta::internal::bimap<mrpt::img::TColormap,std::string>::*)(const enum mrpt::img::TColormap &, std::string &) const) &mrpt::typemeta::internal::bimap<mrpt::img::TColormap, std::string>::direct, "C++: mrpt::typemeta::internal::bimap<mrpt::img::TColormap, std::string>::direct(const enum mrpt::img::TColormap &, std::string &) const --> bool", pybind11::arg("k"), pybind11::arg("out_v"));
		cl.def("inverse", (bool (mrpt::typemeta::internal::bimap<mrpt::img::TColormap,std::string>::*)(const std::string &, enum mrpt::img::TColormap &) const) &mrpt::typemeta::internal::bimap<mrpt::img::TColormap, std::string>::inverse, "C++: mrpt::typemeta::internal::bimap<mrpt::img::TColormap, std::string>::inverse(const std::string &, enum mrpt::img::TColormap &) const --> bool", pybind11::arg("v"), pybind11::arg("out_k"));
		cl.def("insert", (void (mrpt::typemeta::internal::bimap<mrpt::img::TColormap,std::string>::*)(const enum mrpt::img::TColormap &, const std::string &)) &mrpt::typemeta::internal::bimap<mrpt::img::TColormap, std::string>::insert, "C++: mrpt::typemeta::internal::bimap<mrpt::img::TColormap, std::string>::insert(const enum mrpt::img::TColormap &, const std::string &) --> void", pybind11::arg("k"), pybind11::arg("v"));
		cl.def("assign", (struct mrpt::typemeta::internal::bimap<enum mrpt::img::TColormap, std::string > & (mrpt::typemeta::internal::bimap<mrpt::img::TColormap,std::string>::*)(const struct mrpt::typemeta::internal::bimap<enum mrpt::img::TColormap, std::string > &)) &mrpt::typemeta::internal::bimap<mrpt::img::TColormap, std::string>::operator=, "C++: mrpt::typemeta::internal::bimap<mrpt::img::TColormap, std::string>::operator=(const struct mrpt::typemeta::internal::bimap<enum mrpt::img::TColormap, std::string > &) --> struct mrpt::typemeta::internal::bimap<enum mrpt::img::TColormap, std::string > &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::typemeta::internal::bimap file:mrpt/typemeta/TEnumType.h line:22
		pybind11::class_<mrpt::typemeta::internal::bimap<mrpt::slam::TICPAlgorithm,std::string>, std::shared_ptr<mrpt::typemeta::internal::bimap<mrpt::slam::TICPAlgorithm,std::string>>> cl(M("mrpt::typemeta::internal"), "bimap_mrpt_slam_TICPAlgorithm_std_string_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::typemeta::internal::bimap<mrpt::slam::TICPAlgorithm,std::string>(); } ) );
		cl.def( pybind11::init( [](mrpt::typemeta::internal::bimap<mrpt::slam::TICPAlgorithm,std::string> const &o){ return new mrpt::typemeta::internal::bimap<mrpt::slam::TICPAlgorithm,std::string>(o); } ) );
		cl.def_readwrite("m_k2v", &mrpt::typemeta::internal::bimap<mrpt::slam::TICPAlgorithm,std::string>::m_k2v);
		cl.def_readwrite("m_v2k", &mrpt::typemeta::internal::bimap<mrpt::slam::TICPAlgorithm,std::string>::m_v2k);
		cl.def("direct", (bool (mrpt::typemeta::internal::bimap<mrpt::slam::TICPAlgorithm,std::string>::*)(const enum mrpt::slam::TICPAlgorithm &, std::string &) const) &mrpt::typemeta::internal::bimap<mrpt::slam::TICPAlgorithm, std::string>::direct, "C++: mrpt::typemeta::internal::bimap<mrpt::slam::TICPAlgorithm, std::string>::direct(const enum mrpt::slam::TICPAlgorithm &, std::string &) const --> bool", pybind11::arg("k"), pybind11::arg("out_v"));
		cl.def("inverse", (bool (mrpt::typemeta::internal::bimap<mrpt::slam::TICPAlgorithm,std::string>::*)(const std::string &, enum mrpt::slam::TICPAlgorithm &) const) &mrpt::typemeta::internal::bimap<mrpt::slam::TICPAlgorithm, std::string>::inverse, "C++: mrpt::typemeta::internal::bimap<mrpt::slam::TICPAlgorithm, std::string>::inverse(const std::string &, enum mrpt::slam::TICPAlgorithm &) const --> bool", pybind11::arg("v"), pybind11::arg("out_k"));
		cl.def("insert", (void (mrpt::typemeta::internal::bimap<mrpt::slam::TICPAlgorithm,std::string>::*)(const enum mrpt::slam::TICPAlgorithm &, const std::string &)) &mrpt::typemeta::internal::bimap<mrpt::slam::TICPAlgorithm, std::string>::insert, "C++: mrpt::typemeta::internal::bimap<mrpt::slam::TICPAlgorithm, std::string>::insert(const enum mrpt::slam::TICPAlgorithm &, const std::string &) --> void", pybind11::arg("k"), pybind11::arg("v"));
		cl.def("assign", (struct mrpt::typemeta::internal::bimap<enum mrpt::slam::TICPAlgorithm, std::string > & (mrpt::typemeta::internal::bimap<mrpt::slam::TICPAlgorithm,std::string>::*)(const struct mrpt::typemeta::internal::bimap<enum mrpt::slam::TICPAlgorithm, std::string > &)) &mrpt::typemeta::internal::bimap<mrpt::slam::TICPAlgorithm, std::string>::operator=, "C++: mrpt::typemeta::internal::bimap<mrpt::slam::TICPAlgorithm, std::string>::operator=(const struct mrpt::typemeta::internal::bimap<enum mrpt::slam::TICPAlgorithm, std::string > &) --> struct mrpt::typemeta::internal::bimap<enum mrpt::slam::TICPAlgorithm, std::string > &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::typemeta::internal::bimap file:mrpt/typemeta/TEnumType.h line:22
		pybind11::class_<mrpt::typemeta::internal::bimap<mrpt::slam::TICPCovarianceMethod,std::string>, std::shared_ptr<mrpt::typemeta::internal::bimap<mrpt::slam::TICPCovarianceMethod,std::string>>> cl(M("mrpt::typemeta::internal"), "bimap_mrpt_slam_TICPCovarianceMethod_std_string_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::typemeta::internal::bimap<mrpt::slam::TICPCovarianceMethod,std::string>(); } ) );
		cl.def( pybind11::init( [](mrpt::typemeta::internal::bimap<mrpt::slam::TICPCovarianceMethod,std::string> const &o){ return new mrpt::typemeta::internal::bimap<mrpt::slam::TICPCovarianceMethod,std::string>(o); } ) );
		cl.def_readwrite("m_k2v", &mrpt::typemeta::internal::bimap<mrpt::slam::TICPCovarianceMethod,std::string>::m_k2v);
		cl.def_readwrite("m_v2k", &mrpt::typemeta::internal::bimap<mrpt::slam::TICPCovarianceMethod,std::string>::m_v2k);
		cl.def("direct", (bool (mrpt::typemeta::internal::bimap<mrpt::slam::TICPCovarianceMethod,std::string>::*)(const enum mrpt::slam::TICPCovarianceMethod &, std::string &) const) &mrpt::typemeta::internal::bimap<mrpt::slam::TICPCovarianceMethod, std::string>::direct, "C++: mrpt::typemeta::internal::bimap<mrpt::slam::TICPCovarianceMethod, std::string>::direct(const enum mrpt::slam::TICPCovarianceMethod &, std::string &) const --> bool", pybind11::arg("k"), pybind11::arg("out_v"));
		cl.def("inverse", (bool (mrpt::typemeta::internal::bimap<mrpt::slam::TICPCovarianceMethod,std::string>::*)(const std::string &, enum mrpt::slam::TICPCovarianceMethod &) const) &mrpt::typemeta::internal::bimap<mrpt::slam::TICPCovarianceMethod, std::string>::inverse, "C++: mrpt::typemeta::internal::bimap<mrpt::slam::TICPCovarianceMethod, std::string>::inverse(const std::string &, enum mrpt::slam::TICPCovarianceMethod &) const --> bool", pybind11::arg("v"), pybind11::arg("out_k"));
		cl.def("insert", (void (mrpt::typemeta::internal::bimap<mrpt::slam::TICPCovarianceMethod,std::string>::*)(const enum mrpt::slam::TICPCovarianceMethod &, const std::string &)) &mrpt::typemeta::internal::bimap<mrpt::slam::TICPCovarianceMethod, std::string>::insert, "C++: mrpt::typemeta::internal::bimap<mrpt::slam::TICPCovarianceMethod, std::string>::insert(const enum mrpt::slam::TICPCovarianceMethod &, const std::string &) --> void", pybind11::arg("k"), pybind11::arg("v"));
		cl.def("assign", (struct mrpt::typemeta::internal::bimap<enum mrpt::slam::TICPCovarianceMethod, std::string > & (mrpt::typemeta::internal::bimap<mrpt::slam::TICPCovarianceMethod,std::string>::*)(const struct mrpt::typemeta::internal::bimap<enum mrpt::slam::TICPCovarianceMethod, std::string > &)) &mrpt::typemeta::internal::bimap<mrpt::slam::TICPCovarianceMethod, std::string>::operator=, "C++: mrpt::typemeta::internal::bimap<mrpt::slam::TICPCovarianceMethod, std::string>::operator=(const struct mrpt::typemeta::internal::bimap<enum mrpt::slam::TICPCovarianceMethod, std::string > &) --> struct mrpt::typemeta::internal::bimap<enum mrpt::slam::TICPCovarianceMethod, std::string > &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::typemeta::internal::bimap file:mrpt/typemeta/TEnumType.h line:22
		pybind11::class_<mrpt::typemeta::internal::bimap<mrpt::img::DistortionModel,std::string>, std::shared_ptr<mrpt::typemeta::internal::bimap<mrpt::img::DistortionModel,std::string>>> cl(M("mrpt::typemeta::internal"), "bimap_mrpt_img_DistortionModel_std_string_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::typemeta::internal::bimap<mrpt::img::DistortionModel,std::string>(); } ) );
		cl.def( pybind11::init( [](mrpt::typemeta::internal::bimap<mrpt::img::DistortionModel,std::string> const &o){ return new mrpt::typemeta::internal::bimap<mrpt::img::DistortionModel,std::string>(o); } ) );
		cl.def_readwrite("m_k2v", &mrpt::typemeta::internal::bimap<mrpt::img::DistortionModel,std::string>::m_k2v);
		cl.def_readwrite("m_v2k", &mrpt::typemeta::internal::bimap<mrpt::img::DistortionModel,std::string>::m_v2k);
		cl.def("direct", (bool (mrpt::typemeta::internal::bimap<mrpt::img::DistortionModel,std::string>::*)(const enum mrpt::img::DistortionModel &, std::string &) const) &mrpt::typemeta::internal::bimap<mrpt::img::DistortionModel, std::string>::direct, "C++: mrpt::typemeta::internal::bimap<mrpt::img::DistortionModel, std::string>::direct(const enum mrpt::img::DistortionModel &, std::string &) const --> bool", pybind11::arg("k"), pybind11::arg("out_v"));
		cl.def("inverse", (bool (mrpt::typemeta::internal::bimap<mrpt::img::DistortionModel,std::string>::*)(const std::string &, enum mrpt::img::DistortionModel &) const) &mrpt::typemeta::internal::bimap<mrpt::img::DistortionModel, std::string>::inverse, "C++: mrpt::typemeta::internal::bimap<mrpt::img::DistortionModel, std::string>::inverse(const std::string &, enum mrpt::img::DistortionModel &) const --> bool", pybind11::arg("v"), pybind11::arg("out_k"));
		cl.def("insert", (void (mrpt::typemeta::internal::bimap<mrpt::img::DistortionModel,std::string>::*)(const enum mrpt::img::DistortionModel &, const std::string &)) &mrpt::typemeta::internal::bimap<mrpt::img::DistortionModel, std::string>::insert, "C++: mrpt::typemeta::internal::bimap<mrpt::img::DistortionModel, std::string>::insert(const enum mrpt::img::DistortionModel &, const std::string &) --> void", pybind11::arg("k"), pybind11::arg("v"));
		cl.def("assign", (struct mrpt::typemeta::internal::bimap<enum mrpt::img::DistortionModel, std::string > & (mrpt::typemeta::internal::bimap<mrpt::img::DistortionModel,std::string>::*)(const struct mrpt::typemeta::internal::bimap<enum mrpt::img::DistortionModel, std::string > &)) &mrpt::typemeta::internal::bimap<mrpt::img::DistortionModel, std::string>::operator=, "C++: mrpt::typemeta::internal::bimap<mrpt::img::DistortionModel, std::string>::operator=(const struct mrpt::typemeta::internal::bimap<enum mrpt::img::DistortionModel, std::string > &) --> struct mrpt::typemeta::internal::bimap<enum mrpt::img::DistortionModel, std::string > &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::typemeta::internal::bimap file:mrpt/typemeta/TEnumType.h line:22
		pybind11::class_<mrpt::typemeta::internal::bimap<mrpt::opengl::TCullFace,std::string>, std::shared_ptr<mrpt::typemeta::internal::bimap<mrpt::opengl::TCullFace,std::string>>> cl(M("mrpt::typemeta::internal"), "bimap_mrpt_opengl_TCullFace_std_string_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::typemeta::internal::bimap<mrpt::opengl::TCullFace,std::string>(); } ) );
		cl.def( pybind11::init( [](mrpt::typemeta::internal::bimap<mrpt::opengl::TCullFace,std::string> const &o){ return new mrpt::typemeta::internal::bimap<mrpt::opengl::TCullFace,std::string>(o); } ) );
		cl.def_readwrite("m_k2v", &mrpt::typemeta::internal::bimap<mrpt::opengl::TCullFace,std::string>::m_k2v);
		cl.def_readwrite("m_v2k", &mrpt::typemeta::internal::bimap<mrpt::opengl::TCullFace,std::string>::m_v2k);
		cl.def("direct", (bool (mrpt::typemeta::internal::bimap<mrpt::opengl::TCullFace,std::string>::*)(const enum mrpt::opengl::TCullFace &, std::string &) const) &mrpt::typemeta::internal::bimap<mrpt::opengl::TCullFace, std::string>::direct, "C++: mrpt::typemeta::internal::bimap<mrpt::opengl::TCullFace, std::string>::direct(const enum mrpt::opengl::TCullFace &, std::string &) const --> bool", pybind11::arg("k"), pybind11::arg("out_v"));
		cl.def("inverse", (bool (mrpt::typemeta::internal::bimap<mrpt::opengl::TCullFace,std::string>::*)(const std::string &, enum mrpt::opengl::TCullFace &) const) &mrpt::typemeta::internal::bimap<mrpt::opengl::TCullFace, std::string>::inverse, "C++: mrpt::typemeta::internal::bimap<mrpt::opengl::TCullFace, std::string>::inverse(const std::string &, enum mrpt::opengl::TCullFace &) const --> bool", pybind11::arg("v"), pybind11::arg("out_k"));
		cl.def("insert", (void (mrpt::typemeta::internal::bimap<mrpt::opengl::TCullFace,std::string>::*)(const enum mrpt::opengl::TCullFace &, const std::string &)) &mrpt::typemeta::internal::bimap<mrpt::opengl::TCullFace, std::string>::insert, "C++: mrpt::typemeta::internal::bimap<mrpt::opengl::TCullFace, std::string>::insert(const enum mrpt::opengl::TCullFace &, const std::string &) --> void", pybind11::arg("k"), pybind11::arg("v"));
		cl.def("assign", (struct mrpt::typemeta::internal::bimap<enum mrpt::opengl::TCullFace, std::string > & (mrpt::typemeta::internal::bimap<mrpt::opengl::TCullFace,std::string>::*)(const struct mrpt::typemeta::internal::bimap<enum mrpt::opengl::TCullFace, std::string > &)) &mrpt::typemeta::internal::bimap<mrpt::opengl::TCullFace, std::string>::operator=, "C++: mrpt::typemeta::internal::bimap<mrpt::opengl::TCullFace, std::string>::operator=(const struct mrpt::typemeta::internal::bimap<enum mrpt::opengl::TCullFace, std::string > &) --> struct mrpt::typemeta::internal::bimap<enum mrpt::opengl::TCullFace, std::string > &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
