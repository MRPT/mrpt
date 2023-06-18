#include <iterator>
#include <memory>
#include <mrpt/containers/deepcopy_poly_ptr.h>
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CVectorDynamic.h>
#include <mrpt/obs/CAction.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPosePDF.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/poses/CPosePDFSOG.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/typemeta/static_string.h>
#include <ostream>
#include <sstream> // __str__
#include <string>
#include <tuple>
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

void bind_mrpt_containers_deepcopy_poly_ptr(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::containers::deepcopy_poly_ptr file:mrpt/containers/deepcopy_poly_ptr.h line:25
		pybind11::class_<mrpt::containers::deepcopy_poly_ptr<std::shared_ptr<mrpt::poses::CPosePDF>>, std::shared_ptr<mrpt::containers::deepcopy_poly_ptr<std::shared_ptr<mrpt::poses::CPosePDF>>>> cl(M("mrpt::containers"), "deepcopy_poly_ptr_std_shared_ptr_mrpt_poses_CPosePDF_t", "");
		cl.def( pybind11::init<const class std::shared_ptr<class mrpt::poses::CPosePDF> &>(), pybind11::arg("ptr") );

		cl.def( pybind11::init( [](){ return new mrpt::containers::deepcopy_poly_ptr<std::shared_ptr<mrpt::poses::CPosePDF>>(); } ) );
		cl.def( pybind11::init( [](mrpt::containers::deepcopy_poly_ptr<std::shared_ptr<mrpt::poses::CPosePDF>> const &o){ return new mrpt::containers::deepcopy_poly_ptr<std::shared_ptr<mrpt::poses::CPosePDF>>(o); } ) );
		cl.def("assign", (class mrpt::containers::deepcopy_poly_ptr<class std::shared_ptr<class mrpt::poses::CPosePDF> > & (mrpt::containers::deepcopy_poly_ptr<std::shared_ptr<mrpt::poses::CPosePDF>>::*)(const class mrpt::containers::deepcopy_poly_ptr<class std::shared_ptr<class mrpt::poses::CPosePDF> > &)) &mrpt::containers::deepcopy_poly_ptr<std::shared_ptr<mrpt::poses::CPosePDF>>::operator=, "C++: mrpt::containers::deepcopy_poly_ptr<std::shared_ptr<mrpt::poses::CPosePDF>>::operator=(const class mrpt::containers::deepcopy_poly_ptr<class std::shared_ptr<class mrpt::poses::CPosePDF> > &) --> class mrpt::containers::deepcopy_poly_ptr<class std::shared_ptr<class mrpt::poses::CPosePDF> > &", pybind11::return_value_policy::automatic, pybind11::arg("o"));
		cl.def("assign", (class mrpt::containers::deepcopy_poly_ptr<class std::shared_ptr<class mrpt::poses::CPosePDF> > & (mrpt::containers::deepcopy_poly_ptr<std::shared_ptr<mrpt::poses::CPosePDF>>::*)(const class std::shared_ptr<class mrpt::poses::CPosePDF> &)) &mrpt::containers::deepcopy_poly_ptr<std::shared_ptr<mrpt::poses::CPosePDF>>::operator=, "C++: mrpt::containers::deepcopy_poly_ptr<std::shared_ptr<mrpt::poses::CPosePDF>>::operator=(const class std::shared_ptr<class mrpt::poses::CPosePDF> &) --> class mrpt::containers::deepcopy_poly_ptr<class std::shared_ptr<class mrpt::poses::CPosePDF> > &", pybind11::return_value_policy::automatic, pybind11::arg("o_ptr"));
		cl.def("get", (class mrpt::poses::CPosePDF * (mrpt::containers::deepcopy_poly_ptr<std::shared_ptr<mrpt::poses::CPosePDF>>::*)()) &mrpt::containers::deepcopy_poly_ptr<std::shared_ptr<mrpt::poses::CPosePDF>>::get, "C++: mrpt::containers::deepcopy_poly_ptr<std::shared_ptr<mrpt::poses::CPosePDF>>::get() --> class mrpt::poses::CPosePDF *", pybind11::return_value_policy::automatic);
		cl.def("arrow", (class mrpt::poses::CPosePDF * (mrpt::containers::deepcopy_poly_ptr<std::shared_ptr<mrpt::poses::CPosePDF>>::*)()) &mrpt::containers::deepcopy_poly_ptr<std::shared_ptr<mrpt::poses::CPosePDF>>::operator->, "C++: mrpt::containers::deepcopy_poly_ptr<std::shared_ptr<mrpt::poses::CPosePDF>>::operator->() --> class mrpt::poses::CPosePDF *", pybind11::return_value_policy::automatic);
		cl.def("dereference", (class mrpt::poses::CPosePDF & (mrpt::containers::deepcopy_poly_ptr<std::shared_ptr<mrpt::poses::CPosePDF>>::*)()) &mrpt::containers::deepcopy_poly_ptr<std::shared_ptr<mrpt::poses::CPosePDF>>::operator*, "C++: mrpt::containers::deepcopy_poly_ptr<std::shared_ptr<mrpt::poses::CPosePDF>>::operator*() --> class mrpt::poses::CPosePDF &", pybind11::return_value_policy::automatic);
		cl.def("get_ptr", (class std::shared_ptr<class mrpt::poses::CPosePDF> & (mrpt::containers::deepcopy_poly_ptr<std::shared_ptr<mrpt::poses::CPosePDF>>::*)()) &mrpt::containers::deepcopy_poly_ptr<std::shared_ptr<mrpt::poses::CPosePDF>>::get_ptr, "C++: mrpt::containers::deepcopy_poly_ptr<std::shared_ptr<mrpt::poses::CPosePDF>>::get_ptr() --> class std::shared_ptr<class mrpt::poses::CPosePDF> &", pybind11::return_value_policy::automatic);
		cl.def("reset", (void (mrpt::containers::deepcopy_poly_ptr<std::shared_ptr<mrpt::poses::CPosePDF>>::*)()) &mrpt::containers::deepcopy_poly_ptr<std::shared_ptr<mrpt::poses::CPosePDF>>::reset, "C++: mrpt::containers::deepcopy_poly_ptr<std::shared_ptr<mrpt::poses::CPosePDF>>::reset() --> void");
	}
	{ // mrpt::containers::deepcopy_poly_ptr file:mrpt/containers/deepcopy_poly_ptr.h line:25
		pybind11::class_<mrpt::containers::deepcopy_poly_ptr<std::shared_ptr<mrpt::obs::CAction>>, std::shared_ptr<mrpt::containers::deepcopy_poly_ptr<std::shared_ptr<mrpt::obs::CAction>>>> cl(M("mrpt::containers"), "deepcopy_poly_ptr_std_shared_ptr_mrpt_obs_CAction_t", "");
		cl.def( pybind11::init<const class std::shared_ptr<class mrpt::obs::CAction> &>(), pybind11::arg("ptr") );

		cl.def( pybind11::init( [](){ return new mrpt::containers::deepcopy_poly_ptr<std::shared_ptr<mrpt::obs::CAction>>(); } ) );
		cl.def( pybind11::init( [](mrpt::containers::deepcopy_poly_ptr<std::shared_ptr<mrpt::obs::CAction>> const &o){ return new mrpt::containers::deepcopy_poly_ptr<std::shared_ptr<mrpt::obs::CAction>>(o); } ) );
		cl.def("assign", (class mrpt::containers::deepcopy_poly_ptr<class std::shared_ptr<class mrpt::obs::CAction> > & (mrpt::containers::deepcopy_poly_ptr<std::shared_ptr<mrpt::obs::CAction>>::*)(const class mrpt::containers::deepcopy_poly_ptr<class std::shared_ptr<class mrpt::obs::CAction> > &)) &mrpt::containers::deepcopy_poly_ptr<std::shared_ptr<mrpt::obs::CAction>>::operator=, "C++: mrpt::containers::deepcopy_poly_ptr<std::shared_ptr<mrpt::obs::CAction>>::operator=(const class mrpt::containers::deepcopy_poly_ptr<class std::shared_ptr<class mrpt::obs::CAction> > &) --> class mrpt::containers::deepcopy_poly_ptr<class std::shared_ptr<class mrpt::obs::CAction> > &", pybind11::return_value_policy::automatic, pybind11::arg("o"));
		cl.def("assign", (class mrpt::containers::deepcopy_poly_ptr<class std::shared_ptr<class mrpt::obs::CAction> > & (mrpt::containers::deepcopy_poly_ptr<std::shared_ptr<mrpt::obs::CAction>>::*)(const class std::shared_ptr<class mrpt::obs::CAction> &)) &mrpt::containers::deepcopy_poly_ptr<std::shared_ptr<mrpt::obs::CAction>>::operator=, "C++: mrpt::containers::deepcopy_poly_ptr<std::shared_ptr<mrpt::obs::CAction>>::operator=(const class std::shared_ptr<class mrpt::obs::CAction> &) --> class mrpt::containers::deepcopy_poly_ptr<class std::shared_ptr<class mrpt::obs::CAction> > &", pybind11::return_value_policy::automatic, pybind11::arg("o_ptr"));
		cl.def("get", (class mrpt::obs::CAction * (mrpt::containers::deepcopy_poly_ptr<std::shared_ptr<mrpt::obs::CAction>>::*)()) &mrpt::containers::deepcopy_poly_ptr<std::shared_ptr<mrpt::obs::CAction>>::get, "C++: mrpt::containers::deepcopy_poly_ptr<std::shared_ptr<mrpt::obs::CAction>>::get() --> class mrpt::obs::CAction *", pybind11::return_value_policy::automatic);
		cl.def("arrow", (class mrpt::obs::CAction * (mrpt::containers::deepcopy_poly_ptr<std::shared_ptr<mrpt::obs::CAction>>::*)()) &mrpt::containers::deepcopy_poly_ptr<std::shared_ptr<mrpt::obs::CAction>>::operator->, "C++: mrpt::containers::deepcopy_poly_ptr<std::shared_ptr<mrpt::obs::CAction>>::operator->() --> class mrpt::obs::CAction *", pybind11::return_value_policy::automatic);
		cl.def("dereference", (class mrpt::obs::CAction & (mrpt::containers::deepcopy_poly_ptr<std::shared_ptr<mrpt::obs::CAction>>::*)()) &mrpt::containers::deepcopy_poly_ptr<std::shared_ptr<mrpt::obs::CAction>>::operator*, "C++: mrpt::containers::deepcopy_poly_ptr<std::shared_ptr<mrpt::obs::CAction>>::operator*() --> class mrpt::obs::CAction &", pybind11::return_value_policy::automatic);
		cl.def("get_ptr", (class std::shared_ptr<class mrpt::obs::CAction> & (mrpt::containers::deepcopy_poly_ptr<std::shared_ptr<mrpt::obs::CAction>>::*)()) &mrpt::containers::deepcopy_poly_ptr<std::shared_ptr<mrpt::obs::CAction>>::get_ptr, "C++: mrpt::containers::deepcopy_poly_ptr<std::shared_ptr<mrpt::obs::CAction>>::get_ptr() --> class std::shared_ptr<class mrpt::obs::CAction> &", pybind11::return_value_policy::automatic);
		cl.def("reset", (void (mrpt::containers::deepcopy_poly_ptr<std::shared_ptr<mrpt::obs::CAction>>::*)()) &mrpt::containers::deepcopy_poly_ptr<std::shared_ptr<mrpt::obs::CAction>>::reset, "C++: mrpt::containers::deepcopy_poly_ptr<std::shared_ptr<mrpt::obs::CAction>>::reset() --> void");
	}
	{ // mrpt::containers::deepcopy_poly_ptr file:mrpt/containers/deepcopy_poly_ptr.h line:25
		pybind11::class_<mrpt::containers::deepcopy_poly_ptr<std::shared_ptr<mrpt::poses::CPosePDFSOG>>, std::shared_ptr<mrpt::containers::deepcopy_poly_ptr<std::shared_ptr<mrpt::poses::CPosePDFSOG>>>> cl(M("mrpt::containers"), "deepcopy_poly_ptr_std_shared_ptr_mrpt_poses_CPosePDFSOG_t", "");
		cl.def( pybind11::init<const class std::shared_ptr<class mrpt::poses::CPosePDFSOG> &>(), pybind11::arg("ptr") );

		cl.def( pybind11::init( [](){ return new mrpt::containers::deepcopy_poly_ptr<std::shared_ptr<mrpt::poses::CPosePDFSOG>>(); } ) );
		cl.def( pybind11::init( [](mrpt::containers::deepcopy_poly_ptr<std::shared_ptr<mrpt::poses::CPosePDFSOG>> const &o){ return new mrpt::containers::deepcopy_poly_ptr<std::shared_ptr<mrpt::poses::CPosePDFSOG>>(o); } ) );
		cl.def("assign", (class mrpt::containers::deepcopy_poly_ptr<class std::shared_ptr<class mrpt::poses::CPosePDFSOG> > & (mrpt::containers::deepcopy_poly_ptr<std::shared_ptr<mrpt::poses::CPosePDFSOG>>::*)(const class mrpt::containers::deepcopy_poly_ptr<class std::shared_ptr<class mrpt::poses::CPosePDFSOG> > &)) &mrpt::containers::deepcopy_poly_ptr<std::shared_ptr<mrpt::poses::CPosePDFSOG>>::operator=, "C++: mrpt::containers::deepcopy_poly_ptr<std::shared_ptr<mrpt::poses::CPosePDFSOG>>::operator=(const class mrpt::containers::deepcopy_poly_ptr<class std::shared_ptr<class mrpt::poses::CPosePDFSOG> > &) --> class mrpt::containers::deepcopy_poly_ptr<class std::shared_ptr<class mrpt::poses::CPosePDFSOG> > &", pybind11::return_value_policy::automatic, pybind11::arg("o"));
		cl.def("assign", (class mrpt::containers::deepcopy_poly_ptr<class std::shared_ptr<class mrpt::poses::CPosePDFSOG> > & (mrpt::containers::deepcopy_poly_ptr<std::shared_ptr<mrpt::poses::CPosePDFSOG>>::*)(const class std::shared_ptr<class mrpt::poses::CPosePDFSOG> &)) &mrpt::containers::deepcopy_poly_ptr<std::shared_ptr<mrpt::poses::CPosePDFSOG>>::operator=, "C++: mrpt::containers::deepcopy_poly_ptr<std::shared_ptr<mrpt::poses::CPosePDFSOG>>::operator=(const class std::shared_ptr<class mrpt::poses::CPosePDFSOG> &) --> class mrpt::containers::deepcopy_poly_ptr<class std::shared_ptr<class mrpt::poses::CPosePDFSOG> > &", pybind11::return_value_policy::automatic, pybind11::arg("o_ptr"));
		cl.def("get", (class mrpt::poses::CPosePDFSOG * (mrpt::containers::deepcopy_poly_ptr<std::shared_ptr<mrpt::poses::CPosePDFSOG>>::*)()) &mrpt::containers::deepcopy_poly_ptr<std::shared_ptr<mrpt::poses::CPosePDFSOG>>::get, "C++: mrpt::containers::deepcopy_poly_ptr<std::shared_ptr<mrpt::poses::CPosePDFSOG>>::get() --> class mrpt::poses::CPosePDFSOG *", pybind11::return_value_policy::automatic);
		cl.def("arrow", (class mrpt::poses::CPosePDFSOG * (mrpt::containers::deepcopy_poly_ptr<std::shared_ptr<mrpt::poses::CPosePDFSOG>>::*)()) &mrpt::containers::deepcopy_poly_ptr<std::shared_ptr<mrpt::poses::CPosePDFSOG>>::operator->, "C++: mrpt::containers::deepcopy_poly_ptr<std::shared_ptr<mrpt::poses::CPosePDFSOG>>::operator->() --> class mrpt::poses::CPosePDFSOG *", pybind11::return_value_policy::automatic);
		cl.def("dereference", (class mrpt::poses::CPosePDFSOG & (mrpt::containers::deepcopy_poly_ptr<std::shared_ptr<mrpt::poses::CPosePDFSOG>>::*)()) &mrpt::containers::deepcopy_poly_ptr<std::shared_ptr<mrpt::poses::CPosePDFSOG>>::operator*, "C++: mrpt::containers::deepcopy_poly_ptr<std::shared_ptr<mrpt::poses::CPosePDFSOG>>::operator*() --> class mrpt::poses::CPosePDFSOG &", pybind11::return_value_policy::automatic);
		cl.def("get_ptr", (class std::shared_ptr<class mrpt::poses::CPosePDFSOG> & (mrpt::containers::deepcopy_poly_ptr<std::shared_ptr<mrpt::poses::CPosePDFSOG>>::*)()) &mrpt::containers::deepcopy_poly_ptr<std::shared_ptr<mrpt::poses::CPosePDFSOG>>::get_ptr, "C++: mrpt::containers::deepcopy_poly_ptr<std::shared_ptr<mrpt::poses::CPosePDFSOG>>::get_ptr() --> class std::shared_ptr<class mrpt::poses::CPosePDFSOG> &", pybind11::return_value_policy::automatic);
		cl.def("reset", (void (mrpt::containers::deepcopy_poly_ptr<std::shared_ptr<mrpt::poses::CPosePDFSOG>>::*)()) &mrpt::containers::deepcopy_poly_ptr<std::shared_ptr<mrpt::poses::CPosePDFSOG>>::reset, "C++: mrpt::containers::deepcopy_poly_ptr<std::shared_ptr<mrpt::poses::CPosePDFSOG>>::reset() --> void");
	}
}
