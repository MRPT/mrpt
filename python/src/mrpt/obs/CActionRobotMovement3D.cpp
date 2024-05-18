#include <ios>
#include <iterator>
#include <memory>
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CQuaternion.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/obs/CActionRobotMovement3D.h>
#include <mrpt/poses/CPoint2D.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DQuat.h>
#include <mrpt/poses/CPoseOrPoint.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/CSerializable.h>
#include <mrpt/typemeta/static_string.h>
#include <optional>
#include <ostream>
#include <sstream> // __str__
#include <streambuf>
#include <string>
#include <variant>

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

// mrpt::obs::CActionRobotMovement3D file:mrpt/obs/CActionRobotMovement3D.h line:28
struct PyCallBack_mrpt_obs_CActionRobotMovement3D : public mrpt::obs::CActionRobotMovement3D {
	using mrpt::obs::CActionRobotMovement3D::CActionRobotMovement3D;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CActionRobotMovement3D *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CActionRobotMovement3D::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CActionRobotMovement3D *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CActionRobotMovement3D::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CActionRobotMovement3D *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CActionRobotMovement3D::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CActionRobotMovement3D *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CActionRobotMovement3D::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CActionRobotMovement3D *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CActionRobotMovement3D::serializeFrom(a0, a1);
	}
};

void bind_mrpt_obs_CActionRobotMovement3D(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::obs::CActionRobotMovement3D file:mrpt/obs/CActionRobotMovement3D.h line:28
		pybind11::class_<mrpt::obs::CActionRobotMovement3D, std::shared_ptr<mrpt::obs::CActionRobotMovement3D>, PyCallBack_mrpt_obs_CActionRobotMovement3D, mrpt::obs::CAction> cl(M("mrpt::obs"), "CActionRobotMovement3D", "Represents a probabilistic motion increment in SE(3).\n\n Odometry increments might be determined from visual odometry for full 3D, or\n from wheel encoders for 2D movements only.\n\n The implemented model for creating a SE(3) Gaussian from an odometry\n increment is based on \n\n \n\n \n CAction, CActionRobotMovement3D,");
		cl.def( pybind11::init( [](){ return new mrpt::obs::CActionRobotMovement3D(); }, [](){ return new PyCallBack_mrpt_obs_CActionRobotMovement3D(); } ) );
		cl.def( pybind11::init( [](PyCallBack_mrpt_obs_CActionRobotMovement3D const &o){ return new PyCallBack_mrpt_obs_CActionRobotMovement3D(o); } ) );
		cl.def( pybind11::init( [](mrpt::obs::CActionRobotMovement3D const &o){ return new mrpt::obs::CActionRobotMovement3D(o); } ) );

		pybind11::enum_<mrpt::obs::CActionRobotMovement3D::TEstimationMethod>(cl, "TEstimationMethod", pybind11::arithmetic(), "A list of posible ways for estimating the content of a\n CActionRobotMovement3D object.")
			.value("emOdometry", mrpt::obs::CActionRobotMovement3D::emOdometry)
			.value("emVisualOdometry", mrpt::obs::CActionRobotMovement3D::emVisualOdometry)
			.export_values();


		pybind11::enum_<mrpt::obs::CActionRobotMovement3D::TDrawSampleMotionModel>(cl, "TDrawSampleMotionModel", pybind11::arithmetic(), "")
			.value("mmGaussian", mrpt::obs::CActionRobotMovement3D::mmGaussian)
			.value("mm6DOF", mrpt::obs::CActionRobotMovement3D::mm6DOF)
			.export_values();

		cl.def_readwrite("poseChange", &mrpt::obs::CActionRobotMovement3D::poseChange);
		cl.def_readwrite("rawOdometryIncrementReading", &mrpt::obs::CActionRobotMovement3D::rawOdometryIncrementReading);
		cl.def_readwrite("estimationMethod", &mrpt::obs::CActionRobotMovement3D::estimationMethod);
		cl.def_readwrite("motionModelConfiguration", &mrpt::obs::CActionRobotMovement3D::motionModelConfiguration);
		cl.def_readwrite("hasVelocities", &mrpt::obs::CActionRobotMovement3D::hasVelocities);
		cl.def_readwrite("velocities", &mrpt::obs::CActionRobotMovement3D::velocities);
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::obs::CActionRobotMovement3D::GetRuntimeClassIdStatic, "C++: mrpt::obs::CActionRobotMovement3D::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::obs::CActionRobotMovement3D::*)() const) &mrpt::obs::CActionRobotMovement3D::GetRuntimeClass, "C++: mrpt::obs::CActionRobotMovement3D::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::obs::CActionRobotMovement3D::*)() const) &mrpt::obs::CActionRobotMovement3D::clone, "C++: mrpt::obs::CActionRobotMovement3D::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::obs::CActionRobotMovement3D::CreateObject, "C++: mrpt::obs::CActionRobotMovement3D::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("computeFromOdometry", (void (mrpt::obs::CActionRobotMovement3D::*)(const class mrpt::poses::CPose3D &, const struct mrpt::obs::CActionRobotMovement3D::TMotionModelOptions &)) &mrpt::obs::CActionRobotMovement3D::computeFromOdometry, "Computes the PDF of the pose increment from an odometry reading and\n according to the given motion model (speed and encoder ticks information\n is not modified).\n According to the parameters in the passed struct, it will be called one\n the private sampling functions (see \"see also\" next).\n \n\n computeFromOdometry_model6DOF\n\nC++: mrpt::obs::CActionRobotMovement3D::computeFromOdometry(const class mrpt::poses::CPose3D &, const struct mrpt::obs::CActionRobotMovement3D::TMotionModelOptions &) --> void", pybind11::arg("odometryIncrement"), pybind11::arg("options"));
		cl.def("computeFromOdometry_model6DOF", (void (mrpt::obs::CActionRobotMovement3D::*)(const class mrpt::poses::CPose3D &, const struct mrpt::obs::CActionRobotMovement3D::TMotionModelOptions &)) &mrpt::obs::CActionRobotMovement3D::computeFromOdometry_model6DOF, "Computes the PDF of the pose increment from an odometry reading, using\n the motion model for 6 DOF.\n\n Based on: \n\n \n computeFromOdometry\n\nC++: mrpt::obs::CActionRobotMovement3D::computeFromOdometry_model6DOF(const class mrpt::poses::CPose3D &, const struct mrpt::obs::CActionRobotMovement3D::TMotionModelOptions &) --> void", pybind11::arg("odometryIncrement"), pybind11::arg("o"));
		cl.def("assign", (class mrpt::obs::CActionRobotMovement3D & (mrpt::obs::CActionRobotMovement3D::*)(const class mrpt::obs::CActionRobotMovement3D &)) &mrpt::obs::CActionRobotMovement3D::operator=, "C++: mrpt::obs::CActionRobotMovement3D::operator=(const class mrpt::obs::CActionRobotMovement3D &) --> class mrpt::obs::CActionRobotMovement3D &", pybind11::return_value_policy::automatic, pybind11::arg(""));

		{ // mrpt::obs::CActionRobotMovement3D::TMotionModelOptions file:mrpt/obs/CActionRobotMovement3D.h line:65
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::obs::CActionRobotMovement3D::TMotionModelOptions, std::shared_ptr<mrpt::obs::CActionRobotMovement3D::TMotionModelOptions>> cl(enclosing_class, "TMotionModelOptions", "The parameter to be passed to \"computeFromOdometry\".\n See:  ");
			cl.def( pybind11::init( [](){ return new mrpt::obs::CActionRobotMovement3D::TMotionModelOptions(); } ) );
			cl.def( pybind11::init( [](mrpt::obs::CActionRobotMovement3D::TMotionModelOptions const &o){ return new mrpt::obs::CActionRobotMovement3D::TMotionModelOptions(o); } ) );
			cl.def_readwrite("modelSelection", &mrpt::obs::CActionRobotMovement3D::TMotionModelOptions::modelSelection);
			cl.def_readwrite("mm6DOFModel", &mrpt::obs::CActionRobotMovement3D::TMotionModelOptions::mm6DOFModel);
			cl.def("assign", (struct mrpt::obs::CActionRobotMovement3D::TMotionModelOptions & (mrpt::obs::CActionRobotMovement3D::TMotionModelOptions::*)(const struct mrpt::obs::CActionRobotMovement3D::TMotionModelOptions &)) &mrpt::obs::CActionRobotMovement3D::TMotionModelOptions::operator=, "C++: mrpt::obs::CActionRobotMovement3D::TMotionModelOptions::operator=(const struct mrpt::obs::CActionRobotMovement3D::TMotionModelOptions &) --> struct mrpt::obs::CActionRobotMovement3D::TMotionModelOptions &", pybind11::return_value_policy::automatic, pybind11::arg(""));

			{ // mrpt::obs::CActionRobotMovement3D::TMotionModelOptions::TOptions_6DOFModel file:mrpt/obs/CActionRobotMovement3D.h line:72
				auto & enclosing_class = cl;
				pybind11::class_<mrpt::obs::CActionRobotMovement3D::TMotionModelOptions::TOptions_6DOFModel, std::shared_ptr<mrpt::obs::CActionRobotMovement3D::TMotionModelOptions::TOptions_6DOFModel>> cl(enclosing_class, "TOptions_6DOFModel", "");
				cl.def( pybind11::init( [](){ return new mrpt::obs::CActionRobotMovement3D::TMotionModelOptions::TOptions_6DOFModel(); } ) );
				cl.def( pybind11::init( [](mrpt::obs::CActionRobotMovement3D::TMotionModelOptions::TOptions_6DOFModel const &o){ return new mrpt::obs::CActionRobotMovement3D::TMotionModelOptions::TOptions_6DOFModel(o); } ) );
				cl.def_readwrite("nParticlesCount", &mrpt::obs::CActionRobotMovement3D::TMotionModelOptions::TOptions_6DOFModel::nParticlesCount);
				cl.def_readwrite("a1", &mrpt::obs::CActionRobotMovement3D::TMotionModelOptions::TOptions_6DOFModel::a1);
				cl.def_readwrite("a2", &mrpt::obs::CActionRobotMovement3D::TMotionModelOptions::TOptions_6DOFModel::a2);
				cl.def_readwrite("a3", &mrpt::obs::CActionRobotMovement3D::TMotionModelOptions::TOptions_6DOFModel::a3);
				cl.def_readwrite("a4", &mrpt::obs::CActionRobotMovement3D::TMotionModelOptions::TOptions_6DOFModel::a4);
				cl.def_readwrite("a5", &mrpt::obs::CActionRobotMovement3D::TMotionModelOptions::TOptions_6DOFModel::a5);
				cl.def_readwrite("a6", &mrpt::obs::CActionRobotMovement3D::TMotionModelOptions::TOptions_6DOFModel::a6);
				cl.def_readwrite("a7", &mrpt::obs::CActionRobotMovement3D::TMotionModelOptions::TOptions_6DOFModel::a7);
				cl.def_readwrite("a8", &mrpt::obs::CActionRobotMovement3D::TMotionModelOptions::TOptions_6DOFModel::a8);
				cl.def_readwrite("a9", &mrpt::obs::CActionRobotMovement3D::TMotionModelOptions::TOptions_6DOFModel::a9);
				cl.def_readwrite("a10", &mrpt::obs::CActionRobotMovement3D::TMotionModelOptions::TOptions_6DOFModel::a10);
				cl.def_readwrite("additional_std_XYZ", &mrpt::obs::CActionRobotMovement3D::TMotionModelOptions::TOptions_6DOFModel::additional_std_XYZ);
				cl.def_readwrite("additional_std_angle", &mrpt::obs::CActionRobotMovement3D::TMotionModelOptions::TOptions_6DOFModel::additional_std_angle);
				cl.def("assign", (struct mrpt::obs::CActionRobotMovement3D::TMotionModelOptions::TOptions_6DOFModel & (mrpt::obs::CActionRobotMovement3D::TMotionModelOptions::TOptions_6DOFModel::*)(const struct mrpt::obs::CActionRobotMovement3D::TMotionModelOptions::TOptions_6DOFModel &)) &mrpt::obs::CActionRobotMovement3D::TMotionModelOptions::TOptions_6DOFModel::operator=, "C++: mrpt::obs::CActionRobotMovement3D::TMotionModelOptions::TOptions_6DOFModel::operator=(const struct mrpt::obs::CActionRobotMovement3D::TMotionModelOptions::TOptions_6DOFModel &) --> struct mrpt::obs::CActionRobotMovement3D::TMotionModelOptions::TOptions_6DOFModel &", pybind11::return_value_policy::automatic, pybind11::arg(""));
			}

		}

	}
}
