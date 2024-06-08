#include <ios>
#include <iterator>
#include <memory>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/obs/CActionRobotMovement2D.h>
#include <mrpt/poses/CPoint2D.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPoseOrPoint.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/CSerializable.h>
#include <mrpt/typemeta/static_string.h>
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

// mrpt::obs::CActionRobotMovement2D file:mrpt/obs/CActionRobotMovement2D.h line:31
struct PyCallBack_mrpt_obs_CActionRobotMovement2D : public mrpt::obs::CActionRobotMovement2D {
	using mrpt::obs::CActionRobotMovement2D::CActionRobotMovement2D;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CActionRobotMovement2D *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CActionRobotMovement2D::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CActionRobotMovement2D *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CActionRobotMovement2D::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CActionRobotMovement2D *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CActionRobotMovement2D::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CActionRobotMovement2D *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CActionRobotMovement2D::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CActionRobotMovement2D *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CActionRobotMovement2D::serializeFrom(a0, a1);
	}
};

void bind_mrpt_obs_CActionRobotMovement2D(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::obs::CActionRobotMovement2D file:mrpt/obs/CActionRobotMovement2D.h line:31
		pybind11::class_<mrpt::obs::CActionRobotMovement2D, std::shared_ptr<mrpt::obs::CActionRobotMovement2D>, PyCallBack_mrpt_obs_CActionRobotMovement2D, mrpt::obs::CAction> cl(M("mrpt::obs"), "CActionRobotMovement2D", "Represents a probabilistic 2D movement of the robot mobile base\n\n  See docs:\n https://docs.mrpt.org/reference/latest/tutorial-motion-models.html\n\n Velocity is encoded as mrpt::math::TTwist2D in the optional field\n velocityLocal.\n\n \n CAction\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::obs::CActionRobotMovement2D(); }, [](){ return new PyCallBack_mrpt_obs_CActionRobotMovement2D(); } ) );
		cl.def( pybind11::init( [](PyCallBack_mrpt_obs_CActionRobotMovement2D const &o){ return new PyCallBack_mrpt_obs_CActionRobotMovement2D(o); } ) );
		cl.def( pybind11::init( [](mrpt::obs::CActionRobotMovement2D const &o){ return new mrpt::obs::CActionRobotMovement2D(o); } ) );

		pybind11::enum_<mrpt::obs::CActionRobotMovement2D::TEstimationMethod>(cl, "TEstimationMethod", pybind11::arithmetic(), "A list of posible ways for estimating the content of a\n CActionRobotMovement2D object.")
			.value("emOdometry", mrpt::obs::CActionRobotMovement2D::emOdometry)
			.value("emScan2DMatching", mrpt::obs::CActionRobotMovement2D::emScan2DMatching)
			.export_values();


		pybind11::enum_<mrpt::obs::CActionRobotMovement2D::TDrawSampleMotionModel>(cl, "TDrawSampleMotionModel", pybind11::arithmetic(), "")
			.value("mmGaussian", mrpt::obs::CActionRobotMovement2D::mmGaussian)
			.value("mmThrun", mrpt::obs::CActionRobotMovement2D::mmThrun)
			.export_values();

		cl.def_readwrite("poseChange", &mrpt::obs::CActionRobotMovement2D::poseChange);
		cl.def_readwrite("rawOdometryIncrementReading", &mrpt::obs::CActionRobotMovement2D::rawOdometryIncrementReading);
		cl.def_readwrite("estimationMethod", &mrpt::obs::CActionRobotMovement2D::estimationMethod);
		cl.def_readwrite("hasEncodersInfo", &mrpt::obs::CActionRobotMovement2D::hasEncodersInfo);
		cl.def_readwrite("encoderLeftTicks", &mrpt::obs::CActionRobotMovement2D::encoderLeftTicks);
		cl.def_readwrite("encoderRightTicks", &mrpt::obs::CActionRobotMovement2D::encoderRightTicks);
		cl.def_readwrite("hasVelocities", &mrpt::obs::CActionRobotMovement2D::hasVelocities);
		cl.def_readwrite("velocityLocal", &mrpt::obs::CActionRobotMovement2D::velocityLocal);
		cl.def_readwrite("motionModelConfiguration", &mrpt::obs::CActionRobotMovement2D::motionModelConfiguration);
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::obs::CActionRobotMovement2D::GetRuntimeClassIdStatic, "C++: mrpt::obs::CActionRobotMovement2D::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::obs::CActionRobotMovement2D::*)() const) &mrpt::obs::CActionRobotMovement2D::GetRuntimeClass, "C++: mrpt::obs::CActionRobotMovement2D::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::obs::CActionRobotMovement2D::*)() const) &mrpt::obs::CActionRobotMovement2D::clone, "C++: mrpt::obs::CActionRobotMovement2D::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::obs::CActionRobotMovement2D::CreateObject, "C++: mrpt::obs::CActionRobotMovement2D::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("velocityLin", (double (mrpt::obs::CActionRobotMovement2D::*)() const) &mrpt::obs::CActionRobotMovement2D::velocityLin, "C++: mrpt::obs::CActionRobotMovement2D::velocityLin() const --> double");
		cl.def("velocityAng", (double (mrpt::obs::CActionRobotMovement2D::*)() const) &mrpt::obs::CActionRobotMovement2D::velocityAng, "C++: mrpt::obs::CActionRobotMovement2D::velocityAng() const --> double");
		cl.def("computeFromOdometry", (void (mrpt::obs::CActionRobotMovement2D::*)(const class mrpt::poses::CPose2D &, const struct mrpt::obs::CActionRobotMovement2D::TMotionModelOptions &)) &mrpt::obs::CActionRobotMovement2D::computeFromOdometry, "Computes the PDF of the pose increment from an odometry reading and\n according to the given motion model (speed and encoder ticks information\n is not modified).\n According to the parameters in the passed struct, it will be called one\n the private sampling functions (see \"see also\" next).\n \n\n computeFromOdometry_modelGaussian, computeFromOdometry_modelThrun\n\nC++: mrpt::obs::CActionRobotMovement2D::computeFromOdometry(const class mrpt::poses::CPose2D &, const struct mrpt::obs::CActionRobotMovement2D::TMotionModelOptions &) --> void", pybind11::arg("odometryIncrement"), pybind11::arg("options"));
		cl.def("computeFromEncoders", (void (mrpt::obs::CActionRobotMovement2D::*)(double, double, double)) &mrpt::obs::CActionRobotMovement2D::computeFromEncoders, "If \"hasEncodersInfo\"=true, this method updates the pose estimation\n according to the ticks from both encoders and the passed parameters,\n which is passed internally to the method \"computeFromOdometry\" with the\n last used PDF options (or the defualt ones if not explicitly called by\n the user).\n\n \n The meters / tick ratio for the left encoder.\n \n\n The meters / tick ratio for the right encoder.\n \n\n The distance between both wheels, in meters.\n\nC++: mrpt::obs::CActionRobotMovement2D::computeFromEncoders(double, double, double) --> void", pybind11::arg("K_left"), pybind11::arg("K_right"), pybind11::arg("D"));
		cl.def("drawSingleSample", (void (mrpt::obs::CActionRobotMovement2D::*)(class mrpt::poses::CPose2D &) const) &mrpt::obs::CActionRobotMovement2D::drawSingleSample, "Using this method instead of \"poseChange->drawSingleSample()\" may be\n more efficient in most situations.\n \n\n CPosePDF::drawSingleSample\n\nC++: mrpt::obs::CActionRobotMovement2D::drawSingleSample(class mrpt::poses::CPose2D &) const --> void", pybind11::arg("outSample"));
		cl.def("prepareFastDrawSingleSamples", (void (mrpt::obs::CActionRobotMovement2D::*)() const) &mrpt::obs::CActionRobotMovement2D::prepareFastDrawSingleSamples, "Call this before calling a high number of times \"fastDrawSingleSample\",\n which is much faster than \"drawSingleSample\"\n\nC++: mrpt::obs::CActionRobotMovement2D::prepareFastDrawSingleSamples() const --> void");
		cl.def("fastDrawSingleSample", (void (mrpt::obs::CActionRobotMovement2D::*)(class mrpt::poses::CPose2D &) const) &mrpt::obs::CActionRobotMovement2D::fastDrawSingleSample, "Faster version than \"drawSingleSample\", but requires a previous call to\n \"prepareFastDrawSingleSamples\"\n\nC++: mrpt::obs::CActionRobotMovement2D::fastDrawSingleSample(class mrpt::poses::CPose2D &) const --> void", pybind11::arg("outSample"));
		cl.def("assign", (class mrpt::obs::CActionRobotMovement2D & (mrpt::obs::CActionRobotMovement2D::*)(const class mrpt::obs::CActionRobotMovement2D &)) &mrpt::obs::CActionRobotMovement2D::operator=, "C++: mrpt::obs::CActionRobotMovement2D::operator=(const class mrpt::obs::CActionRobotMovement2D &) --> class mrpt::obs::CActionRobotMovement2D &", pybind11::return_value_policy::automatic, pybind11::arg(""));

		{ // mrpt::obs::CActionRobotMovement2D::TMotionModelOptions file:mrpt/obs/CActionRobotMovement2D.h line:79
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::obs::CActionRobotMovement2D::TMotionModelOptions, std::shared_ptr<mrpt::obs::CActionRobotMovement2D::TMotionModelOptions>> cl(enclosing_class, "TMotionModelOptions", "The parameter to be passed to \"computeFromOdometry\". ");
			cl.def( pybind11::init( [](){ return new mrpt::obs::CActionRobotMovement2D::TMotionModelOptions(); } ) );
			cl.def( pybind11::init( [](mrpt::obs::CActionRobotMovement2D::TMotionModelOptions const &o){ return new mrpt::obs::CActionRobotMovement2D::TMotionModelOptions(o); } ) );
			cl.def_readwrite("modelSelection", &mrpt::obs::CActionRobotMovement2D::TMotionModelOptions::modelSelection);
			cl.def_readwrite("gaussianModel", &mrpt::obs::CActionRobotMovement2D::TMotionModelOptions::gaussianModel);
			cl.def_readwrite("thrunModel", &mrpt::obs::CActionRobotMovement2D::TMotionModelOptions::thrunModel);
			cl.def("assign", (struct mrpt::obs::CActionRobotMovement2D::TMotionModelOptions & (mrpt::obs::CActionRobotMovement2D::TMotionModelOptions::*)(const struct mrpt::obs::CActionRobotMovement2D::TMotionModelOptions &)) &mrpt::obs::CActionRobotMovement2D::TMotionModelOptions::operator=, "C++: mrpt::obs::CActionRobotMovement2D::TMotionModelOptions::operator=(const struct mrpt::obs::CActionRobotMovement2D::TMotionModelOptions &) --> struct mrpt::obs::CActionRobotMovement2D::TMotionModelOptions &", pybind11::return_value_policy::automatic, pybind11::arg(""));

			{ // mrpt::obs::CActionRobotMovement2D::TMotionModelOptions::TOptions_GaussianModel file:mrpt/obs/CActionRobotMovement2D.h line:92
				auto & enclosing_class = cl;
				pybind11::class_<mrpt::obs::CActionRobotMovement2D::TMotionModelOptions::TOptions_GaussianModel, std::shared_ptr<mrpt::obs::CActionRobotMovement2D::TMotionModelOptions::TOptions_GaussianModel>> cl(enclosing_class, "TOptions_GaussianModel", "Options for the gaussian model, which generates a CPosePDFGaussian\n object in poseChange using a closed-form linear Gaussian model.\n See docs in:\n https://docs.mrpt.org/reference/latest/tutorial-motion-models.html");
				cl.def( pybind11::init( [](){ return new mrpt::obs::CActionRobotMovement2D::TMotionModelOptions::TOptions_GaussianModel(); } ) );
				cl.def( pybind11::init<double, double, double, double, double, double>(), pybind11::arg("a1_"), pybind11::arg("a2_"), pybind11::arg("a3_"), pybind11::arg("a4_"), pybind11::arg("minStdXY_"), pybind11::arg("minStdPHI_") );

				cl.def( pybind11::init( [](mrpt::obs::CActionRobotMovement2D::TMotionModelOptions::TOptions_GaussianModel const &o){ return new mrpt::obs::CActionRobotMovement2D::TMotionModelOptions::TOptions_GaussianModel(o); } ) );
				cl.def_readwrite("a1", &mrpt::obs::CActionRobotMovement2D::TMotionModelOptions::TOptions_GaussianModel::a1);
				cl.def_readwrite("a2", &mrpt::obs::CActionRobotMovement2D::TMotionModelOptions::TOptions_GaussianModel::a2);
				cl.def_readwrite("a3", &mrpt::obs::CActionRobotMovement2D::TMotionModelOptions::TOptions_GaussianModel::a3);
				cl.def_readwrite("a4", &mrpt::obs::CActionRobotMovement2D::TMotionModelOptions::TOptions_GaussianModel::a4);
				cl.def_readwrite("minStdXY", &mrpt::obs::CActionRobotMovement2D::TMotionModelOptions::TOptions_GaussianModel::minStdXY);
				cl.def_readwrite("minStdPHI", &mrpt::obs::CActionRobotMovement2D::TMotionModelOptions::TOptions_GaussianModel::minStdPHI);
				cl.def("assign", (struct mrpt::obs::CActionRobotMovement2D::TMotionModelOptions::TOptions_GaussianModel & (mrpt::obs::CActionRobotMovement2D::TMotionModelOptions::TOptions_GaussianModel::*)(const struct mrpt::obs::CActionRobotMovement2D::TMotionModelOptions::TOptions_GaussianModel &)) &mrpt::obs::CActionRobotMovement2D::TMotionModelOptions::TOptions_GaussianModel::operator=, "C++: mrpt::obs::CActionRobotMovement2D::TMotionModelOptions::TOptions_GaussianModel::operator=(const struct mrpt::obs::CActionRobotMovement2D::TMotionModelOptions::TOptions_GaussianModel &) --> struct mrpt::obs::CActionRobotMovement2D::TMotionModelOptions::TOptions_GaussianModel &", pybind11::return_value_policy::automatic, pybind11::arg(""));
			}

			{ // mrpt::obs::CActionRobotMovement2D::TMotionModelOptions::TOptions_ThrunModel file:mrpt/obs/CActionRobotMovement2D.h line:122
				auto & enclosing_class = cl;
				pybind11::class_<mrpt::obs::CActionRobotMovement2D::TMotionModelOptions::TOptions_ThrunModel, std::shared_ptr<mrpt::obs::CActionRobotMovement2D::TMotionModelOptions::TOptions_ThrunModel>> cl(enclosing_class, "TOptions_ThrunModel", "Options for the Thrun's model, which generates a CPosePDFParticles\n object in poseChange using a MonteCarlo simulation.\n See docs in:\n https://docs.mrpt.org/reference/latest/tutorial-motion-models.html");
				cl.def( pybind11::init( [](){ return new mrpt::obs::CActionRobotMovement2D::TMotionModelOptions::TOptions_ThrunModel(); } ) );
				cl.def( pybind11::init( [](mrpt::obs::CActionRobotMovement2D::TMotionModelOptions::TOptions_ThrunModel const &o){ return new mrpt::obs::CActionRobotMovement2D::TMotionModelOptions::TOptions_ThrunModel(o); } ) );
				cl.def_readwrite("nParticlesCount", &mrpt::obs::CActionRobotMovement2D::TMotionModelOptions::TOptions_ThrunModel::nParticlesCount);
				cl.def_readwrite("alfa1_rot_rot", &mrpt::obs::CActionRobotMovement2D::TMotionModelOptions::TOptions_ThrunModel::alfa1_rot_rot);
				cl.def_readwrite("alfa2_rot_trans", &mrpt::obs::CActionRobotMovement2D::TMotionModelOptions::TOptions_ThrunModel::alfa2_rot_trans);
				cl.def_readwrite("alfa3_trans_trans", &mrpt::obs::CActionRobotMovement2D::TMotionModelOptions::TOptions_ThrunModel::alfa3_trans_trans);
				cl.def_readwrite("alfa4_trans_rot", &mrpt::obs::CActionRobotMovement2D::TMotionModelOptions::TOptions_ThrunModel::alfa4_trans_rot);
				cl.def_readwrite("additional_std_XY", &mrpt::obs::CActionRobotMovement2D::TMotionModelOptions::TOptions_ThrunModel::additional_std_XY);
				cl.def_readwrite("additional_std_phi", &mrpt::obs::CActionRobotMovement2D::TMotionModelOptions::TOptions_ThrunModel::additional_std_phi);
				cl.def("assign", (struct mrpt::obs::CActionRobotMovement2D::TMotionModelOptions::TOptions_ThrunModel & (mrpt::obs::CActionRobotMovement2D::TMotionModelOptions::TOptions_ThrunModel::*)(const struct mrpt::obs::CActionRobotMovement2D::TMotionModelOptions::TOptions_ThrunModel &)) &mrpt::obs::CActionRobotMovement2D::TMotionModelOptions::TOptions_ThrunModel::operator=, "C++: mrpt::obs::CActionRobotMovement2D::TMotionModelOptions::TOptions_ThrunModel::operator=(const struct mrpt::obs::CActionRobotMovement2D::TMotionModelOptions::TOptions_ThrunModel &) --> struct mrpt::obs::CActionRobotMovement2D::TMotionModelOptions::TOptions_ThrunModel &", pybind11::return_value_policy::automatic, pybind11::arg(""));
			}

		}

	}
}
