/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#include "../include/bindings.h"

/* MRPT */
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DQuat.h>
#include <mrpt/poses/CPosePDF.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/poses/CPosePDFParticles.h>
#include <mrpt/poses/CPose3DPDF.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/poses/CPose3DPDFParticles.h>
#include <mrpt/serialization/CArchive.h>

/* STD */
#include <cmath>
#include <cstdint>

using namespace boost::python;
using namespace mrpt::poses;

// prototypes
#ifdef ROS_EXTENSIONS
object CPose3D_to_ROS_Pose_msg(CPose3D& self);
void CPose3D_from_ROS_Pose_msg(CPose3D& self, object pose_msg);
#endif
// end of prototypes

// CPose2D
double& (CPose2D::*CPose2D_get_x)() = &CPose2D::x;
void (CPose2D::*CPose2D_set_x)(double) = &CPose2D::x;
double& (CPose2D::*CPose2D_get_y)() = &CPose2D::y;
void (CPose2D::*CPose2D_set_y)(double) = &CPose2D::y;
double& (CPose2D::*CPose2D_get_phi)() = &CPose2D::phi;
void (CPose2D::*CPose2D_set_phi)(double) = &CPose2D::phi;

MAKE_AS_STR(CPose2D)

#ifdef ROS_EXTENSIONS
object CPose2D_to_ROS_Pose_msg(CPose2D& self)
{
	CPose3D pose(self);
	return CPose3D_to_ROS_Pose_msg(pose);
}

void CPose2D_from_ROS_Pose_msg(CPose2D& self, object pose_msg)
{
	CPose3D pose;
	CPose3D_from_ROS_Pose_msg(pose, pose_msg);
	self = CPose2D(pose);
}
#endif
// end of CPose2D

// CPose3D
double& (CPose3D::*CPose3D_get_x)() = &CPose3D::x;
void (CPose3D::*CPose3D_set_x)(double) = &CPose3D::x;
double& (CPose3D::*CPose3D_get_y)() = &CPose3D::y;
void (CPose3D::*CPose3D_set_y)(double) = &CPose3D::y;
double& (CPose3D::*CPose3D_get_z)() = &CPose3D::z;
void (CPose3D::*CPose3D_set_z)(double) = &CPose3D::z;

tuple CPose3D_getYawPitchRoll(CPose3D& self)
{
	list ret_val;
	double yaw, pitch, roll;
	self.getYawPitchRoll(yaw, pitch, roll);
	ret_val.append(yaw);
	ret_val.append(pitch);
	ret_val.append(roll);
	return tuple(ret_val);
}

MAKE_AS_STR(CPose3D)
MAKE_GETITEM(CPose3D, double)

#ifdef ROS_EXTENSIONS
object CPose3D_to_ROS_Pose_msg(CPose3D& self)
{
	CPose3DQuat pose_quat(self);
	// import msg
	dict locals;
	exec(
		"from geometry_msgs.msg import Pose\n"
		"pose_msg = Pose()\n",
		object(), locals);
	object pose_msg = locals["pose_msg"];
	pose_msg.attr("position").attr("x") = pose_quat[0];
	pose_msg.attr("position").attr("y") = pose_quat[1];
	pose_msg.attr("position").attr("z") = pose_quat[2];
	pose_msg.attr("orientation").attr("x") = pose_quat[4];
	pose_msg.attr("orientation").attr("y") = pose_quat[5];
	pose_msg.attr("orientation").attr("z") = pose_quat[6];
	pose_msg.attr("orientation").attr("w") = pose_quat[3];
	return pose_msg;
}

void CPose3D_from_ROS_Pose_msg(CPose3D& self, object pose_msg)
{
	CPose3DQuat pose_quat;
	pose_quat[0] = extract<double>(pose_msg.attr("position").attr("x"));
	pose_quat[1] = extract<double>(pose_msg.attr("position").attr("y"));
	pose_quat[2] = extract<double>(pose_msg.attr("position").attr("z"));
	pose_quat[4] = extract<double>(pose_msg.attr("orientation").attr("x"));
	pose_quat[5] = extract<double>(pose_msg.attr("orientation").attr("y"));
	pose_quat[6] = extract<double>(pose_msg.attr("orientation").attr("z"));
	pose_quat[3] = extract<double>(pose_msg.attr("orientation").attr("w"));
	self = CPose3D(pose_quat);
}
#endif
// end of CPose3D

// CPosePDF
// CObject *CPosePDFWrap::duplicate() const
// {
//     return this->get_override("duplicate")();
// }
//
// uint8_t CPosePDFWrap::serializeGetVersion() const { return XX; } void
// CPosePDFWrap::serializeTo(CStream& stream, int32_t* pos) const
// {
//     this->get_override("writeToStream")(stream, pos);
// }
//
// void CPosePDFWrap::serializeFrom(mrpt::serialization::CArchive& in, uint8_t
// version)
// {
//     this->get_override("readFromStream")(stream, pos);
// }
//
// void CPosePDFWrap::getMean(mrpt::poses::CPose2D &mean_point) const
// {
//     this->get_override("getMean")(mean_point);
// }
//
// void
// CPosePDFWrap::getCovarianceAndMean(mrpt::math::CMatrixFixedNumeric<double,3ul,3ul>
// &cov, mrpt::poses::CPose2D &mean_point) const
// {
//     this->get_override("getCovarianceAndMean")(cov, mean_point);
// }
//
// void CPosePDFWrap::saveToTextFile(const std::string &file) const
// {
//     this->get_override("saveToTextFile")(file);
// }
//
// void CPosePDFWrap::drawSingleSample(mrpt::poses::CPose2D &outPart) const
// {
//     this->get_override("drawSingleSample")(outPart);
// }
//
// void CPosePDFWrap::changeCoordinatesReference(const mrpt::poses::CPose3D
// &newReferenceBase)
// {
//     this->get_override("changeCoordinatesReference")(newReferenceBase);
// }
//
// void CPosePDFWrap::copyFrom(const CPosePDF &o)
// {
//     this->get_override("copyFrom")(o);
// }
//
// void CPosePDFWrap::bayesianFusion(const CPosePDF &p1, const CPosePDF &p2,
// const double &minMahalanobisDistToDrop)
// {
//     this->get_override("bayesianFusion")(p1, p2, minMahalanobisDistToDrop);
// }
//
// void CPosePDFWrap::inverse(CPosePDF &o) const
// {
//     this->get_override("inverse")(o);
// }
// // end of CPosePDF
//
// // CPose3DPDF
// struct CPose3DPDFWrap : CPose3DPDF, wrapper<CPose3DPDF>
// {
//     CObject *duplicate() const
//     {
//         return this->get_override("duplicate")();
//     }
//
//     void writeToStream(mrpt::utils::CStream& stream, int32_t* pos) const
//     {
//         this->get_override("writeToStream")(stream, pos);
//     }
//
//     void readFromStream(mrpt::utils::CStream& stream, int32_t pos)
//     {
//         this->get_override("readFromStream")(stream, pos);
//     }
//
//     void getMean(mrpt::poses::CPose3D &mean_point) const
//     {
//         this->get_override("getMean")(mean_point);
//     }
//
//     void getCovariance(mrpt::math::CMatrixFixedNumeric<double,6ul,6ul> &cov)
//     const
//     {
//         this->get_override("getCovariance")(cov);
//     }
//
//     void getCovarianceAndMean(mrpt::math::CMatrixFixedNumeric<double,6ul,6ul>
//     &cov, mrpt::poses::CPose3D &mean_point) const
//     {
//         this->get_override("getCovarianceAndMean")(cov, mean_point);
//     }
//
//     void saveToTextFile(const std::string &file) const
//     {
//         this->get_override("saveToTextFile")(file);
//     }
//
//     void changeCoordinatesReference(const mrpt::poses::CPose3D
//     &newReferenceBase)
//     {
//         this->get_override("changeCoordinatesReference")(newReferenceBase);
//     }
//
//     double getCovarianceEntropy() const
//     {
//         return this->get_override("getCovarianceEntropy")();
//     }
//
//     void drawSingleSample(mrpt::poses::CPose3D &outPart) const
//     {
//         this->get_override("drawSingleSample")(outPart);
//     }
//
//     void copyFrom(const CPose3DPDF &o)
//     {
//         this->get_override("copyFrom")(o);
//     }
//
//     void bayesianFusion(const CPose3DPDF &p1, const CPose3DPDF &p2)
//     {
//         this->get_override("bayesianFusion")(p1, p2);
//     }
//
//     void inverse(CPose3DPDF &o) const
//     {
//         this->get_override("inverse")(o);
//     }
// };

// CPosePDFGaussian
list CPosePDFGaussian_get_cov(CPosePDFGaussian& self)
{
	list cov;
	for (int32_t i = 0; i < 9; ++i)
	{
		cov.append(self.cov(i));
	}
	return cov;
}

void CPosePDFGaussian_set_cov(CPosePDFGaussian& self, list cov)
{
	for (int32_t i = 0; i < 9; ++i)
	{
		self.cov(i) = extract<double>(cov[i]);
	}
}
// end of CPosePDFGaussian

// CPose3DPDFGaussian
list CPose3DPDFGaussian_get_cov(CPose3DPDFGaussian& self)
{
	list cov;
	for (int32_t i = 0; i < 36; ++i)
	{
		cov.append(self.cov(i));
	}
	return cov;
}

void CPose3DPDFGaussian_set_cov(CPose3DPDFGaussian& self, list cov)
{
	for (int32_t i = 0; i < 36; ++i)
	{
		self.cov(i) = extract<double>(cov[i]);
	}
}

MAKE_AS_STR(CPose3DPDFGaussian)

#ifdef ROS_EXTENSIONS
object CPose3DPDFGaussian_to_ROS_PoseWithCovariance_msg(
	CPose3DPDFGaussian& self)
{
	CPose3DQuat pose_quat(self.mean);
	// import msg
	dict locals;
	exec(
		"from geometry_msgs.msg import PoseWithCovariance\n"
		"pose_msg = PoseWithCovariance()\n",
		object(), locals);
	object pose_msg = locals["pose_msg"];
	pose_msg.attr("pose") = CPose3D_to_ROS_Pose_msg(self.mean);

	// Read REP103: http://ros.org/reps/rep-0103.html#covariance-representation
	// # Row-major representation of the 6x6 covariance matrix
	// # The orientation parameters use a fixed-axis representation.
	// # In order, the parameters are:
	// # (x, y, z, rotation about X axis, rotation about Y axis, rotation about
	// Z axis)
	// float64[36] covariance
	// ==> MRPT uses non-fixed z-y-x and ROS uses fixed x-y-z rotations -->
	// which
	//     are equal except for the ordering --> only need to rearrange yaw,
	//     pitch, roll

	const unsigned int indxs_map[6] = {0, 1, 2,
									   5, 4, 3};  // X,Y,Z,YAW,PITCH,ROLL

	list mrptcov = CPose3DPDFGaussian_get_cov(self);
	pose_msg.attr("covariance") = list(mrptcov);

	for (int i = 0; i < 6; i++)
	{
		for (int j = 0; j < 6; j++)
		{
			pose_msg.attr("covariance")[indxs_map[i] * 6 + indxs_map[j]] =
				mrptcov(i * 6 + j);
		}
	}

	return pose_msg;
}

void CPose3DPDFGaussian_from_ROS_PoseWithCovariance_msg(
	CPose3DPDFGaussian& self, object pose_msg)
{
	self = CPose3DPDFGaussian();
	CPose3D_from_ROS_Pose_msg(self.mean, pose_msg.attr("pose"));

	list roscov = list(pose_msg.attr("covariance"));

	// rearrange covariance (see above)
	const unsigned int indxs_map[6] = {0, 1, 2,
									   5, 4, 3};  // X,Y,Z,YAW,PITCH,ROLL

	for (int i = 0; i < 6; i++)
	{
		for (int j = 0; j < 6; j++)
		{
			self.cov(i, j) =
				extract<double>(roscov[indxs_map[i] * 6 + indxs_map[j]]);
		}
	}
}
#endif
// end of CPose3DPDFGaussian

// CPosePDFParticles
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(
	CPosePDFParticles_resetUniform_overloads, resetUniform, 4, 7)
// end of CPosePDFParticles

// smart pointer contents
MAKE_PTR_CTX(CPose2D)
MAKE_PTR_CTX(CPosePDF)
MAKE_PTR_CTX(CPosePDFGaussian)
MAKE_PTR_CTX(CPosePDFParticles)
MAKE_PTR_CTX(CPose3D)
MAKE_PTR_CTX(CPose3DPDF)
MAKE_PTR_CTX(CPose3DPDFGaussian)
MAKE_PTR_CTX(CPose3DPDFParticles)

// exporter
void export_poses()
{
	// map namespace to be submodule of mrpt package
	MAKE_SUBMODULE(poses)

	// CPose2D
	{
		MAKE_PTR(CPose2D)

		class_<CPose2D>("CPose2D", init<>())
			.def(init<CPose2D>())
			.def(init<CPose3D>())
			.def(init<double, double, double>())
			.add_property(
				"x",
				make_function(
					CPose2D_get_x,
					return_value_policy<copy_non_const_reference>()),
				CPose2D_set_x)
			.add_property(
				"y",
				make_function(
					CPose2D_get_y,
					return_value_policy<copy_non_const_reference>()),
				CPose2D_set_y)
			.add_property(
				"phi",
				make_function(
					CPose2D_get_phi,
					return_value_policy<copy_non_const_reference>()),
				CPose2D_set_phi)
			.def(
				"inverse", &CPose2D::inverse,
				"Convert this pose into its inverse, saving the result in "
				"itself.")
			.def("norm", &CPose2D::norm)
			.def(
				"normalizePhi", &CPose2D::normalizePhi,
				"Forces \"phi\" to be in the range [-pi,pi];")
			.def("__str__", &CPose2D_asString)
			.def("distance2DTo", &CPose2D::distance2DTo)
			.def(
				"distance2DFrobeniusTo", &CPose2D::distance2DFrobeniusTo,
				"Returns the 2D distance from this pose/point to a 2D pose "
				"using the Frobenius distance.")
			.def(self + self)
			.def(self - self)
#ifdef ROS_EXTENSIONS
			.def(
				"to_ROS_Pose_msg", &CPose2D_to_ROS_Pose_msg,
				"Convert to ROS geometry_msgs/Pose.")
			.def(
				"from_ROS_Pose_msg", &CPose2D_from_ROS_Pose_msg,
				"Convert from ROS geometry_msgs/Pose.")
#endif
			;
	}

	// CPosePDF
	{
		MAKE_PTR(CPosePDF)

		class_<CPosePDF, boost::noncopyable>("CPosePDF", no_init)
			.def(
				"getMean", &CPosePDF::getMean,
				"Returns the mean, or mathematical expectation of the "
				"probability density distribution (PDF).")
			.def(
				"getCovarianceAndMean", &CPosePDF::getCovarianceAndMean,
				"Returns an estimate of the pose covariance matrix "
				"(STATE_LENxSTATE_LEN cov matrix) and the mean, both at once.")
			.def(
				"saveToTextFile", &CPosePDF::saveToTextFile,
				"Save PDF's particles to a text file. See derived classes for "
				"more information about the format of generated files.")
			.def(
				"copyFrom", &CPosePDF::copyFrom,
				"Copy operator, translating if necesary (for example, between "
				"particles and gaussian representations).")
			.def(
				"bayesianFusion", &CPosePDF::bayesianFusion,
				"Bayesian fusion of two pose distributions (product of two "
				"distributions->new distribution), then save the result in "
				"this object (WARNING: See implementing classes to see classes "
				"that can and cannot be mixtured!).")
			.def(
				"inverse", &CPosePDF::inverse,
				"Returns a new PDF such as: NEW_PDF = (0,0,0) - THIS_PDF");
	}

	// CPosePDFGaussian
	{
		MAKE_PTR(CPosePDFGaussian)

		class_<CPosePDFGaussian, bases<CPosePDF>>(
			"CPosePDFGaussian", init<optional<CPose2D>>())
			.def_readwrite("mean", &CPosePDFGaussian::mean)
			.add_property(
				"cov", CPosePDFGaussian_get_cov, CPosePDFGaussian_set_cov);
	}

	// CPosePDFParticles
	{
		MAKE_PTR(CPosePDFParticles)

		class_<CPosePDFParticles, bases<CPosePDF>>(
			"CPosePDFParticles", init<optional<size_t>>())
			.def(
				"resetUniform", &CPosePDFParticles::resetUniform,
				CPosePDFParticles_resetUniform_overloads())
			.def("getParticlePose", &CPosePDFParticles::getParticlePose);
	}

	// CPose3D
	{
		MAKE_PTR(CPose3D)

		class_<CPose3D>("CPose3D", init<optional<CPose2D>>())
			.def(init<CPose3D>())
			.def(init<double, double, double, double, double, double>())
			.add_property(
				"x",
				make_function(
					CPose3D_get_x,
					return_value_policy<copy_non_const_reference>()),
				CPose3D_set_x)
			.add_property(
				"y",
				make_function(
					CPose3D_get_y,
					return_value_policy<copy_non_const_reference>()),
				CPose3D_set_y)
			.add_property(
				"z",
				make_function(
					CPose3D_get_z,
					return_value_policy<copy_non_const_reference>()),
				CPose3D_set_z)
			.def("__getitem__", &CPose3D_getitem)
			.def(
				"setYawPitchRoll", &CPose3D::setYawPitchRoll,
				"Set the 3 angles of the 3D pose (in radians) - This method "
				"recomputes the internal rotation coordinates matrix.")
			.def(
				"getYawPitchRoll", &CPose3D_getYawPitchRoll,
				"Returns the three angles (yaw, pitch, roll), in radians, from "
				"the rotation matrix.")
			.def(
				"setFromValues", &CPose3D::setFromValues,
				"Set the pose from a 3D position (meters) and yaw/pitch/roll "
				"angles (radians) - This method recomputes the internal "
				"rotation matrix.")
			.def(
				"inverse", &CPose3D::inverse,
				"Convert this pose into its inverse, saving the result in "
				"itself.")
			.def("__str__", &CPose3D_asString)
			.def(self + self)
			.def(self - self)
#ifdef ROS_EXTENSIONS
			.def(
				"to_ROS_Pose_msg", &CPose3D_to_ROS_Pose_msg,
				"Convert to ROS geometry_msgs/Pose.")
			.def(
				"from_ROS_Pose_msg", &CPose3D_from_ROS_Pose_msg,
				"Convert from ROS geometry_msgs/Pose.")
#endif
			;
	}

	// CPose3DPDF
	{
		MAKE_PTR(CPose3DPDF)

		// dealing with overloading
		void (CPose3DPDF::*getCovariance1)(
			mrpt::math::CMatrixFixedNumeric<double, 6ul, 6ul> & cov) const =
			&CPose3DPDF::getCovariance;

		class_<CPose3DPDF, boost::noncopyable>("CPose3DPDF", no_init)
			.def(
				"jacobiansPoseComposition",
				&CPose3DPDF::jacobiansPoseComposition,
				"This static method computes the pose composition Jacobians.")
			.def("getMeanVal", &CPose3DPDF::getMeanVal)
			.def(
				"getMean", &CPose3DPDF::getMean,
				"Returns the mean, or mathematical expectation of the "
				"probability density distribution (PDF).")
			.def(
				"getCovariance", getCovariance1,
				"Returns an estimate of the pose covariance matrix "
				"(STATE_LENxSTATE_LEN cov matrix).")
			.def(
				"getCovarianceAndMean", &CPose3DPDF::getCovarianceAndMean,
				"Returns an estimate of the pose covariance matrix "
				"(STATE_LENxSTATE_LEN cov matrix) and the mean, both at once.")
			.def(
				"saveToTextFile", &CPose3DPDF::saveToTextFile,
				"Save PDF's particles to a text file. See derived classes for "
				"more information about the format of generated files.")
			.def(
				"changeCoordinatesReference",
				&CPose3DPDF::changeCoordinatesReference,
				"this = p (+) this. This can be used to convert a PDF from "
				"local coordinates to global, providing the point "
				"(newReferenceBase) from which 'to project' the current pdf. "
				"Result PDF substituted the currently stored one in the "
				"object.")
			.def(
				"getCovarianceEntropy", &CPose3DPDF::getCovarianceEntropy,
				"Compute the entropy of the estimated covariance matrix.")
			.def(
				"drawSingleSample", &CPose3DPDF::drawSingleSample,
				"Draws a single sample from the distribution")
			.def(
				"copyFrom", &CPose3DPDF::copyFrom,
				"Copy operator, translating if necesary (for example, between "
				"particles and gaussian representations).")
			.def(
				"bayesianFusion", &CPose3DPDF::bayesianFusion,
				"Bayesian fusion of two pose distributions (product of two "
				"distributions->new distribution), then save the result in "
				"this object (WARNING: See implementing classes to see classes "
				"that can and cannot be mixtured!).")
			.def(
				"inverse", &CPose3DPDF::inverse,
				"Returns a new PDF such as: NEW_PDF = (0,0,0) - THIS_PDF");
	}

	// CPose3DPDFGaussian
	{
		MAKE_PTR(CPose3DPDFGaussian)

		// dealing with overloading
		void (CPose3DPDF::*getCovariance1)(
			mrpt::math::CMatrixFixedNumeric<double, 6ul, 6ul> & cov) const =
			&CPose3DPDFGaussian::getCovariance;

		class_<CPose3DPDFGaussian, bases<CPose3DPDF>>(
			"CPose3DPDFGaussian", init<>())
			.def(init<CPose3DPDFGaussian>())
			.def(init<CPose3D>())
			.def(
				"getMean", &CPose3DPDFGaussian::getMean,
				"Returns the mean, or mathematical expectation of the "
				"probability density distribution (PDF).")
			.def(
				"getCovariance", getCovariance1,
				"Returns an estimate of the pose covariance matrix "
				"(STATE_LENxSTATE_LEN cov matrix).")
			.def(
				"getCovarianceAndMean",
				&CPose3DPDFGaussian::getCovarianceAndMean,
				"Returns an estimate of the pose covariance matrix "
				"(STATE_LENxSTATE_LEN cov matrix) and the mean, both at once.")
			.def(
				"saveToTextFile", &CPose3DPDFGaussian::saveToTextFile,
				"Save PDF's particles to a text file. See derived classes for "
				"more information about the format of generated files.")
			.def(
				"changeCoordinatesReference",
				&CPose3DPDF::changeCoordinatesReference,
				"this = p (+) this. This can be used to convert a PDF from "
				"local coordinates to global, providing the point "
				"(newReferenceBase) from which 'to project' the current pdf. "
				"Result PDF substituted the currently stored one in the "
				"object.")
			.def(
				"getCovarianceEntropy",
				&CPose3DPDFGaussian::getCovarianceEntropy,
				"Compute the entropy of the estimated covariance matrix.")
			.def(
				"drawSingleSample", &CPose3DPDFGaussian::drawSingleSample,
				"Draws a single sample from the distribution")
			//            .def("copyFrom", &CPose3DPDFGaussian::copyFrom, "Copy
			//            operator, translating if necesary (for example,
			//            between particles and gaussian representations).")
			.def(
				"bayesianFusion", &CPose3DPDF::bayesianFusion,
				"Bayesian fusion of two pose distributions (product of two "
				"distributions->new distribution), then save the result in "
				"this object (WARNING: See implementing classes to see classes "
				"that can and cannot be mixtured!).")
			.def(
				"inverse", &CPose3DPDFGaussian::inverse,
				"Returns a new PDF such as: NEW_PDF = (0,0,0) - THIS_PDF")
			.def(
				"jacobiansPoseComposition",
				&CPose3DPDFGaussian::jacobiansPoseComposition,
				"This static method computes the pose composition Jacobians.")
			.def("__str__", &CPose3DPDFGaussian_asString)
			.def_readwrite("mean", &CPose3DPDFGaussian::mean)
			// Note: the following does not work unfortunately: p.cov[35] = 1.0
			// FIXME: change cov to return actual matrix object (use
			// def_readwrite)
			.add_property(
				"cov", CPose3DPDFGaussian_get_cov, CPose3DPDFGaussian_set_cov)
			.def(self + self)
			.def(self - self)
			.def(-self)
			.def(self += self)
			.def(self -= self)
#ifdef ROS_EXTENSIONS
			.def(
				"to_ROS_PoseWithCovariance_msg",
				&CPose3DPDFGaussian_to_ROS_PoseWithCovariance_msg,
				"Convert to ROS geometry_msgs/PoseWithCovariance.")
			.def(
				"from_ROS_PoseWithCovariance_msg",
				&CPose3DPDFGaussian_from_ROS_PoseWithCovariance_msg,
				"Convert from ROS geometry_msgs/PoseWithCovariance.")
#endif
			;
	}

	// CPose3DPDFParticles
	{
		MAKE_PTR(CPose3DPDFParticles)

		class_<CPose3DPDFParticles, bases<CPose3DPDF>>(
			"CPose3DPDFParticles", init<>())
			.def(
				"particlesCount", &CPose3DPDFParticles::particlesCount,
				"Get the m_particles count.")
			.def(
				"getMean", &CPose3DPDFParticles::getMean,
				"Returns the mean, or mathematical expectation of the "
				"probability density distribution (PDF).");
	}
}

void export_poses_stl()
{
	MAKE_VEC(CPose2D)
	MAKE_VEC(CPose3D)
}
