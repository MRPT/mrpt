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

#include <mrpt/utils/CStream.h>

/* BOOST */
#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

#include "bindings.h"
#include "poses_bindings.h"

#include "math.h"

/* STD */
#include <stdint.h>

using namespace boost::python;
using namespace mrpt::poses;
using namespace mrpt::utils;

// prototypes
#ifdef ROS_EXTENSIONS
object CPose3D_to_ROS_Pose_msg(CPose3D &self);
void CPose3D_from_ROS_Pose_msg(CPose3D &self, object pose_msg);
#endif
// end of prototypes


// CPose2D
double &(CPose2D::*CPose2D_get_x)()             = &CPose2D::x;
void    (CPose2D::*CPose2D_set_x)(double)       = &CPose2D::x;
double &(CPose2D::*CPose2D_get_y)()             = &CPose2D::y;
void    (CPose2D::*CPose2D_set_y)(double)       = &CPose2D::y;
double &(CPose2D::*CPose2D_get_phi)()           = &CPose2D::phi;
void    (CPose2D::*CPose2D_set_phi)(double)     = &CPose2D::phi;

std::string CPose2D_asString(CPose2D &self)
{
    return self.asString();
}

#ifdef ROS_EXTENSIONS
object CPose2D_to_ROS_Pose_msg(CPose2D &self)
{
    CPose3D pose(self);
    return CPose3D_to_ROS_Pose_msg(pose);
}

void CPose2D_from_ROS_Pose_msg(CPose2D &self, object pose_msg)
{
    CPose3D pose;
    CPose3D_from_ROS_Pose_msg(pose, pose_msg);
    self = CPose2D(pose);
}
#endif
// end of CPose2D

// CPose3D
double &(CPose3D::*CPose3D_get_x)()             = &CPose3D::x;
void    (CPose3D::*CPose3D_set_x)(double)       = &CPose3D::x;
double &(CPose3D::*CPose3D_get_y)()             = &CPose3D::y;
void    (CPose3D::*CPose3D_set_y)(double)       = &CPose3D::y;
double &(CPose3D::*CPose3D_get_z)()             = &CPose3D::z;
void    (CPose3D::*CPose3D_set_z)(double)       = &CPose3D::z;

tuple CPose3D_getYawPitchRoll(CPose3D &self)
{
    list ret_val;
    double yaw, pitch, roll;
    self.getYawPitchRoll(yaw, pitch, roll);
    ret_val.append(yaw);
    ret_val.append(pitch);
    ret_val.append(roll);
    return tuple(ret_val);
}

#ifdef ROS_EXTENSIONS
object CPose3D_to_ROS_Pose_msg(CPose3D &self)
{
    CPose3DQuat pose_quat(self);
    // import msg
    dict locals;
    exec("from geometry_msgs.msg import Pose\n"
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

void CPose3D_from_ROS_Pose_msg(CPose3D &self, object pose_msg)
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
CObject *CPosePDFWrap::duplicate() const
{
    return this->get_override("duplicate")();
}

void CPosePDFWrap::writeToStream(CStream& stream, int32_t* pos) const
{
    this->get_override("writeToStream")(stream, pos);
}

void CPosePDFWrap::readFromStream(CStream& stream, int32_t pos)
{
    this->get_override("readFromStream")(stream, pos);
}

void CPosePDFWrap::getMean(mrpt::poses::CPose2D &mean_point) const
{
    this->get_override("getMean")(mean_point);
}

void CPosePDFWrap::getCovarianceAndMean(mrpt::math::CMatrixFixedNumeric<double,3ul,3ul> &cov, mrpt::poses::CPose2D &mean_point) const
{
    this->get_override("getCovarianceAndMean")(cov, mean_point);
}

void CPosePDFWrap::saveToTextFile(const std::string &file) const
{
    this->get_override("saveToTextFile")(file);
}

void CPosePDFWrap::drawSingleSample(mrpt::poses::CPose2D &outPart) const
{
    this->get_override("drawSingleSample")(outPart);
}

void CPosePDFWrap::changeCoordinatesReference(const mrpt::poses::CPose3D &newReferenceBase)
{
    this->get_override("changeCoordinatesReference")(newReferenceBase);
}

void CPosePDFWrap::copyFrom(const CPosePDF &o)
{
    this->get_override("copyFrom")(o);
}

void CPosePDFWrap::bayesianFusion(const CPosePDF &p1, const CPosePDF &p2, const double &minMahalanobisDistToDrop)
{
    this->get_override("bayesianFusion")(p1, p2, minMahalanobisDistToDrop);
}

void CPosePDFWrap::inverse(CPosePDF &o) const
{
    this->get_override("inverse")(o);
}
// end of CPosePDF

// CPose3DPDF
struct CPose3DPDFWrap : CPose3DPDF, wrapper<CPose3DPDF>
{
    CObject *duplicate() const
    {
        return this->get_override("duplicate")();
    }

    void writeToStream(mrpt::utils::CStream& stream, int32_t* pos) const
    {
        this->get_override("writeToStream")(stream, pos);
    }

    void readFromStream(mrpt::utils::CStream& stream, int32_t pos)
    {
        this->get_override("readFromStream")(stream, pos);
    }

    void getMean(mrpt::poses::CPose3D &mean_point) const
    {
        this->get_override("getMean")(mean_point);
    }

    void getCovarianceAndMean(mrpt::math::CMatrixFixedNumeric<double,6ul,6ul> &cov, mrpt::poses::CPose3D &mean_point) const
    {
        this->get_override("getCovarianceAndMean")(cov, mean_point);
    }

    void saveToTextFile(const std::string &file) const
    {
        this->get_override("saveToTextFile")(file);
    }

    void changeCoordinatesReference(const mrpt::poses::CPose3D &newReferenceBase)
    {
        this->get_override("changeCoordinatesReference")(newReferenceBase);
    }

    void drawSingleSample(mrpt::poses::CPose3D &outPart) const
    {
        this->get_override("drawSingleSample")(outPart);
    }

    void copyFrom(const CPose3DPDF &o)
    {
        this->get_override("copyFrom")(o);
    }

    void bayesianFusion(const CPose3DPDF &p1, const CPose3DPDF &p2)
    {
        this->get_override("bayesianFusion")(p1, p2);
    }

    void inverse(CPose3DPDF &o) const
    {
        this->get_override("inverse")(o);
    }
};

// CPosePDFGaussian
list CPosePDFGaussian_get_cov(CPosePDFGaussian &self)
{
    list cov;
    for (int32_t i = 0; i < 9; ++i) { cov.append(self.cov(i)); }
    return cov;
}

void CPosePDFGaussian_set_cov(CPosePDFGaussian &self, list cov)
{
    for (int32_t i = 0; i < 9; ++i) { self.cov(i) = extract<double>(cov[i]); }
}
// end of CPosePDFGaussian

// CPosePDFParticles
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(CPosePDFParticles_resetUniform_overloads, resetUniform, 4, 7)
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
    object poses_module(handle<>(borrowed(PyImport_AddModule("mrpt.poses"))));
    scope().attr("poses") = poses_module;
    scope poses_scope = poses_module;

    // CPose2D
    {
        class_<CPose2D>("CPose2D", init<>())
            .def(init<CPose2D>())
            .def(init<CPose3D>())
            .def(init<double, double, double>())
            .add_property("x",
                make_function(CPose2D_get_x, return_value_policy<copy_non_const_reference>()),
                CPose2D_set_x
            )
            .add_property("y",
                make_function(CPose2D_get_y, return_value_policy<copy_non_const_reference>()),
                CPose2D_set_y
            )
            .add_property("phi",
                make_function(CPose2D_get_phi, return_value_policy<copy_non_const_reference>()),
                CPose2D_set_phi
            )
            .def("inverse", &CPose2D::inverse, "Convert this pose into its inverse, saving the result in itself.")
            .def("norm", &CPose2D::norm)
            .def("normalizePhi", &CPose2D::normalizePhi, "Forces \"phi\" to be in the range [-pi,pi];")
            .def("__str__", &CPose2D_asString)
            .def("distance2DTo", &CPose2D::distance2DTo)
//          .def("distance2DFrobeniusTo", &CPose2D::distance2DFrobeniusTo, "Returns the 2D distance from this pose/point to a 2D pose using the Frobenius distance.")
            .def(self + self)
            .def(self - self)
#ifdef ROS_EXTENSIONS
            .def("to_ROS_Pose_msg", &CPose2D_to_ROS_Pose_msg, "Convert to ROS geometry_msgs/Pose.")
            .def("from_ROS_Pose_msg", &CPose2D_from_ROS_Pose_msg, "Convert from ROS geometry_msgs/Pose.")
#endif
        ;

        MAKE_PTR(CPose2D)
    }

    // CPosePDF
    {
        class_<CPosePDFWrap, boost::noncopyable>("CPosePDF", no_init)
            .def("writeToStream", &CPosePDFWrap::writeToStream, "Introduces a pure virtual method responsible for writing to a CStream. This can not be used directly be users!")
            .def("readFromStream", &CPosePDFWrap::readFromStream, "Introduces a pure virtual method responsible for loading from a CStream. This can not be used directly be users!")
            .def("getMean", &CPosePDFWrap::getMean, "Returns the mean, or mathematical expectation of the probability density distribution (PDF).")
            .def("getCovarianceAndMean", &CPosePDFWrap::getCovarianceAndMean, "Returns an estimate of the pose covariance matrix (STATE_LENxSTATE_LEN cov matrix) and the mean, both at once.")
            .def("saveToTextFile", &CPosePDFWrap::saveToTextFile, "Save PDF's particles to a text file. See derived classes for more information about the format of generated files.")
            .def("copyFrom", &CPosePDFWrap::copyFrom, "Copy operator, translating if necesary (for example, between particles and gaussian representations).")
            .def("bayesianFusion", &CPosePDFWrap::bayesianFusion, "Bayesian fusion of two pose distributions (product of two distributions->new distribution), then save the result in this object (WARNING: See implementing classes to see classes that can and cannot be mixtured!).")
            .def("inverse", &CPosePDFWrap::inverse, "Returns a new PDF such as: NEW_PDF = (0,0,0) - THIS_PDF")
        ;

        MAKE_PTR(CPosePDF)
    }

    // CPosePDFGaussian
    {
        class_<CPosePDFGaussian, bases<CPosePDFWrap> >("CPosePDFGaussian", init<optional<CPose2D> >())
            .def_readwrite("mean", &CPosePDFGaussian::mean)
            .add_property("cov", CPosePDFGaussian_get_cov, CPosePDFGaussian_set_cov)
        ;

        MAKE_PTR(CPosePDFGaussian)
    }

    // CPosePDFParticles
    {
        class_<CPosePDFParticles, bases<CPosePDFWrap> >("CPosePDFParticles", init<optional<size_t> >())
            .def("resetUniform", &CPosePDFParticles::resetUniform, CPosePDFParticles_resetUniform_overloads())
            .def("getParticlePose", &CPosePDFParticles::getParticlePose)
        ;

        MAKE_PTR(CPosePDFParticles)
    }

    // CPose3D
    {
        class_<CPose3D>("CPose3D", init<optional<CPose2D> >())
            .add_property("x",
                make_function(CPose3D_get_x, return_value_policy<copy_non_const_reference>()),
                CPose3D_set_x
            )
            .add_property("y",
                make_function(CPose3D_get_y, return_value_policy<copy_non_const_reference>()),
                CPose3D_set_y
            )
            .add_property("z",
                make_function(CPose3D_get_z, return_value_policy<copy_non_const_reference>()),
                CPose3D_set_z
            )
            .def(init<CPose3D>())
            .def("setYawPitchRoll", &CPose3D::setYawPitchRoll, "Set the 3 angles of the 3D pose (in radians) - This method recomputes the internal rotation coordinates matrix.")
            .def("getYawPitchRoll", &CPose3D_getYawPitchRoll, "Returns the three angles (yaw, pitch, roll), in radians, from the rotation matrix.")
            .def("setFromValues", &CPose3D::setFromValues, "Set the pose from a 3D position (meters) and yaw/pitch/roll angles (radians) - This method recomputes the internal rotation matrix.")
#ifdef ROS_EXTENSIONS
            .def("to_ROS_Pose_msg", &CPose3D_to_ROS_Pose_msg, "Convert to ROS geometry_msgs/Pose.")
            .def("from_ROS_Pose_msg", &CPose3D_from_ROS_Pose_msg, "Convert from ROS geometry_msgs/Pose.")
#endif
        ;

        MAKE_PTR(CPose3D)
    }

    // CPose3DPDF
    {
        class_<CPose3DPDFWrap, boost::noncopyable>("CPose3DPDF", no_init)
            .def("jacobiansPoseComposition", &CPose3DPDFWrap::jacobiansPoseComposition, "This static method computes the pose composition Jacobians.")
            .def("getMeanVal", &CPose3DPDF::getMeanVal)
        ;

        MAKE_PTR(CPose3DPDF)
    }

    // CPose3DPDFGaussian
    {
        class_<CPose3DPDFGaussian, bases<CPose3DPDF> >("CPose3DPDFGaussian", init<>())
        ;

        MAKE_PTR(CPose3DPDFGaussian)
    }

    // CPose3DPDFParticles
    {
        class_<CPose3DPDFParticles, bases<CPose3DPDF> >("CPose3DPDFParticles", init<>())
            .def("particlesCount", &CPose3DPDFParticles::particlesCount, "Get the m_particles count.")
            .def("getMean", &CPose3DPDFParticles::getMean, "Returns the mean, or mathematical expectation of the probability density distribution (PDF).")
        ;

        MAKE_PTR(CPose3DPDFParticles)
    }
}
