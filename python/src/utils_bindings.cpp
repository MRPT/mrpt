/* bindings */
#include "bindings.h"

/* MRPT */
#include <mrpt/utils/TColor.h>
#include <mrpt/utils/CObject.h>
#include <mrpt/utils/CStream.h>
#include <mrpt/utils/CFileGZInputStream.h>
#include <mrpt/utils/CStdOutStream.h>
#include <mrpt/utils/CSerializable.h>
#include <mrpt/utils/CConfigFileBase.h>
#include <mrpt/utils/CConfigFile.h>
#include <mrpt/utils/CLoadableOptions.h>
#include <mrpt/utils/CRobotSimulator.h>

/* STD */
#include <stdint.h>

using namespace boost::python;
using namespace mrpt::utils;
using namespace mrpt::poses;


// CRobotSimulator
CPose2D CRobotSimulator_getOdometry(CRobotSimulator &self)
{
    CPose2D pose;
    self.getOdometry(pose);
    return pose;
}

CPose2D CRobotSimulator_getRealPose(CRobotSimulator &self)
{
    CPose2D pose;
    self.getRealPose(pose);
    return pose;
}

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(CRobotSimulator_setDelayModelParams_overloads, setDelayModelParams, 0, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(CRobotSimulator_setOdometryErrors_overloads, setOdometryErrors, 1, 7)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(CRobotSimulator_resetOdometry_overloads, resetOdometry, 0, 1)
// end of CRobotSimulator

// CStream
void CStream_ReadObject(CStream& self, CSerializablePtr& obj)
{
    self.ReadObject(obj.pointer());
}

void CStream_WriteObject(CStream& self, CSerializablePtr& obj)
{
    self.WriteObject(obj.pointer());
}
// end of CStream

// CConfigFileBase
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(CConfigFileBase_read_double_overloads, read_double, 3, 4)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(CConfigFileBase_read_float_overloads, read_float, 3, 4)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(CConfigFileBase_read_bool_overloads, read_bool, 3, 4)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(CConfigFileBase_read_int_overloads, read_int, 3, 4)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(CConfigFileBase_read_uint64_t_overloads, read_uint64_t, 3, 4)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(CConfigFileBase_read_string_overloads, read_string, 3, 4)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(CConfigFileBase_read_string_first_word_overloads, read_string_first_word, 3, 4)
//TODO implement read_vector, read_matrix and read_enum
// end of CConfigFileBase

// Utils
double mrpt_utils_DEG2RAD(double deg) { return mrpt::utils::DEG2RAD(deg); }
double mrpt_utils_RAD2DEG(double rad) { return mrpt::utils::RAD2DEG(rad); }
// end of Utils

// smart pointer contents
MAKE_PTR_CTX(CObject)
MAKE_PTR_CTX(CSerializable)


// exporter
void export_utils()
{
    // map namespace to be submodule of mrpt package
    MAKE_SUBMODULE(utils)


    // TColorf
    {
        class_<TColorf>("TColorf", init<optional<float, float, float, float> >(args("r", "g", "b", "alpha"), "A RGB color - floats in the range [0,1]."))
            .def_readwrite("R", &TColorf::R, "red")
            .def_readwrite("G", &TColorf::R, "green")
            .def_readwrite("B", &TColorf::R, "blue")
            .def_readwrite("A", &TColorf::R, "alpha")
        ;
    }

    // CObject
    {
        MAKE_PTR(CObject)

        class_<CObject, boost::noncopyable>("CObject", no_init)
            .def("duplicate", &CObject::duplicate, return_value_policy<manage_new_object>(), "Returns a copy of the object, indepently of its class.")
            .def("GetRuntimeClass", &CObject::GetRuntimeClass, return_internal_reference<>(), "Returns information about the class of an object in runtime.")
        ;
    }

    // CLoadableOptions
    {
        class_<CLoadableOptions, boost::noncopyable>("CLoadableOptions", no_init)
            .def("loadFromConfigFile", &CLoadableOptions::loadFromConfigFile, "This method load the options from a \".ini\"-like file or memory-stored string list.")
            .def("loadFromConfigFileName", &CLoadableOptions::loadFromConfigFileName, "Behaves like loadFromConfigFile, but you can pass directly a file name and a temporary CConfigFile object will be created automatically to load the file.")
            .def("saveToConfigFile", &CLoadableOptions::saveToConfigFile, "This method saves the options to a \".ini\"-like file or memory-stored string list.")
            .def("saveToConfigFileName", &CLoadableOptions::saveToConfigFileName, "Behaves like saveToConfigFile, but you can pass directly a file name and a temporary CConfigFile object will be created automatically to save the file.")
            .def("dumpToConsole", &CLoadableOptions::dumpToConsole, "Just like dumpToTextStream() but sending the text to the console (std::cout)")
            .def("dumpToTextStream", &CLoadableOptions::dumpToTextStream, "This method should clearly display all the contents of the structure in textual form, sending it to a CStream.")
        ;
    }

    // TRuntimeClassId
    {
        class_<TRuntimeClassId, boost::noncopyable>("TRuntimeClassId", no_init)
            .def_readwrite("className", &TRuntimeClassId::className)
        ;
    }

    // CConfigFileBase
    {
        class_<CConfigFileBase, boost::noncopyable>("CConfigFileBase", no_init)
            .def("getAllSections", &CConfigFileBase::getAllSections, "Returns a list with all the section names.")
            .def("getAllKeys", &CConfigFileBase::getAllKeys, "Returs a list with all the keys into a section.")
            .def("sectionExists ", &CConfigFileBase::sectionExists, "Checks if a given section exists (name is case insensitive).")
            .def("read_double", &CConfigFileBase::read_double, CConfigFileBase_read_double_overloads())
            .def("read_float", &CConfigFileBase::read_float, CConfigFileBase_read_float_overloads())
            .def("read_bool", &CConfigFileBase::read_bool, CConfigFileBase_read_bool_overloads())
            .def("read_int", &CConfigFileBase::read_int, CConfigFileBase_read_int_overloads())
            .def("read_uint64_t", &CConfigFileBase::read_uint64_t, CConfigFileBase_read_uint64_t_overloads())
            .def("read_string", &CConfigFileBase::read_string, CConfigFileBase_read_string_overloads())
            .def("read_string_first_word", &CConfigFileBase::read_string_first_word, CConfigFileBase_read_string_first_word_overloads())
        ;
    }

    // CConfigFile
    {
        class_<CConfigFile, bases<CConfigFileBase> >("CConfigFile", init<optional<std::string> >())
        ;
    }

    // CRobotSimulator
    {
        class_<CRobotSimulator>("CRobotSimulator", "This class can be used to simulate the kinematics and dynamics of a differential driven planar mobile robot, including odometry errors and dynamics limitations.", init< optional<float, float> >(args("TAU", "DELAY"), "Constructor with default dynamic model-parameters"))
            .def("setDelayModelParams", &CRobotSimulator::setDelayModelParams, CRobotSimulator_setDelayModelParams_overloads(args("TAU_delay_sec=1.8f", "CMD_delay_sec=0.3f"), "Change the model of delays used for the orders sent to the robot"))
            .def("setOdometryErrors", &CRobotSimulator::setOdometryErrors, CRobotSimulator_setOdometryErrors_overloads( args("enabled", "Ax_err_bias", "Ax_err_std", "Ay_err_bias", "Ay_err_std", "Aphi_err_bias", "Aphi_err_std"), "Enable/Disable odometry errors. Errors in odometry are introduced per millisecond."))
            .def("setRealPose", &CRobotSimulator::setRealPose, args("pose"), "Reset actual robot pose (inmediately, without simulating the movement along time).")
            .def("getX", &CRobotSimulator::getX, "Read the instantaneous, error-free status of the simulated robot.")
            .def("getY", &CRobotSimulator::getY, "Read the instantaneous, error-free status of the simulated robot.")
            .def("getPHI", &CRobotSimulator::getPHI, "Read the instantaneous, error-free status of the simulated robot.")
            .def("getT", &CRobotSimulator::getT, "Read the instantaneous, error-free status of the simulated robot.")
            .def("getV", &CRobotSimulator::getV, "Read the instantaneous, error-free status of the simulated robot.")
            .def("getW", &CRobotSimulator::getW, "Read the instantaneous, error-free status of the simulated robot.")
            .def("setV", &CRobotSimulator::setV, args("v"), "Set actual robot velocity, error-free status of the simulated robot.")
            .def("setW", &CRobotSimulator::setW, args("w"), "Set actual robot turnrate, error-free status of the simulated robot.")
            .def("movementCommand", &CRobotSimulator::movementCommand, args("lin_vel", "ang_vel"), "Used to command the robot a desired movement (velocities).")
            .def("resetStatus", &CRobotSimulator::resetStatus, "Reset all the simulator variables to 0 (All but current simulator time).")
            .def("resetTime", &CRobotSimulator::resetTime, "Reset time counter.")
            .def("simulateInterval", &CRobotSimulator::simulateInterval, args("dt"), "This method must be called periodically to simulate discrete time intervals.")
            .def("resetOdometry", &CRobotSimulator::resetOdometry,  CRobotSimulator_resetOdometry_overloads(args("newOdo"), "Forces odometry to be set to a specified values."))
            .def("getOdometry", &CRobotSimulator_getOdometry, "Reads the simulated robot odometry (this is NOT equal to the actual error-free robot coordinates).")
            .def("getRealPose", &CRobotSimulator_getRealPose, "Reads the real robot pose.")
        ;
    }

    // CSerializable
    {
        MAKE_PTR_BASE(CSerializable, CObject)

        class_<CSerializable, boost::noncopyable, bases<CObject> >("CSerializable", no_init)
        ;
    }

    // CStream
    {
        class_<CStream, boost::noncopyable>("CStream", "This base class is used to provide a unified interface to files,memory buffers.", no_init)
            .def("ReadObject", &CStream_ReadObject, "Reads an object from stream, its class determined at runtime, and returns a smart pointer to the object.")
            .def("WriteObject", &CStream_WriteObject, "Writes an object to the stream.")
        ;
    }

    // CStdOutStream
    {
        class_<CStdOutStream, bases<CStream> >("CStdOutStream", "This CStdOutStream derived class allow printing to standard out, normally the console text output.", init<>())
        ;
    }

    // CFileGZInputStream
    {
        class_<CFileGZInputStream, boost::noncopyable, bases<CStream> >("CFileGZInputStream", "Transparently opens a compressed \"gz\" file and reads uncompressed data from it.", init< optional<std::string> >(args("filename")))
            .def("open", &CFileGZInputStream::open, "Opens the file for read.")
            .def("close", &CFileGZInputStream::close, "Closes the file.")
        ;
    }

    // static module functions
    def("DEG2RAD", &mrpt_utils_DEG2RAD, args("deg"), "Convert degrees to radiants.");
    def("RAD2DEG", &mrpt_utils_RAD2DEG, args("rad"), "Convert radiants to degrees.");
}

void export_utils_stl()
{
    MAKE_VEC_NAMED(float, FloatVector)
    MAKE_VEC_NAMED(double, DoubleVector)
    MAKE_VEC_NAMED(uint8_t, UInt8Vector)
    MAKE_VEC_NAMED(uint16_t, UInt16Vector)
    MAKE_VEC_NAMED(uint32_t, UInt32Vector)
    MAKE_VEC_NAMED(uint64_t, UInt64Vector)
    MAKE_VEC_NAMED(int8_t, Int8Vector)
    MAKE_VEC_NAMED(int16_t, Int16Vector)
    MAKE_VEC_NAMED(int32_t, Int32Vector)
    MAKE_VEC_NAMED(int64_t, Int64Vector)
}
