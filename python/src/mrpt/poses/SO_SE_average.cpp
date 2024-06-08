#include <any>
#include <functional>
#include <ios>
#include <istream>
#include <iterator>
#include <map>
#include <memory>
#include <mrpt/containers/CommentPosition.h>
#include <mrpt/containers/YamlEmitOptions.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CQuaternion.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/math/math_frwds.h>
#include <mrpt/math/matrix_size_t.h>
#include <mrpt/poses/CPoint2D.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DQuat.h>
#include <mrpt/poses/CPoseOrPoint.h>
#include <mrpt/poses/SO_SE_average.h>
#include <mrpt/poses/sensor_poses.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/typemeta/static_string.h>
#include <optional>
#include <ostream>
#include <sstream> // __str__
#include <streambuf>
#include <string>
#include <string_view>
#include <type_traits>
#include <typeinfo>
#include <utility>
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

void bind_mrpt_poses_SO_SE_average(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::poses::SO_average file:mrpt/poses/SO_SE_average.h line:41
		pybind11::class_<mrpt::poses::SO_average<2UL>, std::shared_ptr<mrpt::poses::SO_average<2UL>>> cl(M("mrpt::poses"), "SO_average_2UL_t", "Computes weighted and un-weighted averages of SO(2) orientations.\n Add values to average with  when done call \n Use  to reset the accumulator and start a new average computation.\n Theoretical base: Average on SO(2) manifolds is computed by averaging the\n corresponding 2D points, then projecting the result back to the closest-point\n in the manifold.\n Shortly explained in [these\n slides](http://ingmec.ual.es/~jlblanco/papers/blanco2013tutorial-manifolds-introduction-robotics.pdf)\n \n\n Class introduced in MRPT 1.3.1\n \n\n SE_traits ");
		cl.def( pybind11::init( [](){ return new mrpt::poses::SO_average<2UL>(); } ) );
		cl.def_readwrite("enable_exception_on_undeterminate", &mrpt::poses::SO_average<2UL>::enable_exception_on_undeterminate);
		cl.def("clear", (void (mrpt::poses::SO_average<2UL>::*)()) &mrpt::poses::SO_average<2>::clear, "Resets the accumulator \n\nC++: mrpt::poses::SO_average<2>::clear() --> void");
		cl.def("append", (void (mrpt::poses::SO_average<2UL>::*)(const double)) &mrpt::poses::SO_average<2>::append, "Adds a new orientation (radians) to the computation \n get_average \n\nC++: mrpt::poses::SO_average<2>::append(const double) --> void", pybind11::arg("orientation_rad"));
		cl.def("append", (void (mrpt::poses::SO_average<2UL>::*)(const double, const double)) &mrpt::poses::SO_average<2>::append, "Adds a new orientation (radians) to the weighted-average computation \n\n get_average \n\nC++: mrpt::poses::SO_average<2>::append(const double, const double) --> void", pybind11::arg("orientation_rad"), pybind11::arg("weight"));
		cl.def("get_average", (double (mrpt::poses::SO_average<2UL>::*)() const) &mrpt::poses::SO_average<2>::get_average, "Returns the average orientation (radians).\n \n\n std::logic_error If no data point were inserted.\n \n\n std::runtime_error Upon undeterminate average value (ie the\n average lays exactly on the origin point) and \n is set to true (otherwise, the 0\n orientation would be returned)\n \n\n append \n\nC++: mrpt::poses::SO_average<2>::get_average() const --> double");
	}
	{ // mrpt::poses::SO_average file:mrpt/poses/SO_SE_average.h line:83
		pybind11::class_<mrpt::poses::SO_average<3UL>, std::shared_ptr<mrpt::poses::SO_average<3UL>>> cl(M("mrpt::poses"), "SO_average_3UL_t", "Computes weighted and un-weighted averages of SO(3) orientations.\n Add values to average with  when done call \n Use  to reset the accumulator and start a new average computation.\n Theoretical base: Average on SO(3) manifolds is computed by averaging the\n corresponding matrices, then projecting the result back to the closest matrix\n in the manifold.\n Shortly explained in [these\n slides](http://ingmec.ual.es/~jlblanco/papers/blanco2013tutorial-manifolds-introduction-robotics.pdf)\n See also: eq. (3.7) in \"MEANS AND AVERAGING IN THE GROUP OF ROTATIONS\",\n MAHER MOAKHER, 2002.\n \n\n Class introduced in MRPT 1.3.1\n \n\n SE_traits ");
		cl.def( pybind11::init( [](){ return new mrpt::poses::SO_average<3UL>(); } ) );
		cl.def_readwrite("enable_exception_on_undeterminate", &mrpt::poses::SO_average<3UL>::enable_exception_on_undeterminate);
		cl.def("clear", (void (mrpt::poses::SO_average<3UL>::*)()) &mrpt::poses::SO_average<3>::clear, "Resets the accumulator \n\nC++: mrpt::poses::SO_average<3>::clear() --> void");
		cl.def("append", (void (mrpt::poses::SO_average<3UL>::*)(const class mrpt::math::CMatrixFixed<double, 3, 3> &)) &mrpt::poses::SO_average<3>::append, "Adds a new orientation to the computation \n get_average \n\nC++: mrpt::poses::SO_average<3>::append(const class mrpt::math::CMatrixFixed<double, 3, 3> &) --> void", pybind11::arg("M"));
		cl.def("append", (void (mrpt::poses::SO_average<3UL>::*)(const class mrpt::math::CMatrixFixed<double, 3, 3> &, const double)) &mrpt::poses::SO_average<3>::append, "Adds a new orientation to the weighted-average computation \n\n get_average \n\nC++: mrpt::poses::SO_average<3>::append(const class mrpt::math::CMatrixFixed<double, 3, 3> &, const double) --> void", pybind11::arg("M"), pybind11::arg("weight"));
		cl.def("get_average", (class mrpt::math::CMatrixFixed<double, 3, 3> (mrpt::poses::SO_average<3UL>::*)() const) &mrpt::poses::SO_average<3>::get_average, "Returns the average orientation.\n \n\n std::logic_error If no data point were inserted.\n \n\n std::runtime_error Upon undeterminate average value (ie there\n was a problem with the SVD) and  is\n set to true (otherwise, the 0 orientation would be returned)\n \n\n append \n\nC++: mrpt::poses::SO_average<3>::get_average() const --> class mrpt::math::CMatrixFixed<double, 3, 3>");
	}
	{ // mrpt::poses::SE_average file:mrpt/poses/SO_SE_average.h line:119
		pybind11::class_<mrpt::poses::SE_average<2UL>, std::shared_ptr<mrpt::poses::SE_average<2UL>>> cl(M("mrpt::poses"), "SE_average_2UL_t", "Computes weighted and un-weighted averages of SE(2) poses.\n Add values to average with  when done call \n Use  to reset the accumulator and start a new average computation.\n Theoretical base: See SO_average<2> for the rotation part. The translation\n is a simple arithmetic mean in Euclidean space.\n \n\n Class introduced in MRPT 1.3.1\n \n\n SE_traits ");
		cl.def( pybind11::init( [](){ return new mrpt::poses::SE_average<2UL>(); } ) );
		cl.def_readwrite("enable_exception_on_undeterminate", &mrpt::poses::SE_average<2UL>::enable_exception_on_undeterminate);
		cl.def("clear", (void (mrpt::poses::SE_average<2UL>::*)()) &mrpt::poses::SE_average<2>::clear, "Resets the accumulator \n\nC++: mrpt::poses::SE_average<2>::clear() --> void");
		cl.def("append", (void (mrpt::poses::SE_average<2UL>::*)(const class mrpt::poses::CPose2D &)) &mrpt::poses::SE_average<2>::append, "Adds a new pose to the computation \n get_average \n\nC++: mrpt::poses::SE_average<2>::append(const class mrpt::poses::CPose2D &) --> void", pybind11::arg("p"));
		cl.def("append", (void (mrpt::poses::SE_average<2UL>::*)(const class mrpt::poses::CPose2D &, const double)) &mrpt::poses::SE_average<2>::append, "Adds a new pose to the weighted-average computation \n get_average \n\nC++: mrpt::poses::SE_average<2>::append(const class mrpt::poses::CPose2D &, const double) --> void", pybind11::arg("p"), pybind11::arg("weight"));
		cl.def("append", (void (mrpt::poses::SE_average<2UL>::*)(const struct mrpt::math::TPose2D &, const double)) &mrpt::poses::SE_average<2>::append, "C++: mrpt::poses::SE_average<2>::append(const struct mrpt::math::TPose2D &, const double) --> void", pybind11::arg("p"), pybind11::arg("weight"));
		cl.def("get_average", (void (mrpt::poses::SE_average<2UL>::*)(class mrpt::poses::CPose2D &) const) &mrpt::poses::SE_average<2>::get_average, "Returns the average pose.\n \n\n std::logic_error If no data point were inserted.\n \n\n std::runtime_error Upon undeterminate average value (ie the\n average lays exactly on the origin point) and \n is set to true (otherwise, the 0\n orientation would be returned)\n \n\n append \n\nC++: mrpt::poses::SE_average<2>::get_average(class mrpt::poses::CPose2D &) const --> void", pybind11::arg("out_mean"));
	}
	{ // mrpt::poses::SE_average file:mrpt/poses/SO_SE_average.h line:157
		pybind11::class_<mrpt::poses::SE_average<3UL>, std::shared_ptr<mrpt::poses::SE_average<3UL>>> cl(M("mrpt::poses"), "SE_average_3UL_t", "Computes weighted and un-weighted averages of SE(3) poses.\n Add values to average with  when done call \n Use  to reset the accumulator and start a new average computation.\n Theoretical base: See SO_average<3> for the rotation part. The translation\n is a simple arithmetic mean in Euclidean space.\n \n\n Class introduced in MRPT 1.3.1\n \n\n SE_traits ");
		cl.def( pybind11::init( [](){ return new mrpt::poses::SE_average<3UL>(); } ) );
		cl.def_readwrite("enable_exception_on_undeterminate", &mrpt::poses::SE_average<3UL>::enable_exception_on_undeterminate);
		cl.def("clear", (void (mrpt::poses::SE_average<3UL>::*)()) &mrpt::poses::SE_average<3>::clear, "Resets the accumulator \n\nC++: mrpt::poses::SE_average<3>::clear() --> void");
		cl.def("append", (void (mrpt::poses::SE_average<3UL>::*)(const class mrpt::poses::CPose3D &)) &mrpt::poses::SE_average<3>::append, "Adds a new pose to the computation \n get_average \n\nC++: mrpt::poses::SE_average<3>::append(const class mrpt::poses::CPose3D &) --> void", pybind11::arg("p"));
		cl.def("append", (void (mrpt::poses::SE_average<3UL>::*)(const class mrpt::poses::CPose3D &, const double)) &mrpt::poses::SE_average<3>::append, "Adds a new pose to the weighted-average computation \n get_average \n\nC++: mrpt::poses::SE_average<3>::append(const class mrpt::poses::CPose3D &, const double) --> void", pybind11::arg("p"), pybind11::arg("weight"));
		cl.def("append", (void (mrpt::poses::SE_average<3UL>::*)(const struct mrpt::math::TPose3D &, const double)) &mrpt::poses::SE_average<3>::append, "C++: mrpt::poses::SE_average<3>::append(const struct mrpt::math::TPose3D &, const double) --> void", pybind11::arg("p"), pybind11::arg("weight"));
		cl.def("get_average", (void (mrpt::poses::SE_average<3UL>::*)(class mrpt::poses::CPose3D &) const) &mrpt::poses::SE_average<3>::get_average, "Returns the average pose.\n \n\n std::logic_error If no data point were inserted.\n \n\n std::runtime_error Upon undeterminate average value (ie the\n average lays exactly on the origin point) and \n is set to true (otherwise, the 0\n orientation would be returned)\n \n\n append \n\nC++: mrpt::poses::SE_average<3>::get_average(class mrpt::poses::CPose3D &) const --> void", pybind11::arg("out_mean"));
	}
	// mrpt::poses::sensor_poses_from_yaml(const class mrpt::containers::yaml &, const std::string &) file:mrpt/poses/sensor_poses.h line:28
	M("mrpt::poses").def("sensor_poses_from_yaml", [](const class mrpt::containers::yaml & a0) -> std::map<std::string, class mrpt::poses::CPose3D> { return mrpt::poses::sensor_poses_from_yaml(a0); }, "", pybind11::arg("d"));
	M("mrpt::poses").def("sensor_poses_from_yaml", (class std::map<std::string, class mrpt::poses::CPose3D> (*)(const class mrpt::containers::yaml &, const std::string &)) &mrpt::poses::sensor_poses_from_yaml, "Alternative to sensor_poses_from_yaml_file() where the yaml map inside\n `sensors: ...` is directly passed programatically.\n\n \n CPose3D, mrpt::obs::CObservation, sensor_poses_from_yaml_file()\n \n\n\n \n\nC++: mrpt::poses::sensor_poses_from_yaml(const class mrpt::containers::yaml &, const std::string &) --> class std::map<std::string, class mrpt::poses::CPose3D>", pybind11::arg("d"), pybind11::arg("referenceFrame"));

	// mrpt::poses::sensor_poses_from_yaml_file(const std::string &, const std::string &) file:mrpt/poses/sensor_poses.h line:69
	M("mrpt::poses").def("sensor_poses_from_yaml_file", [](const std::string & a0) -> std::map<std::string, class mrpt::poses::CPose3D> { return mrpt::poses::sensor_poses_from_yaml_file(a0); }, "", pybind11::arg("filename"));
	M("mrpt::poses").def("sensor_poses_from_yaml_file", (class std::map<std::string, class mrpt::poses::CPose3D> (*)(const std::string &, const std::string &)) &mrpt::poses::sensor_poses_from_yaml_file, "Utility to parse a YAML file with the extrinsic calibration of sensors.\n\n  Each YAML map entry defines a sensorLabel, and for each one an `extrinsics`\n  map containing the SE(3) relative pose between the `parent` frame and this\n  sensor. The pose is given as a quaternion and a translation.\n\n  The expected file contents is like:\n\n  \n\n\n\n\n\n\n\n\n\n\n\n\n\n Following the common ROS conventions, the robot reference frame is assumed\n to be `base_link` (default).\n\n Of course, this mechanism of defining a tree of sensor poses in a YAML file\n only works for static (rigid) sensor assemblies, where the transformations\n between them is always static.\n\n The data is returned as a `std::map` from sensor labels to poses within the\n robot reference frame.\n\n This function takes as input the YAML filename to load.\n\n \n CPose3D, mrpt::obs::CObservation, sensor_poses_from_yaml()\n \n\n\n \n\nC++: mrpt::poses::sensor_poses_from_yaml_file(const std::string &, const std::string &) --> class std::map<std::string, class mrpt::poses::CPose3D>", pybind11::arg("filename"), pybind11::arg("referenceFrame"));

}
