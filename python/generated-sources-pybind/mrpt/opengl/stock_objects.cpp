#include <iterator>
#include <memory>
#include <mrpt/containers/yaml.h>
#include <mrpt/img/TColor.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/TBoundingBox.h>
#include <mrpt/opengl/CRenderizable.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/opengl/stock_objects.h>
#include <mrpt/poses/CPointPDF.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DPDF.h>
#include <mrpt/poses/CPose3DQuatPDF.h>
#include <mrpt/poses/CPosePDF.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/typemeta/static_string.h>
#include <string>
#include <type_traits>
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

void bind_mrpt_opengl_stock_objects(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	// mrpt::opengl::stock_objects::RobotRhodon() file:mrpt/opengl/stock_objects.h line:34
	M("mrpt::opengl::stock_objects").def("RobotRhodon", (class std::shared_ptr<class mrpt::opengl::CSetOfObjects> (*)()) &mrpt::opengl::stock_objects::RobotRhodon, "Returns a representation of Rhodon.\n  The generated object must be inserted in a opengl::Scene or\n opengl::CSetOfObjects.\n  \n  \n      mrpt::opengl::stock_objects::RobotRhodon()   \n\n\n html preview_stock_objects_RobotRhodon.png  \n  \n  \n\nC++: mrpt::opengl::stock_objects::RobotRhodon() --> class std::shared_ptr<class mrpt::opengl::CSetOfObjects>");

	// mrpt::opengl::stock_objects::RobotGiraff() file:mrpt/opengl/stock_objects.h line:47
	M("mrpt::opengl::stock_objects").def("RobotGiraff", (class std::shared_ptr<class mrpt::opengl::CSetOfObjects> (*)()) &mrpt::opengl::stock_objects::RobotGiraff, "Returns a representation of RobotGiraff.\n  The generated object must be inserted in a opengl::Scene or\n opengl::CSetOfObjects.\n  \n  \n      mrpt::opengl::stock_objects::RobotGiraff()   \n\n\n html preview_stock_objects_RobotGiraff.png  \n  \n  \n\nC++: mrpt::opengl::stock_objects::RobotGiraff() --> class std::shared_ptr<class mrpt::opengl::CSetOfObjects>");

	// mrpt::opengl::stock_objects::RobotPioneer() file:mrpt/opengl/stock_objects.h line:60
	M("mrpt::opengl::stock_objects").def("RobotPioneer", (class std::shared_ptr<class mrpt::opengl::CSetOfObjects> (*)()) &mrpt::opengl::stock_objects::RobotPioneer, "Returns a representation of a Pioneer II mobile base.\n  The generated object must be inserted in a opengl::Scene or\n opengl::CSetOfObjects.\n  \n  \n      mrpt::opengl::stock_objects::RobotPioneer()   \n\n\n html preview_stock_objects_RobotPioneer.png  \n  \n  \n\nC++: mrpt::opengl::stock_objects::RobotPioneer() --> class std::shared_ptr<class mrpt::opengl::CSetOfObjects>");

	// mrpt::opengl::stock_objects::CornerXYZ(float) file:mrpt/opengl/stock_objects.h line:74
	M("mrpt::opengl::stock_objects").def("CornerXYZ", []() -> std::shared_ptr<class mrpt::opengl::CSetOfObjects> { return mrpt::opengl::stock_objects::CornerXYZ(); }, "");
	M("mrpt::opengl::stock_objects").def("CornerXYZ", (class std::shared_ptr<class mrpt::opengl::CSetOfObjects> (*)(float)) &mrpt::opengl::stock_objects::CornerXYZ, "Returns three arrows representing a X,Y,Z 3D corner.\n  The generated object must be inserted in a opengl::Scene or\n opengl::CSetOfObjects.\n \n\n CornerXYZSimple, CornerXYSimple, CornerXYZEye\n  \n  \n      mrpt::opengl::stock_objects::CornerXYZ()   \n\n\n preview_stock_objects_CornerXYZ.png  \n  \n  \n\nC++: mrpt::opengl::stock_objects::CornerXYZ(float) --> class std::shared_ptr<class mrpt::opengl::CSetOfObjects>", pybind11::arg("scale"));

	// mrpt::opengl::stock_objects::CornerXYZEye() file:mrpt/opengl/stock_objects.h line:92
	M("mrpt::opengl::stock_objects").def("CornerXYZEye", (class std::shared_ptr<class mrpt::opengl::CSetOfObjects> (*)()) &mrpt::opengl::stock_objects::CornerXYZEye, "Returns three arrows representing a X,Y,Z 3D corner.\n  Differently from CornerXYZ the arrowhead of Z axis ends where the object is\n placed.\n  This is useful if you want to place this object with the same position and\n orientation of a camera.\n  The generated object must be inserted in a opengl::Scene or\n opengl::CSetOfObjects.\n \n\n CornerXYZSimple, CornerXYSimple\n  \n  \n      mrpt::opengl::stock_objects::CornerXYZ()   \n\n\n preview_stock_objects_CornerXYZ.png  \n  \n  \n\nC++: mrpt::opengl::stock_objects::CornerXYZEye() --> class std::shared_ptr<class mrpt::opengl::CSetOfObjects>");

	// mrpt::opengl::stock_objects::CornerXYZSimple(float, float) file:mrpt/opengl/stock_objects.h line:107
	M("mrpt::opengl::stock_objects").def("CornerXYZSimple", []() -> std::shared_ptr<class mrpt::opengl::CSetOfObjects> { return mrpt::opengl::stock_objects::CornerXYZSimple(); }, "");
	M("mrpt::opengl::stock_objects").def("CornerXYZSimple", [](float const & a0) -> std::shared_ptr<class mrpt::opengl::CSetOfObjects> { return mrpt::opengl::stock_objects::CornerXYZSimple(a0); }, "", pybind11::arg("scale"));
	M("mrpt::opengl::stock_objects").def("CornerXYZSimple", (class std::shared_ptr<class mrpt::opengl::CSetOfObjects> (*)(float, float)) &mrpt::opengl::stock_objects::CornerXYZSimple, "Returns three arrows representing a X,Y,Z 3D corner (just thick lines\n instead of complex arrows for faster rendering than CornerXYZ).\n  The generated object must be inserted in a opengl::Scene or\n opengl::CSetOfObjects.\n \n\n CornerXYZ, CornerXYSimple\n  \n  \n      mrpt::opengl::stock_objects::CornerXYZSimple()  \n \n\n\n  \n  \n\nC++: mrpt::opengl::stock_objects::CornerXYZSimple(float, float) --> class std::shared_ptr<class mrpt::opengl::CSetOfObjects>", pybind11::arg("scale"), pybind11::arg("lineWidth"));

	// mrpt::opengl::stock_objects::CornerXYSimple(float, float) file:mrpt/opengl/stock_objects.h line:122
	M("mrpt::opengl::stock_objects").def("CornerXYSimple", []() -> std::shared_ptr<class mrpt::opengl::CSetOfObjects> { return mrpt::opengl::stock_objects::CornerXYSimple(); }, "");
	M("mrpt::opengl::stock_objects").def("CornerXYSimple", [](float const & a0) -> std::shared_ptr<class mrpt::opengl::CSetOfObjects> { return mrpt::opengl::stock_objects::CornerXYSimple(a0); }, "", pybind11::arg("scale"));
	M("mrpt::opengl::stock_objects").def("CornerXYSimple", (class std::shared_ptr<class mrpt::opengl::CSetOfObjects> (*)(float, float)) &mrpt::opengl::stock_objects::CornerXYSimple, "Returns two arrows representing a X,Y 2D corner (just thick lines, fast to\n render).\n  The generated object must be inserted in a opengl::Scene or\n opengl::CSetOfObjects.\n \n\n CornerXYZSimple, CornerXYZ, CornerXYZEye\n  \n  \n      mrpt::opengl::stock_objects::CornerXYSimple()   \n\n\n html preview_stock_objects_CornerXYSimple.png  \n  \n  \n\nC++: mrpt::opengl::stock_objects::CornerXYSimple(float, float) --> class std::shared_ptr<class mrpt::opengl::CSetOfObjects>", pybind11::arg("scale"), pybind11::arg("lineWidth"));

	// mrpt::opengl::stock_objects::BumblebeeCamera() file:mrpt/opengl/stock_objects.h line:135
	M("mrpt::opengl::stock_objects").def("BumblebeeCamera", (class std::shared_ptr<class mrpt::opengl::CSetOfObjects> (*)()) &mrpt::opengl::stock_objects::BumblebeeCamera, "Returns a simple 3D model of a PointGrey Bumblebee stereo camera.\n  The generated object must be inserted in a opengl::Scene or\n opengl::CSetOfObjects.\n  \n  \n      mrpt::opengl::stock_objects::BumblebeeCamera()  \n \n\n\n  \n  \n\nC++: mrpt::opengl::stock_objects::BumblebeeCamera() --> class std::shared_ptr<class mrpt::opengl::CSetOfObjects>");

	// mrpt::opengl::stock_objects::Hokuyo_URG() file:mrpt/opengl/stock_objects.h line:148
	M("mrpt::opengl::stock_objects").def("Hokuyo_URG", (class std::shared_ptr<class mrpt::opengl::CSetOfObjects> (*)()) &mrpt::opengl::stock_objects::Hokuyo_URG, "Returns a simple 3D model of a Hokuyo URG scanner.\n  The generated object must be inserted in a opengl::Scene or\n opengl::CSetOfObjects.\n  \n  \n      mrpt::opengl::stock_objects::Hokuyo_URG()   \n\n\n html preview_stock_objects_Hokuyo_URG.png  \n  \n  \n\nC++: mrpt::opengl::stock_objects::Hokuyo_URG() --> class std::shared_ptr<class mrpt::opengl::CSetOfObjects>");

	// mrpt::opengl::stock_objects::Hokuyo_UTM() file:mrpt/opengl/stock_objects.h line:161
	M("mrpt::opengl::stock_objects").def("Hokuyo_UTM", (class std::shared_ptr<class mrpt::opengl::CSetOfObjects> (*)()) &mrpt::opengl::stock_objects::Hokuyo_UTM, "Returns a simple 3D model of a Hokuyo UTM scanner.\n  The generated object must be inserted in a opengl::Scene or\n opengl::CSetOfObjects.\n  \n  \n      mrpt::opengl::stock_objects::Hokuyo_UTM()   \n\n\n html preview_stock_objects_Hokuyo_UTM.png  \n  \n  \n\nC++: mrpt::opengl::stock_objects::Hokuyo_UTM() --> class std::shared_ptr<class mrpt::opengl::CSetOfObjects>");

	// mrpt::opengl::stock_objects::Househam_Sprayer() file:mrpt/opengl/stock_objects.h line:174
	M("mrpt::opengl::stock_objects").def("Househam_Sprayer", (class std::shared_ptr<class mrpt::opengl::CSetOfObjects> (*)()) &mrpt::opengl::stock_objects::Househam_Sprayer, "Returns a simple 3D model of a househam sprayer.\n  The generated object must be inserted in a opengl::Scene or\n opengl::CSetOfObjects.\n  \n  \n      mrpt::opengl::stock_objects::Househam_Sprayer()  \n \n\n\n  \n  \n\nC++: mrpt::opengl::stock_objects::Househam_Sprayer() --> class std::shared_ptr<class mrpt::opengl::CSetOfObjects>");

}
