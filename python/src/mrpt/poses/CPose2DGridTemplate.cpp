#include <iterator>
#include <memory>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CQuaternion.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/poses/CPose2DGridTemplate.h>
#include <mrpt/poses/CPose3DGridTemplate.h>
#include <optional>
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

void bind_mrpt_poses_CPose2DGridTemplate(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::poses::CPose2DGridTemplate file:mrpt/poses/CPose2DGridTemplate.h line:24
		pybind11::class_<mrpt::poses::CPose2DGridTemplate<double>, std::shared_ptr<mrpt::poses::CPose2DGridTemplate<double>>> cl(M("mrpt::poses"), "CPose2DGridTemplate_double_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::poses::CPose2DGridTemplate<double>(); } ), "doc" );
		cl.def( pybind11::init( [](double const & a0){ return new mrpt::poses::CPose2DGridTemplate<double>(a0); } ), "doc" , pybind11::arg("xMin"));
		cl.def( pybind11::init( [](double const & a0, double const & a1){ return new mrpt::poses::CPose2DGridTemplate<double>(a0, a1); } ), "doc" , pybind11::arg("xMin"), pybind11::arg("xMax"));
		cl.def( pybind11::init( [](double const & a0, double const & a1, double const & a2){ return new mrpt::poses::CPose2DGridTemplate<double>(a0, a1, a2); } ), "doc" , pybind11::arg("xMin"), pybind11::arg("xMax"), pybind11::arg("yMin"));
		cl.def( pybind11::init( [](double const & a0, double const & a1, double const & a2, double const & a3){ return new mrpt::poses::CPose2DGridTemplate<double>(a0, a1, a2, a3); } ), "doc" , pybind11::arg("xMin"), pybind11::arg("xMax"), pybind11::arg("yMin"), pybind11::arg("yMax"));
		cl.def( pybind11::init( [](double const & a0, double const & a1, double const & a2, double const & a3, double const & a4){ return new mrpt::poses::CPose2DGridTemplate<double>(a0, a1, a2, a3, a4); } ), "doc" , pybind11::arg("xMin"), pybind11::arg("xMax"), pybind11::arg("yMin"), pybind11::arg("yMax"), pybind11::arg("resolutionXY"));
		cl.def( pybind11::init( [](double const & a0, double const & a1, double const & a2, double const & a3, double const & a4, double const & a5){ return new mrpt::poses::CPose2DGridTemplate<double>(a0, a1, a2, a3, a4, a5); } ), "doc" , pybind11::arg("xMin"), pybind11::arg("xMax"), pybind11::arg("yMin"), pybind11::arg("yMax"), pybind11::arg("resolutionXY"), pybind11::arg("resolutionPhi"));
		cl.def( pybind11::init( [](double const & a0, double const & a1, double const & a2, double const & a3, double const & a4, double const & a5, double const & a6){ return new mrpt::poses::CPose2DGridTemplate<double>(a0, a1, a2, a3, a4, a5, a6); } ), "doc" , pybind11::arg("xMin"), pybind11::arg("xMax"), pybind11::arg("yMin"), pybind11::arg("yMax"), pybind11::arg("resolutionXY"), pybind11::arg("resolutionPhi"), pybind11::arg("phiMin"));
		cl.def( pybind11::init<double, double, double, double, double, double, double, double>(), pybind11::arg("xMin"), pybind11::arg("xMax"), pybind11::arg("yMin"), pybind11::arg("yMax"), pybind11::arg("resolutionXY"), pybind11::arg("resolutionPhi"), pybind11::arg("phiMin"), pybind11::arg("phiMax") );

		cl.def( pybind11::init( [](mrpt::poses::CPose2DGridTemplate<double> const &o){ return new mrpt::poses::CPose2DGridTemplate<double>(o); } ) );
		cl.def("x2idx", (size_t (mrpt::poses::CPose2DGridTemplate<double>::*)(double) const) &mrpt::poses::CPose2DGridTemplate<double>::x2idx, "C++: mrpt::poses::CPose2DGridTemplate<double>::x2idx(double) const --> size_t", pybind11::arg("x"));
		cl.def("y2idx", (size_t (mrpt::poses::CPose2DGridTemplate<double>::*)(double) const) &mrpt::poses::CPose2DGridTemplate<double>::y2idx, "C++: mrpt::poses::CPose2DGridTemplate<double>::y2idx(double) const --> size_t", pybind11::arg("y"));
		cl.def("phi2idx", (size_t (mrpt::poses::CPose2DGridTemplate<double>::*)(double) const) &mrpt::poses::CPose2DGridTemplate<double>::phi2idx, "C++: mrpt::poses::CPose2DGridTemplate<double>::phi2idx(double) const --> size_t", pybind11::arg("phi"));
		cl.def("idx2x", (double (mrpt::poses::CPose2DGridTemplate<double>::*)(size_t) const) &mrpt::poses::CPose2DGridTemplate<double>::idx2x, "C++: mrpt::poses::CPose2DGridTemplate<double>::idx2x(size_t) const --> double", pybind11::arg("x"));
		cl.def("idx2y", (double (mrpt::poses::CPose2DGridTemplate<double>::*)(size_t) const) &mrpt::poses::CPose2DGridTemplate<double>::idx2y, "C++: mrpt::poses::CPose2DGridTemplate<double>::idx2y(size_t) const --> double", pybind11::arg("y"));
		cl.def("idx2phi", (double (mrpt::poses::CPose2DGridTemplate<double>::*)(size_t) const) &mrpt::poses::CPose2DGridTemplate<double>::idx2phi, "C++: mrpt::poses::CPose2DGridTemplate<double>::idx2phi(size_t) const --> double", pybind11::arg("phi"));
		cl.def("setSize", [](mrpt::poses::CPose2DGridTemplate<double> &o, double const & a0, double const & a1, double const & a2, double const & a3, double const & a4, double const & a5) -> void { return o.setSize(a0, a1, a2, a3, a4, a5); }, "", pybind11::arg("xMin"), pybind11::arg("xMax"), pybind11::arg("yMin"), pybind11::arg("yMax"), pybind11::arg("resolutionXY"), pybind11::arg("resolutionPhi"));
		cl.def("setSize", [](mrpt::poses::CPose2DGridTemplate<double> &o, double const & a0, double const & a1, double const & a2, double const & a3, double const & a4, double const & a5, double const & a6) -> void { return o.setSize(a0, a1, a2, a3, a4, a5, a6); }, "", pybind11::arg("xMin"), pybind11::arg("xMax"), pybind11::arg("yMin"), pybind11::arg("yMax"), pybind11::arg("resolutionXY"), pybind11::arg("resolutionPhi"), pybind11::arg("phiMin"));
		cl.def("setSize", (void (mrpt::poses::CPose2DGridTemplate<double>::*)(double, double, double, double, double, double, double, double)) &mrpt::poses::CPose2DGridTemplate<double>::setSize, "C++: mrpt::poses::CPose2DGridTemplate<double>::setSize(double, double, double, double, double, double, double, double) --> void", pybind11::arg("xMin"), pybind11::arg("xMax"), pybind11::arg("yMin"), pybind11::arg("yMax"), pybind11::arg("resolutionXY"), pybind11::arg("resolutionPhi"), pybind11::arg("phiMin"), pybind11::arg("phiMax"));
		cl.def("getByPos", (double * (mrpt::poses::CPose2DGridTemplate<double>::*)(double, double, double)) &mrpt::poses::CPose2DGridTemplate<double>::getByPos, "C++: mrpt::poses::CPose2DGridTemplate<double>::getByPos(double, double, double) --> double *", pybind11::return_value_policy::automatic, pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("phi"));
		cl.def("idx2absidx", (size_t (mrpt::poses::CPose2DGridTemplate<double>::*)(size_t, size_t, size_t) const) &mrpt::poses::CPose2DGridTemplate<double>::idx2absidx, "C++: mrpt::poses::CPose2DGridTemplate<double>::idx2absidx(size_t, size_t, size_t) const --> size_t", pybind11::arg("cx"), pybind11::arg("cy"), pybind11::arg("cPhi"));
		cl.def("absidx2idx", (class std::tuple<unsigned long, unsigned long, unsigned long> (mrpt::poses::CPose2DGridTemplate<double>::*)(size_t) const) &mrpt::poses::CPose2DGridTemplate<double>::absidx2idx, "C++: mrpt::poses::CPose2DGridTemplate<double>::absidx2idx(size_t) const --> class std::tuple<unsigned long, unsigned long, unsigned long>", pybind11::arg("absIdx"));
		cl.def("getByIndex", (double * (mrpt::poses::CPose2DGridTemplate<double>::*)(size_t, size_t, size_t)) &mrpt::poses::CPose2DGridTemplate<double>::getByIndex, "C++: mrpt::poses::CPose2DGridTemplate<double>::getByIndex(size_t, size_t, size_t) --> double *", pybind11::return_value_policy::automatic, pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("phi"));
		cl.def("getXMin", (double (mrpt::poses::CPose2DGridTemplate<double>::*)() const) &mrpt::poses::CPose2DGridTemplate<double>::getXMin, "C++: mrpt::poses::CPose2DGridTemplate<double>::getXMin() const --> double");
		cl.def("getXMax", (double (mrpt::poses::CPose2DGridTemplate<double>::*)() const) &mrpt::poses::CPose2DGridTemplate<double>::getXMax, "C++: mrpt::poses::CPose2DGridTemplate<double>::getXMax() const --> double");
		cl.def("getYMin", (double (mrpt::poses::CPose2DGridTemplate<double>::*)() const) &mrpt::poses::CPose2DGridTemplate<double>::getYMin, "C++: mrpt::poses::CPose2DGridTemplate<double>::getYMin() const --> double");
		cl.def("getYMax", (double (mrpt::poses::CPose2DGridTemplate<double>::*)() const) &mrpt::poses::CPose2DGridTemplate<double>::getYMax, "C++: mrpt::poses::CPose2DGridTemplate<double>::getYMax() const --> double");
		cl.def("getPhiMin", (double (mrpt::poses::CPose2DGridTemplate<double>::*)() const) &mrpt::poses::CPose2DGridTemplate<double>::getPhiMin, "C++: mrpt::poses::CPose2DGridTemplate<double>::getPhiMin() const --> double");
		cl.def("getPhiMax", (double (mrpt::poses::CPose2DGridTemplate<double>::*)() const) &mrpt::poses::CPose2DGridTemplate<double>::getPhiMax, "C++: mrpt::poses::CPose2DGridTemplate<double>::getPhiMax() const --> double");
		cl.def("getResolutionXY", (double (mrpt::poses::CPose2DGridTemplate<double>::*)() const) &mrpt::poses::CPose2DGridTemplate<double>::getResolutionXY, "C++: mrpt::poses::CPose2DGridTemplate<double>::getResolutionXY() const --> double");
		cl.def("getResolutionPhi", (double (mrpt::poses::CPose2DGridTemplate<double>::*)() const) &mrpt::poses::CPose2DGridTemplate<double>::getResolutionPhi, "C++: mrpt::poses::CPose2DGridTemplate<double>::getResolutionPhi() const --> double");
		cl.def("getSizeX", (size_t (mrpt::poses::CPose2DGridTemplate<double>::*)() const) &mrpt::poses::CPose2DGridTemplate<double>::getSizeX, "C++: mrpt::poses::CPose2DGridTemplate<double>::getSizeX() const --> size_t");
		cl.def("getSizeY", (size_t (mrpt::poses::CPose2DGridTemplate<double>::*)() const) &mrpt::poses::CPose2DGridTemplate<double>::getSizeY, "C++: mrpt::poses::CPose2DGridTemplate<double>::getSizeY() const --> size_t");
		cl.def("getSizePhi", (size_t (mrpt::poses::CPose2DGridTemplate<double>::*)() const) &mrpt::poses::CPose2DGridTemplate<double>::getSizePhi, "C++: mrpt::poses::CPose2DGridTemplate<double>::getSizePhi() const --> size_t");
		cl.def("assign", (class mrpt::poses::CPose2DGridTemplate<double> & (mrpt::poses::CPose2DGridTemplate<double>::*)(const class mrpt::poses::CPose2DGridTemplate<double> &)) &mrpt::poses::CPose2DGridTemplate<double>::operator=, "C++: mrpt::poses::CPose2DGridTemplate<double>::operator=(const class mrpt::poses::CPose2DGridTemplate<double> &) --> class mrpt::poses::CPose2DGridTemplate<double> &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::poses::CPose3DGridTemplate file:mrpt/poses/CPose3DGridTemplate.h line:25
		pybind11::class_<mrpt::poses::CPose3DGridTemplate<double>, std::shared_ptr<mrpt::poses::CPose3DGridTemplate<double>>> cl(M("mrpt::poses"), "CPose3DGridTemplate_double_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::poses::CPose3DGridTemplate<double>(); } ), "doc" );
		cl.def( pybind11::init( [](const struct mrpt::math::TPose3D & a0){ return new mrpt::poses::CPose3DGridTemplate<double>(a0); } ), "doc" , pybind11::arg("bb_min"));
		cl.def( pybind11::init( [](const struct mrpt::math::TPose3D & a0, const struct mrpt::math::TPose3D & a1){ return new mrpt::poses::CPose3DGridTemplate<double>(a0, a1); } ), "doc" , pybind11::arg("bb_min"), pybind11::arg("bb_max"));
		cl.def( pybind11::init( [](const struct mrpt::math::TPose3D & a0, const struct mrpt::math::TPose3D & a1, double const & a2){ return new mrpt::poses::CPose3DGridTemplate<double>(a0, a1, a2); } ), "doc" , pybind11::arg("bb_min"), pybind11::arg("bb_max"), pybind11::arg("resolution_XYZ"));
		cl.def( pybind11::init<const struct mrpt::math::TPose3D &, const struct mrpt::math::TPose3D &, double, double>(), pybind11::arg("bb_min"), pybind11::arg("bb_max"), pybind11::arg("resolution_XYZ"), pybind11::arg("resolution_YPR") );

		cl.def( pybind11::init( [](mrpt::poses::CPose3DGridTemplate<double> const &o){ return new mrpt::poses::CPose3DGridTemplate<double>(o); } ) );
		cl.def("x2idx", (int (mrpt::poses::CPose3DGridTemplate<double>::*)(double) const) &mrpt::poses::CPose3DGridTemplate<double>::x2idx, "C++: mrpt::poses::CPose3DGridTemplate<double>::x2idx(double) const --> int", pybind11::arg("x"));
		cl.def("y2idx", (int (mrpt::poses::CPose3DGridTemplate<double>::*)(double) const) &mrpt::poses::CPose3DGridTemplate<double>::y2idx, "C++: mrpt::poses::CPose3DGridTemplate<double>::y2idx(double) const --> int", pybind11::arg("y"));
		cl.def("z2idx", (int (mrpt::poses::CPose3DGridTemplate<double>::*)(double) const) &mrpt::poses::CPose3DGridTemplate<double>::z2idx, "C++: mrpt::poses::CPose3DGridTemplate<double>::z2idx(double) const --> int", pybind11::arg("z"));
		cl.def("yaw2idx", (int (mrpt::poses::CPose3DGridTemplate<double>::*)(double) const) &mrpt::poses::CPose3DGridTemplate<double>::yaw2idx, "C++: mrpt::poses::CPose3DGridTemplate<double>::yaw2idx(double) const --> int", pybind11::arg("yaw"));
		cl.def("pitch2idx", (int (mrpt::poses::CPose3DGridTemplate<double>::*)(double) const) &mrpt::poses::CPose3DGridTemplate<double>::pitch2idx, "C++: mrpt::poses::CPose3DGridTemplate<double>::pitch2idx(double) const --> int", pybind11::arg("pitch"));
		cl.def("roll2idx", (int (mrpt::poses::CPose3DGridTemplate<double>::*)(double) const) &mrpt::poses::CPose3DGridTemplate<double>::roll2idx, "C++: mrpt::poses::CPose3DGridTemplate<double>::roll2idx(double) const --> int", pybind11::arg("roll"));
		cl.def("idx2x", (double (mrpt::poses::CPose3DGridTemplate<double>::*)(uint32_t) const) &mrpt::poses::CPose3DGridTemplate<double>::idx2x, "C++: mrpt::poses::CPose3DGridTemplate<double>::idx2x(uint32_t) const --> double", pybind11::arg("cx"));
		cl.def("idx2y", (double (mrpt::poses::CPose3DGridTemplate<double>::*)(uint32_t) const) &mrpt::poses::CPose3DGridTemplate<double>::idx2y, "C++: mrpt::poses::CPose3DGridTemplate<double>::idx2y(uint32_t) const --> double", pybind11::arg("cy"));
		cl.def("idx2z", (double (mrpt::poses::CPose3DGridTemplate<double>::*)(uint32_t) const) &mrpt::poses::CPose3DGridTemplate<double>::idx2z, "C++: mrpt::poses::CPose3DGridTemplate<double>::idx2z(uint32_t) const --> double", pybind11::arg("cz"));
		cl.def("idx2yaw", (double (mrpt::poses::CPose3DGridTemplate<double>::*)(uint32_t) const) &mrpt::poses::CPose3DGridTemplate<double>::idx2yaw, "C++: mrpt::poses::CPose3DGridTemplate<double>::idx2yaw(uint32_t) const --> double", pybind11::arg("cY"));
		cl.def("idx2pitch", (double (mrpt::poses::CPose3DGridTemplate<double>::*)(uint32_t) const) &mrpt::poses::CPose3DGridTemplate<double>::idx2pitch, "C++: mrpt::poses::CPose3DGridTemplate<double>::idx2pitch(uint32_t) const --> double", pybind11::arg("cP"));
		cl.def("idx2roll", (double (mrpt::poses::CPose3DGridTemplate<double>::*)(uint32_t) const) &mrpt::poses::CPose3DGridTemplate<double>::idx2roll, "C++: mrpt::poses::CPose3DGridTemplate<double>::idx2roll(uint32_t) const --> double", pybind11::arg("cR"));
		cl.def("setSize", (void (mrpt::poses::CPose3DGridTemplate<double>::*)(const struct mrpt::math::TPose3D &, const struct mrpt::math::TPose3D &, double, double)) &mrpt::poses::CPose3DGridTemplate<double>::setSize, "C++: mrpt::poses::CPose3DGridTemplate<double>::setSize(const struct mrpt::math::TPose3D &, const struct mrpt::math::TPose3D &, double, double) --> void", pybind11::arg("bb_min"), pybind11::arg("bb_max"), pybind11::arg("resolution_XYZ"), pybind11::arg("resolution_YPR"));
		cl.def("getByPos", (double * (mrpt::poses::CPose3DGridTemplate<double>::*)(double, double, double, double, double, double)) &mrpt::poses::CPose3DGridTemplate<double>::getByPos, "C++: mrpt::poses::CPose3DGridTemplate<double>::getByPos(double, double, double, double, double, double) --> double *", pybind11::return_value_policy::automatic, pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("z"), pybind11::arg("yaw"), pybind11::arg("pitch"), pybind11::arg("roll"));
		cl.def("getByPos", (double * (mrpt::poses::CPose3DGridTemplate<double>::*)(const struct mrpt::math::TPose3D &)) &mrpt::poses::CPose3DGridTemplate<double>::getByPos, "C++: mrpt::poses::CPose3DGridTemplate<double>::getByPos(const struct mrpt::math::TPose3D &) --> double *", pybind11::return_value_policy::automatic, pybind11::arg("p"));
		cl.def("getByIndex", (double * (mrpt::poses::CPose3DGridTemplate<double>::*)(int, int, int, int, int, int)) &mrpt::poses::CPose3DGridTemplate<double>::getByIndex, "C++: mrpt::poses::CPose3DGridTemplate<double>::getByIndex(int, int, int, int, int, int) --> double *", pybind11::return_value_policy::automatic, pybind11::arg("cx"), pybind11::arg("cy"), pybind11::arg("cz"), pybind11::arg("cY"), pybind11::arg("cP"), pybind11::arg("cR"));
		cl.def("idx2absidx", (size_t (mrpt::poses::CPose3DGridTemplate<double>::*)(size_t, size_t, size_t, size_t, size_t, size_t) const) &mrpt::poses::CPose3DGridTemplate<double>::idx2absidx, "C++: mrpt::poses::CPose3DGridTemplate<double>::idx2absidx(size_t, size_t, size_t, size_t, size_t, size_t) const --> size_t", pybind11::arg("cx"), pybind11::arg("cy"), pybind11::arg("cz"), pybind11::arg("cYaw"), pybind11::arg("cPitch"), pybind11::arg("cRoll"));
		cl.def("absidx2idx", (class std::tuple<unsigned long, unsigned long, unsigned long, unsigned long, unsigned long, unsigned long> (mrpt::poses::CPose3DGridTemplate<double>::*)(size_t) const) &mrpt::poses::CPose3DGridTemplate<double>::absidx2idx, "C++: mrpt::poses::CPose3DGridTemplate<double>::absidx2idx(size_t) const --> class std::tuple<unsigned long, unsigned long, unsigned long, unsigned long, unsigned long, unsigned long>", pybind11::arg("absIdx"));
		cl.def("getMinBoundingBox", (struct mrpt::math::TPose3D (mrpt::poses::CPose3DGridTemplate<double>::*)() const) &mrpt::poses::CPose3DGridTemplate<double>::getMinBoundingBox, "C++: mrpt::poses::CPose3DGridTemplate<double>::getMinBoundingBox() const --> struct mrpt::math::TPose3D");
		cl.def("getMaxBoundingBox", (struct mrpt::math::TPose3D (mrpt::poses::CPose3DGridTemplate<double>::*)() const) &mrpt::poses::CPose3DGridTemplate<double>::getMaxBoundingBox, "C++: mrpt::poses::CPose3DGridTemplate<double>::getMaxBoundingBox() const --> struct mrpt::math::TPose3D");
		cl.def("getResolutionXYZ", (double (mrpt::poses::CPose3DGridTemplate<double>::*)() const) &mrpt::poses::CPose3DGridTemplate<double>::getResolutionXYZ, "C++: mrpt::poses::CPose3DGridTemplate<double>::getResolutionXYZ() const --> double");
		cl.def("getResolutionAngles", (double (mrpt::poses::CPose3DGridTemplate<double>::*)() const) &mrpt::poses::CPose3DGridTemplate<double>::getResolutionAngles, "C++: mrpt::poses::CPose3DGridTemplate<double>::getResolutionAngles() const --> double");
		cl.def("fill", (void (mrpt::poses::CPose3DGridTemplate<double>::*)(const double &)) &mrpt::poses::CPose3DGridTemplate<double>::fill, "C++: mrpt::poses::CPose3DGridTemplate<double>::fill(const double &) --> void", pybind11::arg("val"));
		cl.def("getSizeX", (uint32_t (mrpt::poses::CPose3DGridTemplate<double>::*)() const) &mrpt::poses::CPose3DGridTemplate<double>::getSizeX, "C++: mrpt::poses::CPose3DGridTemplate<double>::getSizeX() const --> uint32_t");
		cl.def("getSizeY", (uint32_t (mrpt::poses::CPose3DGridTemplate<double>::*)() const) &mrpt::poses::CPose3DGridTemplate<double>::getSizeY, "C++: mrpt::poses::CPose3DGridTemplate<double>::getSizeY() const --> uint32_t");
		cl.def("getSizeZ", (uint32_t (mrpt::poses::CPose3DGridTemplate<double>::*)() const) &mrpt::poses::CPose3DGridTemplate<double>::getSizeZ, "C++: mrpt::poses::CPose3DGridTemplate<double>::getSizeZ() const --> uint32_t");
		cl.def("getSizeYaw", (uint32_t (mrpt::poses::CPose3DGridTemplate<double>::*)() const) &mrpt::poses::CPose3DGridTemplate<double>::getSizeYaw, "C++: mrpt::poses::CPose3DGridTemplate<double>::getSizeYaw() const --> uint32_t");
		cl.def("getSizePitch", (uint32_t (mrpt::poses::CPose3DGridTemplate<double>::*)() const) &mrpt::poses::CPose3DGridTemplate<double>::getSizePitch, "C++: mrpt::poses::CPose3DGridTemplate<double>::getSizePitch() const --> uint32_t");
		cl.def("getSizeRoll", (uint32_t (mrpt::poses::CPose3DGridTemplate<double>::*)() const) &mrpt::poses::CPose3DGridTemplate<double>::getSizeRoll, "C++: mrpt::poses::CPose3DGridTemplate<double>::getSizeRoll() const --> uint32_t");
		cl.def("getTotalVoxelCount", (uint32_t (mrpt::poses::CPose3DGridTemplate<double>::*)() const) &mrpt::poses::CPose3DGridTemplate<double>::getTotalVoxelCount, "C++: mrpt::poses::CPose3DGridTemplate<double>::getTotalVoxelCount() const --> uint32_t");
		cl.def("assign", (class mrpt::poses::CPose3DGridTemplate<double> & (mrpt::poses::CPose3DGridTemplate<double>::*)(const class mrpt::poses::CPose3DGridTemplate<double> &)) &mrpt::poses::CPose3DGridTemplate<double>::operator=, "C++: mrpt::poses::CPose3DGridTemplate<double>::operator=(const class mrpt::poses::CPose3DGridTemplate<double> &) --> class mrpt::poses::CPose3DGridTemplate<double> &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
