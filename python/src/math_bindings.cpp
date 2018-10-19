/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#include "bindings.h"

/* MRPT */
#include <mrpt/poses/CPoint2D.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DQuat.h>
#include <mrpt/math/lightweight_geom_data.h>
#include <mrpt/math/CMatrixFixedNumeric.h>

/* macros */
#define MAKE_FIXED_DOUBLE_MATRIX(rows, cols)                                \
	class_<CMatrixDouble##rows##cols>(                                      \
		STRINGIFY(CMatrixDouble##rows##cols), init<>())                     \
		.def(STRINGIFY(__getitem__), &CMatrixDouble##rows##cols##_getitem1) \
		.def(STRINGIFY(__getitem__), &CMatrixDouble##rows##cols##_getitem2) \
		.def(STRINGIFY(__setitem__), &CMatrixDouble##rows##cols##_setitem1) \
		.def(STRINGIFY(__setitem__), &CMatrixDouble##rows##cols##_setitem2);

#define MAKE_FIXED_DOUBLE_MATRIX_GETITEM(rows, cols)                  \
	double CMatrixDouble##rows##cols##_getitem1(                      \
		CMatrixDouble##rows##cols& self, size_t i)                    \
	{                                                                 \
		return self(i);                                               \
	}                                                                 \
                                                                      \
	double CMatrixDouble##rows##cols##_getitem2(                      \
		CMatrixDouble##rows##cols& self, const tuple& t)              \
	{                                                                 \
		extract<int> eir(t[0]);                                       \
		extract<int> eic(t[1]);                                       \
		if (eir.check() && eic.check())                               \
		{                                                             \
			int row = eir();                                          \
			int col = eic();                                          \
			printf(STRINGIFY(rows cols % i % i), row, col);           \
			if ((0 <= row && row < rows) && (0 <= col && col < cols)) \
				return self(row, col);                                \
			else                                                      \
			{                                                         \
				printf("IndexError");                                 \
				IndexError();                                         \
			}                                                         \
		}                                                             \
		else                                                          \
		{                                                             \
			TypeError("Indeces must be integers!");                   \
		}                                                             \
		return self(0, 0);                                            \
	}

#define MAKE_FIXED_DOUBLE_MATRIX_SETITEM(rows, cols)                   \
	void CMatrixDouble##rows##cols##_setitem1(                         \
		CMatrixDouble##rows##cols& self, size_t i, double value)       \
	{                                                                  \
		self(i) = value;                                               \
	}                                                                  \
                                                                       \
	void CMatrixDouble##rows##cols##_setitem2(                         \
		CMatrixDouble##rows##cols& self, const tuple& t, double value) \
	{                                                                  \
		self(extract<int>(t[0]), extract<int>(t[1])) = value;          \
	}

#define MAKE_FIXED_DOUBLE_MATRIX_GETSET(rows, cols) \
	MAKE_FIXED_DOUBLE_MATRIX_GETITEM(rows, cols)    \
	MAKE_FIXED_DOUBLE_MATRIX_SETITEM(rows, cols)

/* namespaces */
using namespace boost::python;
using namespace mrpt::math;
using namespace mrpt::poses;

// TPoint2D
MAKE_AS_STR(TPoint2D)
MAKE_GETITEM(TPoint2D, double)
MAKE_SETITEM(TPoint2D, double)
// end of TPoint2D

// TPose2D
MAKE_AS_STR(TPose2D)
MAKE_GETITEM(TPose2D, double)
MAKE_SETITEM(TPose2D, double)
// end of TPose2D

// TPoint3D
MAKE_AS_STR(TPoint3D)
MAKE_GETITEM(TPoint3D, double)
MAKE_SETITEM(TPoint3D, double)
// end of TPoint3D

// TPose3D
MAKE_AS_STR(TPose3D)
MAKE_GETITEM(TPose3D, double)
MAKE_SETITEM(TPose3D, double)
// end of TPose3D

// TTwist2D
MAKE_AS_STR(TTwist2D)
MAKE_GETITEM(TTwist2D, double)
MAKE_SETITEM(TTwist2D, double)
// end of TTwist2D

// TTwist3D
MAKE_AS_STR(TTwist3D)
MAKE_GETITEM(TTwist3D, double)
MAKE_SETITEM(TTwist3D, double)
// end of TTwist2D

// TPose3DQuat
MAKE_AS_STR(TPose3DQuat)
MAKE_GETITEM(TPose3DQuat, double)
MAKE_SETITEM(TPose3DQuat, double)
// end of TPose3DQuat

// CMatrix
MAKE_FIXED_DOUBLE_MATRIX_GETSET(3, 3)
MAKE_FIXED_DOUBLE_MATRIX_GETSET(6, 6)

// TODO: add conversion from/to list for convenience

// end of CMatrix

// Explicit instantiations:
namespace mrpt
{
namespace math
{
template void wrapTo2PiInPlace<double>(double& a);
template double wrapTo2Pi<double>(double a);
template double wrapToPi<double>(double a);
template void wrapToPiInPlace<double>(double& a);
template double angDistance<double>(double from, double to);
}  // namespace math
}  // namespace mrpt

// exporter
void export_math()
{
	// map namespace to be submodule of mrpt package
	MAKE_SUBMODULE(math)

	// wrapToPi.h
	{
		def("wrapTo2Pi", &mrpt::math::wrapTo2Pi<double>);
		def("wrapToPi", &mrpt::math::wrapToPi<double>);
		def("wrapTo2PiInPlace", &mrpt::math::wrapTo2PiInPlace<double>);
		def("wrapToPiInPlace", &mrpt::math::wrapToPiInPlace<double>);
		def("angDistance", &mrpt::math::angDistance<double>);
	}

	// TPoint2D
	{
		class_<TPoint2D>("TPoint2D", init<>())
			.def(init<TPoint3D>())
			.def(init<TPose2D>())
			.def(init<TPose3D>())
			.def(init<double, double>())
			.def_readwrite("x", &TPoint2D::x)
			.def_readwrite("y", &TPoint2D::y)
			.def(self + self)
			.def(self - self)
			.def(self * float())
			.def(self / float())
			.def(self += self)
			.def(self -= self)
			.def(self *= float())
			.def(self /= float())
			.def("norm", &TPoint2D::norm, "Point norm.")
			.def("__getitem__", &TPoint2D_getitem)
			.def("__setitem__", &TPoint2D_setitem)
			.def("__str__", &TPoint2D_asString);
	}

	// TPose2D
	{
		class_<TPose2D>("TPose2D", init<>())
			.def(init<TPoint2D>())
			.def(init<TPoint3D>())
			.def(init<TPose3D>())
			.def(init<double, double, double>())
			.def_readwrite("x", &TPose2D::x)
			.def_readwrite("y", &TPose2D::y)
			.def_readwrite("phi", &TPose2D::phi)
			.def("__getitem__", &TPose2D_getitem)
			.def("__setitem__", &TPose2D_setitem)
			.def("__str__", &TPose2D_asString);
	}

	// TPoint3D
	{
		class_<TPoint3D>("TPoint3D", init<>())
			.def(init<TPoint2D>())
			.def(init<TPose2D>())
			.def(init<TPose3D>())
			.def(init<double, double, double>())
			.def_readwrite("x", &TPoint3D::x)
			.def_readwrite("y", &TPoint3D::y)
			.def_readwrite("z", &TPoint3D::z)
			.def("__getitem__", &TPose3D_getitem)
			.def("__setitem__", &TPose3D_setitem)
			.def("__str__", &TPoint3D_asString);
	}

	// TPose3D
	{
		class_<TPose3D>("TPose3D", init<>())
			.def(init<TPoint2D>())
			.def(init<TPoint3D>())
			.def(init<TPose2D>())
			.def(init<double, double, double, double, double, double>())
			.def_readwrite("x", &TPose3D::x)
			.def_readwrite("y", &TPose3D::y)
			.def_readwrite("z", &TPose3D::z)
			.def_readwrite("yaw", &TPose3D::yaw)
			.def_readwrite("pitch", &TPose3D::pitch)
			.def_readwrite("roll", &TPose3D::roll)
			.def("__getitem__", &TPose3D_getitem)
			.def("__setitem__", &TPose3D_setitem)
			.def("__str__", &TPose3D_asString);
	}

	// TPose3DQuat
	{
		class_<TPose3DQuat>("TPose3DQuat", init<>())
			.def(init<double, double, double, double, double, double, double>())
			.def_readwrite("x", &TPose3DQuat::x)
			.def_readwrite("y", &TPose3DQuat::y)
			.def_readwrite("z", &TPose3DQuat::z)
			.def_readwrite("qr", &TPose3DQuat::qr)
			.def_readwrite("qx", &TPose3DQuat::qx)
			.def_readwrite("qy", &TPose3DQuat::qy)
			.def_readwrite("qz", &TPose3DQuat::qz)
			.def("__getitem__", &TPose3DQuat_getitem)
			.def("__setitem__", &TPose3DQuat_setitem)
			.def("__str__", &TPose3DQuat_asString);
	}

	// CMatrixFixedNumeric
	{
		MAKE_FIXED_DOUBLE_MATRIX(3, 3)
		MAKE_FIXED_DOUBLE_MATRIX(6, 6)
	}
}

void export_math_stl()
{
	MAKE_VEC(TPoint2D)
	MAKE_VEC(TPoint3D)
	MAKE_VEC(TPose2D)
	MAKE_VEC(TPose3D)
	//     MAKE_VEC(TPose3DQuat) // Compiler complains about something missing
	//     on this type, but I can't figure out what.
}
