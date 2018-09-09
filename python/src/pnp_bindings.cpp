/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <boost/python.hpp>
#include <numpy/arrayobject.h>
using namespace boost::python;

#include <Eigen/Core>
#include <Eigen/Dense>
using namespace Eigen;

#include <mrpt/config.h>

#include <mrpt/vision/pnp_algos.h>
mrpt::vision::pnp::CPnP pnp_algos;

#if MRPT_HAS_OPENCV
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
using namespace cv;
#endif

#include <iostream>

class PnPAlgos
{
   public:
	PnPAlgos(int new_m);
	~PnPAlgos();
#if MRPT_HAS_OPENCV
	int epnp_solve(
		PyObject* obj_pts, PyObject* img_pts, int n, PyObject* cam_intrinsic,
		PyObject* pose_mat);
	int dls_solve(
		PyObject* obj_pts, PyObject* img_pts, int n, PyObject* cam_intrinsic,
		PyObject* pose_mat);
	int upnp_solve(
		PyObject* obj_pts, PyObject* img_pts, int n, PyObject* cam_intrinsic,
		PyObject* pose_mat);
#endif
	int p3p_solve(
		PyObject* obj_pts, PyObject* img_pts, int n, PyObject* cam_intrinsic,
		PyObject* pose_mat);
	int ppnp_solve(
		PyObject* obj_pts, PyObject* img_pts, int n, PyObject* cam_intrinsic,
		PyObject* pose_mat);
	int rpnp_solve(
		PyObject* obj_pts, PyObject* img_pts, int n, PyObject* cam_intrinsic,
		PyObject* pose_mat);
	int posit_solve(
		PyObject* obj_pts, PyObject* img_pts, int n, PyObject* cam_intrinsic,
		PyObject* pose_mat);
	int lhm_solve(
		PyObject* obj_pts, PyObject* img_pts, int n, PyObject* cam_intrinsic,
		PyObject* pose_mat);

   private:
	int dummy;
};

PnPAlgos::PnPAlgos(int new_m)
{
	dummy = new_m;
#if MRPT_HAS_OPENCV
	std::cout << " Using OpenCV dependency for PnP Algorithms - EPnP, DLS-PnP, "
				 "UPnP(Broken) "
			  << std::endl
			  << std::endl;
#else
	std::cout << " Initializing PnP class " << std::endl << std::endl;
#endif
}
PnPAlgos::~PnPAlgos() = default;
#if MRPT_HAS_OPENCV
int PnPAlgos::epnp_solve(
	PyObject* obj_pts, PyObject* img_pts, int n, PyObject* cam_intrinsic,
	PyObject* pose_mat)
{
	Map<MatrixXd> _obj_pts(
		(double*)PyArray_DATA((PyArrayObject*)obj_pts), 3, n);
	Map<MatrixXd> _img_pts(
		(double*)PyArray_DATA((PyArrayObject*)img_pts), 3, n);
	Map<MatrixXd> _pose_mat(
		(double*)PyArray_DATA((PyArrayObject*)pose_mat), 6, 1);
	Map<MatrixXd> _cam_intrinsic(
		(double*)PyArray_DATA((PyArrayObject*)cam_intrinsic), 3, 3);

	return pnp_algos.epnp(_obj_pts, _img_pts, n, _cam_intrinsic, _pose_mat);
}

int PnPAlgos::dls_solve(
	PyObject* obj_pts, PyObject* img_pts, int n, PyObject* cam_intrinsic,
	PyObject* pose_mat)
{
	Map<MatrixXd> _obj_pts(
		(double*)PyArray_DATA((PyArrayObject*)obj_pts), 3, n);
	Map<MatrixXd> _img_pts(
		(double*)PyArray_DATA((PyArrayObject*)img_pts), 3, n);
	Map<MatrixXd> _pose_mat(
		(double*)PyArray_DATA((PyArrayObject*)pose_mat), 6, 1);
	Map<MatrixXd> _cam_intrinsic(
		(double*)PyArray_DATA((PyArrayObject*)cam_intrinsic), 3, 3);

	return pnp_algos.dls(_obj_pts, _img_pts, n, _cam_intrinsic, _pose_mat);
}

int PnPAlgos::upnp_solve(
	PyObject* obj_pts, PyObject* img_pts, int n, PyObject* cam_intrinsic,
	PyObject* pose_mat)
{
	Map<MatrixXd> _obj_pts(
		(double*)PyArray_DATA((PyArrayObject*)obj_pts), 3, n);
	Map<MatrixXd> _img_pts(
		(double*)PyArray_DATA((PyArrayObject*)img_pts), 3, n);
	Map<MatrixXd> _pose_mat(
		(double*)PyArray_DATA((PyArrayObject*)pose_mat), 6, 1);
	Map<MatrixXd> _cam_intrinsic(
		(double*)PyArray_DATA((PyArrayObject*)cam_intrinsic), 3, 3);

	return pnp_algos.upnp(_obj_pts, _img_pts, n, _cam_intrinsic, _pose_mat);
}
#endif

int PnPAlgos::p3p_solve(
	PyObject* obj_pts, PyObject* img_pts, int n, PyObject* cam_intrinsic,
	PyObject* pose_mat)
{
	Map<MatrixXd> _obj_pts(
		(double*)PyArray_DATA((PyArrayObject*)obj_pts), 3, n);
	Map<MatrixXd> _img_pts(
		(double*)PyArray_DATA((PyArrayObject*)img_pts), 3, n);
	Map<MatrixXd> _pose_mat(
		(double*)PyArray_DATA((PyArrayObject*)pose_mat), 6, 1);
	Map<MatrixXd> _cam_intrinsic(
		(double*)PyArray_DATA((PyArrayObject*)cam_intrinsic), 3, 3);

	return pnp_algos.p3p(_obj_pts, _img_pts, n, _cam_intrinsic, _pose_mat);
}

int PnPAlgos::ppnp_solve(
	PyObject* obj_pts, PyObject* img_pts, int n, PyObject* cam_intrinsic,
	PyObject* pose_mat)
{
	Map<MatrixXd> _obj_pts(
		(double*)PyArray_DATA((PyArrayObject*)obj_pts), 3, n);
	Map<MatrixXd> _img_pts(
		(double*)PyArray_DATA((PyArrayObject*)img_pts), 3, n);
	Map<MatrixXd> _pose_mat(
		(double*)PyArray_DATA((PyArrayObject*)pose_mat), 6, 1);
	Map<MatrixXd> _cam_intrinsic(
		(double*)PyArray_DATA((PyArrayObject*)cam_intrinsic), 3, 3);

	return pnp_algos.ppnp(_obj_pts, _img_pts, n, _cam_intrinsic, _pose_mat);
}

int PnPAlgos::rpnp_solve(
	PyObject* obj_pts, PyObject* img_pts, int n, PyObject* cam_intrinsic,
	PyObject* pose_mat)
{
	Map<MatrixXd> _obj_pts(
		(double*)PyArray_DATA((PyArrayObject*)obj_pts), 3, n);
	Map<MatrixXd> _img_pts(
		(double*)PyArray_DATA((PyArrayObject*)img_pts), 3, n);
	Map<MatrixXd> _pose_mat(
		(double*)PyArray_DATA((PyArrayObject*)pose_mat), 6, 1);
	Map<MatrixXd> _cam_intrinsic(
		(double*)PyArray_DATA((PyArrayObject*)cam_intrinsic), 3, 3);

	return pnp_algos.rpnp(_obj_pts, _img_pts, n, _cam_intrinsic, _pose_mat);
}

int PnPAlgos::posit_solve(
	PyObject* obj_pts, PyObject* img_pts, int n, PyObject* cam_intrinsic,
	PyObject* pose_mat)
{
	Map<MatrixXd> _obj_pts(
		(double*)PyArray_DATA((PyArrayObject*)obj_pts), 3, n);
	Map<MatrixXd> _img_pts(
		(double*)PyArray_DATA((PyArrayObject*)img_pts), 3, n);
	Map<MatrixXd> _pose_mat(
		(double*)PyArray_DATA((PyArrayObject*)pose_mat), 6, 1);
	Map<MatrixXd> _cam_intrinsic(
		(double*)PyArray_DATA((PyArrayObject*)cam_intrinsic), 3, 3);

	return pnp_algos.posit(_obj_pts, _img_pts, n, _cam_intrinsic, _pose_mat);
}

int PnPAlgos::lhm_solve(
	PyObject* obj_pts, PyObject* img_pts, int n, PyObject* cam_intrinsic,
	PyObject* pose_mat)
{
	Map<MatrixXd> _obj_pts(
		(double*)PyArray_DATA((PyArrayObject*)obj_pts), 3, n);
	Map<MatrixXd> _img_pts(
		(double*)PyArray_DATA((PyArrayObject*)img_pts), 3, n);
	Map<MatrixXd> _pose_mat(
		(double*)PyArray_DATA((PyArrayObject*)pose_mat), 6, 1);
	Map<MatrixXd> _cam_intrinsic(
		(double*)PyArray_DATA((PyArrayObject*)cam_intrinsic), 3, 3);

	return pnp_algos.lhm(_obj_pts, _img_pts, n, _cam_intrinsic, _pose_mat);
}

void export_pnp()
{
	class_<PnPAlgos>("pnp", init<int>(args("m")))
#if MRPT_HAS_OPENCV
		.def("epnp", &PnPAlgos::epnp_solve)
		.def("dls", &PnPAlgos::dls_solve)
		.def("upnp", &PnPAlgos::upnp_solve)
#endif
		.def("p3p", &PnPAlgos::p3p_solve)
		.def("ppnp", &PnPAlgos::ppnp_solve)
		.def("rpnp", &PnPAlgos::rpnp_solve)
		.def("posit", &PnPAlgos::posit_solve)
		.def("lhm", &PnPAlgos::lhm_solve);
}
