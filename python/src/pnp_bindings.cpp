#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <boost/python.hpp>
#include <numpy/arrayobject.h> 
using namespace boost::python;

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
using namespace Eigen;

#include <mrpt/vision/pnp/pnp_algos.h>
using namespace pnp;
CPnP pnp_algos;

#include <mrpt/vision/pnp/epnp.h>
#include <mrpt/vision/pnp/dls.h>
#include <mrpt/vision/pnp/upnp.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
using namespace cv;

class PnPAlgos
{
public:
	PnPAlgos( int new_m );
	~PnPAlgos();
	
	int epnp_solve(PyObject* obj_pts, PyObject* img_pts, int n, PyObject* cam_intrinsic, PyObject* pose_mat);
	int dls_solve(PyObject* obj_pts, PyObject* img_pts, int n, PyObject* cam_intrinsic, PyObject* pose_mat);
	int upnp_solve(PyObject* obj_pts, PyObject* img_pts, int n, PyObject* cam_intrinsic, PyObject* pose_mat);
	
private:
	int m;
};

PnPAlgos::PnPAlgos( int new_m ){
	m = new_m;
}
PnPAlgos::~PnPAlgos(){
}

int PnPAlgos::epnp_solve(PyObject* obj_pts, PyObject* img_pts, int n, PyObject* cam_intrinsic, PyObject* pose_mat){
	Map<MatrixXd> _obj_pts((double *) PyArray_DATA((PyArrayObject*)obj_pts),3,n);
	Map<MatrixXd> _img_pts((double *) PyArray_DATA((PyArrayObject*)img_pts),2,n);
	Map<MatrixXd> _pose_mat((double *) PyArray_DATA((PyArrayObject*)pose_mat),6,1);
	Map<MatrixXd> _cam_intrinsic((double *) PyArray_DATA((PyArrayObject*)cam_intrinsic),3,3);
	
	return pnp_algos.CPnP_epnp((MatrixXd&)_obj_pts, (MatrixXd&)_img_pts, n, (MatrixXd&)_cam_intrinsic, (MatrixXd&)_pose_mat);
}

int PnPAlgos::dls_solve(PyObject* obj_pts, PyObject* img_pts, int n, PyObject* cam_intrinsic, PyObject* pose_mat){
	Map<MatrixXd> _obj_pts((double *) PyArray_DATA((PyArrayObject*)obj_pts),3,n);
	Map<MatrixXd> _img_pts((double *) PyArray_DATA((PyArrayObject*)img_pts),2,n);
	Map<MatrixXd> _pose_mat((double *) PyArray_DATA((PyArrayObject*)pose_mat),6,1);
	Map<MatrixXd> _cam_intrinsic((double *) PyArray_DATA((PyArrayObject*)cam_intrinsic),3,3);
	
	return pnp_algos.CPnP_dls((MatrixXd&)_obj_pts, (MatrixXd&)_img_pts, n, (MatrixXd&)_cam_intrinsic, (MatrixXd&)_pose_mat);
}


int PnPAlgos::upnp_solve(PyObject* obj_pts, PyObject* img_pts, int n, PyObject* cam_intrinsic, PyObject* pose_mat){
	Map<MatrixXd> _obj_pts((double *) PyArray_DATA((PyArrayObject*)obj_pts),3,n);
	Map<MatrixXd> _img_pts((double *) PyArray_DATA((PyArrayObject*)img_pts),2,n);
	Map<MatrixXd> _pose_mat((double *) PyArray_DATA((PyArrayObject*)pose_mat),6,1);
	Map<MatrixXd> _cam_intrinsic((double *) PyArray_DATA((PyArrayObject*)cam_intrinsic),3,3);
	
	return pnp_algos.CPnP_upnp((MatrixXd&)_obj_pts, (MatrixXd&)_img_pts, n, (MatrixXd&)_cam_intrinsic, (MatrixXd&)_pose_mat);
}

void export_pnp()
{
    class_<PnPAlgos>("pnp", init<int>(args("m")))
        .def("epnp_solve", &PnPAlgos::epnp_solve)
        .def("dls_solve", &PnPAlgos::dls_solve)
        .def("upnp_solve", &PnPAlgos::upnp_solve)
    ;
}
