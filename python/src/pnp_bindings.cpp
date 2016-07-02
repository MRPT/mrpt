#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <boost/python.hpp>
#include <numpy/arrayobject.h> 
using namespace boost::python;

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
using namespace Eigen;

#include <mrpt/vision/pnp_algos.h>
pnp::CPnP pnp_algos;

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
	int p3p_solve(PyObject* obj_pts, PyObject* img_pts, int n, PyObject* cam_intrinsic, PyObject* pose_mat);
	int ppnp_solve(PyObject* obj_pts, PyObject* img_pts, int n, PyObject* cam_intrinsic, PyObject* pose_mat);
	int posit_solve(PyObject* obj_pts, PyObject* img_pts, int n, PyObject* cam_intrinsic, PyObject* pose_mat);
	int lhm_solve(PyObject* obj_pts, PyObject* img_pts, int n, PyObject* cam_intrinsic, PyObject* pose_mat);
	
private:
	int dummy;
};

PnPAlgos::PnPAlgos( int new_m ){
	dummy = new_m;
}
PnPAlgos::~PnPAlgos(){
}

int PnPAlgos::epnp_solve(PyObject* obj_pts, PyObject* img_pts, int n, PyObject* cam_intrinsic, PyObject* pose_mat){
	Map<MatrixXd> _obj_pts((double *) PyArray_DATA((PyArrayObject*)obj_pts),3,n);
	Map<MatrixXd> _img_pts((double *) PyArray_DATA((PyArrayObject*)img_pts),2,n);
	Map<MatrixXd> _pose_mat((double *) PyArray_DATA((PyArrayObject*)pose_mat),6,1);
	Map<MatrixXd> _cam_intrinsic((double *) PyArray_DATA((PyArrayObject*)cam_intrinsic),3,3);
	
	return pnp_algos.CPnP_epnp(_obj_pts, _img_pts, n, _cam_intrinsic, _pose_mat);
}

int PnPAlgos::dls_solve(PyObject* obj_pts, PyObject* img_pts, int n, PyObject* cam_intrinsic, PyObject* pose_mat){
	Map<MatrixXd> _obj_pts((double *) PyArray_DATA((PyArrayObject*)obj_pts),3,n);
	Map<MatrixXd> _img_pts((double *) PyArray_DATA((PyArrayObject*)img_pts),2,n);
	Map<MatrixXd> _pose_mat((double *) PyArray_DATA((PyArrayObject*)pose_mat),6,1);
	Map<MatrixXd> _cam_intrinsic((double *) PyArray_DATA((PyArrayObject*)cam_intrinsic),3,3);
	
	return pnp_algos.CPnP_dls(_obj_pts, _img_pts, n, _cam_intrinsic, _pose_mat);
}


int PnPAlgos::upnp_solve(PyObject* obj_pts, PyObject* img_pts, int n, PyObject* cam_intrinsic, PyObject* pose_mat){
	Map<MatrixXd> _obj_pts((double *) PyArray_DATA((PyArrayObject*)obj_pts),3,n);
	Map<MatrixXd> _img_pts((double *) PyArray_DATA((PyArrayObject*)img_pts),2,n);
	Map<MatrixXd> _pose_mat((double *) PyArray_DATA((PyArrayObject*)pose_mat),6,1);
	Map<MatrixXd> _cam_intrinsic((double *) PyArray_DATA((PyArrayObject*)cam_intrinsic),3,3);
	
	return pnp_algos.CPnP_upnp(_obj_pts, _img_pts, n, _cam_intrinsic, _pose_mat);
}

int PnPAlgos::p3p_solve(PyObject* obj_pts, PyObject* img_pts, int n, PyObject* cam_intrinsic, PyObject* pose_mat){
	Map<MatrixXd> _obj_pts((double *) PyArray_DATA((PyArrayObject*)obj_pts),3,n);
	Map<MatrixXd> _img_pts((double *) PyArray_DATA((PyArrayObject*)img_pts),2,n);
	Map<MatrixXd> _pose_mat((double *) PyArray_DATA((PyArrayObject*)pose_mat),6,1);
	Map<MatrixXd> _cam_intrinsic((double *) PyArray_DATA((PyArrayObject*)cam_intrinsic),3,3);
	
	return pnp_algos.CPnP_p3p(_obj_pts, _img_pts, n, _cam_intrinsic, _pose_mat);
}

int PnPAlgos::ppnp_solve(PyObject* obj_pts, PyObject* img_pts, int n, PyObject* cam_intrinsic, PyObject* pose_mat){
	Map<MatrixXd> _obj_pts((double *) PyArray_DATA((PyArrayObject*)obj_pts),3,n);
	Map<MatrixXd> _img_pts((double *) PyArray_DATA((PyArrayObject*)img_pts),3,n);
	Map<MatrixXd> _pose_mat((double *) PyArray_DATA((PyArrayObject*)pose_mat),6,1);
	Map<MatrixXd> _cam_intrinsic((double *) PyArray_DATA((PyArrayObject*)cam_intrinsic),3,3);
	
	return pnp_algos.CPnP_ppnp(_obj_pts, _img_pts, n, _cam_intrinsic, _pose_mat);
}

int PnPAlgos::posit_solve(PyObject* obj_pts, PyObject* img_pts, int n, PyObject* cam_intrinsic, PyObject* pose_mat){
	Map<MatrixXd> _obj_pts((double *) PyArray_DATA((PyArrayObject*)obj_pts),3,n);
	Map<MatrixXd> _img_pts((double *) PyArray_DATA((PyArrayObject*)img_pts),3,n);
	Map<MatrixXd> _pose_mat((double *) PyArray_DATA((PyArrayObject*)pose_mat),6,1);
	Map<MatrixXd> _cam_intrinsic((double *) PyArray_DATA((PyArrayObject*)cam_intrinsic),3,3);
	
	return pnp_algos.CPnP_posit(_obj_pts, _img_pts, n, _cam_intrinsic, _pose_mat);
}

int PnPAlgos::lhm_solve(PyObject* obj_pts, PyObject* img_pts, int n, PyObject* cam_intrinsic, PyObject* pose_mat){
	Map<MatrixXd> _obj_pts((double *) PyArray_DATA((PyArrayObject*)obj_pts),3,n);
	Map<MatrixXd> _img_pts((double *) PyArray_DATA((PyArrayObject*)img_pts),3,n);
	Map<MatrixXd> _pose_mat((double *) PyArray_DATA((PyArrayObject*)pose_mat),6,1);
	Map<MatrixXd> _cam_intrinsic((double *) PyArray_DATA((PyArrayObject*)cam_intrinsic),3,3);
	
	return pnp_algos.CPnP_lhm(_obj_pts, _img_pts, n, _cam_intrinsic, _pose_mat);
}

void export_pnp()
{
    class_<PnPAlgos>("pnp", init<int>(args("m")))
        .def("epnp_solve", &PnPAlgos::epnp_solve)
        .def("dls_solve", &PnPAlgos::dls_solve)
        .def("upnp_solve", &PnPAlgos::upnp_solve)
        .def("p3p_solve", &PnPAlgos::p3p_solve)
        .def("ppnp_solve", &PnPAlgos::ppnp_solve)
        .def("posit_solve", &PnPAlgos::posit_solve)
        .def("lhm_solve", &PnPAlgos::lhm_solve)
    ;
}
