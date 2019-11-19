#include <cassert>
#include <iostream>
#include <fstream>
#include <sstream>
#include <random>

#include <TooN/helpers.h>
#include <TooN/Cholesky.h>
#include <TooN/se3.h>

#include <cvd/camera.h>
#include <cvd/convolution.h>
#include <cvd/videosource.h>
#include <cvd/gl_helpers.h>
#include <cvd/vision.h>
#include <cvd/image_io.h>
#include <cvd/image_interpolate.h>
#include <cvd/glwindow.h>
#include <cvd/timer.h>
#include <cvd/colourspaces.h>
#include <cvd/colourspace_convert.h>
#include <cvd/colourspacebuffer.h>
#include <cvd/videosource.h>

using namespace std;
using namespace TooN;
using namespace CVD;

#include <X11/keysym.h>
#include <X11/Xlib.h>

typedef byte CAMERA_PIXEL;
VideoBuffer<CAMERA_PIXEL>* videoBuffer=0;

// global configuration variables, can be set via command line options
#ifdef CVD_HAVE_QTBUFFER
string videoDevice = "colourspace:[from=yuv422]//qt://0";
#else
string videoDevice = "v4l2:///dev/video0";
#endif

typedef Camera::OldCameraAdapter<Camera::Harris> CameraModel;
static const int NumCameraParams = CameraModel::num_parameters;

Vector<NumCameraParams> badVector()
{
	Vector<NumCameraParams> v;
	for(int i=0; i < NumCameraParams; i++)
		v[i] = -1;

	return v;
}


Vector<NumCameraParams> cameraParameters = badVector();
int bottomLeftWhite = 1;
int gridx = 11;
int gridy = 7;
double cellSize = 0.02;
double interval = 0.5;
int generateErrorImage = 0;
string output;

void drawGrid (vector<Vector<2> > grid, int cols, int rows)
{
    glBegin (GL_LINES);
    for (unsigned int i = 0; i < grid.size(); i++)
    {
	if (((i+1) % (cols+1)))
        {
	    //Not last column, draw a horiz line
	    glVertex(grid[i]);
	    glVertex(grid[i+1]);
        }

	if (i < (unsigned int)rows * (cols+1))
        {
	    //Not top row, draw a vert line
	    glVertex(grid[i]);
	    glVertex(grid[i+cols+1]);
        }
    }
    glEnd ();
}

void drawCross(Vector<2> pos, double size)
{
    glBegin (GL_LINES);
    glVertex2f (pos[0] - size / 2, pos[1]);
    glVertex2f (pos[0] + size / 2, pos[1]);
    glVertex2f (pos[0] , pos[1] - size / 2);
    glVertex2f (pos[0] , pos[1] + size / 2);
    glEnd ();
}

void drawCross(ImageRef pos, int size)
{
    glBegin (GL_LINES);
    glVertex2i (pos.x - size / 2, pos.y);
    glVertex2i (pos.x + size / 2, pos.y);
    glVertex2i (pos.x , pos.y - size / 2);
    glVertex2i (pos.x , pos.y + size / 2);
    glEnd ();
}

template <typename B>
Matrix<2> inverse(const Matrix<2, 2, double, B>& A)
{
	Matrix<2> result;
	const double idet = 1/(A(0,0)*A(1,1)-A(0,1)*A(1,0));
	result(0,0) = A(1,1);
	result(1,0) = - A(0,1);
	result(0,1) = - A(1,0);
	result(1,1) = A(0,0);
	result *= idet;
	return result;
}

template <class A1, class A2, class A3> inline
Vector<2> project_transformed_point(const SE3<> & pose, const Vector<3,double,A1>& in_frame, Matrix<2,3,double,A2>& J_x, Matrix<2,6,double,A3>& J_pose)
{
	const double z_inv = 1.0/in_frame[2];
	const double x_z_inv = in_frame[0]*z_inv;
	const double y_z_inv = in_frame[1]*z_inv;
	const double cross = x_z_inv * y_z_inv;
	J_pose[0][0] = J_pose[1][1] = z_inv;
	J_pose[0][1] = J_pose[1][0] = 0;
	J_pose[0][2] = -x_z_inv * z_inv;
	J_pose[1][2] = -y_z_inv * z_inv;
	J_pose[0][3] = -cross;
	J_pose[0][4] = 1 + x_z_inv*x_z_inv; 
	J_pose[0][5] = -y_z_inv;  
	J_pose[1][3] = -1 - y_z_inv*y_z_inv;
	J_pose[1][4] =  cross;
	J_pose[1][5] =  x_z_inv;    
	
	const TooN::Matrix<3>& R = pose.get_rotation().get_matrix();
	J_x[0][0] = z_inv*(R[0][0] - x_z_inv * R[2][0]);
	J_x[0][1] = z_inv*(R[0][1] - x_z_inv * R[2][1]);
	J_x[0][2] = z_inv*(R[0][2] - x_z_inv * R[2][2]);
	J_x[1][0] = z_inv*(R[1][0] - y_z_inv * R[2][0]);
	J_x[1][1] = z_inv*(R[1][1] - y_z_inv * R[2][1]);
	J_x[1][2] = z_inv*(R[1][2] - y_z_inv * R[2][2]);
	
	return makeVector(x_z_inv, y_z_inv);
}


template <class A1> inline
Vector<2> transform_and_project(const SE3<>& pose, const Vector<3,double,A1>& x)
{
	return project(pose * x);
}

template <class A1, class A2, class A3> inline
Vector<2> transform_and_project(const SE3<>& pose, const Vector<3,double,A1>& x, Matrix<2,3,double,A2>& J_x, Matrix<2,6,double,A3>& J_pose)
{
return project_transformed_point(pose, pose * x, J_x, J_pose);
}

template <class CamModel, class P>
SE3<> find_pose(const SE3<>& start, const vector<Vector<3> >& x, const vector<pair<size_t,P> >& p, CamModel& camModel, double noise)
{
    vector<pair<Vector<2>, Matrix<2> > > unprojected(p.size());

    for (size_t i=0; i<p.size(); ++i) {
	unprojected[i].first = camModel.unproject(p[i].second);
	Matrix<2> Jinv = inverse(camModel.get_derivative());
	unprojected[i].second = Jinv * (noise * Jinv.T());
    }

    SE3<> bestSE3;
    double bestError = numeric_limits<double>::max();
    vector<Vector<2> > up(p.size());
    for (size_t i=0; i<p.size(); i++)
	up[i] = camModel.unproject(p[i].second);
    SE3<> se3;
    se3 = start;

    for (int iter=0; iter<4; ++iter) {
	Matrix<6> I = Zeros;
	Vector<6> b = Zeros;
	for (size_t i=0; i<p.size(); ++i) {
	    Matrix<2,3> J_x;
	    Matrix<2,6> J_pose;
	    Vector<2> v = unprojected[i].first - transform_and_project(se3, x[p[i].first], J_x, J_pose);
	    const Matrix<2> Rinv = inverse(unprojected[i].second);
	    I += J_pose.T() * Rinv * J_pose;
	    //transformCovariance<util::PlusEquals>(J_pose.T(), Rinv, I);
	    b += J_pose.T() * (Rinv * v);
	}
	Cholesky<6> chol(I);
	se3.left_multiply_by(SE3<>::exp(chol.backsub(b)));

	double residual = 0;
	for (size_t i=0; i<p.size(); ++i) {
	    Vector<2> v = unprojected[i].first - transform_and_project(se3, x[p[i].first]);
	    const Matrix<2> Rinv = inverse(unprojected[i].second);
	    residual += v * (Rinv * v);
	}
	if (residual < bestError) {
	    bestError = residual;
	    bestSE3 = se3;
	}
    }
    return bestSE3;
}

template <class CamModel, class P>
SE3<> find_pose_and_params(const SE3<>& start, const vector<Vector<3> >& x, const vector<pair<size_t,P> >& p, CamModel& camModel, double noise)
{
    static const int NCP = CamModel::num_parameters;
    static const int NP = NCP + 6;
    //const Matrix<2> Rinv = Identity<2>(1.0/noise);
    Matrix<2> Rinv = Identity;
    Rinv /= noise;
    SE3<> best_pose = start;
    Vector<NCP> best_params = camModel.get_parameters();
    SE3<> pose = start;
  
    double residual = 0;
    for (size_t i=0; i<p.size(); ++i) {
	Vector<2> v = p[i].second - camModel.project(transform_and_project(pose, x[p[i].first]));
	residual += v*Rinv*v;
    }
    double best_residual = residual;

    double lambda = 1.0;
    for (int iter=0; iter<10; ++iter) { 
	Matrix<NP> I = Identity;
	I *= lambda;
	Vector<NP> b = Zeros;
	for (size_t i=0; i<p.size(); ++i) {
	    Matrix<2,3> J_x;
	    Matrix<2,6> J_pose;
	    Vector<2> v = p[i].second - camModel.project(transform_and_project(pose, x[p[i].first], J_x, J_pose));
	    Matrix<2,NP> J;
	    J.template slice<0,0,2,NCP>() = camModel.get_parameter_derivs().T();
	    J.template slice<0,NCP,2,6>() = camModel.get_derivative() * J_pose;
	  
	  	I += J.T() * Rinv * J;
	    // transformCovariance<util::PlusEquals>(J.T(), Rinv, I);
	    b += J.T() * (Rinv * v);
	}
	Cholesky<NP> chol(I);
	Vector<NP> delta = chol.backsub(b);
	camModel.get_parameters() += delta.template slice<0,NCP>();
	pose.left_multiply_by(SE3<>::exp(delta.template slice<NCP,6>()));

	double residual = 0;
	for (size_t i=0; i<p.size(); ++i) {
	    Vector<2> v = p[i].second - camModel.project(transform_and_project(pose, x[p[i].first]));
	    residual += v*Rinv*v;
	}
	if (residual < best_residual) {
	    best_residual = residual;
	    best_pose = pose;
	    best_params = camModel.get_parameters();
	    lambda *= 0.5;
	} else {
	    pose = best_pose;
	    camModel.get_parameters() = best_params;
	    lambda *= 1e2;
	}
    }
    camModel.get_parameters() = best_params;
    return best_pose;
}

void getOptions(int argc, char* argv[])
{
    for (int i = 1; i<argc; i++){
	string arg = argv[i];
	if (arg == "-o") {
	    output = argv[++i];
	} else if (arg == "-d") {
	    videoDevice = argv[++i];
	} else if (arg == "-x") {
	    gridx = atoi(argv[++i]);
	} else if (arg == "-y") {
	    gridy = atoi(argv[++i]);
	} else if (arg == "-s") {
	    cellSize = atof(argv[++i]);
	} else if (arg == "-t") {
	    interval = atof(argv[++i]);
	} else if (arg == "-c") {
	    istringstream sin(argv[++i]);
	    sin >> cameraParameters;
	} else if (arg == "-e") {
	    generateErrorImage = 1;
	} else {
	    cerr << argv[0] << " [options]\n\n"
		"Move virtual grid over real grid until it snaps on.\n"
		"Record a number of frames ~100.\n"
		"Press SPACE to calculate camera parameters.\n\n"
		"A calibration grid for the default parameters is in libcvd/doc/cameracalib2cm.pdf\n\n"
		"flag  description\t\t\t\tdefault\n"
		"  -d  device to open, CVD video source URL\tv4l2:///dev/video0\n"
		"  -x  grid dimension in x (width) direction\t11\n"
		"  -y  grid dimension in y (height) direction\t7\n"
		"  -s  grid cell size\t\t\t\t0.02 m\n"
		"  -t  interval between captured frames\t\t0.5 s\n"
		"  -c  initial camera parameters\t\t\t" << cameraParameters << "\n"
		"  -e  generate image showing errors per pixel\n";
	    exit(0);
	}
    }
    try {
        cerr << "opening " << videoDevice << endl;
        videoBuffer = open_video_source<CAMERA_PIXEL>(videoDevice);
    }
    catch (CVD::Exceptions::All& e) {
	cerr << e.what << endl;
	exit(1);
    }
}

vector<Vector<2> > makeGrid(int gx, int gy, double cellSize)
{
    vector<Vector<2> > grid;
    Vector<2> center = makeVector(gx,gy);
    center *= cellSize/2.0;
    for (int y=0; y<=gy; y++)
    {
	for (int x=0; x<=gx; x++)
        {
	    grid.push_back(Vector<2>(makeVector(x,y))*cellSize - center);
        }
    }
    return grid;
}

template <class CM>
void drawPoints(const vector<Vector<2> >& points, const SE3<>& pose, CM& cm)
{
    glColor3f(0,1,0);
    for (size_t i=0; i<points.size(); i++)
    {
	Vector<3> x = unproject(points[i]);
	Vector<2> plane = project(pose * x);
	Vector<2> im = cm.project(plane);
	drawCross(im, 6);
    }
}

struct MeasurementSet
{
    vector<Vector<2> > im;
    vector<Vector<3> > world;
};

template <class CM>
double getReprojectionError(const vector<MeasurementSet>& ms, const vector<SE3<> >& poses, CM& cm)
{
    double error = 0;
    for (size_t i=0; i<ms.size(); i++)
    {
	for (size_t j=0; j<ms[i].im.size(); j++)
        {
	    Vector<3> camFrame = poses[i] * ms[i].world[j];
	    Vector<2> im = cm.project(project(camFrame));
	    Vector<2> v = ms[i].im[j] - im;
	    error += v*v;
        }
    }
    return error;
}

template <class CM>
pair<double, double> getReprojectionError(const vector<MeasurementSet>& ms, const vector<SE3<> >& poses, CM& cm,
                                          vector<pair<Vector<2>,Vector<2> > >& v)
{
    double error = 0, maxError = 0;
    for (size_t i=0; i<ms.size(); i++)
    {
	for (size_t j=0; j<ms[i].im.size(); j++)
        {
	    Vector<3> camFrame = poses[i] * ms[i].world[j];
	    Vector<2> im = cm.project(project(camFrame));
	    Vector<2> err = ms[i].im[j] - im;
	    v.push_back(make_pair(im, err));
	    double thisError = err*err;
	    maxError = std::max(maxError, thisError);
	    error += thisError;
        }
    }
    return make_pair(error, maxError);
}

template <class CM>
double getReprojectionError(const vector<MeasurementSet>& ms, const vector<vector<Vector<2> > >& plane, CM& cm)
{
    assert(plane.size() == ms.size());
    double error = 0;
    for (size_t i=0; i<ms.size(); i++)
    {
	for (size_t j=0; j<plane[i].size(); j++)
        {
	    Vector<2> im = cm.project(plane[i][j]);
	    Vector<2> v = ms[i].im[j] - im;
	    error += v*v;
        }
    }
    return error;
}

template <class CM>
void improveLM(vector<MeasurementSet>& ms, vector<SE3<> >& pose, CM& cm, double lambda)
{
    Matrix<> JTJ(CM::num_parameters+ms.size()*6,CM::num_parameters+ms.size()*6);
    Vector<> JTe(JTJ.num_rows());

    JTJ= Zeros;
    Vector<CM::num_parameters> JTep = Zeros;

    for (size_t i=0; i<ms.size(); i++)
    {
	Matrix<6> poseBlock = Zeros;
	Matrix<CM::num_parameters> paramBlock = Zeros;
	Matrix<CM::num_parameters, 6> offDiag = Zeros;
      
	Vector<6> JTei = Zeros;
      
	for (size_t j=0; j<ms[i].im.size(); j++)
	{
	    Vector<3> camFrame = pose[i] * ms[i].world[j];
	    Matrix<2,3> J_x;
	    Matrix<2,6> J_pose;
	    Vector<2> v = ms[i].im[j] - cm.project(transform_and_project(pose[i], ms[i].world[j], J_x, J_pose));
	    //Vector<2> im = cm.project(project(camFrame));
	    //Vector<2> v = ms[i].im[j] - im;
	    //makeProjectionParameterJacobian(camFrame, /*pose[i],*/ J_pose);
	    J_pose = cm.get_derivative() * J_pose;
	    Matrix<2,CM::num_parameters> J_param = cm.get_parameter_derivs().T();
	    poseBlock += J_pose.T()*J_pose;
	    paramBlock += J_param.T() * J_param;
	    offDiag += J_param.T() * J_pose;
	    JTei += J_pose.T() * v;
	    JTep += J_param.T() * v;
	}
	JTe.slice(CM::num_parameters + i*6, 6) = JTei;
	JTJ.slice(CM::num_parameters + i*6, CM::num_parameters + i*6, 6,6) = poseBlock;
	JTJ.slice(0,0,CM::num_parameters, CM::num_parameters) += paramBlock;
	JTJ.slice(0,CM::num_parameters + i*6, CM::num_parameters, 6) = offDiag;
	JTJ.slice(CM::num_parameters + i*6,0, 6, CM::num_parameters) = offDiag.T();
    }
    JTe.slice(0,CM::num_parameters) = JTep;
  
    for (int i=0; i<JTJ.num_rows(); i++)
	JTJ[i][i] += lambda;
    Vector<> delta = Cholesky<>(JTJ).backsub(JTe);
    cm.get_parameters() += delta.template slice<0,CM::num_parameters>();
    for (size_t i=0; i<pose.size(); i++)
    {
	pose[i].left_multiply_by(SE3<>::exp(delta.slice(CM::num_parameters+i*6, 6)));
    }
}

template <class CM>
void getUncertainty(const vector<MeasurementSet>& ms, const vector<SE3<> >& pose, CM& cm, Matrix<CM::num_parameters>& C)
{
    Matrix<> JTJ(CM::num_parameters+ms.size()*6,CM::num_parameters+ms.size()*6);
    JTJ = Zeros;
    for (size_t i=0; i<ms.size(); i++)
    {
	Matrix<6> poseBlock = Zeros;
	Matrix<CM::num_parameters> paramBlock = Zeros;
	Matrix<CM::num_parameters, 6> offDiag = Zeros;
	for (size_t j=0; j<ms[i].im.size(); j++)
        {
	    Matrix<2,3> J_x;
	    Matrix<2,6> J_pose;
	    Vector<2> v = ms[i].im[j] - cm.project(transform_and_project(pose[i], ms[i].world[j], J_x, J_pose));

	    J_pose = cm.get_derivative() * J_pose;
	    Matrix<2,CM::num_parameters> J_param = cm.get_parameter_derivs().T();
	    poseBlock += J_pose.T()*J_pose;
	    paramBlock += J_param.T() * J_param;
	    offDiag += J_param.T() * J_pose;
        }
	JTJ.slice(CM::num_parameters + i*6, CM::num_parameters + i*6, 6,6) = poseBlock;
	JTJ.slice(0,0,CM::num_parameters, CM::num_parameters) = JTJ.slice(0,0,CM::num_parameters, CM::num_parameters) + paramBlock;
	JTJ.slice(0,CM::num_parameters + i*6, CM::num_parameters, 6) = offDiag;
	JTJ.slice(CM::num_parameters + i*6,0, 6, CM::num_parameters) = offDiag.T();
    }
    Cholesky<> chol(JTJ);
    Vector<> v(JTJ.num_cols());
    v = Zeros;
  
    for (int i=0; i<CM::num_parameters; ++i) {
	v[i]=1;
	Vector<> Cv = chol.backsub(v);
	v[i]=0;
	C.T()[i] = Cv.template slice<0,CM::num_parameters>();
    }
}

inline Vector<2> imagePoint(const Vector<2>& inPoint, const SE3<> & pose, CameraModel& cm, const double& factor)
{
    Vector<3> point3D = unproject(inPoint);
    Vector<2> plane = project(pose*point3D);
    Vector<2> im = cm.project(plane) / factor;

    return im;
}

inline float imageVal(image_interpolate<Interpolate::Bilinear, float> &imgInter, const Vector<2>& inPoint, const SE3<> & pose,
                      CameraModel& cm, const double& factor, const bool& boundsCheck)
{
    Vector<2> imageVec = imagePoint(inPoint, pose, cm, factor);
    if(!boundsCheck) //Slight speed-up
	return imgInter[imageVec];

    if (imgInter.in_image(imageVec))
	return imgInter[imageVec];
    else
	return -1;

}

//l,t,r,b
inline bool inSquareX(const Vector<2>& inPoint, const Vector<4>& calibSquare)
{
    if(inPoint[0] < calibSquare[0])
	return false;
    if(inPoint[0] > calibSquare[2])
	return false;

    return true;
}
//l,t,r,b
inline bool inSquareY(const Vector<2>& inPoint, const Vector<4>& calibSquare)
{
    if(inPoint[1] > calibSquare[1])
	return false;
    if(inPoint[1] < calibSquare[3])
	return false;

    return true;
}

inline float minMargin(const Vector<2>& imagePoint, const Vector<2>& minCoord, const Vector<2>& maxCoord)
{
    return std::min(imagePoint[0]-minCoord[0],
		    std::min(imagePoint[1]-minCoord[1],
			     std::min(maxCoord[0] - imagePoint[0], maxCoord[1] - imagePoint[1])));
}

inline float minMarginSquare(const Vector<2>& inPoint, image_interpolate<Interpolate::Bilinear, float>& imgInter, const SE3<> & pose,
                             CameraModel& cm, double factor, float cellSize)
{
    Vector<2> posVec = makeVector( cellSize/2, cellSize/2);
    Vector<2> negVec = makeVector( cellSize/2, -cellSize/2);

    Vector<2> minCoord = imgInter.min();
    Vector<2> maxCoord = imgInter.max();

    float minVal = minMargin(imagePoint(inPoint, pose, cm, factor), minCoord, maxCoord);
    minVal = std::min(minVal, minMargin(imagePoint(inPoint - negVec, pose, cm, factor), minCoord, maxCoord)); //top-left
    minVal = std::min(minVal, minMargin(imagePoint(inPoint + posVec, pose, cm, factor), minCoord, maxCoord)); //top-right
    minVal = std::min(minVal, minMargin(imagePoint(inPoint - posVec, pose, cm, factor), minCoord, maxCoord)); //bottom-left
    minVal = std::min(minVal, minMargin(imagePoint(inPoint + negVec, pose, cm, factor), minCoord, maxCoord)); //bottom-right

    return minVal;
}

bool sanityCheck(const Vector<2>& inPoint, image_interpolate<Interpolate::Bilinear, float>& imgInter, const SE3<> & pose,
                 CameraModel& cm, double factor, bool blWhite, float cellSize)
{
    Vector<2> posVec = makeVector( cellSize/2, cellSize/2);
    Vector<2> negVec = makeVector( cellSize/2, -cellSize/2);

    // Don't need to bounds check these as it's done after the minMarginSquare > 0 check
    float midVal = imageVal(imgInter, inPoint, pose, cm, factor, false);
    float tlVal = imageVal(imgInter, inPoint - negVec, pose, cm, factor, false);
    float trVal = imageVal(imgInter, inPoint + posVec, pose, cm, factor, false);
    float blVal = imageVal(imgInter, inPoint - posVec, pose, cm, factor, false);
    float brVal = imageVal(imgInter, inPoint + negVec, pose, cm, factor, false);

    if(blWhite)
    {
	if(midVal - tlVal < 0.02)
	    return false;
	if(trVal - midVal < 0.02)
	    return false;
	if(blVal - midVal < 0.02)
	    return false;
	if(midVal - brVal < 0.02)
	    return false;
    }
    else
    {
	if(tlVal - midVal < 0.02)
	    return false;
	if(midVal - trVal < 0.02)
	    return false;
	if(midVal - blVal < 0.02)
	    return false;
	if(brVal - midVal < 0.02)
	    return false;
    }

    return true;
}

bool findInitialIntersectionEstimate(image_interpolate<Interpolate::Bilinear, float> &imgInter, Vector<2>& initialPoint,
                                     const SE3<> & pose, CameraModel& cm, double factor,
                                     bool boundsCheck, const Vector<4>& likelySquare, double cellSize)
{
    bool looksOK = true;

    //very roughly find a sensible starting place
    Vector<2> testPoint = makeVector( likelySquare[0], likelySquare[1]);
    float startPx = imageVal(imgInter, testPoint, pose, cm, factor, boundsCheck);
    while(looksOK && fabs(imageVal(imgInter, testPoint, pose, cm, factor, boundsCheck) - startPx) < 0.1)
    {
	testPoint[0] += cellSize/10;
	if(testPoint[0] > likelySquare[2])
	    looksOK = false;
    }
    if(looksOK)
	initialPoint[0] = testPoint[0];

    testPoint[0] = likelySquare[0];
    while(looksOK && fabs(imageVal(imgInter, testPoint, pose, cm, factor, boundsCheck) - startPx) < 0.1)
    {
	testPoint[1] -= cellSize/10;
	if(testPoint[1] < likelySquare[3])
	    looksOK = false;
    }
    if(looksOK)
	initialPoint[1] = testPoint[1];

    return looksOK;
}

bool optimiseIntersection(image_interpolate<Interpolate::Bilinear, float> &imgInter, Vector<2>& inPoint,
                          const SE3<> & pose, CameraModel& cm, double factor, bool boundsCheck,
                          const Vector<4>& likelySquare, double cellSize, bool blWhite)
{
    Vector<2> largeX = makeVector( 0.004, 0);
    Vector<2> largeY = makeVector( 0, 0.004);
    Vector<2> smallX = makeVector( cellSize/10, 0);
    Vector<2> smallY = makeVector( 0, cellSize/10);

    float aboveVal = imageVal(imgInter, inPoint + largeY, pose, cm, factor, boundsCheck);
    float belowVal = imageVal(imgInter, inPoint - largeY, pose, cm, factor, boundsCheck);
    float leftVal = imageVal(imgInter, inPoint - largeX, pose, cm, factor, boundsCheck);
    float rightVal = imageVal(imgInter, inPoint + largeX, pose, cm, factor, boundsCheck);

    while(fabs(aboveVal - belowVal) > 0.2 || fabs(leftVal - rightVal) > 0.2)
    {
	//reduce step size
	smallY = smallY/2;
	smallX = smallX/2;

	while(belowVal < aboveVal)
        {
	    //lighter above - move left if bottom-left white, otherwise right
	    blWhite ? inPoint -= smallX : inPoint += smallX;
	    if(!inSquareX(inPoint, likelySquare))
		break;
	    aboveVal = imageVal(imgInter, inPoint + largeY, pose, cm, factor, boundsCheck);
	    belowVal = imageVal(imgInter, inPoint - largeY, pose, cm, factor, boundsCheck);
        }
	while(belowVal > aboveVal)
        {
	    //darker above - move right if bottom-left white, otherwise left
	    blWhite ? inPoint += smallX : inPoint -= smallX;
	    if(!inSquareX(inPoint, likelySquare))
		break;
	    aboveVal = imageVal(imgInter, inPoint + largeY, pose, cm, factor, boundsCheck);
	    belowVal = imageVal(imgInter, inPoint - largeY, pose, cm, factor, boundsCheck);
        }

	while(rightVal > leftVal)
        {
	    //darker on left - move down if bottom-left white, otherwise up
	    blWhite ? inPoint -= smallY : inPoint += smallY;
	    if(!inSquareY(inPoint, likelySquare))
		break;
	    leftVal = imageVal(imgInter, inPoint - largeX, pose, cm, factor, boundsCheck);
	    rightVal = imageVal(imgInter, inPoint + largeX, pose, cm, factor, boundsCheck);
        }
	while(rightVal < leftVal)
        {
	    //lighter on left - move up if bottom-left white, otherwise down
	    blWhite ? inPoint += smallY : inPoint -= smallY;
	    if(!inSquareY(inPoint, likelySquare))
		break;
	    leftVal = imageVal(imgInter, inPoint - largeX, pose, cm, factor, boundsCheck);
	    rightVal = imageVal(imgInter, inPoint + largeX, pose, cm, factor, boundsCheck);
        }

	if(!(inSquareX(inPoint, likelySquare) && inSquareY(inPoint, likelySquare)))
	    break;

	if(smallX[0] < 0.000002)
	    break;  //prevents infinite loop with certain patterns
    }

    return (fabs(aboveVal - belowVal) < 1 && fabs(leftVal - rightVal) < 1);
}

mt19937 engine;
normal_distribution<double> gaussian;

double rand_g()
{
	return gaussian(engine);
}


int main(int argc, char* argv[])
{
    getOptions(argc, argv);
	cvd_timer timer;

    string titlePrefix = "Calibrate: Align grid ([ and ] to resize)";
    GLWindow disp(videoBuffer->size(), titlePrefix);
    GLWindow::EventSummary events;
    //VideoDisplay disp (0.0, 0.0, 640.0, 480.0);
    CameraModel cameraModel;
    double factor=1.0;
    vector<MeasurementSet> measurementSets;
    vector<SE3<> > poses;
    ImageRef imageSize;

	if(cameraParameters[0] == -1)
	{
		cameraParameters = Zeros;
		cameraParameters[0] = 500;
		cameraParameters[1] = 500;
		cameraParameters[2] = videoBuffer->size().x/2;
		cameraParameters[3] = videoBuffer->size().y/2;
	}

    //XEvent e;
    //disp.select_events(KeyPressMask);
    bool doParams = true;
    int stage = 2; // 2 is init, 1 record, 0 done

    disp.set_title(titlePrefix);

    double curr = timer.get_time();
    imageSize = videoBuffer->size();

    cameraModel.get_parameters() = cameraParameters;
    factor = 1.0/std::max(imageSize.x, imageSize.y);
    cameraModel.get_parameters().slice<0,4>() *= factor;

    vector<Vector<2> > grid = makeGrid(gridx, gridy, cellSize);
    vector<Vector<3> > grid3d(grid.size());
    for (size_t i=0; i<grid.size(); i++)
        grid3d[i] = unproject(grid[i]);

    SE3<> pose;
    pose.get_translation()[2] = -0.7;
    const SE3<> init_pose = pose;

    glPixelStorei(GL_UNPACK_ALIGNMENT,1);
    glDrawBuffer(GL_BACK);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);

    double lastMeasure = timer.get_time();
    bool prevFrameTracked = false;
    while (stage)
    {
	    
	events.clear();
	disp.get_events(events);

	for (GLWindow::EventSummary::key_iterator it = events.key_down.begin(); it!=events.key_down.end(); ++it) {
	    switch (it->first) {
	    case XK_Home:
		pose = init_pose;
		break;
	    case XK_bracketleft:
		pose.get_translation()[2] += 0.02;
		break;
	    case XK_bracketright:
		pose.get_translation()[2] -= 0.02;
		break;
	    case XK_Escape:
	    case XK_q:
	    case XK_Q:
		exit(0);
		break;
	    case XK_space:
		if(stage == 1) {
		    stage = 0;
		    titlePrefix = "Calibrate: Calculating Camera Parameters";
		    disp.set_title(titlePrefix);
		}
		break;
	    case XK_p:
		doParams = !doParams;
		if(stage == 1)
		{
		    if(doParams)
			titlePrefix = "Calibrate: Tracking Pose and Internal Params";
		    else
			titlePrefix = "Calibrate: Tracking Pose";
		}
		break;
	    }
	}
	    
	
	//videoBuffer->flush();
	VideoFrame<CAMERA_PIXEL>* vframe = videoBuffer->get_frame();
	
	// leave this in, we cannot assume that vframe has a datatype that can be
	// directly used in the glTexImage call later on (e.g. its yuv422 on OSX)
	Image<byte> temp = convert_image(*vframe);
	Image<float> gray = convert_image(temp);
	videoBuffer->put_frame(vframe);

	glDisable(GL_BLEND);
	GLenum texTarget;
	#ifdef GL_TEXTURE_RECTANGLE_ARB
		texTarget=GL_TEXTURE_RECTANGLE_ARB;
	#elif defined GL_TEXTURE_RECTANGLE_NV
		texTarget=GL_TEXTURE_RECTANGLE_NV;
	#else
		texTarget=GL_TEXTURE_RECTANGLE_EXT;
	#endif
	glEnable(texTarget);
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
	glTexParameterf( texTarget, GL_TEXTURE_MIN_FILTER, GL_NEAREST );
	glTexParameterf( texTarget, GL_TEXTURE_MAG_FILTER, GL_NEAREST );
	glPixelStorei(GL_UNPACK_ALIGNMENT,1);
	glTexImage2D(temp, 0, texTarget);
	glBegin(GL_QUADS);
	glTexCoord2i(0, 0);
	glVertex2i(0,0);
	glTexCoord2i(temp.size().x, 0);
	glVertex2i(disp.size().x,0);
	glTexCoord2i(temp.size().x,temp.size().y);
	glVertex2i(disp.size().x,disp.size().y);
	glTexCoord2i(0, temp.size().y);
	glVertex2i(0, disp.size().y);
	glEnd ();
	glDisable(texTarget);

	glEnable(GL_BLEND);

	//this is the bit that does the calibrating
	vector<pair<size_t, Vector<2> > > measurements;
	vector<Vector<2> > failedPoints;

	//smooth and prepare image for interpolation
	convolveGaussian(gray, 0.5);
	image_interpolate<Interpolate::Bilinear, float> imgInter =
            image_interpolate<Interpolate::Bilinear, float>(gray);

	int guessCount = 0;
	int totalPass = 0, totalFail = 0;
	SE3<> guessPose = pose;
	bool correctPose = false;

	do
	{
	    measurements.clear();
	    failedPoints.clear();

	    int rowPass[gridy+1];
	    for(int i = 0; i < gridy+1; i++)
                rowPass[i] = 0;
	    int rowFail[gridy+1];
	    for(int i = 0; i < gridy+1; i++)
                rowFail[i] = 0;
	    int colPass[gridx+1];
	    for(int i = 0; i < gridx+1; i++)
                colPass[i] = 0;
	    int colFail[gridx+1];
	    for(int i = 0; i < gridx+1; i++)
                colFail[i] = 0;

	    totalPass = 0;
	    totalFail = 0;

	    for (size_t i=0; i<grid.size(); i++)
	    {
		int yNo = i/(gridx+1);
		int xNo = i%(gridx+1);
		bool blWhite = (xNo+yNo) % 2;
		if (bottomLeftWhite)
                    blWhite = !blWhite;

		Vector<2> inPoint = makeVector( xNo*cellSize - gridx*cellSize/2, yNo*cellSize - gridy*cellSize/2);
		Vector<4> likelySquare = makeVector( inPoint[0] - cellSize/2, inPoint[1] + cellSize/2,
					  inPoint[0] + cellSize/2, inPoint[1] - cellSize/2); //l,t,r,b

		float minMarginDist = minMarginSquare(inPoint, imgInter, guessPose, cameraModel, factor, cellSize);
		if(minMarginDist > 0)
		{
		    bool boundsCheck = (minMarginDist < 40) ? true : false;

		    bool okEst = findInitialIntersectionEstimate(imgInter, inPoint, guessPose, cameraModel,
								 factor, boundsCheck, likelySquare, cellSize);

		    bool pass = false;
		    bool fail = true;

		    if(okEst)
		    {
			//Find the accurate intersection
			bool converged = optimiseIntersection(imgInter, inPoint, guessPose, cameraModel, factor,
							      boundsCheck, likelySquare, cellSize, blWhite);

			//Check the found position - inside the image bounds?
			if(minMarginSquare(inPoint, imgInter, guessPose, cameraModel, factor, cellSize) > 0)
			{
			    //In expected square?
			    if(inSquareX(inPoint, likelySquare) && inSquareY(inPoint, likelySquare))
			    {
				//Optimisation suitably converged?
				if(converged)
				{
				    //Sensible difference between the black and white squares?
				    if(sanityCheck(inPoint, imgInter, guessPose, cameraModel, factor, blWhite, cellSize))
				    {
					pass = true;
					fail = false;
					rowPass[yNo]++;
					colPass[xNo]++;
					totalPass++;

					//add to the list of points to be used to optimise the pose
					measurements.push_back(make_pair(i, imagePoint(inPoint, guessPose, cameraModel, factor)*factor));
				    }
				}
			    }
			}
			else
                            fail = false; //Points outside the image haven't really failed
		    }

		    if(fail)
		    {
			rowFail[yNo]++;
			colFail[xNo]++;
			totalFail++;

			failedPoints.push_back(imagePoint(inPoint, pose, cameraModel, factor));
		    }
		}
	    }

	    //If previous frame tracked OK, decision on next one is based on no. of failures
	    //Otherwise base on no. of passes - this means if tracking is going OK it will track
	    //even when not all points are visible. However if tracking is lost, it will not
	    //lock on again until it passes suitably on all rows
	    if(prevFrameTracked)
	    {
		int worstRow = rowFail[0];
		for(int i =0; i<gridy+1; i++)
                    if(rowFail[i] > worstRow)
			worstRow = rowFail[i];

		int worstCol = colFail[0];
		for(int i =0; i<gridx+1; i++)
                    if(colFail[i] > worstCol)
			worstCol = colFail[i];

		//allow half of the worst column or row to fail, as long as overall 5x more points pass than fail
		if(worstCol <= gridx/2 && worstRow <= gridy/2 && static_cast<float>(totalFail)/totalPass < 0.2)
                    correctPose = true;
	    }
	    else
	    {
		int worstRow = rowPass[0]
		    ;
		for(int i =0; i<gridy+1; i++)
                    if(rowPass[i] < worstRow)
			worstRow = rowPass[i];

		int worstCol = colPass[0];
		for(int i =0; i<gridx+1; i++)
                    if(colPass[i] < worstCol)
			worstCol = colPass[i];

		//If tracking failed, apply more restrictive matching constraints:
		//Need at least half the worst/row column to be OK, and 10x more points to pass than fail
		if(worstCol >= (gridx + 1)/2 && worstRow >= (gridy + 1)/2 &&  static_cast<float>(totalFail)/totalPass < 0.1)
                    correctPose = true;
	    }

	    if(correctPose)
	    {
		//This pose worked - let's use it
		pose = guessPose;
	    }
	    else
	    {
		//Didn't track correctly - let's guess! - just change the pose a bit
		guessCount++;
		guessPose = SE3<>::exp(makeVector( rand_g()/300, rand_g()/300, rand_g()/40, rand_g()/10, rand_g()/10, rand_g()/10));
		//Apply the change in grid coordinates
		//Grid coordinates actually centred on (0,0,1) in 3D, so need to shift before and after
		guessPose = SE3<>::exp(makeVector( 0, 0, 1, 0, 0, 0)) * guessPose * SE3<>::exp(makeVector( 0, 0, -1, 0, 0, 0));
		guessPose = pose * guessPose;
	    }

	}
	while((!correctPose) && (!videoBuffer->frame_pending()));

	if(correctPose)
	{
	    if(stage == 2)
	    {
		stage = 1;
		if(doParams)
                    titlePrefix = "Calibrate: Tracking Pose and Internal Params";
		else
                    titlePrefix = "Calibrate: Tracking Pose";
	    }

	    if (doParams)
		pose = find_pose_and_params(pose, grid3d, measurements, cameraModel, factor*factor);
	    else		  
		pose = find_pose(pose, grid3d, measurements, cameraModel, factor*factor);

	    double now = timer.get_time();
	    if (now - lastMeasure > interval)
	    {
		lastMeasure = now;
		MeasurementSet ms;
		for (size_t i=0; i<measurements.size(); i++)
		{
		    ms.im.push_back(measurements[i].second);
		    ms.world.push_back(grid3d[measurements[i].first]);
		}
		measurementSets.push_back(ms);
		poses.push_back(pose);
		char buf[50];
		sprintf(buf, " (%u views)", (unsigned int)measurementSets.size());
		disp.set_title(titlePrefix+buf);
	    }
	    prevFrameTracked = true;
	}
	else
            prevFrameTracked = false;

	//Draw the grid of estimated pose, and the points used to come up with it
	vector<Vector<2> > img_grid(grid.size());
	for (size_t i=0; i<grid.size(); i++)
	{
	    Vector<2> plane = project(pose*grid3d[i]);
	    img_grid[i] = cameraModel.project(plane) / factor;
	}
	if(correctPose)
	{
	    glColor3f(1,1,1);
	    drawGrid (img_grid, gridx, gridy);
	    //plot the points used to estimate this pose
	    glColor3f(0,1,0);
	    for (size_t i=0; i<measurements.size(); i++)
                drawCross(measurements[i].second/factor, 8);
	    glColor3f(1,0,0);
	    for (size_t i=0; i<failedPoints.size(); i++)
                drawCross(failedPoints[i], 8);
	}
	else
	{
	    glColor3f(1,0.2,0.2);
	    drawGrid (img_grid, gridx, gridy);
	}

	//Coloured box to signify stage - init, record or calc
	switch(stage)
	{
	case 0:
	    glColor4f(0,1,0,0.5);
	    break;
	case 1:
	    glColor4f(1,0,0,0.5);
	    break;
	case 2:
	    glColor4f(1,1,0,0.5);
	    break;
	}
	glBegin(GL_QUADS);
	glVertex2i(15, 450);
	glVertex2i(30, 450);
	glVertex2i(30, 465);
	glVertex2i(15, 465);
	glEnd();
	disp.swap_buffers();
    }

    size_t numMeasurements = 0;
    for (size_t i=0; i<measurementSets.size(); i++)
	numMeasurements += measurementSets[i].im.size();

    cout.precision(10);
    cerr.precision(10);
    size_t numPoints = 0;
    for (size_t i=0; i<measurementSets.size(); i++)
    {
	numPoints += measurementSets[i].im.size();
    }
    cerr << measurementSets.size() << " sets of measurements, " << numPoints << " total points" << endl;

    double minError = getReprojectionError(measurementSets, poses, cameraModel);
    cerr << sqrt(minError/numPoints) / factor << " initial reprojection error" << endl;

    cerr << "Optimizing with Levenberg-Marquardt..." << endl;
    double lambda = 1;
    while (lambda < 1e12)
    {
	Vector<NumCameraParams> params = cameraModel.get_parameters();
	vector<SE3<> > oldposes = poses;
	improveLM(measurementSets, poses, cameraModel, lambda);
	double error = getReprojectionError(measurementSets, poses, cameraModel);
	if (minError - error > 1e-19)
        {
	    minError = error;
	    lambda *= 0.5;
	    cerr << "rms reprojection error = " << sqrt(minError/numPoints) / factor << " pixels" << endl;
        }
	else
        {
	    poses = oldposes;
	    cameraModel.get_parameters() = params;
	    lambda *= 10;
        }
	//Vector<6> v = cameraModel.get_parameters();
	//v.slice<0,4>() /= factor;
	//cerr << v << endl;
    }

    vector<pair<Vector<2>, Vector<2> > > errors;
    pair<double,double> rperr = getReprojectionError(measurementSets, poses, cameraModel, errors);

    if(generateErrorImage){
	cerr << "Generating errors.pgm..." << endl;
	Image<float> errorImage(imageSize);
	errorImage.zero();
	for (size_t i=0; i<errors.size(); i++)
        {
	    ImageRef p = ir_rounded(errors[i].first/factor);
	    double mag = sqrt(errors[i].second * errors[i].second / rperr.second);
	    errorImage[p] = mag;
        }
	img_save(errorImage, "errors.pgm");
    }

    cerr << "Estimating uncertainty..." << endl;
    Matrix<NumCameraParams> Cov;
    getUncertainty(measurementSets, poses, cameraModel, Cov);
    static const int NumRadialParams = NumCameraParams - 4;
    Cov.slice<4,4,NumRadialParams,NumRadialParams>() *= factor*factor;
    Cov.slice<0,4,4,NumRadialParams>() *= factor;
    Cov.slice<4,0,NumRadialParams,4>() *= factor;

    Vector<NumCameraParams> uncertainty;
    for (int i=0; i<NumCameraParams; i++)
	uncertainty[i]= 3*sqrt(Cov[i][i]);

    cameraModel.get_parameters().slice<0,4>() /= factor;

    cout.precision(14);
    cout << sqrt(rperr.first/numPoints) / factor << " average reprojection error" << endl;
    cout << "Three sigma uncertainties: " << endl;
    cout << uncertainty << endl << endl;
    cout << "Parameters:" << endl;
    cout << cameraModel.get_parameters() << endl << endl;
    cout << "Covariance:" << endl;
    cout << endl << Cov << endl << endl;

    cout << "Camera Model:" << endl;
    cameraModel.save(cout);
    cout << endl << endl;

    if (output.length())
    {
	ofstream fout(output.c_str());
	fout.precision(19);
	cameraModel.save (fout);
    }

    return 0;
}
