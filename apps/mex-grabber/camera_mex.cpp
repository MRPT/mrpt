/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

/*-----------------------------------------------------------------------------
	APPLICATION: mex-grabber
	FILE: mexgrabber_main.cpp
    AUTHORS: Jesus Briales Garcia <jesusbriales@gmail.com>
             Jose Luis Blanco Claraco <joseluisblancoc@gmail.com>


	For instructions and details, see:
	 http://
  -----------------------------------------------------------------------------*/

#include <mrpt/hwdrivers/CGenericSensor.h>
#include <mrpt/hwdrivers/CHokuyoURG.h>
#include <mrpt/hwdrivers/CImageGrabber_dc1394.h>
#include <mrpt/utils/CConfigFile.h>
#include <mrpt/utils/CFileGZOutputStream.h>
#include <mrpt/utils/CImage.h>
#include <mrpt/utils/round.h>
#include <mrpt/slam/CActionCollection.h>
#include <mrpt/slam/CSensoryFrame.h>
#include <mrpt/slam/CObservationOdometry.h>
#include <mrpt/slam/CObservationGPS.h>
#include <mrpt/slam/CObservationIMU.h>
#include <mrpt/slam/CActionRobotMovement2D.h>
#include <mrpt/system/os.h>
#include <mrpt/system/filesystem.h>

// MRPT headers for conversion tools
#include <opencv2/core.hpp>
#include <opencv2/core/types_c.h>
#include <opencv2/core/core_c.h>
#include <mrpt/slam/CSimplePointsMap.h>

// Matlab MEX interface headers
#include <mexplus.h>

// Matlab-OpenCV interface from mexopencv: headers
//#include <MxArray.hpp>

using namespace mrpt;
using namespace mrpt::system;
using namespace mrpt::hwdrivers;
using namespace mrpt::utils;
using namespace mrpt::slam;
using namespace std;
using namespace mexplus;

// Add cv::Mat method
namespace mexplus {
/** mxArray* importer methods.
 */
//template <typename T>
//static mxArray* from(const T& value) { return fromInternal<T>(value); }
//static mxArray* from(const char* value) {
//  mxArray* array = mxCreateString(value);
//  MEXPLUS_CHECK_NOTNULL(array);
//  return array;
//}
//static mxArray* from(int32_t value) {
//  mxArray* array = mxCreateNumericMatrix(1, 1, mxINT32_CLASS, mxREAL);
//  MEXPLUS_CHECK_NOTNULL(array);
//  *reinterpret_cast<int32_t*>(mxGetData(array)) = value;
//  return array;
//}

/** std::map wrapper with one-line initialization and lookup method.
 * Initialization
 * @code
 * const ConstMap<std::string,int> BorderType = ConstMap<std::string,int>
 *     ("Replicate",  cv::BORDER_REPLICATE)
 *     ("Constant",   cv::BORDER_CONSTANT)
 *     ("Reflect",    cv::BORDER_REFLECT);
 * @endcode
 * Lookup
 * @code
 * BorderType["Constant"] // => cv::BORDER_CONSTANT
 * @endcode
 */
template <typename T, typename U>
class ConstMap
{
  public:
    /// Constructor with a single key-value pair
    ConstMap(const T& key, const U& val)
    {
        m_[key] = val;
    }
    /// Consecutive insertion operator
    ConstMap<T, U>& operator()(const T& key, const U& val)
    {
        m_[key] = val;
        return *this;
    }
    /// Implicit converter to std::map
    operator std::map<T, U>() { return m_; }
    /// Lookup operator; fail if not found
    U operator [](const T& key) const
    {
        typename std::map<T,U>::const_iterator it = m_.find(key);
        if (it==m_.end())
            mexErrMsgIdAndTxt("mexopencv:error", "Value not found");
        return (*it).second;
    }
  private:
    std::map<T, U> m_;
};

/** Translates data type definition used in Matlab to that of OpenCV.
 * @param depth data depth of opencv's Mat class. e.g., CV_32F.
 * @return data type of matlab's mxArray. e.g., mxDOUBLE_CLASS.
 */
const ConstMap<int,mxClassID> ClassIDOf = ConstMap<int,mxClassID>
    (CV_64F,    mxDOUBLE_CLASS)
    (CV_32F,    mxSINGLE_CLASS)
    (CV_8S,     mxINT8_CLASS)
    (CV_8U,     mxUINT8_CLASS)
    (CV_16S,    mxINT16_CLASS)
    (CV_16U,    mxUINT16_CLASS)
    (CV_32S,    mxINT32_CLASS);

/** Translates data type definition used in OpenCV to that of Matlab.
 * @param classid data type of matlab's mxArray. e.g., mxDOUBLE_CLASS.
 * @return opencv's data type. e.g., CV_8U.
 */
const ConstMap<mxClassID, int> DepthOf = ConstMap<mxClassID, int>
    (mxDOUBLE_CLASS,   CV_64F)
    (mxSINGLE_CLASS,   CV_32F)
    (mxINT8_CLASS,     CV_8S)
    (mxUINT8_CLASS,    CV_8U)
    (mxINT16_CLASS,    CV_16S)
    (mxUINT16_CLASS,   CV_16U)
    (mxINT32_CLASS,    CV_32S)
    (mxUINT32_CLASS,   CV_32S)
    (mxLOGICAL_CLASS,  CV_8U);

template <>
mxArray* MxArray::from(const cv::Mat& mat)
{
    mxArray* p_; // Create pointer
    if (mat.empty())
    {
        p_ = mxCreateNumericArray(0, 0, mxDOUBLE_CLASS, mxREAL);
        if (!p_)
            mexErrMsgIdAndTxt("mexopencv:error", "Allocation error");
        return p_;
    }
    // Optional arguments:
    mxClassID classid = mxUNKNOWN_CLASS;
    bool transpose = true;

    cv::Mat input = (mat.dims == 2 && transpose) ? mat.t() : mat;
    // Create a new mxArray.
    const int nchannels = input.channels();
    const int* dims_ = input.size;
    std::vector<mwSize> d(dims_, dims_ + input.dims);
    d.push_back(nchannels);
    classid = (classid == mxUNKNOWN_CLASS)
        ? ClassIDOf[input.depth()] : classid;
    std::swap(d[0], d[1]);
    if (classid == mxLOGICAL_CLASS)
    {
        // OpenCV's logical true is any nonzero while matlab's true is 1.
        cv::compare(input, 0, input, cv::CMP_NE);
        input.setTo(1, input);
        p_ = mxCreateLogicalArray(d.size(), &d[0]);
    }
    else {
        p_ = mxCreateNumericArray(d.size(), &d[0], classid, mxREAL);
    }
    if (!p_)
        mexErrMsgIdAndTxt("mexopencv:error", "Allocation error");
    // Copy each channel.
    std::vector<cv::Mat> channels;
    split(input, channels);
    std::vector<mwSize> si(d.size(), 0); // subscript index.
    int type = CV_MAKETYPE(DepthOf[classid], 1); // destination type.
    for (int i = 0; i < nchannels; ++i)
    {
        si[si.size() - 1] = i; // last dim is a channel index.

        mwIndex subs_si = mxCalcSingleSubscript(p_, si.size(), &si[0]);
//        void *ptr = reinterpret_cast<void*>(
//                reinterpret_cast<size_t>(mxGetData(p_)) +
//                mxGetElementSize(p_) * subs(si));
        void *ptr = reinterpret_cast<void*>(
                reinterpret_cast<size_t>(mxGetData(p_)) +
                mxGetElementSize(p_) * subs_si);
        cv::Mat m(input.dims, dims_, type, ptr);
        channels[i].convertTo(m, type); // Write to mxArray through m.
    }
    return p_;
}

} // namespace mexplus

// Redefining cout
class mstream : public std::streambuf {
public:
protected:
  virtual std::streamsize xsputn(const char *s, std::streamsize n);
  virtual int overflow(int c = EOF);
};

std::streamsize
mstream::xsputn(const char *s, std::streamsize n)
{
  mexPrintf("%.*s",n,s);
  return n;
}

int
mstream::overflow(int c)
{
    if (c != EOF) {
      mexPrintf("%.1s",&c);
    }
    return 1;
}

//#define CLASS CHokuyoURG
#define CLASS CGenericSensor

template class mexplus::Session<CLASS>;

namespace {
// Defines MEX API for new.
MEX_DEFINE(new) (int nlhs, mxArray* plhs[],
                 int nrhs, const mxArray* prhs[]) {
    // For cout in Matlab
    mstream mout;
    std::streambuf *outbuf = std::cout.rdbuf(&mout);
    //std::cout << "This is a Matlab test for cout" << std::endl;

    InputArguments input(nrhs, prhs, 1);
    OutputArguments output(nlhs, plhs, 1);

    const std::string GLOBAL_SECTION_NAME = "global";

    CGenericSensor::TListObservations		global_list_obs;
    synch::CCriticalSection					cs_global_list_obs;

    bool									allThreadsMustExit = false;

    string 		rawlog_ext_imgs_dir;		// Directory where to save externally stored images, only for CCameraSensor's.

    printf(" rawlog-grabber - Part of the MRPT\n");
    printf(" MRPT C++ Library: %s - BUILD DATE %s\n", MRPT_getVersion().c_str(), MRPT_getCompilationDate().c_str());
    printf("-------------------------------------------------------------------\n");

    string INI_FILENAME( input.get<string>(0) );
    ASSERT_FILE_EXISTS_(INI_FILENAME)
    CConfigFile	iniFile( INI_FILENAME );
    printf("Using ini file %s\n", INI_FILENAME.c_str());

    // ------------------------------------------
    //			Load config from file:
    // ------------------------------------------
    string			rawlog_prefix = "dataset";
    int				time_between_launches = 300;
    double			SF_max_time_span = 0.25;			// Seconds
    bool			use_sensoryframes = false;
    int				GRABBER_PERIOD_MS = 1000;

    MRPT_LOAD_CONFIG_VAR( rawlog_prefix, string, iniFile, GLOBAL_SECTION_NAME );
    MRPT_LOAD_CONFIG_VAR( time_between_launches, int, iniFile, GLOBAL_SECTION_NAME );
    MRPT_LOAD_CONFIG_VAR( SF_max_time_span, float,		iniFile, GLOBAL_SECTION_NAME );
    MRPT_LOAD_CONFIG_VAR( use_sensoryframes, bool,		iniFile, GLOBAL_SECTION_NAME );
    MRPT_LOAD_CONFIG_VAR( GRABBER_PERIOD_MS, int, iniFile, GLOBAL_SECTION_NAME );

    // Build full rawlog file name:
    string	rawlog_postfix = "_";

    //rawlog_postfix += dateTimeToString( now() );
    mrpt::system::TTimeParts parts;
    mrpt::system::timestampToParts(now(), parts, true);
    rawlog_postfix += format("%04u-%02u-%02u_%02uh%02um%02us",
                             (unsigned int)parts.year,
                             (unsigned int)parts.month,
                             (unsigned int)parts.day,
                             (unsigned int)parts.hour,
                             (unsigned int)parts.minute,
                             (unsigned int)parts.second );

    rawlog_postfix = mrpt::system::fileNameStripInvalidChars( rawlog_postfix );

    // Only set this if we want externally stored images:
    rawlog_ext_imgs_dir = rawlog_prefix+fileNameStripInvalidChars( rawlog_postfix+string("_Images") );

    // Also, set the path in CImage to enable online visualization in a GUI window:
    CImage::IMAGES_PATH_BASE = rawlog_ext_imgs_dir;

    cout << endl ;
    cout << "External image storage: " << rawlog_ext_imgs_dir << endl << endl;

    vector_string	sections;
    iniFile.getAllSections( sections );

    // TODO: Extract sections here
    //string driver_name = params.cfgFile->read_string(params.sensor_label,"driver","",true);

    // Create mexplus handler
    mexPrintf("Before sensor\n");
    //CGenericSensorPtr sensor = CGenericSensor::createSensorPtr("CHokuyoURG");

    string sensor_label;
    for (vector_string::iterator it=sections.begin();it!=sections.end();++it)
    {
        if (*it==GLOBAL_SECTION_NAME || it->empty() || iniFile.read_bool(*it,"rawlog-grabber-ignore",false,false) )
            continue;	// This is not a sensor:

        sensor_label = *it;
        break; // Exit when valid iterator is reached
    }
    mexPrintf("Launching thread for sensor '%s'\n", sensor_label.c_str());

    string driver_name = iniFile.read_string(sensor_label,"driver","",true);
    mexPrintf("Driver name: %s\n\n", driver_name.c_str());

    CLASS* sensor;
    sensor = CGenericSensor::createSensor( driver_name );

    sensor->loadConfig( iniFile, sensor_label );
    //cout << format("[thread_%s] Starting...",sensor_label.c_str()) << " at " << sensor->getProcessRate() <<  " Hz" << endl;

    sensor->initialize();
    mexPrintf("After sensor initialization\n");

    //delete var;
    output.set(0, Session<CLASS>::create( sensor ));

    mexPrintf("Done new, assigning to output\n");

    std::cout.rdbuf(outbuf); // For cout in Matlab
}

//// Defines MEX API for query (const method).
//MEX_DEFINE(read) (int nlhs, mxArray* plhs[],
//                  int nrhs, const mxArray* prhs[]) {
//  InputArguments input(nrhs, prhs, 1);
//  OutputArguments output(nlhs, plhs, 0);

//  const CLASS& sensor = Session<CLASS>::getConst(input.get(0));

//  mrpt::slam::CObservationImage obs;
//  sensor.getObservation( obs );
//  obs.image.saveToFile( "/home/jesus/image.jpg" );

//  //output.set(0, database.query(input.get<string>(1)));
//}

// Defines MEX API for set (non const method).
MEX_DEFINE(read) (int nlhs, mxArray* plhs[],
                  int nrhs, const mxArray* prhs[]) {
  InputArguments input(nrhs, prhs, 1);
  OutputArguments output(nlhs, plhs, 3);
  CLASS* sensor = Session<CLASS>::get(input.get(0));

  TTimeStamp t0= now();

  // Process
  sensor->doProcess();

  // Get new observations
  CGenericSensor::TListObservations lstObjs;
  sensor->getObservations( lstObjs );
  cout << "lstObjs container size: " << lstObjs.size() << endl;

  // Create an object the same type as interest object in the multimap (TListObservations)
  // TListObservations is a multimap with elements <TTimeStamp,CSerializablePtr>
  CSerializablePtr obj;
  // Gets a TListObservations::iterator pointing to the wanted pair
  CGenericSensor::TListObservations::iterator it=lstObjs.begin();
  // Interest object can be extracted from iterator pointed object, as second element of the pair
  obj = (*it).second;

  // Check if got object is of the searched class
  if( IS_CLASS(obj, CObservation2DRangeScan) )
  {
      // Casting by constructing new SmartPtr from existing data pointer
      CObservation2DRangeScanPtr LRF_obs = CObservation2DRangeScanPtr(obj);

      //mrpt::slam::CObservation2DRangeScan	outObservation;
      CSimplePointsMap map;
      //map.insertObservation( &outObservation );
      map.insertObservation( LRF_obs.pointer() );
      vector<float> xpts;
      vector<float> ypts;
      vector<float> zpts;
      map.getAllPoints(xpts,ypts,zpts);

      output.set(0, xpts);
      output.set(1, ypts);
      output.set(2, zpts);
  }
  else if( IS_CLASS(obj, CObservationImage) )
  {
      //mexPrintf("Reading image\n");

      CObservationImagePtr cam_obs = CObservationImagePtr(obj);

      // Both core/core_c.h and core/types_c.h have been included
      // to use cvarrToMat and IplImage, respectively
      cv::Mat cvImg = cv::cvarrToMat( cam_obs->image.getAs<IplImage>() );

//      cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );// Create a window for display.
//      cv::imshow( "Display window", cvImg );                   // Show our image inside it.
//      cv::waitKey(0);                                          // Wait for a keystroke in the window

      //CMatrixFloat mat;
      //cam_obs->image.grayscale().getAsMatrix( mat );

      //cam_obs->image.saveToFile("/home/jesus/output_image.jpg");

      /*
      mexPrintf( "Rows %d and columns %d\n", mat.rows(), mat.cols() );
      int num_of_pixels = mat.rows() * mat.cols();
      //vector<float> pixels = vector<float>( mat.data(), mat.data()+num_of_pixels );

      output.set(0, vector<float>( mat.data(), mat.data()+num_of_pixels ) );
      */

      output.set(0, cvImg);
      output.set(1, 1);
      output.set(2, 2);

      //plhs[0] = mexopencv::MxArray(cvImg);

      // Delete useless objects
  }
  else
  {
      mexPrintf("Bad class\n");

      output.set(0, 0);
      output.set(1, 1);
      output.set(2, 2);
  }
}

/*
// Defines MEX API for set (non const method).
MEX_DEFINE(read) (int nlhs, mxArray* plhs[],
                  int nrhs, const mxArray* prhs[]) {
  InputArguments input(nrhs, prhs, 1);
  OutputArguments output(nlhs, plhs, 3);
  CLASS* sensor = Session<CLASS>::get(input.get(0));

  bool outThereIsObservation;
  mrpt::slam::CObservation2DRangeScan	outObservation;
  bool hardwareError;

  //sensor->purgeBuffers();
  mrpt::system::sleep(100);
  sensor->doProcessSimple(outThereIsObservation,
                          outObservation,
                          hardwareError);
  //output.set(0, outObservation.scan);

  CSimplePointsMap map;
  map.insertObservation( &outObservation );
  vector<float> xpts;
  vector<float> ypts;
  vector<float> zpts;
  map.getAllPoints(xpts,ypts,zpts);
  printf("Map points recovered\n");

  output.set(0, xpts);
  output.set(1, ypts);
  output.set(2, zpts);

  //database->put(input.get<string>(1), input.get<string>(2));
}
*/

// Defines MEX API for delete.
MEX_DEFINE(delete) (int nlhs, mxArray* plhs[],
                    int nrhs, const mxArray* prhs[]) {
    InputArguments input(nrhs, prhs, 1);
    OutputArguments output(nlhs, plhs, 0);

    // Delete pointed object prior to removing pointer
    MRPT_TODO("Delete object prior to destroy handler")
    Session<CLASS>::destroy(input.get(0));
    //Session<CGenericSensor>::destroy(input.get(0));
    //Session<CHokuyoURG>::destroy(input.get(0));
    //Session<CImage>::destroy(input.get(0));
}

}

MEX_DISPATCH // Don't forget to add this if MEX_DEFINE() is used.

int main( int argc, const char* argv[] )
{
    printf(" mex-grabber - Part of the MRPT\n");
    printf(" MRPT C++ Library: %s - BUILD DATE %s\n", MRPT_getVersion().c_str(), MRPT_getCompilationDate().c_str());
    printf("-------------------------------------------------------------------\n");
    printf(" This is a test for Matlab MEX functionalities\n");
    printf("-------------------------------------------------------------------\n");

    // Create driver object
    const mxArray* pnew[2];
    pnew[0] = MxArray::from("new");
    //pnew[0] = mxCreateString("new");
    pnew[1] = MxArray::from( argv[1] ); // Read config file path, first argv is function name
    //pnew[1] = mxCreateString( argv[1] );
    mxArray* id_[1]; // id for object handling
    mexFunction( 1, id_, 2, pnew );

    // Read image from camera
    const mxArray* pread[2];
    pread[0] = MxArray::from("read");
    //pread[0] = mxCreateString("read");
    pread[1] = id_[0];
    mxArray* im[3];
    for(int i=0; i<100; ++i)
    {
        mexFunction( 3, im, 2, pread);
        printf("Iteration %d done\n",i);
    }

    return 0;
}
