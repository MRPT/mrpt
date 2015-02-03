/**
 * @file mexopencv.hpp
 * @brief Global constant definitions
 * @author Kota Yamaguchi
 * @date 2012
 *
 * The header file for a Matlab mex function that uses OpenCV library.
 * The file includes definition of MxArray class that converts between mxArray
 * and a couple of std:: and cv:: data types including cv::Mat.
 */
#ifndef __MEXOPENCV_HPP__
#define __MEXOPENCV_HPP__

#include <functional>
#include <map>
#include <stdint.h>
#include <string>
#include "mex.h"
#include "opencv2/opencv.hpp"
#include "opencv/cv.hpp"

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

// Global constants

/** BorderType map for option processing
 */
const ConstMap<std::string,int> BorderType = ConstMap<std::string,int>
    ("Replicate",   cv::BORDER_REPLICATE)
    ("Constant",    cv::BORDER_CONSTANT)
    ("Reflect",     cv::BORDER_REFLECT)
    ("Wrap",        cv::BORDER_WRAP)
    ("Reflect101",  cv::BORDER_REFLECT_101)
    ("Transparent", cv::BORDER_TRANSPARENT)
    ("Default",     cv::BORDER_DEFAULT)
    ("Isolated",    cv::BORDER_ISOLATED);

/** Interpolation type map for option processing
 */
const ConstMap<std::string,int> InterType = ConstMap<std::string,int>
    ("Nearest",  cv::INTER_NEAREST)  //!< nearest neighbor interpolation
    ("Linear",   cv::INTER_LINEAR)   //!< bilinear interpolation
    ("Cubic",    cv::INTER_CUBIC)    //!< bicubic interpolation
    ("Area",     cv::INTER_AREA)     //!< area-based (or super) interpolation
    ("Lanczos4", cv::INTER_LANCZOS4) //!< Lanczos interpolation over 8x8 neighborhood
    ("Max",      cv::INTER_MAX);
    //("WarpInverseMap",    cv::WARP_INVERSE_MAP);

/** Thresholding type map for option processing
 */
const ConstMap<std::string,int> ThreshType = ConstMap<std::string,int>
    ("Binary",    cv::THRESH_BINARY)
    ("BinaryInv", cv::THRESH_BINARY_INV)
    ("Trunc",     cv::THRESH_TRUNC)
    ("ToZero",    cv::THRESH_TOZERO)
    ("ToZeroInv", cv::THRESH_TOZERO_INV)
    ("Mask",      cv::THRESH_MASK);
    //("Otsu",    cv::THRESH_OTSU);

/** Distance types for Distance Transform and M-estimators
 */
const ConstMap<std::string,int> DistType = ConstMap<std::string,int>
    ("User",   CV_DIST_USER)
    ("L1",     CV_DIST_L1)
    ("L2",     CV_DIST_L2)
    ("C",      CV_DIST_C)
    ("L12",    CV_DIST_L12)
    ("Fair",   CV_DIST_FAIR)
    ("Welsch", CV_DIST_WELSCH)
    ("Huber",  CV_DIST_HUBER);

/** Line type for drawing
 */
const ConstMap<std::string,int> LineType = ConstMap<std::string,int>
    ("8",  8)
    ("4",  4)
    ("AA", CV_AA);

/** Font faces for drawing
 */
const ConstMap<std::string,int> FontFace = ConstMap<std::string,int>
    ("HersheySimplex",       cv::FONT_HERSHEY_SIMPLEX)
    ("HersheyPlain",         cv::FONT_HERSHEY_PLAIN)
    ("HersheyDuplex",        cv::FONT_HERSHEY_DUPLEX)
    ("HersheyComplex",       cv::FONT_HERSHEY_COMPLEX)
    ("HersheyTriplex",       cv::FONT_HERSHEY_TRIPLEX)
    ("HersheyComplexSmall",  cv::FONT_HERSHEY_COMPLEX_SMALL)
    ("HersheyScriptSimplex", cv::FONT_HERSHEY_SCRIPT_SIMPLEX)
    ("HersheyScriptComplex", cv::FONT_HERSHEY_SCRIPT_COMPLEX);

/** Font styles for drawing
 */
const ConstMap<std::string,int> FontStyle = ConstMap<std::string,int>
    ("Regular", 0)
    ("Italic",  cv::FONT_ITALIC);

// Code from MxArray.cpp

///** Field names for cv::Moments.
// */
//const char *cv_moments_fields[10] = {"m00", "m10", "m01", "m20", "m11", "m02",
//									 "m30", "m21", "m12", "m03"};
///** Field names for cv::RotatedRect.
// */
//const char *cv_rotated_rect_fields[3] = {"center", "size", "angle"};
///** Field names for cv::TermCriteria.
// */
//const char *cv_term_criteria_fields[3] = {"type", "maxCount", "epsilon"};
///** Field names for cv::Keypoint.
// */
//const char *cv_keypoint_fields[6] = {"pt", "size", "angle", "response",
//									 "octave", "class_id"};
///** Field names for cv::DMatch.
// */
//const char *cv_dmatch_fields[4] = {"queryIdx", "trainIdx", "imgIdx",
//								   "distance"};

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

#endif
