/** MxArray data conversion library.
 *
 * The library provides mexplus::MxArray class for data conversion between
 * mxArray* and C++ types. The static API's are the core of the high-level
 * conversions.
 *
 *    int value = MxArray::to<int>(prhs[0]);
 *    string value = MxArray::to<string>(prhs[0]);
 *    vector<double> value = MxArray::to<vector<double> >(prhs[0]);
 *
 *    plhs[0] = MxArray::from(20);
 *    plhs[0] = MxArray::from("text value.");
 *    plhs[0] = MxArray::from(vector<double>(20, 0));
 *
 * Additionally, object API's are there to wrap around a complicated data
 * access.
 *
 * ### Read access
 *
 *    MxArray cell(prhs[0]);   // Assumes cell array in prhs[0].
 *    int x = cell.at<int>(0);
 *    vector<double> y = cell.at<vector<double> >(1);
 *
 *    MxArray numeric(prhs[0]);   // Assumes numeric array in prhs[0].
 *    double x = numeric.at<double>(0);
 *    int y = numeric.at<int>(1);
 *
 * ### Write access
 *
 *    MxArray cell(MxArray::Cell(1, 3));
 *    cell.set(0, 12);
 *    cell.set(1, "text value.");
 *    cell.set(2, vector<double>(4, 0));
 *    plhs[0] = cell.release();
 *
 *    MxArray numeric(MxArray::Numeric<double>(2, 2));
 *    numeric.set(0, 0, 1);
 *    numeric.set(0, 1, 2);
 *    numeric.set(1, 0, 3);
 *    numeric.set(1, 1, 4);
 *    plhs[0] = numeric.release();
 *
 * To add your own data conversion, define in namespace mexplus a template
 * specialization of MxArray::from() and MxArray::to().
 *
 * Kota Yamaguchi 2014 <kyamagu@cs.stonybrook.edu>
 */

#ifndef __MEXPLUS_MXARRAY_EXTRA_H__
#define __MEXPLUS_MXARRAY_EXTRA_H__

#include <mex.h>
//#include <mexplus/mxtypes.h>
#include "mxtypes.h"
#include <stdint.h>
#include <string>
#include <typeinfo>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/core/types_c.h>
#include <opencv2/core/core_c.h>

namespace mexplus {
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

/*
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
        //channels[i].convertTo(m, type); // Write to mxArray through m.
        // Swap R and B channels
        MRPT_TODO("Do in other place where it is more clear")
        channels[nchannels-1-i].convertTo(m, type); // Write to mxArray through m.
    }
    return p_;
}
*/

} // namespace mexplus

#endif // __MEXPLUS_MXARRAY_EXTRA_H__
