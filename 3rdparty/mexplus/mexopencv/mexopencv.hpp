/**
 * @file mexopencv.hpp
 * @brief Global constant definitions
 * @author Kota Yamaguchi
 * @date 2012
 *
 * Header file for MATLAB MEX-functions that use OpenCV library.
 * The file includes definition of MxArray class that converts between mxArray
 * and a couple of std:: and cv:: data types including cv::Mat.
 */
#ifndef MEXOPENCV_HPP
#define MEXOPENCV_HPP

#include "MxArray.hpp"

/**************************************************************\
*                       Global constants                       *
\**************************************************************/

/** Translates class name used in MATLAB to equivalent OpenCV depth.
 * @param classname numeric MATLAB data type, e.g. \c uint8.
 * @return equivalent OpenCV data type, e.g. \c CV_8U.
 *
 * Note: 64-bit integer types are not supported by OpenCV.
 *       Also, OpenCV only has signed 32-bit integer type.
 */
const ConstMap<std::string,int> ClassNameMap = ConstMap<std::string,int>
    ("uint8",   CV_8U)
    ("int8",    CV_8S)
    ("uint16",  CV_16U)
    ("int16",   CV_16S)
  //("uint32",  CV_32S)
    ("int32",   CV_32S)
    ("single",  CV_32F)
    ("double",  CV_64F)
    ("logical", CV_8U);

/** Translates data type definition used in OpenCV to that of MATLAB.
 * @param depth OpenCV cv::Mat data depth, e.g. \c CV_8U.
 * @return equivalent MATLAB class name, e.g. \c uint8.
 */
const ConstMap<int,std::string> ClassNameInvMap = ConstMap<int,std::string>
    (CV_8U,  "uint8")
    (CV_8S,  "int8")
    (CV_16U, "uint16")
    (CV_16S, "int16")
    (CV_32S, "int32")
    (CV_32F, "single")
    (CV_64F, "double");

/** Translates MATLAB color names (see \c ColorSpec) into OpenCV scalars.
 * @param name short name of MATLAB colors. One of eight predefined colors.
 * @return BGR triplet as an OpenCV Scalar.
 */
const ConstMap<std::string,cv::Scalar> ColorType = ConstMap<std::string,cv::Scalar>
    ("r", cv::Scalar(  0,  0,255))
    ("g", cv::Scalar(  0,255,  0))
    ("b", cv::Scalar(255,  0,  0))
    ("c", cv::Scalar(255,255,  0))
    ("m", cv::Scalar(255,  0,255))
    ("y", cv::Scalar(  0,255,255))
    ("k", cv::Scalar(  0,  0,  0))
    ("w", cv::Scalar(255,255,255));

/// Border type map for option processing
const ConstMap<std::string,int> BorderType = ConstMap<std::string,int>
    ("Constant",    cv::BORDER_CONSTANT)    // iiiiii|abcdefgh|iiiiiii for some i
    ("Replicate",   cv::BORDER_REPLICATE)   // aaaaaa|abcdefgh|hhhhhhh
    ("Reflect",     cv::BORDER_REFLECT)     // fedcba|abcdefgh|hgfedcb
    ("Reflect101",  cv::BORDER_REFLECT_101) // gfedcb|abcdefgh|gfedcba
    ("Wrap",        cv::BORDER_WRAP)        // cdefgh|abcdefgh|abcdefg
    ("Transparent", cv::BORDER_TRANSPARENT) // uvwxyz|abcdefgh|ijklmno
    ("Default",     cv::BORDER_DEFAULT);    // same as "Reflect101"

/// Inverse border type map for option processing
const ConstMap<int,std::string> BorderTypeInv = ConstMap<int,std::string>
    (cv::BORDER_CONSTANT,    "Constant")
    (cv::BORDER_REPLICATE,   "Replicate")
    (cv::BORDER_REFLECT,     "Reflect")
    (cv::BORDER_REFLECT_101, "Reflect101")
    (cv::BORDER_WRAP,        "Wrap")
    (cv::BORDER_TRANSPARENT, "Transparent");

/// Interpolation type map for option processing
const ConstMap<std::string,int> InterpType = ConstMap<std::string,int>
    ("Nearest",     cv::INTER_NEAREST)       // nearest neighbor interpolation
    ("Linear",      cv::INTER_LINEAR)        // bilinear interpolation
    ("Cubic",       cv::INTER_CUBIC)         // bicubic interpolation
    ("Area",        cv::INTER_AREA)          // area-based (or super) interpolation
    ("Lanczos4",    cv::INTER_LANCZOS4)      // Lanczos interpolation over 8x8 neighborhood
    ("LinearExact", cv::INTER_LINEAR_EXACT); // Bit exact bilinear interpolation

/// Thresholding type map for option processing
const ConstMap<std::string,int> ThreshType = ConstMap<std::string,int>
    ("Binary",    cv::THRESH_BINARY)      // val = (val > thresh) ? maxVal : 0
    ("BinaryInv", cv::THRESH_BINARY_INV)  // val = (val > thresh) ? 0 : maxVal
    ("Trunc",     cv::THRESH_TRUNC)       // val = (val > thresh) ? thresh : val
    ("ToZero",    cv::THRESH_TOZERO)      // val = (val > thresh) ? val : 0
    ("ToZeroInv", cv::THRESH_TOZERO_INV); // val = (val > thresh) ? 0 : val

/// Distance types for Distance Transform and M-estimators
const ConstMap<std::string,int> DistType = ConstMap<std::string,int>
    ("User",   cv::DIST_USER)   // user-defined distance
    ("L1",     cv::DIST_L1)     // distance = |x1-x2| + |y1-y2|
    ("L2",     cv::DIST_L2)     // the simple euclidean distance
    ("C",      cv::DIST_C)      // distance = max(|x1-x2|,|y1-y2|)
    ("L12",    cv::DIST_L12)    // distance = 2*(sqrt(1+x*x/2) - 1)
    ("Fair",   cv::DIST_FAIR)   // distance = c^2*(|x|/c-log(1+|x|/c)), c = 1.3998
    ("Welsch", cv::DIST_WELSCH) // distance = c^2/2*(1-exp(-(x/c)^2)), c = 2.9846
    ("Huber",  cv::DIST_HUBER); // distance = |x|<c ? x^2/2 : c(|x|-c/2), c=1.345

/// Inverse Distance types for Distance Transform and M-estimators
const ConstMap<int,std::string> DistTypeInv = ConstMap<int,std::string>
    (cv::DIST_USER,   "User")
    (cv::DIST_L1,     "L1")
    (cv::DIST_L2,     "L2")
    (cv::DIST_C,      "C")
    (cv::DIST_L12,    "L12")
    (cv::DIST_FAIR,   "Fair")
    (cv::DIST_WELSCH, "Welsch")
    (cv::DIST_HUBER,  "Huber");

/// Line type for drawing
const ConstMap<std::string,int> LineType = ConstMap<std::string,int>
    ("4",  cv::LINE_4)
    ("8",  cv::LINE_8)
    ("AA", cv::LINE_AA);

/// Thickness type for drawing
const ConstMap<std::string,int> ThicknessType = ConstMap<std::string,int>
    ("Filled", cv::FILLED);

/// Font faces for drawing
const ConstMap<std::string,int> FontFace = ConstMap<std::string,int>
    ("HersheySimplex",       cv::FONT_HERSHEY_SIMPLEX)
    ("HersheyPlain",         cv::FONT_HERSHEY_PLAIN)
    ("HersheyDuplex",        cv::FONT_HERSHEY_DUPLEX)
    ("HersheyComplex",       cv::FONT_HERSHEY_COMPLEX)
    ("HersheyTriplex",       cv::FONT_HERSHEY_TRIPLEX)
    ("HersheyComplexSmall",  cv::FONT_HERSHEY_COMPLEX_SMALL)
    ("HersheyScriptSimplex", cv::FONT_HERSHEY_SCRIPT_SIMPLEX)
    ("HersheyScriptComplex", cv::FONT_HERSHEY_SCRIPT_COMPLEX);

/// Font styles for drawing
const ConstMap<std::string,int> FontStyle = ConstMap<std::string,int>
    ("Regular", 0)
    ("Italic",  cv::FONT_ITALIC);

/// Norm type map for option processing
const ConstMap<std::string,int> NormType = ConstMap<std::string,int>
    ("Inf",      cv::NORM_INF)
    ("L1",       cv::NORM_L1)
    ("L2",       cv::NORM_L2)
    ("L2Sqr",    cv::NORM_L2SQR)
    ("Hamming",  cv::NORM_HAMMING)
    ("Hamming2", cv::NORM_HAMMING2)
    ("MinMax",   cv::NORM_MINMAX);

/// Inverse norm type map for option processing
const ConstMap<int,std::string> NormTypeInv = ConstMap<int,std::string>
    (cv::NORM_INF,      "Inf")
    (cv::NORM_L1,       "L1")
    (cv::NORM_L2,       "L2")
    (cv::NORM_L2SQR,    "L2Sqr")
    (cv::NORM_HAMMING,  "Hamming")
    (cv::NORM_HAMMING2, "Hamming2")
    (cv::NORM_MINMAX,   "MinMax");

/**************************************************************\
*                  Helper Macros & Functions                   *
\**************************************************************/

/// set or clear a bit in flag depending on bool value
#define UPDATE_FLAG(NUM, TF, BIT)       \
    do {                                \
        if ((TF)) { (NUM) |=  (BIT); }  \
        else      { (NUM) &= ~(BIT); }  \
    } while (0)

/// Alias for input/output arguments number check
inline void nargchk(bool cond)
{
    if (!cond) {
        mexErrMsgIdAndTxt("mexopencv:error", "Wrong number of arguments");
    }
}

/**************************************************************\
*           Conversion Functions: MxArray to vector            *
\**************************************************************/

/** Convert an MxArray to std::vector<cv::Point_<T>>
 *
 * @param arr MxArray object. In one of the following forms:
 * - a cell-array of 2D points (2-element vectors) of length \c N,
 *   e.g: <tt>{[x,y], [x,y], ...}</tt>
 * - a numeric matrix of size \c Nx2, \c Nx1x2, or \c 1xNx2 in the form:
 *   <tt>[x,y; x,y; ...]</tt> or <tt>cat(3, [x,y], [x,y], ...)</tt>
 * @return vector of 2D points of size \c N
 *
 * Example:
 * @code
 * MxArray cellArray(prhs[0]);
 * vector<Point2d> vp = MxArrayToVectorPoint<double>(cellArray);
 * @endcode
 */
template <typename T>
std::vector<cv::Point_<T> > MxArrayToVectorPoint(const MxArray& arr)
{
    std::vector<cv::Point_<T> > vp;
    if (arr.isNumeric()) {
        if (arr.numel() == 2)
            vp.push_back(arr.toPoint_<T>());
        else
            arr.toMat(cv::traits::Depth<cv::Point_<T> >::value).reshape(2, 0).copyTo(vp);
    }
    else if (arr.isCell()) {
        /*
        std::vector<MxArray> va(arr.toVector<MxArray>());
        vp.reserve(va.size());
        for (std::vector<MxArray>::const_iterator it = va.begin(); it != va.end(); ++it)
            vp.push_back(it->toPoint_<T>());
        */
        vp = arr.toVector(
            std::const_mem_fun_ref_t<cv::Point_<T>, MxArray>(
                &MxArray::toPoint_<T>));
    }
    else
        mexErrMsgIdAndTxt("mexopencv:error",
            "Unable to convert MxArray to std::vector<cv::Point_<T>>");
    return vp;
}

/** Convert an MxArray to std::vector<cv::Point3_<T>>
 *
 * @param arr MxArray object. In one of the following forms:
 * - a cell-array of 3D points (3-element vectors) of length \c N,
 *   e.g: <tt>{[x,y,z], [x,y,z], ...}</tt>
 * - a numeric matrix of size \c Nx3, \c Nx1x3, or \c 1xNx3 in the form:
 *   <tt>[x,y,z; x,y,z; ...]</tt> or <tt>cat(3, [x,y,z], [x,y,z], ...)</tt>
 * @return vector of 3D points of size \c N
 *
 * Example:
 * @code
 * MxArray cellArray(prhs[0]);
 * vector<Point3f> vp = MxArrayToVectorPoint3<float>(cellArray);
 * @endcode
 */
template <typename T>
std::vector<cv::Point3_<T> > MxArrayToVectorPoint3(const MxArray& arr)
{
    std::vector<cv::Point3_<T> > vp;
    if (arr.isNumeric()) {
        if (arr.numel() == 3)
            vp.push_back(arr.toPoint3_<T>());
        else
            arr.toMat(cv::traits::Depth<cv::Point3_<T> >::value).reshape(3, 0).copyTo(vp);
    }
    else if (arr.isCell()) {
        /*
        std::vector<MxArray> va(arr.toVector<MxArray>());
        vp.reserve(va.size());
        for (std::vector<MxArray>::const_iterator it = va.begin(); it != va.end(); ++it)
            vp.push_back(it->toPoint3_<T>());
        */
        vp = arr.toVector(
            std::const_mem_fun_ref_t<cv::Point3_<T>, MxArray>(
                &MxArray::toPoint3_<T>));
    }
    else
        mexErrMsgIdAndTxt("mexopencv:error",
            "Unable to convert MxArray to std::vector<cv::Point3_<T>>");
    return vp;
}

/** Convert an MxArray to std::vector<cv::Size_<T>>
 *
 * @param arr MxArray object. In one of the following forms:
 * - a cell-array of sizes (2-element vectors) of length \c N,
 *   e.g: <tt>{[w,h], [w,h], ...}</tt>
 * - a numeric matrix of size \c Nx2, \c Nx1x2, or \c 1xNx2 in the form:
 *   <tt>[w,h; w,h; ...]</tt> or <tt>cat(3, [w,h], [w,h], ...)</tt>
 * @return vector of sizes of size \c N
 *
 * Example:
 * @code
 * MxArray cellArray(prhs[0]);
 * vector<Size2d> vs = MxArrayToVectorSize<double>(cellArray);
 * @endcode
 */
template <typename T>
std::vector<cv::Size_<T> > MxArrayToVectorSize(const MxArray& arr)
{
    std::vector<cv::Size_<T> > vs;
    if (arr.isNumeric()) {
        if (arr.numel() == 2)
            vs.push_back(arr.toSize_<T>());
        else
            arr.toMat(cv::traits::Depth<cv::Size_<T> >::value).reshape(2, 0).copyTo(vs);
    }
    else if (arr.isCell()) {
        /*
        std::vector<MxArray> va(arr.toVector<MxArray>());
        vs.reserve(va.size());
        for (std::vector<MxArray>::const_iterator it = va.begin(); it != va.end(); ++it)
            vs.push_back(it->toSize_<T>());
        */
        vs = arr.toVector(
            std::const_mem_fun_ref_t<cv::Size_<T>, MxArray>(
                &MxArray::toSize_<T>));
    }
    else
        mexErrMsgIdAndTxt("mexopencv:error",
            "Unable to convert MxArray to std::vector<cv::Size_<T>>");
    return vs;
}

/** Convert an MxArray to std::vector<cv::Rect_<T>>
 *
 * @param arr MxArray object. In one of the following forms:
 * - a cell-array of rectangles (4-element vectors) of length \c N,
 *   e.g: <tt>{[x,y,w,h], [x,y,w,h], ...}</tt>
 * - a numeric matrix of size \c Nx4, \c Nx1x4, or \c 1xNx4 in the form:
 *   <tt>[x,y,w,h; x,y,w,h; ...]</tt> or
 *   <tt>cat(3, [x,y,w,h], [x,y,w,h], ...)</tt>
 * @return vector of rectangles of size \c N
 *
 * Example:
 * @code
 * MxArray cellArray(prhs[0]);
 * vector<Rect2f> vr = MxArrayToVectorRect<float>(cellArray);
 * @endcode
 */
template <typename T>
std::vector<cv::Rect_<T> > MxArrayToVectorRect(const MxArray& arr)
{
    std::vector<cv::Rect_<T> > vr;
    if (arr.isNumeric()) {
        if (arr.numel() == 4)
            vr.push_back(arr.toRect_<T>());
        else
            arr.toMat(cv::traits::Depth<cv::Rect_<T> >::value).reshape(4, 0).copyTo(vr);
    }
    else if (arr.isCell()) {
        /*
        std::vector<MxArray> va(arr.toVector<MxArray>());
        vr.reserve(va.size());
        for (std::vector<MxArray>::const_iterator it = va.begin(); it != va.end(); ++it)
            vr.push_back(it->toRect_<T>());
        */
        vr = arr.toVector(
            std::const_mem_fun_ref_t<cv::Rect_<T>, MxArray>(
                &MxArray::toRect_<T>));
    }
    else
        mexErrMsgIdAndTxt("mexopencv:error",
            "Unable to convert MxArray to std::vector<cv::Rect_<T>>");
    return vr;
}

/** Convert an MxArray to std::vector<cv::Vec<T,cn>>
 *
 * @param arr MxArray object. In one of the following forms:
 * - a cell-array of vecs (\c cn -element vectors) of length \c N,
 *   e.g: <tt>{[v_1,v_2,...,v_cn], [v_1,v_2,...,v_cn], ...}</tt>
 * - a numeric matrix of size \c Nxcn, \c Nx1xcn, or \c 1xNxcn in the form:
 *   <tt>[v_1,v_2,...,v_cn; v_1,v_2,...,v_cn; ...]</tt> or
 *   <tt>cat(3, [v_1,v_2,...,v_cn], [v_1,v_2,...,v_cn], ...)</tt>
 * @return vector of vecs of size \c N
 *
 * Example:
 * @code
 * MxArray cellArray(prhs[0]);
 * vector<Vec4i> vv = MxArrayToVectorVec<int,4>(cellArray);
 * @endcode
 */
template <typename T, int cn>
std::vector<cv::Vec<T,cn> > MxArrayToVectorVec(const MxArray& arr)
{
    std::vector<cv::Vec<T,cn> > vv;
    if (arr.isNumeric()) {
        if (arr.numel() == cn)
            vv.push_back(arr.toVec<T,cn>());
        else
            arr.toMat(cv::traits::Depth<cv::Vec<T,cn> >::value).reshape(cn, 0).copyTo(vv);
    }
    else if (arr.isCell()) {
        /*
        std::vector<MxArray> va(arr.toVector<MxArray>());
        vv.reserve(va.size());
        for (std::vector<MxArray>::const_iterator it = va.begin(); it != va.end(); ++it)
            vv.push_back(it->toVec<T,cn>());
        */
        vv = arr.toVector(
            std::const_mem_fun_ref_t<cv::Vec<T,cn>, MxArray>(
                &MxArray::toVec<T,cn>));
    }
    else
        mexErrMsgIdAndTxt("mexopencv:error",
            "Unable to convert MxArray to std::vector<cv::Vec<T,cn>>");
    return vv;
}

/** Convert an MxArray to std::vector<cv::Matx<T,m,n>>
 *
 * @param arr MxArray object. In one of the following forms:
 * - a cell-array of mats (\c mxn matrices) of length \c N,
 *   e.g: <tt>{[mat_11, ..., mat_1n; ....; mat_m1, ..., mat_mn], ...}</tt>
 * - a sole numeric matrix (\c N=1) of size \c mxn in the form:
 *   <tt>[mat_11, ..., mat_1n; ....; mat_m1, ..., mat_mn]</tt>
 * @return vector of mats of size \c N
 *
 * Example:
 * @code
 * MxArray cellArray(prhs[0]);
 * vector<Matx32f> vx = MxArrayToVectorMatx<float,3,2>(cellArray);
 * @endcode
 */
template <typename T, int m, int n>
std::vector<cv::Matx<T,m,n> > MxArrayToVectorMatx(const MxArray& arr)
{
    std::vector<cv::Matx<T,m,n> > vx;
    if (arr.isNumeric()) {
        vx.push_back(arr.toMatx<T,m,n>());
    }
    else if (arr.isCell()) {
        /*
        std::vector<MxArray> va(arr.toVector<MxArray>());
        vx.reserve(va.size());
        for (std::vector<MxArray>::const_iterator it = va.begin(); it != va.end(); ++it)
            vx.push_back(it->toMatx<T,m,n>());
        */
        vx = arr.toVector(
            std::const_mem_fun_ref_t<cv::Matx<T,m,n>, MxArray>(
                &MxArray::toMatx<T,m,n>));
    }
    else
        mexErrMsgIdAndTxt("mexopencv:error",
            "Unable to convert MxArray to std::vector<cv::Matx<T,m,n>>");
    return vx;
}

/**************************************************************\
*      Conversion Functions: MxArray to vector of vectors      *
\**************************************************************/

/** Convert an MxArray to std::vector<std::vector<T>>
 *
 * @param arr MxArray object. In one of the following forms:
 * - a cell-array of cell-arrays of numeric scalars,
 *   e.g: <tt>{{s1, s2, ...}, {s1, ...}, ...}</tt>
 * - a cell-array of numeric vectors,
 *   e.g: <tt>{[s1, s2, ...], [s1, ...], ...}</tt>
 * @return vector of vectors of primitives of type T
 *
 * Example:
 * @code
 * MxArray cellArray(prhs[0]);
 * vector<vector<int>> vvi = MxArrayToVectorVectorPrimitive<int>(cellArray);
 * @endcode
 */
template <typename T>
std::vector<std::vector<T> > MxArrayToVectorVectorPrimitive(const MxArray& arr)
{
    /*
    std::vector<MxArray> vva(arr.toVector<MxArray>());
    std::vector<std::vector<T> > vv;
    vv.reserve(vva.size());
    for (std::vector<MxArray>::const_iterator it = vva.begin(); it != vva.end(); ++it) {
        vv.push_back(it->toVector<T>());
    }
    return vv;
    */
    typedef std::vector<T> VecT;
    std::const_mem_fun_ref_t<VecT, MxArray> func(&MxArray::toVector<T>);
    return arr.toVector(func);
}

/** Convert an MxArray to std::vector<std::vector<cv::Point_<T>>>
 *
 * @param arr MxArray object. In one of the following forms:
 * - a cell-array of cell-arrays of 2D points (2-element vectors),
 *   e.g: <tt>{{[x,y], [x,y], ..}, {[x,y], [x,y], ..}, ...}</tt>
 * - a cell-array of numeric matrices of size \c Mx2, \c Mx1x2, or \c 1xMx2,
 *   e.g: <tt>{[x,y; x,y; ...], [x,y; x,y; ...], ...}</tt> or
 *   <tt>{cat(3, [x,y], [x,y], ...), cat(3, [x,y], [x,y], ...), ...}</tt>
 * @return vector of vectors of 2D points
 *
 * Example:
 * @code
 * MxArray cellArray(prhs[0]);
 * vector<vector<Point2d>> vvp = MxArrayToVectorVectorPoint<double>(cellArray);
 * @endcode
 */
template <typename T>
std::vector<std::vector<cv::Point_<T> > > MxArrayToVectorVectorPoint(const MxArray& arr)
{
    std::vector<MxArray> vva(arr.toVector<MxArray>());
    std::vector<std::vector<cv::Point_<T> > > vvp;
    vvp.reserve(vva.size());
    for (std::vector<MxArray>::const_iterator it = vva.begin(); it != vva.end(); ++it) {
        /*
        std::vector<MxArray> va(it->toVector<MxArray());
        std::vector<cv::Point_<T> > vp;
        for (std::vector<MxArray>::const_iterator jt = va.begin(); jt != va.end(); ++jt) {
            vp.push_back(jt->toPoint_<T>());
        }
        vvp.push_back(vp);
        */
        vvp.push_back(MxArrayToVectorPoint<T>(*it));
    }
    return vvp;
}

/** Convert an MxArray to std::vector<std::vector<cv::Point3_<T>>>
 *
 * @param arr MxArray object. In one of the following forms:
 * - a cell-array of cell-arrays of 3D points (3-element vectors),
 *   e.g: <tt>{{[x,y,z], [x,y,z], ..}, {[x,y,z], [x,y,z], ..}, ...}</tt>
 * - a cell-array of numeric matrices of size \c Mx3, \c Mx1x3, or \c 1xMx3,
 *   e.g: <tt>{[x,y,z; x,y,z; ...], [x,y,z; x,y,z; ...], ...}</tt> or
 *   <tt>{cat(3, [x,y,z], [x,y,z], ...), cat(3, [x,y,z], [x,y,z], ...), ...}</tt>
 * @return vector of vectors of 3D points
 *
 * Example:
 * @code
 * MxArray cellArray(prhs[0]);
 * vector<vector<Point3d>> vvp = MxArrayToVectorVectorPoint3<double>(cellArray);
 * @endcode
 */
template <typename T>
std::vector<std::vector<cv::Point3_<T> > > MxArrayToVectorVectorPoint3(const MxArray& arr)
{
    std::vector<MxArray> vva(arr.toVector<MxArray>());
    std::vector<std::vector<cv::Point3_<T> > > vvp;
    vvp.reserve(vva.size());
    for (std::vector<MxArray>::const_iterator it = vva.begin(); it != vva.end(); ++it) {
        /*
        std::vector<MxArray> va(it->toVector<MxArray());
        std::vector<cv::Point3_<T> > vp;
        for (std::vector<MxArray>::const_iterator jt = va.begin(); jt != va.end(); ++jt) {
            vp.push_back(jt->toPoint3_<T>());
        }
        vvp.push_back(vp);
        */
        vvp.push_back(MxArrayToVectorPoint3<T>(*it));
    }
    return vvp;
}

#endif
