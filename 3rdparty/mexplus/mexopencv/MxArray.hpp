/** MxArray and ConstMap declaration.
 * @file MxArray.hpp
 * @author Kota Yamaguchi
 * @date 2012
 */
#ifndef MXARRAY_HPP
#define MXARRAY_HPP

#include <stdint.h>
#include <functional>
#include <map>
#include <string>
#include "mex.h"
#include "opencv2/opencv.hpp"

/** Type traits for mxArray.
 */
template <typename T>
struct MxTypes
{
	/// maps general template parameter to unkown MATLAB type.
	static const mxClassID type = mxUNKNOWN_CLASS;
};

/** int8_t traits.
 */
template <>
struct MxTypes<int8_t>
{
	/// maps \c int8_t to \c int8 MATLAB type.
	static const mxClassID type = mxINT8_CLASS;
};

/** uint8_t traits.
 */
template <>
struct MxTypes<uint8_t>
{
	/// maps \c uint8_t to \c uint8 MATLAB type.
	static const mxClassID type = mxUINT8_CLASS;
};

/** int16_t traits.
 */
template <>
struct MxTypes<int16_t>
{
	/// maps \c int16_t to \c int16 MATLAB type.
	static const mxClassID type = mxINT16_CLASS;
};

/** uint16_t traits.
 */
template <>
struct MxTypes<uint16_t>
{
	/// maps \c uint16_t to \c uint16 MATLAB type.
	static const mxClassID type = mxUINT16_CLASS;
};

/** int32_t traits.
 */
template <>
struct MxTypes<int32_t>
{
	/// maps \c int32_t to \c int32 MATLAB type.
	static const mxClassID type = mxINT32_CLASS;
};

/** uint32_t traits.
 */
template <>
struct MxTypes<uint32_t>
{
	/// maps \c uint32_t to \c uint32 MATLAB type.
	static const mxClassID type = mxUINT32_CLASS;
};

/** int64_t traits.
 */
template <>
struct MxTypes<int64_t>
{
	/// maps \c int64_t to \c int64 MATLAB type.
	static const mxClassID type = mxINT64_CLASS;
};

/** uint64_t traits.
 */
template <>
struct MxTypes<uint64_t>
{
	/// maps \c uint64_t to \c uint64 MATLAB type.
	static const mxClassID type = mxUINT64_CLASS;
};

/** float traits.
 */
template <>
struct MxTypes<float>
{
	/// maps \c float to \c single MATLAB type.
	static const mxClassID type = mxSINGLE_CLASS;
};

/** double traits.
 */
template <>
struct MxTypes<double>
{
	/// maps \c double to \c double MATLAB type.
	static const mxClassID type = mxDOUBLE_CLASS;
};

/** char traits.
 */
template <>
struct MxTypes<char>
{
	/// maps \c char to \c mxChar MATLAB type.
	static const mxClassID type = mxCHAR_CLASS;
};

/** bool traits.
 */
template <>
struct MxTypes<bool>
{
	/// maps \c bool to \c mxLogical MATLAB type.
	static const mxClassID type = mxLOGICAL_CLASS;
};

/** Cutom error callback to be invoked by cv::error(), CV_Assert, etc.
 * @param status status code.
 * @param func_name function name.
 * @param err_msg error message.
 * @param file_name filename path.
 * @param line line number.
 * @param userdata optional user data pointer (unused).
 * @return zero code.
 * @sa cv::redirectError
 */
int MexErrorHandler(
	int status, const char* func_name, const char* err_msg,
	const char* file_name, int line, void* userdata);

/** mxArray object wrapper for data conversion and manipulation.
 */
class MxArray
{
   public:
	/** MxArray constructor from mxArray*.
	 * @param arr mxArray pointer given by mexFunction.
	 */
	MxArray(const mxArray* arr) : p_(arr) {}
	/** Copy constructor.
	 * @param arr Another MxArray.
	 */
	MxArray(const MxArray& arr) : p_(arr.p_) {}
	/** Assignment operator.
	 * @param rhs Reference to another MxArray.
	 * @return reference to current MxArray object (for chained calls).
	 */
	MxArray& operator=(const MxArray& rhs);
	/** MxArray constructor from int.
	 * @param i int value.
	 * @return MxArray object, a scalar double array.
	 */
	explicit MxArray(const int i);
	/** MxArray constructor from double.
	 * @param d double value.
	 * @return MxArray object, a scalar double array.
	 */
	explicit MxArray(const double d);
	/** MxArray constructor from bool.
	 * @param b bool value.
	 * @return MxArray object, a scalar logical array.
	 */
	explicit MxArray(const bool b);
	/** MxArray constructor from std::string.
	 * @param s reference to a string value.
	 * @return MxArray object.
	 */
	explicit MxArray(const std::string& s);
	/** Convert cv::Mat to MxArray.
	 * @param mat cv::Mat object.
	 * @param classid classid of mxArray. e.g., \c mxDOUBLE_CLASS. When
	 *    \c mxUNKNOWN_CLASS is specified, classid will be automatically
	 *    determined from the type of cv::Mat. default: \c mxUNKNOWN_CLASS.
	 * @param transpose Optional transposition to the return value so that
	 *    rows and columns of the 2D Mat are mapped to the 2nd and 1st
	 *    dimensions in MxArray, respectively. This does not apply the N-D
	 *    array conversion. default true.
	 * @return MxArray object.
	 *
	 * Convert cv::Mat object to an MxArray. When the cv::Mat object is 2D, the
	 * width, height, and channels are mapped to the first, second, and third
	 * dimensions of the MxArray unless \p transpose flag is set to false. When
	 * the cv::Mat object is N-D, <tt>(dim 1, dim 2,...dim N, channels)</tt>
	 * are mapped to <tt>(dim 2, dim 1, ..., dim N, dim N+1)</tt>,
	 * respectively.
	 *
	 * Example:
	 * @code
	 * cv::Mat x(120, 90, CV_8UC3, Scalar(0));
	 * mxArray* plhs[0] = MxArray(x);
	 * @endcode
	 */
	explicit MxArray(
		const cv::Mat& mat, mxClassID classid = mxUNKNOWN_CLASS,
		bool transpose = true);
	/** Convert float cv::SparseMat to MxArray.
	 * @param mat cv::SparseMat object.
	 * @return MxArray object, a 2D sparse array.
	 */
	explicit MxArray(const cv::SparseMat& mat);
	/** Convert cv::Moments to MxArray.
	 * @param m cv::Moments object.
	 * @return MxArray object, a scalar struct array.
	 */
	explicit MxArray(const cv::Moments& m);
	/** Convert cv::KeyPoint to MxArray.
	 * @param p cv::KeyPoint object.
	 * @return MxArray object, a scalar struct array.
	 */
	explicit MxArray(const cv::KeyPoint& p);
	/** Convert cv::DMatch to MxArray.
	 * @param m cv::DMatch object.
	 * @return MxArray object, a scalar struct array.
	 */
	explicit MxArray(const cv::DMatch& m);
	/** Convert cv::RotatedRect to MxArray.
	 * @param r cv::RotatedRect object.
	 * @return MxArray object, a scalar struct array.
	 */
	explicit MxArray(const cv::RotatedRect& r);
	/** Convert cv::TermCriteria to MxArray.
	 * @param t cv::TermCriteria object.
	 * @return MxArray object, a scalar struct array.
	 */
	explicit MxArray(const cv::TermCriteria& t);
	/** MxArray constructor from vector<T>.
	 * @param v vector of type T.
	 * @return MxArray object, a numeric or a cell array.
	 *
	 * The constructor forwards the call to an appropriate overloaded method
	 * according to the parameter type (tag dispatching by instance).
	 * In the general case, vectors of primitive types are converted to
	 * numeric arrays of the equivalent MATLAB type, while other types are
	 * converted to cell arrays.
	 *
	 * Example:
	 * @code
	 * std::vector<double> v(10, 1.0);
	 * mxArray* plhs[0] = MxArray(v);
	 * @endcode
	 */
	template <typename T>
	explicit MxArray(const std::vector<T>& v)
	{
		// we do this: fromVector(v) as opposed to: fromVector<T>(v),
		// that way the call gets resolved to overloaded or
		// template-specialized version appropriately.
		// (although we dont currently have an overloaded version)
		fromVector(v);
	}
	/** MxArray constructor from cv::Point_<T>.
	 * @param p cv::Point_<T> object.
	 * @return two-element numeric MxArray <tt>[x, y]</tt>.
	 */
	template <typename T>
	explicit MxArray(const cv::Point_<T>& p);
	/** MxArray constructor from cv::Point3_<T>.
	 * @param p cv::Point3_<T> object.
	 * @return three-element numeric MxArray <tt>[x, y, z]</tt>.
	 */
	template <typename T>
	explicit MxArray(const cv::Point3_<T>& p);
	/** MxArray constructor from cv::Size_<T>.
	 * @param s cv::Size_<T> object.
	 * @return two-element numeric MxArray.
	 */
	template <typename T>
	explicit MxArray(const cv::Size_<T>& s);
	/** MxArray constructor from cv::Rect_<T>.
	 * @param r cv::Rect_<T> object.
	 * @return four-element numeric MxArray <tt>[x, y, width, height]</tt>.
	 */
	template <typename T>
	explicit MxArray(const cv::Rect_<T>& r);
	/** MxArray constructor from cv::Scalar_<T>.
	 * @param s cv::Scalar_<T> object.
	 * @return four-element numeric MxArray <tt>[v0, v1, v2, v3]</tt>.
	 */
	template <typename T>
	explicit MxArray(const cv::Scalar_<T>& s);
	/** MxArray constructor from cv::Vec<T,cn>.
	 * @param vec cv::Vec<T,cn> object.
	 * @return \c cn -element numeric MxArray <tt>[v0, v1, ...]</tt>.
	 */
	template <typename T, int cn>
	explicit MxArray(const cv::Vec<T, cn>& vec);
	/** MxArray constructor from cv::Matx<T,m,n>.
	 * @param mat cv::Mat<T,m,n> object.
	 * @return \c m-by-n numeric MxArray matrix
	 *    <tt>[mat_11, ..., mat_1n; ....; mat_m1, ..., mat_mn]</tt>.
	 */
	template <typename T, int m, int n>
	explicit MxArray(const cv::Matx<T, m, n>& mat);
	/** Destructor. This does not free the underlying mxArray*.
	 */
	virtual ~MxArray() {}
	/** Create a new cell array.
	 * @param m Number of rows.
	 * @param n Number of cols.
	 * @return MxArray object, a 2D cell array with uninitialized cells.
	 *
	 * Example:
	 * @code
	 * MxArray c = MxArray::Cell(2,1);
	 * c.set(0, MxArray(1));
	 * c.set(1, MxArray(std::string("some value")));
	 * @endcode
	 */
	static inline MxArray Cell(mwSize m = 1, mwSize n = 1)
	{
		mxArray* pm = mxCreateCellMatrix(m, n);
		if (!pm) mexErrMsgIdAndTxt("mexopencv:error", "Allocation error");
		return MxArray(pm);
	}
	/** Create a new struct array.
	 * @param fields Field names.
	 * @param nfields Number of fields.
	 * @param m Number of rows (size of the first dimension).
	 * @param n Number of cols (size of the second dimension).
	 * @return MxArray object, a 2D struct array with uninitialized fields.
	 *
	 * Example:
	 * @code
	 * const char* fields[] = {"field1", "field2"};
	 * MxArray s = MxArray::Struct(fields, 2);
	 * s.set("field1", 1);
	 * s.set("field2", "field2 value");
	 * @endcode
	 */
	static inline MxArray Struct(
		const char** fields = NULL, int nfields = 0, mwSize m = 1, mwSize n = 1)
	{
		mxArray* pm = mxCreateStructMatrix(m, n, nfields, fields);
		if (!pm) mexErrMsgIdAndTxt("mexopencv:error", "Allocation error");
		return MxArray(pm);
	}
	/** Clone mxArray. This allocates new mxArray*.
	 * @return MxArray object, a deep-copy clone.
	 */
	MxArray clone() const;
	/** Deallocate memory occupied by mxArray.
	 *
	 * Use this to destroy a temporary mxArray. Do not call this on arrays
	 * you are returning to MATLAB as left-hand side.
	 */
	void destroy() { mxDestroyArray(const_cast<mxArray*>(p_)); }
	/** Implicit conversion to const mxArray*.
	 * @return const mxArray* pointer.
	 */
	operator const mxArray*() const { return p_; };
	/** Implicit conversion to mxArray*.
	 * @return mxArray* pointer.
	 *
	 * Be careful that this internally casts away mxArray* constness.
	 */
	operator mxArray*() const { return const_cast<mxArray*>(p_); };
	/** Convert MxArray to int.
	 * @return int value.
	 */
	int toInt() const;
	/** Convert MxArray to double.
	 * @return double value.
	 */
	double toDouble() const;
	/** Convert MxArray to float.
	 * @return float value.
	 */
	float toFloat() const;
	/** Convert MxArray to bool.
	 * @return bool value.
	 */
	bool toBool() const;
	/** Convert MxArray to std::string.
	 * @return std::string value.
	 */
	std::string toString() const;
	/** Convert MxArray to cv::Mat.
	 * @param depth depth of cv::Mat. e.g., \c CV_8U, \c CV_32F. When
	 *    \c CV_USERTYPE1 is specified, depth will be automatically determined
	 *    from the classid of the MxArray. default: \c CV_USERTYPE1.
	 * @param transpose Optional transposition to the return value so that
	 *     rows and columns of the 2D Mat are mapped to the 2nd and 1st
	 *     dimensions in MxArray, respectively. This does not apply the N-D
	 *     array conversion. default true.
	 * @return cv::Mat object.
	 *
	 * Convert a MxArray object to a cv::Mat object. When the dimensionality
	 * of the MxArray is more than 2, the last dimension will be mapped to the
	 * channels of the cv::Mat. Also, if the resulting cv::Mat is 2D, the 1st
	 * and 2nd dimensions of the MxArray are mapped to rows and columns of the
	 * cv::Mat unless \p transpose flag is false. That is, when MxArray is 3D,
	 * <tt>(dim 1, dim 2, dim 3)</tt> are mapped to
	 * <tt>(cols, rows, channels)</tt> of the cv::Mat by default, whereas if
	 * MxArray is more than 4D, <tt>(dim 1, dim 2, ..., dim N-1, dim N)</tt>
	 * are mapped to <tt>(dim 2, dim 1, ..., dim N-1, channels)</tt> of the
	 * cv::Mat, respectively.
	 *
	 * Example:
	 * @code
	 * cv::Mat x(MxArray(prhs[0]).toMat());
	 * @endcode
	 */
	cv::Mat toMat(int depth = CV_USRTYPE1, bool transpose = true) const;
	/** Convert MxArray to a single-channel cv::Mat.
	 * @param depth depth of cv::Mat. e.g., \c CV_8U, \c CV_32F. When
	 *    \c CV_USERTYPE1 is specified, depth will be automatically determined
	 *    from the the classid of the MxArray. default: \c CV_USERTYPE1.
	 * @param transpose Optional transposition to the return value so that
	 *    rows and columns of the 2D Mat are mapped to the 2nd and 1st
	 *    dimensions in MxArray, respectively. This does not apply the N-D
	 *    array conversion. default true.
	 * @return const cv::Mat object.
	 *
	 * Convert a MxArray object to a single-channel cv::Mat object. If the
	 * MxArray is 2D, the 1st and 2nd dimensions of the MxArray are mapped to
	 * rows and columns of the cv::Mat unless \p transpose flag is false. If
	 * the MxArray is more than 3D, the 1st and 2nd dimensions of the MxArray
	 * are mapped to 2nd and 1st dimensions of the cv::Mat. That is, when
	 * MxArray is 2D, <tt>(dim 1, dim 2)</tt> are mapped to
	 * <tt>(cols, rows)</tt> of the cv::Mat by default, whereas if MxArray is
	 * more than 3D, <tt>(dim 1, dim 2, dim 3, ..., dim N)</tt> are mapped to
	 * <tt>(dim 2, dim 1, dim 3, ..., dim N)</tt> of the cv::Mat,
	 * respectively.
	 *
	 * Example:
	 * @code
	 * cv::Mat x(MxArray(prhs[0]).toMatND());
	 * @endcode
	 */
	cv::MatND toMatND(int depth = CV_USRTYPE1, bool transpose = true) const;
	/** Convert double sparse MxArray to 2D single-channel cv::SparseMat.
	 * @param depth depth of cv::SparseMat. e.g., \c CV_32F, \c CV_64F. When
	 *    \c CV_USERTYPE1 is specified, depth will be automatically determined
	 *    from the the classid of the MxArray (which is double, the only
	 *    supported type for MATLAB sparse arrays). default: \c CV_USERTYPE1.
	 * @return cv::SparseMat object.
	 */
	cv::SparseMat toSparseMat(int depth = CV_USRTYPE1) const;
	/** Convert MxArray to cv::Moments.
	 * @param index linear index of the struct array element.
	 * @return cv::Moments object.
	 */
	cv::Moments toMoments(mwIndex index = 0) const;
	/** Convert MxArray to cv::KeyPoint.
	 * @param index linear index of the struct array element.
	 * @return cv::KeyPoint object.
	 */
	cv::KeyPoint toKeyPoint(mwIndex index = 0) const;
	/** Convert MxArray to cv::DMatch.
	 * @param index linear index of the struct array element.
	 * @return cv::DMatch object.
	 */
	cv::DMatch toDMatch(mwIndex index = 0) const;
	/** Convert MxArray to cv::Range.
	 * @return cv::Range object.
	 */
	cv::Range toRange() const;
	/** Convert MxArray to cv::RotatedRect.
	 * @param index linear index of the struct array element.
	 * @return cv::RotatedRect object.
	 */
	cv::RotatedRect toRotatedRect(mwIndex index = 0) const;
	/** Convert MxArray to cv::TermCriteria.
	 * @param index linear index of the struct array element.
	 * @return cv::TermCriteria object.
	 */
	cv::TermCriteria toTermCriteria(mwIndex index = 0) const;
	/** Convert MxArray to Point_<T>.
	 * @return cv::Point_<T> value.
	 */
	template <typename T>
	cv::Point_<T> toPoint_() const;
	/** Convert MxArray to Point3_<T>.
	 * @return cv::Poin3_<T> value.
	 */
	template <typename T>
	cv::Point3_<T> toPoint3_() const;
	/** Convert MxArray to Size_<T>.
	 * @return cv::Size_<T> value.
	 */
	template <typename T>
	cv::Size_<T> toSize_() const;
	/** Convert MxArray to Rect_<T>.
	 * @return cv::Rect_<T> value.
	 */
	template <typename T>
	cv::Rect_<T> toRect_() const;
	/** Convert MxArray to Scalar_<T>.
	 * @return cv::Scalar_<T> value.
	 */
	template <typename T>
	cv::Scalar_<T> toScalar_() const;
	/** Convert MxArray to Vec<T,cn>.
	 * @return cv::Vec<T,cn> value.
	 */
	template <typename T, int cn>
	cv::Vec<T, cn> toVec() const;
	/** Convert MxArray to Matx<T,m,n>.
	 * @return cv::Matx<T,m,n> value.
	 */
	template <typename T, int m, int n>
	cv::Matx<T, m, n> toMatx() const;
	/** Convert MxArray to std::vector<T> of primitive types.
	 * @return std::vector<T> value.
	 *
	 * The method is intended for conversion to a raw numeric vector such
	 * as std::vector<int> or std::vector<double>. Example:
	 *
	 * @code
	 * MxArray numArray(prhs[0]);
	 * vector<double> vd = numArray.toVector<double>();
	 * @endcode
	 */
	template <typename T>
	std::vector<T> toVector() const;
	/** Convert MxArray to std::vector<T> by a specified conversion method.
	 * @param f member function of MxArray (e.g., &MxArray::toMat,
	 *          &MxArray::toInt).
	 * @return std::vector<T> value.
	 *
	 * The method constructs std::vector<T> by applying conversion method \p f
	 * to each cell array element. This is similar to std::transform function.
	 * An example usage is shown below:
	 *
	 * @code
	 * MxArray cellArray(prhs[0]);
	 * const_mem_fun_ref_t<Point3i,MxArray> convert(&MxArray::toPoint3_<int>);
	 * vector<Point3i> v = cellArray.toVector(convert);
	 * @endcode
	 */
#if 0  // JLBC for MRPT (C++17 incompatible)
    template <typename T>
    std::vector<T> toVector(std::const_mem_fun_ref_t<T, MxArray> f) const;
#endif
	/** Alias to toPoint_<int>.
	 * @return cv::Point object.
	 */
	inline cv::Point toPoint() const { return toPoint_<int>(); }
	/** Alias to toPoint_<float>.
	 * @return cv::Point2f object.
	 */
	inline cv::Point2f toPoint2f() const { return toPoint_<float>(); }
	/** Alias to toPoint3_<float>.
	 * @return cv::Point3f object.
	 */
	inline cv::Point3f toPoint3f() const { return toPoint3_<float>(); }
	/** Alias to toSize_<int>.
	 * @return cv::Size object.
	 */
	inline cv::Size toSize() const { return toSize_<int>(); }
	/** Alias to toRect_<int>.
	 * @return cv::Rect object.
	 */
	inline cv::Rect toRect() const { return toRect_<int>(); }
	/** Alias to toScalar_<double>
	 * @return cv::Scalar object.
	 */
	inline cv::Scalar toScalar() const { return toScalar_<double>(); }
	/** Class ID of mxArray.
	 * @return identifier of the array class, enum value of type mxClassID.
	 */
	inline mxClassID classID() const { return mxGetClassID(p_); }
	/** Class name of mxArray.
	 * @return class name of the array, as a string.
	 */
	inline const std::string className() const
	{
		return std::string(mxGetClassName(p_));
	}
	/** Number of elements in an array.
	 * @return number of elements in the array
	 */
	inline mwSize numel() const { return mxGetNumberOfElements(p_); }
	/** Number of dimensions.
	 * @return number of dimension in the array, always >= 2.
	 */
	inline mwSize ndims() const { return mxGetNumberOfDimensions(p_); }
	/** Array of each dimension.
	 * @return array of dimensions, number of elements in each dimension.
	 */
	inline const mwSize* dims() const { return mxGetDimensions(p_); }
	/** Number of rows in an array.
	 * @return number of rows in the array (first dimension).
	 */
	inline mwSize rows() const { return mxGetM(p_); }
	/** Number of columns in an array.
	 * @return number of cols in the array.
	 *
	 * If the array is N-dimensional, this returns the product of dimensions
	 * 2 through N.
	 */
	inline mwSize cols() const { return mxGetN(p_); }
	/** Number of fields in a struct array.
	 * @return number of fields in the structure array.
	 */
	inline int nfields() const { return mxGetNumberOfFields(p_); }
	/** Get specified field name from a struct array.
	 * @param fieldnumber position of the desired field.
	 * @return std::string name of n-th field.
	 */
	std::string fieldname(int fieldnumber) const;
	/** Get field names of a struct array.
	 * @return std::vector<std::string> vector of all field names.
	 */
	std::vector<std::string> fieldnames() const;
	/** Number of elements in \c IR, \c PR, and \c PI arrays.
	 * @return number of elements allocated to hold nonzero entries
	 *         in the sparse array.
	 */
	inline mwSize nzmax() const { return mxGetNzmax(p_); }
	/** Offset from first element to desired element.
	 * @param i index of the first dimension of the array.
	 * @param j index of the second dimension of the array.
	 * @return linear offset of the specified subscript index.
	 *
	 * Return the offset (in number of elements) from the beginning of
	 * the array to the specified <tt>(i,j)</tt> subscript.
	 */
	mwIndex subs(mwIndex i, mwIndex j = 0) const;
	/** Offset from first element to desired element.
	 * @param si vector of subscripts for each dimension of the array.
	 * @return linear offset of the specified subscripts.
	 *
	 * Return the offset (in number of elements) from the beginning of
	 * the array to the specified subscript
	 * <tt>(si[0], si[1], ..., si[n])</tt>.
	 */
	mwIndex subs(const std::vector<mwIndex>& si) const;
	/** Determine whether the array is initialized or not.
	 * @return true if the internal mxArray pointer is \c NULL, false
	 *         otherwise.
	 */
	inline bool isNull() const { return (p_ == NULL); }
	/** Determine whether input is cell array.
	 * @return true if array is of type \c mxCELL_CLASS, false otherwise.
	 */
	inline bool isCell() const { return mxIsCell(p_); }
	/** Determine whether input is string array.
	 * @return true if array is of type \c mxCHAR_CLASS, false otherwise.
	 */
	inline bool isChar() const { return mxIsChar(p_); }
	/** Determine whether array is member of specified class.
	 * @param s class name as a string
	 * @return true if array is of specified class, false otherwise.
	 *
	 * Example:
	 * @code
	 * bool b = MxArray(prhs[0]).isClass("uint8");
	 * @endcode
	 */
	inline bool isClass(std::string s) const
	{
		return mxIsClass(p_, s.c_str());
	}
	/** Determine whether data is complex.
	 * @return true if array is numeric containing complex data,
	 *         false otherwise
	 */
	inline bool isComplex() const { return mxIsComplex(p_); }
	/** Determine whether mxArray represents data as double-precision,
	 * floating-point numbers.
	 * @return true if array is of type \c mxDOUBLE_CLASS, false otherwise.
	 */
	inline bool isDouble() const { return mxIsDouble(p_); }
	/** Determine whether array is empty.
	 * @return true if array is empty, false otherwise.
	 *
	 * An array is empty if the size of any of its dimensions is 0.
	 */
	inline bool isEmpty() const { return mxIsEmpty(p_); }
	/** Determine whether input is finite.
	 * @param d double-precision floating-point number
	 * @return true if value is finite, false otherwise.
	 */
	static inline bool isFinite(double d) { return mxIsFinite(d); }
	/** Determine whether array was copied from MATLAB global workspace.
	 * @return true if the array was copied out of the global workspace,
	 *         false otherwise.
	 */
	inline bool isFromGlobalWS() const { return mxIsFromGlobalWS(p_); }
	/** Determine whether input is infinite.
	 * @param d double-precision floating-point number
	 * @return true if value is infinity, false otherwise.
	 */
	static inline bool isInf(double d) { return mxIsInf(d); }
	/** Determine whether array represents data as signed 8-bit integers.
	 * @return true if array is of type \c mxINT8_CLASS, false otherwise.
	 */
	inline bool isInt8() const { return mxIsInt8(p_); }
	/** Determine whether array represents data as signed 16-bit integers.
	 * @return true if array is of type \c mxINT16_CLASS, false otherwise.
	 */
	inline bool isInt16() const { return mxIsInt16(p_); }
	/** Determine whether array represents data as signed 32-bit integers.
	 * @return true if array is of type \c mxINT32_CLASS, false otherwise.
	 */
	inline bool isInt32() const { return mxIsInt32(p_); }
	/** Determine whether array represents data as signed 64-bit integers.
	 * @return true if array is of type \c mxINT64_CLASS, false otherwise.
	 */
	inline bool isInt64() const { return mxIsInt64(p_); }
	/** Determine whether array is of type mxLogical.
	 * @return true if array is of type \c mxLOGICAL_CLASS, false otherwise.
	 */
	inline bool isLogical() const { return mxIsLogical(p_); }
	/** Determine whether scalar array is of type mxLogical.
	 * @return true if array is 1-by-1 of type \c mxLOGICAL_CLASS,
	 *         false otherwise.
	 */
	inline bool isLogicalScalar() const { return mxIsLogicalScalar(p_); }
	/** Determine whether scalar array of type \c mxLogical is true.
	 * @return true if array is 1-by-1 of type \c mxLOGICAL_CLASS
	 *         containing a value of true, returns false otherwise.
	 */
	inline bool isLogicalScalarTrue() const
	{
		return mxIsLogicalScalarTrue(p_);
	}
	/** Determine whether array is numeric.
	 * @return true if array is numeric, false otherwise.
	 */
	inline bool isNumeric() const { return mxIsNumeric(p_); }
	/** Determine whether array represents data as single-precision,
	 * floating-point numbers.
	 * @return true if array is of type \c mxSINGLE_CLASS, false otherwise.
	 */
	inline bool isSingle() const { return mxIsSingle(p_); }
	/** Determine whether input is sparse array.
	 * @return true if array is numeric sparse array, false otherwise.
	 */
	inline bool isSparse() const { return mxIsSparse(p_); }
	/** Determine whether input is structure array.
	 * @return true if array is of type \c mxSTRUCT_CLASS, false otherwise.
	 */
	inline bool isStruct() const { return mxIsStruct(p_); }
	/** Determine whether array represents data as unsigned 8-bit integers.
	 * @return true if array is of type \c mxUINT8_CLASS, false otherwise.
	 */
	inline bool isUint8() const { return mxIsUint8(p_); }
	/** Determine whether array represents data as unsigned 16-bit integers.
	 * @return true if array is of type \c mxUINT16_CLASS, false otherwise.
	 */
	inline bool isUint16() const { return mxIsUint16(p_); }
	/** Determine whether array represents data as unsigned 32-bit integers.
	 * @return true if array is of type \c mxUINT32_CLASS, false otherwise.
	 */
	inline bool isUint32() const { return mxIsUint32(p_); }
	/** Determine whether array represents data as unsigned 64-bit integers.
	 * @return true if array is of type \c mxUINT64_CLASS, false otherwise.
	 */
	inline bool isUint64() const { return mxIsUint64(p_); }
	/** Determine whether array represents data as integer types
	 * (8-bit, 16-bit, 32-bit or 64-bit, both signed and unsigned).
	 * @return true for integer numeric arrays, false otherwise.
	 */
	inline bool isInteger() const
	{
		return (
			isUint8() || isInt8() || isUint16() || isInt16() || isUint32() ||
			isInt32() || isUint64() || isInt64());
	}
	/** Determine whether array represents data as floating-point numbers,
	 * both single and double precision.
	 * @return true for floating-point numeric arrays, false otherwise.
	 */
	inline bool isFloat() const { return (isDouble() || isSingle()); }
	/** Determine whether a struct array has a specified field.
	 * @param fieldName name of field to check
	 * @return true if struct array has specified field, false otherwise.
	 */
	inline bool isField(const std::string& fieldName) const
	{
		return isStruct() && mxGetFieldNumber(p_, fieldName.c_str()) != -1;
	}
	/** Template for numeric array element accessor.
	 * @param index linear index of the array element.
	 * @return value of the element at specified index.
	 *
	 * This getter method is intended for accessing elements of primitive
	 * types of a numeric array. Use an appropriate specialized/overloaded
	 * method version for accessing elements of struct or cell arrays.
	 *
	 * In MATLAB, this is equivalent to getting:
	 * @code
	 * value = arr(index)
	 * @endcode
	 *
	 * Example:
	 * @code
	 * MxArray m(prhs[0]);
	 * double d = m.at<double>(0);
	 * @endcode
	 */
	template <typename T>
	T at(mwIndex index) const;
	/** Template for numeric array element accessor.
	 * @param i index of the first dimension.
	 * @param j index of the second dimension.
	 * @return value of the element at (i,j) subscript.
	 *
	 * This getter method is intended for accessing elements of primitive
	 * types of a numeric array.
	 *
	 * In MATLAB, this is equivalent to getting:
	 * @code
	 * value = arr(i,j)
	 * @endcode
	 */
	template <typename T>
	T at(mwIndex i, mwIndex j) const;
	/** Template for numeric array element accessor.
	 * @param si vector of subscripts for each dimension of the array.
	 * @return value of the element at subscript index.
	 *
	 * This getter method is intended for accessing elements of primitive
	 * types of a numeric array.
	 *
	 * In MATLAB, this is equivalent to getting:
	 * @code
	 * value = arr(dim1Sub, dim2Sub, dim3Sub, ...)
	 * @endcode
	 */
	template <typename T>
	T at(const std::vector<mwIndex>& si) const;
	/** Struct array element accessor.
	 * @param fieldName name of field in the structure.
	 * @param index linear index of the struct array element.
	 * @return MxArray of the field at the specified index.
	 *
	 * In MATLAB, this is equivalent to getting:
	 * @code
	 * value = st(index).fieldname
	 * @endcode
	 *
	 * Example:
	 * @code
	 * MxArray structArray(prhs[0]);
	 * MxArray x = structArray.at<MxArray>("some_field");
	 * @endcode
	 */
	MxArray at(const std::string& fieldName, mwIndex index = 0) const;
	/** Template for numeric array element write accessor.
	 * @param index linear index of the array element.
	 * @param value value to assign to the element at the specified index.
	 *
	 * (If the array is a cell array, \p value type must be convertible to
	 * MxArray using an existing constructor.)
	 *
	 * In MATLAB, this is equivalent to setting:
	 * @code
	 * arr(index) = value
	 * @endcode
	 *
	 * Example:
	 * @code
	 * MxArray arr(prhs[0]);
	 * arr.set<double>(0, 3.14);
	 * @endcode
	 */
	template <typename T>
	void set(mwIndex index, const T& value);
	/** Template for numeric array element write accessor.
	 * @param i index of the first dimension of the array element.
	 * @param j index of the first dimension of the array element.
	 * @param value value to assign to the element at the specified index.
	 *
	 * In MATLAB, this is equivalent to setting:
	 * @code
	 * arr(i,j) = value
	 * @endcode
	 */
	template <typename T>
	void set(mwIndex i, mwIndex j, const T& value);
	/** Template for numeric array element write accessor.
	 * @param si vector of subscripts for each dimension of the array.
	 * @param value value to assign to the element at the specified subscript.
	 *
	 * In MATLAB, this is equivalent to setting:
	 * @code
	 * arr(dim1Sub, dim2Sub, dim3Sub, ...) = value
	 * @endcode
	 */
	template <typename T>
	void set(const std::vector<mwIndex>& si, const T& value);
	/** Template for struct array element write accessor.
	 * @param fieldName name of field in the structure.
	 * @param value value to assign to the field.
	 * @param index linear index of the struct array element.
	 *
	 * The value type must be convertible to MxArray using an existing
	 * constructor.
	 *
	 * In MATLAB, this is equivalent to setting:
	 * @code
	 * st(index).fieldname = value
	 * @endcode
	 *
	 * Example:
	 * @code
	 * MxArray structArray = MxArray::Struct();
	 * structArray.set<double>("some_field", 3.14);
	 * @endcode
	 */
	template <typename T>
	void set(const std::string& fieldName, const T& value, mwIndex index = 0);
	/** Determine whether input is NaN (Not-a-Number).
	 * @param d double-precision floating-point number
	 * @return true if value is \c NaN, false otherwise.
	 */
	static inline bool isNaN(double d) { return mxIsNaN(d); }
	/** Value of infinity.
	 * @return double-precision value representing infinity.
	 */
	static inline double Inf() { return mxGetInf(); }
	/** Value of \c NaN (Not-a-Number).
	 * @return double-precision value representing \c NaN.
	 */
	static inline double NaN() { return mxGetNaN(); }
	/** Value of \c EPS.
	 * @return double-precision value representing MATLAB \c eps.
	 *
	 * This variable holds the distance from 1.0 to the next largest
	 * floating-point number. As such, it is a measure of floating-point
	 * accuracy.
	 */
	static inline double Eps() { return mxGetEps(); }

   private:
	/** Internal converter from std::vector to MxArray.
	 * @param v vector of type \c T.
	 * @return MxArray object, representing a numeric or a cell array.
	 *
	 * Vectors of primitive types are converted to numeric arrays of the
	 * equivalent MATLAB type, while other types are converted to cell arrays
	 * (assuming an appropriate constructor exists that converts each element
	 * of type \c T to MxArray).
	 */
	template <typename T>
	void fromVector(const std::vector<T>& v);
	/** const pointer to the mxArray opaque object.
	 */
	const mxArray* p_;
};

/** std::map wrapper with one-line initialization and lookup method.
 *
 * Initialization:
 * @code
 * const ConstMap<std::string,int> BorderType = ConstMap<std::string,int>
 *     ("Replicate",  cv::BORDER_REPLICATE)
 *     ("Constant",   cv::BORDER_CONSTANT)
 *     ("Reflect",    cv::BORDER_REFLECT);
 * @endcode
 *
 * Lookup:
 * @code
 * BorderType["Constant"] // => cv::BORDER_CONSTANT
 * @endcode
 */
template <typename T, typename U>
class ConstMap
{
   public:
	/// Constructor with a single key-value pair
	ConstMap(const T& key, const U& val) { m_[key] = val; }
	/// Consecutive insertion operator
	ConstMap<T, U>& operator()(const T& key, const U& val)
	{
		m_[key] = val;
		return *this;
	}
	/// Implicit converter to std::map
	operator std::map<T, U>() const { return m_; }
	/// Lookup operator; fail if not found
	const U& operator[](const T& key) const
	{
		typename std::map<T, U>::const_iterator it = m_.find(key);
		if (it == m_.end())
			mexErrMsgIdAndTxt("mexopencv:error", "ConstMap: Value not found");
		return it->second;
	}

   private:
	/// private map object
	std::map<T, U> m_;
};

template <typename T>
void MxArray::fromVector(const std::vector<T>& v)
{
	if (MxTypes<T>::type == mxUNKNOWN_CLASS)
	{
		p_ = mxCreateCellMatrix(1, v.size());
		if (!p_) mexErrMsgIdAndTxt("mexopencv:error", "Allocation error");
		for (mwIndex i = 0; i < v.size(); ++i)
			mxSetCell(const_cast<mxArray*>(p_), i, MxArray(v[i]));
	}
	else
	{
		p_ = mxCreateNumericMatrix(1, v.size(), MxTypes<T>::type, mxREAL);
		if (!p_) mexErrMsgIdAndTxt("mexopencv:error", "Allocation error");
		std::copy(v.begin(), v.end(), reinterpret_cast<T*>(mxGetData(p_)));
	}
}

template <typename T>
MxArray::MxArray(const cv::Point_<T>& p)
	: p_(mxCreateNumericMatrix(1, 2, mxDOUBLE_CLASS, mxREAL))
{
	if (!p_) mexErrMsgIdAndTxt("mexopencv:error", "Allocation error");
	double* x = mxGetPr(p_);
	x[0] = static_cast<double>(p.x);
	x[1] = static_cast<double>(p.y);
}

template <typename T>
MxArray::MxArray(const cv::Point3_<T>& p)
	: p_(mxCreateNumericMatrix(1, 3, mxDOUBLE_CLASS, mxREAL))
{
	if (!p_) mexErrMsgIdAndTxt("mexopencv:error", "Allocation error");
	double* x = mxGetPr(p_);
	x[0] = static_cast<double>(p.x);
	x[1] = static_cast<double>(p.y);
	x[2] = static_cast<double>(p.z);
}

template <typename T>
MxArray::MxArray(const cv::Size_<T>& s)
	: p_(mxCreateNumericMatrix(1, 2, mxDOUBLE_CLASS, mxREAL))
{
	if (!p_) mexErrMsgIdAndTxt("mexopencv:error", "Allocation error");
	double* x = mxGetPr(p_);
	x[0] = static_cast<double>(s.width);
	x[1] = static_cast<double>(s.height);
}

template <typename T>
MxArray::MxArray(const cv::Rect_<T>& r)
	: p_(mxCreateNumericMatrix(1, 4, mxDOUBLE_CLASS, mxREAL))
{
	if (!p_) mexErrMsgIdAndTxt("mexopencv:error", "Allocation error");
	double* x = mxGetPr(p_);
	x[0] = static_cast<double>(r.x);
	x[1] = static_cast<double>(r.y);
	x[2] = static_cast<double>(r.width);
	x[3] = static_cast<double>(r.height);
}

template <typename T>
MxArray::MxArray(const cv::Scalar_<T>& s)
	: p_(mxCreateNumericMatrix(1, 4, mxDOUBLE_CLASS, mxREAL))
{
	if (!p_) mexErrMsgIdAndTxt("mexopencv:error", "Allocation error");
	double* x = mxGetPr(p_);
	x[0] = static_cast<double>(s[0]);
	x[1] = static_cast<double>(s[1]);
	x[2] = static_cast<double>(s[2]);
	x[3] = static_cast<double>(s[3]);
}

template <typename T, int cn>
MxArray::MxArray(const cv::Vec<T, cn>& vec)
	: p_(mxCreateNumericMatrix(1, cn, mxDOUBLE_CLASS, mxREAL))
{
	if (!p_) mexErrMsgIdAndTxt("mexopencv:error", "Allocation error");
	/*
	double *x = mxGetPr(p_);
	for (mwIndex i=0; i<cn; i++)
		//set<double>(i, static_cast<double>(vec[i]));
		x[i] = static_cast<double>(vec[i]);
	*/
	std::copy(vec.val, vec.val + cn, mxGetPr(p_));
}

template <typename T, int m, int n>
MxArray::MxArray(const cv::Matx<T, m, n>& mat)
	: p_(mxCreateNumericMatrix(m, n, mxDOUBLE_CLASS, mxREAL))
{
	if (!p_) mexErrMsgIdAndTxt("mexopencv:error", "Allocation error");
	/*
	double *x = mxGetPr(p_);
	for (mwIndex j=0; j<n; j++)
	   for (mwIndex i=0; i<m; i++)
		   //set<double>(i, j, static_cast<double>(mat(i,j)));
		   x[j*m+i] = static_cast<double>(mat(i,j));
	*/
	// Note: C is row-major, MATLAB uses column-major order
	const cv::Matx<T, n, m> mat_t = mat.t();
	std::copy(mat_t.val, mat_t.val + m * n, mxGetPr(p_));
}

template <typename T>
cv::Point_<T> MxArray::toPoint_() const
{
	if (!isNumeric() || numel() != 2)
		mexErrMsgIdAndTxt("mexopencv:error", "MxArray is not a cv::Point");
	return cv::Point_<T>(at<T>(0), at<T>(1));
}

template <typename T>
cv::Point3_<T> MxArray::toPoint3_() const
{
	if (!isNumeric() || numel() != 3)
		mexErrMsgIdAndTxt("mexopencv:error", "MxArray is not a cv::Point3");
	return cv::Point3_<T>(at<T>(0), at<T>(1), at<T>(2));
}

template <typename T>
cv::Size_<T> MxArray::toSize_() const
{
	if (!isNumeric() || numel() != 2)
		mexErrMsgIdAndTxt("mexopencv:error", "MxArray is not a cv::Size");
	return cv::Size_<T>(at<T>(0), at<T>(1));
}

template <typename T>
cv::Rect_<T> MxArray::toRect_() const
{
	if (!isNumeric() || numel() != 4)
		mexErrMsgIdAndTxt("mexopencv:error", "MxArray is not a cv::Rect");
	return cv::Rect_<T>(at<T>(0), at<T>(1), at<T>(2), at<T>(3));
}

template <typename T>
cv::Scalar_<T> MxArray::toScalar_() const
{
	const mwSize n = numel();
	if (!isNumeric() || n < 1 || 4 < n)
		mexErrMsgIdAndTxt("mexopencv:error", "MxArray is not a cv::Scalar");
	switch (n)
	{
		case 1:
			return cv::Scalar_<T>(at<T>(0));
		case 2:
			return cv::Scalar_<T>(at<T>(0), at<T>(1));
		case 3:
			return cv::Scalar_<T>(at<T>(0), at<T>(1), at<T>(2));
		case 4:
			return cv::Scalar_<T>(at<T>(0), at<T>(1), at<T>(2), at<T>(3));
		default:
			return cv::Scalar_<T>();
	}
}

template <typename T, int cn>
cv::Vec<T, cn> MxArray::toVec() const
{
	if (!isNumeric() || numel() != cn)
		mexErrMsgIdAndTxt("mexopencv:error", "MxArray is not a cv::Vec%d", cn);
	/*
	std::vector<T> v(toVector<T>());
	return (!v.empty() && v.size() == cn) ?
		cv::Vec<T,cn>(&v[0]) : cv::Vec<T,cn>();
	*/
	cv::Vec<T, cn> vec;
	for (mwIndex i = 0; i < cn; i++) vec[i] = at<T>(i);
	return vec;
}

template <typename T, int m, int n>
cv::Matx<T, m, n> MxArray::toMatx() const
{
	if (!isNumeric() || numel() != m * n || rows() != m || cols() != n)
		mexErrMsgIdAndTxt(
			"mexopencv:error", "MxArray is not a cv::Matx%d%d", m, n);
	/*
	// C is row-major, MATLAB is column-major order
	std::vector<T> v(toVector<T>());
	return (!v.empty() && v.size() == m*n) ?
		cv::Matx<T,n,m>(&v[0]).t() : cv::Matx<T,m,n>();
	*/
	cv::Matx<T, m, n> mat;
	for (mwIndex j = 0; j < n; j++)
		for (mwIndex i = 0; i < m; i++)
			// mat(i,j) = at<T>(j*m+i);
			mat(i, j) = at<T>(i, j);
	return mat;
}

template <typename T>
std::vector<T> MxArray::toVector() const
{
	const mwSize n = numel();
	std::vector<T> vt;
	vt.reserve(n);
	if (isNumeric() || isLogical() || isChar())
	{
		/*
		// shorter but slower implementation
		for (mwIndex i = 0; i < n; ++i)
			vt.push_back(at<T>(i));
		*/
		switch (classID())
		{
			case mxDOUBLE_CLASS:
			{
				const double* data = mxGetPr(p_);
				// std::copy(data, data + n, vt.begin());
				vt.assign(data, data + n);
				break;
			}
			case mxSINGLE_CLASS:
			{
				const float* data = reinterpret_cast<float*>(mxGetData(p_));
				vt.assign(data, data + n);
				break;
			}
			case mxUINT8_CLASS:
			{
				const uint8_t* data = reinterpret_cast<uint8_t*>(mxGetData(p_));
				vt.assign(data, data + n);
				break;
			}
			case mxINT8_CLASS:
			{
				const int8_t* data = reinterpret_cast<int8_t*>(mxGetData(p_));
				vt.assign(data, data + n);
				break;
			}
			case mxUINT16_CLASS:
			{
				const uint16_t* data =
					reinterpret_cast<uint16_t*>(mxGetData(p_));
				vt.assign(data, data + n);
				break;
			}
			case mxINT16_CLASS:
			{
				const int16_t* data = reinterpret_cast<int16_t*>(mxGetData(p_));
				vt.assign(data, data + n);
				break;
			}
			case mxUINT32_CLASS:
			{
				const uint32_t* data =
					reinterpret_cast<uint32_t*>(mxGetData(p_));
				vt.assign(data, data + n);
				break;
			}
			case mxINT32_CLASS:
			{
				const int32_t* data = reinterpret_cast<int32_t*>(mxGetData(p_));
				vt.assign(data, data + n);
				break;
			}
			case mxUINT64_CLASS:
			{
				const uint64_t* data =
					reinterpret_cast<uint64_t*>(mxGetData(p_));
				vt.assign(data, data + n);
				break;
			}
			case mxINT64_CLASS:
			{
				const int64_t* data = reinterpret_cast<int64_t*>(mxGetData(p_));
				vt.assign(data, data + n);
				break;
			}
			case mxCHAR_CLASS:
			{
				const mxChar* data = mxGetChars(p_);
				vt.assign(data, data + n);
				break;
			}
			case mxLOGICAL_CLASS:
			{
				const mxLogical* data = mxGetLogicals(p_);
				vt.assign(data, data + n);
				break;
			}
			default:
				break;  // should never reach this case
		}
	}
	else if (isCell())
	{
		for (mwIndex i = 0; i < n; ++i)
			// vt.push_back(at<MxArray>(i).at<T>(0));
			vt.push_back(MxArray(mxGetCell(p_, i)).at<T>(0));
	}
	else
		mexErrMsgIdAndTxt("mexopencv:error", "MxArray is not a std::vector");
	return vt;
}

#if 0
template <typename T>
std::vector<T> MxArray::toVector(std::const_mem_fun_ref_t<T, MxArray> f) const
{
	const std::vector<MxArray> v(toVector<MxArray>());
	std::vector<T> vt;
	vt.reserve(v.size());
	for (std::vector<MxArray>::const_iterator it = v.begin(); it != v.end();
		 ++it)
		vt.push_back(f(*it));
	return vt;
}
#endif

template <typename T>
T MxArray::at(mwIndex index) const
{
	if (!p_) mexErrMsgIdAndTxt("mexopencv:error", "Null pointer error");
	if (!isNumeric() && !isLogical() && !isChar())
		mexErrMsgIdAndTxt("mexopencv:error", "MxArray is not primitive");
	if (numel() <= index)
		mexErrMsgIdAndTxt("mexopencv:error", "Index out of range");
	switch (classID())
	{
		case mxCHAR_CLASS:
			return static_cast<T>(*(mxGetChars(p_) + index));
		case mxDOUBLE_CLASS:
			return static_cast<T>(*(mxGetPr(p_) + index));
		case mxINT8_CLASS:
			return static_cast<T>(
				*(reinterpret_cast<int8_t*>(mxGetData(p_)) + index));
		case mxUINT8_CLASS:
			return static_cast<T>(
				*(reinterpret_cast<uint8_t*>(mxGetData(p_)) + index));
		case mxINT16_CLASS:
			return static_cast<T>(
				*(reinterpret_cast<int16_t*>(mxGetData(p_)) + index));
		case mxUINT16_CLASS:
			return static_cast<T>(
				*(reinterpret_cast<uint16_t*>(mxGetData(p_)) + index));
		case mxINT32_CLASS:
			return static_cast<T>(
				*(reinterpret_cast<int32_t*>(mxGetData(p_)) + index));
		case mxUINT32_CLASS:
			return static_cast<T>(
				*(reinterpret_cast<uint32_t*>(mxGetData(p_)) + index));
		case mxINT64_CLASS:
			return static_cast<T>(
				*(reinterpret_cast<int64_t*>(mxGetData(p_)) + index));
		case mxUINT64_CLASS:
			return static_cast<T>(
				*(reinterpret_cast<uint64_t*>(mxGetData(p_)) + index));
		case mxSINGLE_CLASS:
			return static_cast<T>(
				*(reinterpret_cast<float*>(mxGetData(p_)) + index));
		case mxLOGICAL_CLASS:
			return static_cast<T>(*(mxGetLogicals(p_) + index));
		default:
			return static_cast<T>(0);  // should never reach this case
	}
}

template <typename T>
T MxArray::at(mwIndex i, mwIndex j) const
{
	return at<T>(subs(i, j));
}

template <typename T>
T MxArray::at(const std::vector<mwIndex>& si) const
{
	return at<T>(subs(si));
}

template <typename T>
void MxArray::set(mwIndex index, const T& value)
{
	if (!p_) mexErrMsgIdAndTxt("mexopencv:error", "Null pointer error");
	if (!isNumeric() && !isLogical() && !isChar())
		mexErrMsgIdAndTxt("mexopencv:error", "MxArray is not primitive");
	if (numel() <= index)
		mexErrMsgIdAndTxt("mexopencv:error", "Index out of range");
	switch (classID())
	{
		case mxCHAR_CLASS:
			*(mxGetChars(p_) + index) = static_cast<mxChar>(value);
			break;
		case mxDOUBLE_CLASS:
			*(mxGetPr(p_) + index) = static_cast<double>(value);
			break;
		case mxINT8_CLASS:
			*(reinterpret_cast<int8_t*>(mxGetData(p_)) + index) =
				static_cast<int8_t>(value);
			break;
		case mxUINT8_CLASS:
			*(reinterpret_cast<uint8_t*>(mxGetData(p_)) + index) =
				static_cast<uint8_t>(value);
			break;
		case mxINT16_CLASS:
			*(reinterpret_cast<int16_t*>(mxGetData(p_)) + index) =
				static_cast<int16_t>(value);
			break;
		case mxUINT16_CLASS:
			*(reinterpret_cast<uint16_t*>(mxGetData(p_)) + index) =
				static_cast<uint16_t>(value);
			break;
		case mxINT32_CLASS:
			*(reinterpret_cast<int32_t*>(mxGetData(p_)) + index) =
				static_cast<int32_t>(value);
			break;
		case mxUINT32_CLASS:
			*(reinterpret_cast<uint32_t*>(mxGetData(p_)) + index) =
				static_cast<uint32_t>(value);
			break;
		case mxINT64_CLASS:
			*(reinterpret_cast<int64_t*>(mxGetData(p_)) + index) =
				static_cast<int64_t>(value);
			break;
		case mxUINT64_CLASS:
			*(reinterpret_cast<uint64_t*>(mxGetData(p_)) + index) =
				static_cast<uint64_t>(value);
			break;
		case mxSINGLE_CLASS:
			*(reinterpret_cast<float*>(mxGetData(p_)) + index) =
				static_cast<float>(value);
			break;
		case mxLOGICAL_CLASS:
			*(mxGetLogicals(p_) + index) = static_cast<mxLogical>(value);
			break;
		default:
			break;  // should never reach this case
	}
}

template <typename T>
void MxArray::set(mwIndex i, mwIndex j, const T& value)
{
	set<T>(subs(i, j), value);
}

template <typename T>
void MxArray::set(const std::vector<mwIndex>& si, const T& value)
{
	set<T>(subs(si), value);
}

template <typename T>
void MxArray::set(const std::string& fieldName, const T& value, mwIndex index)
{
	if (!isStruct())
		mexErrMsgIdAndTxt("mexopencv:error", "MxArray is not struct");
	if (numel() <= index)
		mexErrMsgIdAndTxt("mexopencv:error", "Index out of range");
	if (!isField(fieldName))
	{
		if (mxAddField(const_cast<mxArray*>(p_), fieldName.c_str()) < 0)
			mexErrMsgIdAndTxt(
				"mexopencv:error", "Failed to create a field '%s'",
				fieldName.c_str());
	}
	mxSetField(
		const_cast<mxArray*>(p_), index, fieldName.c_str(),
		static_cast<mxArray*>(MxArray(value)));
}

/** MxArray specialized constructor from vector<char>.
 * @param v vector of type char.
 * @return a char array MxArray object.
 */
template <>
void MxArray::fromVector(const std::vector<char>& v);

/** MxArray specialized constructor from vector<bool>.
 * @param v vector of type bool.
 * @return a logical array MxArray object.
 */
template <>
void MxArray::fromVector(const std::vector<bool>& v);

/** MxArray specialized constructor from vector<DMatch>.
 * @param v vector of type DMatch.
 * @return a struct array MxArray object.
 */
template <>
MxArray::MxArray(const std::vector<cv::DMatch>& v);

/** MxArray specialized constructor from vector<KeyPoint>.
 * @param v vector of type KeyPoint.
 * @return a struct array MxArray object.
 */
template <>
MxArray::MxArray(const std::vector<cv::KeyPoint>& v);

/** MxArray specialized constructor from vector<RotatedRect>.
 * @param v vector of type RotatedRect.
 * @return a struct array MxArray object.
 */
template <>
MxArray::MxArray(const std::vector<cv::RotatedRect>& v);

/** Cell array element accessor.
 * @param index linear index of the cell array element.
 * @return MxArray of the element at the specified index.
 *
 * Example:
 * @code
 * MxArray cellArray(prhs[0]);
 * MxArray m = cellArray.at<MxArray>(0);
 * @endcode
 */
template <>
MxArray MxArray::at(mwIndex index) const;

/** Cell array element write accessor.
 * @param index linear index of the cell array element.
 * @param value MxArray to assign to the element at the specified index.
 *
 * Example:
 * @code
 * MxArray cellArray = MxArray::Cell(1,2);
 * cellArray.set<MxArray>(0, MxArray(3.14));
 * cellArray.set<MxArray>(1, MxArray(std::string("hello")));
 * @endcode
 */
template <>
void MxArray::set(mwIndex index, const MxArray& value);

/** Convert MxArray to std::vector<MxArray>.
 * @return std::vector<MxArray> value.
 *
 * Example:
 * @code
 * MxArray cellArray(prhs[0]);
 * vector<MxArray> v = cellArray.toVector<MxArray>();
 * @endcode
 */
template <>
std::vector<MxArray> MxArray::toVector() const;

/** Convert MxArray to std::vector<std::string>.
 * @return std::vector<std::string> value.
 *
 * The input MxArray is expected to be a cell-array of strings, e.g:
 * <tt>{'str1', 'str2', ...}</tt>, or a single string <tt>'str'</tt>.
 *
 * Example:
 * @code
 * MxArray cellArray(prhs[0]);
 * vector<string> v = cellArray.toVector<string>();
 * @endcode
 */
template <>
std::vector<std::string> MxArray::toVector() const;

/** Convert MxArray to std::vector<cv::Mat>.
 * @return std::vector<cv::Mat> value.
 *
 * The input MxArray can be either:
 * - a cell-array of matrices of length \c N, e.g: <tt>{m1, m2, ...}</tt>
 * - a single numeric matrix (hence \c N=1)
 * .
 * where \c N will be the output vector size.
 *
 * Example:
 * @code
 * MxArray cellArray(prhs[0]);
 * vector<Mat> v = cellArray.toVector<Mat>();
 * @endcode
 */
template <>
std::vector<cv::Mat> MxArray::toVector() const;

/** Convert MxArray to std::vector<Point>.
 * @return std::vector<Point> value.
 *
 * The input MxArray can be either:
 * - a cell-array of 2D integer points (2-element vectors) of length \c N,
 *   e.g: <tt>{[x,y], [x,y], ...}</tt>
 * - a numeric matrix of size \c Nx2, \c Nx1x2, or \c 1xNx2 in the form:
 *   <tt>[x,y; x,y; ...]</tt> or <tt>cat(3, [x,x,...], [y,y,...])</tt>
 * .
 * where \c N will be the output vector size.
 *
 * Example:
 * @code
 * MxArray cellArray(prhs[0]);
 * vector<Point> v = cellArray.toVector<Point>();
 * @endcode
 */
template <>
std::vector<cv::Point> MxArray::toVector() const;

/** Convert MxArray to std::vector<Point2f>.
 * @return std::vector<Point2f> value.
 *
 * The input MxArray can be either:
 * - a cell-array of 2D float points (2-element vectors) of length \c N, e.g:
 *   <tt>{[x,y], [x,y], ...}</tt>
 * - a numeric matrix of size \c Nx2, \c Nx1x2, or \c 1xNx2 in the form:
 *   <tt>[x,y; x,y; ...]</tt> or <tt>cat(3, [x,x,...], [y,y,...])</tt>
 * .
 * where \c N will be the output vector size.
 *
 * Example:
 * @code
 * MxArray cellArray(prhs[0]);
 * vector<Point2f> v = cellArray.toVector<Point2f>();
 * @endcode
 */
template <>
std::vector<cv::Point2f> MxArray::toVector() const;

/** Convert MxArray to std::vector<Point2d>.
 * @return std::vector<Point2d> value.
 *
 * The input MxArray can be either:
 * - a cell-array of 2D double points (2-element vectors) of length \c N, e.g:
 *   <tt>{[x,y], [x,y], ...}</tt>
 * - a numeric matrix of size \c Nx2, \c Nx1x2, or \c 1xNx2 in the form:
 *   <tt>[x,y; x,y; ...]</tt> or <tt>cat(3, [x,x,...], [y,y,...])</tt>
 * .
 * where \c N will be the output vector size.
 *
 * Example:
 * @code
 * MxArray cellArray(prhs[0]);
 * vector<Point2d> v = cellArray.toVector<Point2d>();
 * @endcode
 */
template <>
std::vector<cv::Point2d> MxArray::toVector() const;

/** Convert MxArray to std::vector<Point3i>.
 * @return std::vector<Point3i> value.
 *
 * The input MxArray can be either:
 * - a cell-array of 3D integer points (3-element vectors) of length \c N, e.g:
 *   <tt>{[x,y,z], [x,y,z], ...}</tt>
 * - a numeric matrix of size \c Nx3, \c Nx1x3, or \c 1xNx3 in the form:
 *   <tt>[x,y,z; x,y,z; ...]</tt> or
 *   <tt>cat(3, [x,x,...], [y,y,...], [z,z,...])</tt>
 * .
 * where \c N will be the output vector size.
 *
 * Example:
 * @code
 * MxArray cellArray(prhs[0]);
 * vector<Point3i> v = cellArray.toVector<Point3i>();
 * @endcode
 */
template <>
std::vector<cv::Point3i> MxArray::toVector() const;

/** Convert MxArray to std::vector<Point3f>.
 * @return std::vector<Point3f> value.
 *
 * The input MxArray can be either:
 * - a cell-array of 3D float points (3-element vectors) of length \c N, e.g:
 *   <tt>{[x,y,z], [x,y,z], ...}</tt>
 * - a numeric matrix of size \c Nx3, \c Nx1x3, or \c 1xNx3 in the form:
 *   <tt>[x,y,z; x,y,z; ...]</tt> or
 *   <tt>cat(3, [x,x,...], [y,y,...], [z,z,...])</tt>
 * .
 * where \c N will be the output vector size.
 *
 * Example:
 * @code
 * MxArray cellArray(prhs[0]);
 * vector<Point3f> v = cellArray.toVector<Point3f>();
 * @endcode
 */
template <>
std::vector<cv::Point3f> MxArray::toVector() const;

/** Convert MxArray to std::vector<Point3d>.
 * @return std::vector<Point3d> value.
 *
 * The input MxArray can be either:
 * - a cell-array of 3D double points (3-element vectors) of length \c N, e.g:
 *   <tt>{[x,y,z], [x,y,z], ...}</tt>
 * - a numeric matrix of size \c Nx3, \c Nx1x3, or \c 1xNx3 in the form:
 *   <tt>[x,y,z; x,y,z; ...]</tt> or
 *   <tt>cat(3, [x,x,...], [y,y,...], [z,z,...])</tt>
 * .
 * where \c N will be the output vector size.
 *
 * Example:
 * @code
 * MxArray cellArray(prhs[0]);
 * vector<Point3d> v = cellArray.toVector<Point3d>();
 * @endcode
 */
template <>
std::vector<cv::Point3d> MxArray::toVector() const;

/** Convert MxArray to std::vector<Size>.
 * @return std::vector<Size> value.
 *
 * The input MxArray can be either:
 * - a cell-array of sizes (2-element vectors) of length \c N, e.g:
 *   <tt>{[w,h], [w,h], ...}</tt>
 * - a numeric matrix of size \c Nx2, \c Nx1x2, or \c 1xNx2 in the form:
 *   <tt>[w,h; w,h; ...]</tt> or <tt>cat(3, [w,w,...], [h,h,...])</tt>
 * .
 * where \c N will be the output vector size.
 *
 * Example:
 * @code
 * MxArray cellArray(prhs[0]);
 * vector<Size> v = cellArray.toVector<Size>();
 * @endcode
 */
template <>
std::vector<cv::Size> MxArray::toVector() const;

/** Convert MxArray to std::vector<Rect>.
 * @return std::vector<Rect> value.
 *
 * The input MxArray can be either:
 * - a cell-array of rectangles (4-element vectors) of length \c N, e.g:
 *   <tt>{[x,y,w,h], [x,y,w,h], ...}</tt>
 * - a numeric matrix of size \c Nx4, \c Nx1x4, or \c 1xNx4 in the form:
 *   <tt>[x,y,w,h; x,y,w,h; ...]</tt> or
 *   <tt>cat(3, [x,x,...], [y,y,...] ,[w,w,...], [h,h,...])</tt>
 * .
 * where \c N will be the output vector size.
 *
 * Example:
 * @code
 * MxArray cellArray(prhs[0]);
 * vector<Rect> v = cellArray.toVector<Rect>();
 * @endcode
 */
template <>
std::vector<cv::Rect> MxArray::toVector() const;

/** Convert MxArray to std::vector<Vec2i>.
 * @return std::vector<Vec2i> value.
 *
 * The input MxArray can be either:
 * - a cell-array of 2-element vectors of length \c N, e.g:
 *   <tt>{[v1,v2], [v1,v2], ...}</tt>
 * - a numeric matrix of size \c Nx2, \c Nx1x2, or \c 1xNx2 in the form:
 *   <tt>[v1,v2; v1,v2; ...]</tt> or <tt>cat(3, [v1,v1,...], [v2,v2,...])</tt>
 * .
 * where \c N will be the output vector size.
 *
 * Example:
 * @code
 * MxArray cellArray(prhs[0]);
 * vector<Vec2i> v = cellArray.toVector<Vec2i>();
 * @endcode
 */
template <>
std::vector<cv::Vec2i> MxArray::toVector() const;

/** Convert MxArray to std::vector<Vec2f>.
 * @return std::vector<Vec2f> value.
 *
 * The input MxArray can be either:
 * - a cell-array of 2-element vectors of length \c N, e.g:
 *   <tt>{[v1,v2], [v1,v2], ...}</tt>
 * - a numeric matrix of size \c Nx2, \c Nx1x2, or \c 1xNx2 in the form:
 *   <tt>[v1,v2; v1,v2; ...]</tt> or <tt>cat(3, [v1,v1,...], [v2,v2,...])</tt>
 * .
 * where \c N will be the output vector size.
 *
 * Example:
 * @code
 * MxArray cellArray(prhs[0]);
 * vector<Vec2f> v = cellArray.toVector<Vec2f>();
 * @endcode
 */
template <>
std::vector<cv::Vec2f> MxArray::toVector() const;

/** Convert MxArray to std::vector<Vec3i>.
 * @return std::vector<Vec3i> value.
 *
 * The input MxArray can be either:
 * - a cell-array of 3-element vectors of length \c N, e.g:
 *   <tt>{[v1,v2,v3], [v1,v2,v3], ...}</tt>
 * - a numeric matrix of size \c Nx3, \c Nx1x3, or \c 1xNx3 in the form:
 *   <tt>[v1,v2,v3; v1,v2,v3; ...]</tt> or
 *   <tt>cat(3, [v1,v1,...], [v2,v2,...], [v3,v3,...])</tt>
 * .
 * where \c N will be the output vector size.
 *
 * Example:
 * @code
 * MxArray cellArray(prhs[0]);
 * vector<Vec3i> v = cellArray.toVector<Vec3i>();
 * @endcode
 */
template <>
std::vector<cv::Vec3i> MxArray::toVector() const;

/** Convert MxArray to std::vector<Vec3f>.
 * @return std::vector<Vec3f> value.
 *
 * The input MxArray can be either:
 * - a cell-array of 3-element vectors of length \c N, e.g:
 *   <tt>{[v1,v2,v3], [v1,v2,v3], ...}</tt>
 * - a numeric matrix of size \c Nx3, \c Nx1x3, or \c 1xNx3 in the form:
 *   <tt>[v1,v2,v3; v1,v2,v3; ...]</tt> or
 *   <tt>cat(3, [v1,v1,...], [v2,v2,...], [v3,v3,...])</tt>
 * .
 * where \c N will be the output vector size.
 *
 * Example:
 * @code
 * MxArray cellArray(prhs[0]);
 * vector<Vec3f> v = cellArray.toVector<Vec3f>();
 * @endcode
 */
template <>
std::vector<cv::Vec3f> MxArray::toVector() const;

/** Convert MxArray to std::vector<Vec4i>.
 * @return std::vector<Vec4i> value.
 *
 * The input MxArray can be either:
 * - a cell-array of 4-element vectors of length \c N, e.g:
 *   <tt>{[v1,v2,v3,v4], [v1,v2,v3,v4], ...}</tt>
 * - a numeric matrix of size \c Nx4, \c Nx1x4, or \c 1xNx4 in the form:
 *   <tt>[v1,v2,v3,v4; v1,v2,v3,v4; ...]</tt> or
 *   <tt>cat(3, [v1,v1,...], [v2,v2,...], [v3,v3,...], [v4,v4,...])</tt>
 * .
 * where \c N will be the output vector size.
 *
 * Example:
 * @code
 * MxArray cellArray(prhs[0]);
 * vector<Vec4i> v = cellArray.toVector<Vec4i>();
 * @endcode
 */
template <>
std::vector<cv::Vec4i> MxArray::toVector() const;

/** Convert MxArray to std::vector<Vec4f>.
 * @return std::vector<Vec4f> value.
 *
 * The input MxArray can be either:
 * - a cell-array of 4-element vectors of length \c N, e.g:
 *   <tt>{[v1,v2,v3,v4], [v1,v2,v3,v4], ...}</tt>
 * - a numeric matrix of size \c Nx4, \c Nx1x4, or \c 1xNx4 in the form:
 *   <tt>[v1,v2,v3,v4; v1,v2,v3,v4; ...]</tt> or
 *   <tt>cat(3, [v1,v1,...], [v2,v2,...], [v3,v3,...], [v4,v4,...])</tt>
 * .
 * where \c N will be the output vector size.
 *
 * Example:
 * @code
 * MxArray cellArray(prhs[0]);
 * vector<Vec4f> v = cellArray.toVector<Vec4f>();
 * @endcode
 */
template <>
std::vector<cv::Vec4f> MxArray::toVector() const;

/** Convert MxArray to std::vector<cv::RotatedRect>.
 * @return std::vector<cv::RotatedRect> value.
 *
 * The input MxArray can be either:
 * - a cell-array of rotated rectangles (scalar structs) of length \c N, e.g:
 *   <tt>{struct('center',[x,y], 'size',[a,b], 'angle',t), ...}</tt>
 * - a structure-array of length \c N, in the form:
 *   <tt>struct('center',{[x,y],...}, 'size',{[a,b],...}, 'angle',{t,...})</tt>
 * .
 * where \c N will be the output vector size.
 *
 * Example:
 * @code
 * MxArray structArray(prhs[0]);
 * vector<RotatedRect> v = structArray.toVector<RotatedRect>();
 * @endcode
 */
template <>
std::vector<cv::RotatedRect> MxArray::toVector() const;

/** Convert MxArray to std::vector<cv::KeyPoint>.
 * @return std::vector<cv::KeyPoint> value.
 *
 * The input MxArray can be either:
 * - a cell-array of keypoints (scalar structs) of length \c N, e.g:
 *   <tt>{struct('pt',[x,y], 'size',[a,b], 'angle',t), ...}</tt>
 * - a structure-array of length \c N, in the form:
 *   <tt>struct('pt',{[x,y],...}, 'size',{[a,b],...}, 'angle',{t,...})</tt>
 * .
 * where \c N will be the output vector size.
 *
 * Example:
 * @code
 * MxArray structArray(prhs[0]);
 * vector<KeyPoint> v = structArray.toVector<KeyPoint>();
 * @endcode
 */
template <>
std::vector<cv::KeyPoint> MxArray::toVector() const;

/** Convert MxArray to std::vector<cv::DMatch>.
 * @return std::vector<cv::DMatch> value.
 *
 * The input MxArray can be either:
 * - a cell-array of dmatches (scalar structs) of length \c N, e.g:
 *   <tt>{struct('queryIdx',i, 'trainIdx',j, 'distance',d), ...}</tt>
 * - a structure-array of length \c N, in the form:
 *   <tt>struct('queryIdx',{i,...}, 'trainIdx',{j,...}, 'distance',{d,...})</tt>
 * .
 * where \c N will be the output vector size.
 *
 * Example:
 * @code
 * MxArray structArray(prhs[0]);
 * vector<DMatch> v = structArray.toVector<DMatch>();
 * @endcode
 */
template <>
std::vector<cv::DMatch> MxArray::toVector() const;

#endif  // MXARRAY_HPP
