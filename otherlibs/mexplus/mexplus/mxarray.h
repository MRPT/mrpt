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

#ifndef __MEXPLUS_MXARRAY_H__
#define __MEXPLUS_MXARRAY_H__

#include <mex.h>
#include <mexplus/mxtypes.h>
#include <stdint.h>
#include <string>
#include <typeinfo>
#include <vector>

#include <mrpt/config.h> // Control MRPT_HAS_... defines
#if MRPT_HAS_OPENCV
#include <mexopencv/mexopencv.hpp>
#endif

/** Macro definitions.
 */
#define MEXPLUS_CHECK_NOTNULL(pointer) \
	if (!(pointer)) \
	mexErrMsgIdAndTxt("mexplus:error", \
	"Null pointer exception: %s:%d:%s `" #pointer "`.", \
	__FILE__, \
	__LINE__, \
	__FUNCTION__)

#define MEXPLUS_ERROR(...) mexErrMsgIdAndTxt("mexplus:error", __VA_ARGS__)
#define MEXPLUS_WARNING(...) mexWarnMsgIdAndTxt("mexplus:warning", __VA_ARGS__)
#define MEXPLUS_ASSERT(condition, ...) \
	if (!(condition)) mexErrMsgIdAndTxt("mexplus:error", __VA_ARGS__)

namespace mexplus {

/** Templated mxArray importers.
*/
template <typename T>
mxArray* fromInternal(const typename std::enable_if<
							 std::is_same<typename MxTypes<T>::array_type, mxNumeric>::value,
							 T>::type& value);
template <typename Container>
mxArray* fromInternal(const typename std::enable_if<
							 std::is_same<typename MxTypes<
							 typename Container::value_type>::array_type, mxNumeric>::value &&
							 std::is_compound<Container>::value,
							 Container>::type& value);
template <typename T>
mxArray* fromInternal(const typename std::enable_if<
							 std::is_same<typename MxTypes<T>::array_type, mxChar>::value,
							 T>::type& value);
template <typename Container>
mxArray* fromInternal(const typename std::enable_if<
							 std::is_same<typename MxTypes<
							 typename Container::value_type>::array_type, mxChar>::value &&
							 std::is_compound<Container>::value,
							 Container>::type& value);
template <typename T>
mxArray* fromInternal(const typename std::enable_if<
							 std::is_same<typename MxTypes<T>::array_type, mxLogical>::value,
							 T>::type& value);
template <typename Container>
mxArray* fromInternal(const typename std::enable_if<
							 std::is_same<typename MxTypes<
							 typename Container::value_type>::array_type, mxLogical>::value &&
							 std::is_compound<Container>::value,
							 Container>::type& value);
template <typename Container>
mxArray* fromInternal(const typename std::enable_if<
							 std::is_same<typename MxTypes<
							 typename Container::value_type>::array_type, mxCell>::value &&
							 std::is_compound<Container>::value,
							 Container>::type& value);

/** mxArray* importer methods.
*/
template <typename T>
mxArray* from(const T& value) { return fromInternal<T>(value); }
inline mxArray* from(const char* value) {
	mxArray* array = mxCreateString(value);
	MEXPLUS_CHECK_NOTNULL(array);
	return array;
}

// TODO: Change inline functions to .cpp file and build own library?
inline mxArray* from(int32_t value) {
	mxArray* array = mxCreateNumericMatrix(1, 1, mxINT32_CLASS, mxREAL);
	MEXPLUS_CHECK_NOTNULL(array);
	*reinterpret_cast<int32_t*>(mxGetData(array)) = value;
	return array;
}

// Extra code for OpenCV classes
#if MRPT_HAS_OPENCV
inline mxArray* from(const cv::Mat& mat)
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
//			void *ptr = reinterpret_cast<void*>(
//						reinterpret_cast<size_t>(mxGetData(p_)) +
//						mxGetElementSize(p_) * this->subscriptIndex(si));
		mwIndex subs_si = mxCalcSingleSubscript(p_, si.size(), &si[0]);
		void *ptr = reinterpret_cast<void*>(
					reinterpret_cast<size_t>(mxGetData(p_)) +
					mxGetElementSize(p_) * subs_si);
		cv::Mat m(input.dims, dims_, type, ptr);
		//channels[i].convertTo(m, type); // Write to mxArray through m.
		// Swap R and B channels
		MRPT_TODO("Do RGB to BGR swapping in other place where it is more clear")
				channels[nchannels-1-i].convertTo(m, type); // Write to mxArray through m.
	}
	return p_;
}
#endif

// Implementation of internal importers
template <typename T>
mxArray* fromInternal(const typename std::enable_if<
							   std::is_same<typename MxTypes<T>::array_type, mxNumeric>::value,
							   T>::type& value) {
	mxArray* array = mxCreateNumericMatrix(1,
										   1,
										   MxTypes<T>::class_id,
										   MxTypes<T>::complexity);
	MEXPLUS_CHECK_NOTNULL(array);
	*reinterpret_cast<T*>(mxGetData(array)) = value;
	return array;
}

template <typename Container>
mxArray* fromInternal(const typename std::enable_if<
							   std::is_same<typename MxTypes<typename Container::value_type>::array_type,
							   mxNumeric>::value &&
							   std::is_compound<Container>::value,
							   Container>::type& value) {
	typedef typename Container::value_type ValueType;
	mxArray* array = mxCreateNumericMatrix(1,
										   value.size(),
										   MxTypes<ValueType>::class_id,
										   MxTypes<ValueType>::complexity);
	MEXPLUS_CHECK_NOTNULL(array);
	std::copy(value.begin(),
			  value.end(),
			  reinterpret_cast<ValueType*>(mxGetData(array)));
	return array;
}

template <typename T>
mxArray* fromInternal(const typename std::enable_if<
							   std::is_same<typename MxTypes<T>::array_type, mxChar>::value,
							   T>::type& value) {
	const char char_array[] = {static_cast<char>(value), 0};
	mxArray* array = mxCreateString(char_array);
	MEXPLUS_CHECK_NOTNULL(array);
	return array;
}

template <typename Container>
mxArray* fromInternal(const typename std::enable_if<
							   std::is_same<typename MxTypes<typename Container::value_type>::array_type,
							   mxChar>::value &&
							   std::is_compound<Container>::value,
							   Container>::type& value) {
	const mwSize dimensions[] = {1, static_cast<mwSize>(value.size())};
	mxArray* array = mxCreateCharArray(2, dimensions);
	MEXPLUS_CHECK_NOTNULL(array);
	std::copy(value.begin(), value.end(), mxGetChars(array));
	return array;
}

template <typename T>
mxArray* fromInternal(const typename std::enable_if<
							   std::is_same<typename MxTypes<T>::array_type, mxLogical>::value,
							   T>::type& value) {
	mxArray* array = mxCreateLogicalScalar(value);
	MEXPLUS_CHECK_NOTNULL(array);
	return array;
}

template <typename Container>
mxArray* fromInternal(const typename std::enable_if<
							   std::is_same<typename MxTypes<typename Container::value_type>::array_type,
							   mxLogical>::value &&
							   std::is_compound<Container>::value,
							   Container>::type& value) {
	mxArray* array = mxCreateLogicalMatrix(1, value.size());
	MEXPLUS_CHECK_NOTNULL(array);
	std::copy(value.begin(), value.end(), mxGetLogicals(array));
	return array;
}

template <typename Container>
mxArray* fromInternal(const typename std::enable_if<
							   std::is_same<typename MxTypes<typename Container::value_type>::array_type,
							   mxCell>::value &&
							   std::is_compound<Container>::value,
							   Container>::type& value) {
	mxArray* array = mxCreateCellMatrix(1, value.size());
	MEXPLUS_CHECK_NOTNULL(array);
	mwIndex index = 0;
	for (typename Container::const_iterator it = value.begin();
		 it != value.end();
		 ++it)
		mxSetCell(array, index++, from(*it));
	return array;
}

/** mxArray object wrapper for data conversion and manipulation.
 *
 * The class is similar to a combination of unique_ptr and wrapper around
 * Matlab's matrix API. An MxArray object created from a mutable mxArray*
 * pointer automatically frees its internal memory unless explicitly
 * released. When MxArray is created from a const mxArray*, the object does not
 * manage memory but still provides the same matrix API.
 */
class MxArray {
public:
	/** Empty MxArray constructor. Use reset() to set a pointer.
   */
	MxArray() : array_(nullptr), owner_(false) {}
	/** nullptr assignment.
   */
	MxArray& operator= (std::nullptr_t) {
		reset();
		return *this;
	}
	/** Move constructor.
   */
	MxArray(MxArray&& array) : array_(nullptr), owner_(false) {
		*this = std::move(array);
	}
	/** Move assignment.
   */
	MxArray& operator= (MxArray&& rhs) {
		if (this != &rhs) {
			array_ = rhs.array_;
			owner_ = rhs.owner_;
			rhs.array_ = nullptr;
			rhs.owner_ = false;
		}
		return *this;
	}
	/** MxArray constructor from mutable mxArray*. MxArray will manage memory.
   * @param array mxArray pointer.
   */
	explicit MxArray(const mxArray* array) :
		array_(const_cast<mxArray*>(array)),
		owner_(false) {}
	/** MxArray constructor from const mxArray*. MxArray will not manage memory.
   * @param array mxArray pointer given by mexFunction.
   */
	explicit MxArray(mxArray* array) : array_(array), owner_(array!=false) {}
	/** MxArray constructor from scalar.
   */
	template <typename T>
	explicit MxArray(const T& value) : array_(from(value)), owner_(true) {}
	/** Destructor. Unreleased pointers will be destroyed.
   */
	virtual ~MxArray() {
		if (array_ && owner_)
			mxDestroyArray(array_);
	}
	/** Swap operation.
   */
	void swap(MxArray& rhs)  {
		if (this != &rhs) {
			mxArray* array = rhs.array_;
			bool owner = rhs.owner_;
			rhs.array_ = array_;
			rhs.owner_ = owner_;
			array_ = array;
			owner_ = owner;
		}
	}
	/** Reset an mxArray to a const mxArray*.
   *
   * Caller must be VERY careful with this, as the behavior is undefined when
   * the original mxArray* is destroyed. For example, the following will crash.
   * @code
   *     MxArray foo;
   *     {
   *       MxArray bar(1);
   *       foo.reset(bar.get());
   *     }
   *     foo.toInt(); // Error!
   * @endcode
   */
	void reset(const mxArray* array = nullptr) {
		if (array_ && owner_)
			mxDestroyArray(array_);
		array_ = const_cast<mxArray*>(array);
		owner_ = false;
	}
	/** Reset an mxArray.
   */
	void reset(mxArray* array) {
		if (array_ && owner_)
			mxDestroyArray(array_);
		array_ = array;
		owner_ = array!=nullptr;
	}
	/** Release managed mxArray* pointer, or clone if not owner.
   * @return Unmanaged mxArray*. Always caller must destroy.
   */
	mxArray* release()  {
		MEXPLUS_CHECK_NOTNULL(array_);
		mxArray* array = (owner_) ? array_ : clone();
		array_ = nullptr;
		owner_ = false;
		return array;
	}
	/** Clone mxArray. This always allocates new mxArray*.
   * @return Unmanaged mxArray*. Always caller must destroy.
   */
	mxArray* clone() const {
		MEXPLUS_CHECK_NOTNULL(array_);
		mxArray* array = mxDuplicateArray(array_);
		MEXPLUS_CHECK_NOTNULL(array);
		return array;
	}
	/** Conversion to const mxArray*.
   * @return const mxArray* pointer.
   */
	inline const mxArray* get() const { return array_; }
	/** Return true if the array is not NULL.
   */
	operator bool() const { return array_ != nullptr; }
	/** Return true if owner.
   */
	inline bool isOwner() const { return owner_; }
	/** Create a new numeric matrix.
   * @param rows Number of rows.
   * @param columns Number of cols.
   */
	template <typename T>
	static mxArray* Numeric(int rows = 1, int columns = 1);
	/** Create a new logical matrix.
   * @param rows Number of rows.
   * @param columns Number of cols.
   */
	static mxArray* Logical(int rows = 1, int columns = 1) {
		mxArray* logical_array = mxCreateLogicalMatrix(rows, columns);
		MEXPLUS_CHECK_NOTNULL(logical_array);
		return logical_array;
	}
	/** Create a new cell matrix.
   * @param rows Number of rows.
   * @param columns Number of cols.
   *
   * Example:
   * @code
   *     MxArray cell_array = MxArray::Cell(1, 2);
   *     cell_array.set(0, 1);
   *     cell_array.set(1, "another value");
   *     plhs[0] = cell_array.release();
   * @endcode
   */
	static mxArray* Cell(int rows = 1, int columns = 1) {
		mxArray* cell_array = mxCreateCellMatrix(rows, columns);
		MEXPLUS_CHECK_NOTNULL(cell_array);
		return cell_array;
	}
	/** Generic constructor for a struct matrix.
   * @param fields field names.
   * @param nfields number of field names.
   * @param rows size of the first dimension.
   * @param columns size of the second dimension.
   *
   * Example:
   * @code
   *     const char* fields[] = {"field1", "field2"};
   *     MxArray struct_array(MxArray::Struct(2, fields));
   *     struct_array.set("field1", 1);
   *     struct_array.set("field2", "field2 value");
   *     plhs[0] = struct_array.release();
   * @endcode
   */
	static mxArray* Struct(int nfields = 0,
						   const char** fields = nullptr,
						   int rows = 1,
						   int columns = 1) {
		mxArray* struct_array = mxCreateStructMatrix(rows,
													 columns,
													 nfields,
													 fields);
		MEXPLUS_CHECK_NOTNULL(struct_array);
		return struct_array;
	}


	/** mxArray* exporter methods.
   */
	template <typename T>
	static void to(const mxArray* array, T* value) {
		toInternal<T>(array, value);
	}
	template <typename T>
	static T to(const mxArray* array) {
		T value;
		toInternal<T>(array, &value);
		return value;
	}
	/** mxArray* element reader methods.
   */
	template <typename T>
	static T at(const mxArray* array, mwIndex index) {
		T value;
		atInternal<T>(array, index, &value);
		return value;
	}
	template <typename T>
	static void at(const mxArray* array, mwIndex index, T* value) {
		atInternal<T>(array, index, value);
	}
	static const mxArray* at(const mxArray* array, mwIndex index) {
		MEXPLUS_CHECK_NOTNULL(array);
		MEXPLUS_ASSERT(mxIsCell(array), "Expected a cell array.");
		return mxGetCell(array, index);
	}
	template <typename T>
	static void at(const mxArray* array,
				   const std::string& field,
				   T* value,
				   mwIndex index = 0) {
		atInternal<T>(array, field, index, value);
	}
	static const mxArray* at(const mxArray* array,
							 const std::string& field,
							 mwIndex index = 0)  {
		MEXPLUS_CHECK_NOTNULL(array);
		MEXPLUS_ASSERT(mxIsStruct(array), "Expected a struct array.");
		return mxGetField(array, index, field.c_str());
	}
	/** mxArray* element writer methods.
   */
	template <typename T>
	static void set(mxArray* array, mwIndex index, const T& value) {
		setInternal<T>(array, index, value);
	}
	static void set(mxArray* array, mwIndex index, mxArray* value) {
		MEXPLUS_CHECK_NOTNULL(array);
		MEXPLUS_CHECK_NOTNULL(value);
		MEXPLUS_ASSERT(mxIsCell(array), "Expected a cell array.");
		MEXPLUS_ASSERT(index < mxGetNumberOfElements(array),
					   "Index out of range: %u.",
					   index);
		mxSetCell(array, index, value);
	}
	template <typename T>
	static void set(mxArray* array,
					const std::string& field,
					const T& value,
					mwIndex index = 0) {
		setInternal<T>(array, field, index, value);
	}
	static void set(mxArray* array,
					const std::string& field,
					mxArray* value,
					mwIndex index = 0) {
		MEXPLUS_CHECK_NOTNULL(array);
		MEXPLUS_CHECK_NOTNULL(value);
		MEXPLUS_ASSERT(mxIsStruct(array), "Expected a struct array.");
		MEXPLUS_ASSERT(index < mxGetNumberOfElements(array),
					   "Index out of range: %u.",
					   index);
		if (!mxGetField(array, index, field.c_str()))
			MEXPLUS_ASSERT(mxAddField(array, field.c_str()) >= 0,
						   "Failed to create a field '%s'",
						   field.c_str());
		mxSetField(array, index, field.c_str(), value);
	}

	/** Convert MxArray to a specified type.
   */
	template <typename T>
	T to() const {
		T value;
		toInternal<T>(array_, &value);
		return value;
	}
	template <typename T>
	void to(T* value) const { toInternal<T>(array_, value); }
	/** Template for element accessor.
   * @param index index of the array element.
   * @return value of the element at index.
   *
   *
   * Example:
   * @code
   *     MxArray array(prhs[0]);
   *     double value = array.at<double>(0);
   * @endcode
   */
	template <typename T>
	T at(mwIndex index) const {
		T value;
		atInternal<T>(array_, index, &value);
		return value;
	}
	template <typename T>
	void at(mwIndex index, T* value) const {
		atInternal<T>(array_, index, value);
	}
	const mxArray* at(mwIndex index) const {
		return at(array_, index);
	}
	/** Template for element accessor.
   * @param row index of the first dimension.
   * @param column index of the second dimension.
   * @return value of the element at (row, column).
   */
	template <typename T>
	T at(mwIndex row, mwIndex column) const;
	/** Template for element accessor.
   * @param si subscript index of the element.
   * @return value of the element at subscript index.
   */
	template <typename T>
	T at(const std::vector<mwIndex>& subscripts) const;
	/** Struct element accessor.
   * @param field field name of the struct array.
   * @param index index of the struct array.
   * @return value of the element at the specified field.
   */
	template <typename T>
	T at(const std::string& field, mwIndex index = 0) const {
		T value;
		atInternal<T>(array_, field, index, &value);
		return value;
	}
	template <typename T>
	void at(const std::string& field, T* value, mwIndex index = 0) const {
		atInternal<T>(array_, field, index, value);
	}
	const mxArray* at(const std::string& field, mwIndex index = 0) const {
		return at(array_, field, index);
	}

	/** Template for element write accessor.
   * @param index offset of the array element.
   * @param value value of the field.
   */
	template <typename T>
	void set(mwIndex index, const T& value) {
		setInternal<T>(array_, index, value);
	}
	/** Template for element write accessor.
   * @param row index of the first dimension of the array element.
   * @param column index of the first dimension of the array element.
   * @param value value of the field.
   */
	template <typename T>
	void set(mwIndex row, mwIndex column, const T& value);
	/** Template for element write accessor.
   * @param subscripts subscript index of the element.
   * @param value value of the field.
   */
	template <typename T>
	void set(const std::vector<mwIndex>& subscripts, const T& value);
	/** Cell element write accessor.
   * @param index index of the element.
   * @param value cell element to be inserted.
   */
	void set(mwIndex index, mxArray* value) {
		MEXPLUS_ASSERT(isOwner(), "Must be an owner to set.");
		set(array_, index, value);
	}
	/** Struct element write accessor.
   * @param field field name of the struct array.
   * @param value value of the field.
   * @param index linear index of the struct array element.
   */
	template <typename T>
	void set(const std::string& field, const T& value, mwIndex index = 0) {
		MEXPLUS_ASSERT(isOwner(), "Must be an owner to set.");
		setInternal<T>(array_, field, index, value);
	}
	/** Struct element write accessor.
   * @param field field name of the struct array.
   * @param value value of the field to be inserted.
   * @param index linear index of the struct array element.
   */
	void set(const std::string& field, mxArray* value, mwIndex index = 0) {
		MEXPLUS_ASSERT(isOwner(), "Must be an owner to set.");
		set(array_, field, value, index);
	}
	/** Get raw data pointer.
   * @return pointer T*. If MxArray is not compatible, return NULL.
   */
	template <typename T>
	T* getData() const;
	mxLogical* getLogicals() const {
		MEXPLUS_CHECK_NOTNULL(array_);
		MEXPLUS_ASSERT(isLogical(),
					   "Expected a logical array but %s.",
					   className().c_str());
		return mxGetLogicals(array_);
	}
	mxChar* getChars() const {
		MEXPLUS_CHECK_NOTNULL(array_);
		MEXPLUS_ASSERT(isChar(),
					   "Expected a char array but %s.",
					   className().c_str());
		return mxGetChars(array_);
	}
	/** Class ID of mxArray.
   */
	inline mxClassID classID() const { return mxGetClassID(array_); }
	/** Class name of mxArray.
   */
	inline const std::string className() const {
		return std::string(mxGetClassName(array_));
	}
	/** Number of elements in an array.
   */
	inline mwSize size() const { return mxGetNumberOfElements(array_); }
	/** Number of dimensions.
   */
	inline mwSize dimensionSize() const {
		return mxGetNumberOfDimensions(array_);
	}
	/** Array of each dimension.
   */
	inline std::vector<mwSize> dimensions() const {
		const mwSize* dimensions = mxGetDimensions(array_);
		return std::vector<mwSize>(dimensions, dimensions + dimensionSize());
	}
	/** Number of rows in an array.
   */
	inline mwSize rows() const { return mxGetM(array_); }
	/** Number of columns in an array.
   */
	inline mwSize cols() const { return mxGetN(array_); }
	/** Number of fields in a struct array.
   */
	inline int fieldSize() const { return mxGetNumberOfFields(array_); }
	/** Get field name of a struct array.
   * @param index index of the struct array.
   * @return std::string.
   */
	std::string fieldName(int index) const {
		const char* field = mxGetFieldNameByNumber(array_, index);
		MEXPLUS_ASSERT(field, "Failed to get field name at %d.", index);
		return std::string(field);
	}
	/** Get field names of a struct array.
   * @params field_nams std::vector<std::string> of struct field names.
   */
	std::vector<std::string> fieldNames() const {
		MEXPLUS_ASSERT(isStruct(), "Expected a struct array.");
		std::vector<std::string> fields(fieldSize());
		for (size_t i = 0; i < fields.size(); ++i)
			fields[i] = fieldName(i);
		return fields;
	}
	/** Number of elements in IR, PR, and PI arrays.
   */
	inline mwSize nonZeroMax() const { return mxGetNzmax(array_); }
	/** Offset from first element to desired element.
   * @param row index of the first dimension of the array.
   * @param column index of the second dimension of the array.
   * @return linear offset of the specified subscript index.
   */
	mwIndex subscriptIndex(mwIndex row, mwIndex column) const {
		MEXPLUS_ASSERT(row < rows() && column < cols(),
					   "Subscript is out of range.");
		mwIndex subscripts[] = {row, column};
		return mxCalcSingleSubscript(array_, 2, subscripts);
	}
	/** Offset from first element to desired element.
   * @param si subscript index of the array.
   * @return linear offset of the specified subscript index.
   */
	mwIndex subscriptIndex(const std::vector<mwIndex>& subscripts) const {
		return mxCalcSingleSubscript(array_, subscripts.size(), &subscripts[0]);
	}
	/** Determine whether input is cell array.
   */
	inline bool isCell() const { return mxIsCell(array_); }
	/** Determine whether input is string array.
   */
	inline bool isChar() const { return mxIsChar(array_); }
	/** Determine whether array is member of specified class.
   */
	inline bool isClass(const char* name) const {
		return mxIsClass(array_, name);
	}
	/** Determine whether data is complex.
   */
	inline bool isComplex() const { return mxIsComplex(array_); }
	/** Determine whether mxArray represents data as double-precision,
   * floating-point numbers.
   */
	inline bool isDouble() const { return mxIsDouble(array_); }
	/** Determine whether array is empty.
   */
	inline bool isEmpty() const { return mxIsEmpty(array_); }
	/** Determine whether input is finite.
   */
	static inline bool IsFinite(double value) { return mxIsFinite(value); }
	/** Determine whether array was copied from MATLAB global workspace.
   */
	inline bool isFromGlobalWS() const { return mxIsFromGlobalWS(array_); };
	/** Determine whether input is infinite.
   */
	static inline bool IsInf(double value) { return mxIsInf(value); }
	/** Determine whether array represents data as signed 8-bit integers.
   */
	inline bool isInt8() const { return mxIsInt8(array_); }
	/** Determine whether array represents data as signed 16-bit integers.
   */
	inline bool isInt16() const { return mxIsInt16(array_); }
	/** Determine whether array represents data as signed 32-bit integers.
   */
	inline bool isInt32() const { return mxIsInt32(array_); }
	/** Determine whether array represents data as signed 64-bit integers.
   */
	inline bool isInt64() const { return mxIsInt64(array_); }
	/** Determine whether array is of type mxLogical.
   */
	inline bool isLogical() const { return mxIsLogical(array_); }
	/** Determine whether scalar array is of type mxLogical.
   */
	inline bool isLogicalScalar() const { return mxIsLogicalScalar(array_); }
	/** Determine whether scalar array of type mxLogical is true.
   */
	inline bool isLogicalScalarTrue() const {
		return mxIsLogicalScalarTrue(array_);
	}
	/** Determine whether array is numeric.
   */
	inline bool isNumeric() const { return mxIsNumeric(array_); }
	/** Determine whether array represents data as single-precision,
   * floating-point numbers.
   */
	inline bool isSingle() const { return mxIsSingle(array_); }
	/** Determine whether input is sparse array.
   */
	inline bool isSparse() const { return mxIsSparse(array_); }
	/** Determine whether input is structure array.
   */
	inline bool isStruct() const { return mxIsStruct(array_); }
	/** Determine whether array represents data as unsigned 8-bit integers.
   */
	inline bool isUint8() const { return mxIsUint8(array_); }
	/** Determine whether array represents data as unsigned 16-bit integers.
   */
	inline bool isUint16() const { return mxIsUint16(array_); }
	/** Determine whether array represents data as unsigned 32-bit integers.
   */
	inline bool isUint32() const { return mxIsUint32(array_); }
	/** Determine whether array represents data as unsigned 64-bit integers.
   */
	inline bool isUint64() const { return mxIsUint64(array_); }
	/** Determine whether a struct array has a specified field.
   */
	bool hasField(const std::string& field_name, mwIndex index = 0) const {
		return isStruct() &&
				mxGetField(array_, index, field_name.c_str()) != nullptr;
	}
	/** Determine whether input is NaN (Not-a-Number).
   */
	static inline bool IsNaN(double value) { return mxIsNaN(value); }
	/** Value of infinity.
   */
	static inline double Inf() { return mxGetInf(); }
	/** Value of NaN (Not-a-Number).
   */
	static inline double NaN() { return mxGetNaN(); }
	/** Value of EPS.
   */
	static inline double Eps() { return mxGetEps(); }

private:
	/** Copy constructor is prohibited except internally.
   */
	MxArray(const MxArray& array);
	//MxArray(const MxArray& array) = delete;
	/** Copy assignment operator is prohibited.
   */
	MxArray& operator=(const MxArray& rhs);
	//MxArray& operator=(const MxArray& rhs) = delete;

	/** Templated mxArray exporters.
   */
	template <typename T>
	static void toInternal(const mxArray* array, typename std::enable_if<
						   std::is_arithmetic<T>::value, T>::type* value) {
		atInternal<T>(array, 0, value);
	}
	template <typename T>
	static void toInternal(const mxArray* array, typename std::enable_if<
						   std::is_compound<T>::value &&
						   std::is_arithmetic<typename T::value_type>::value,
						   T>::type* value);
	template <typename T>
	static void toInternal(const mxArray* array, typename std::enable_if<
						   std::is_compound<T>::value &&
						   !std::is_arithmetic<typename T::value_type>::value,
						   T>::type* value);
	template <typename T>
	static void atInternal(const mxArray* array,
						   mwIndex index,
						   typename std::enable_if<
						   std::is_arithmetic<T>::value, T>::type* value);
	template <typename T>
	static void atInternal(const mxArray* array,
						   mwIndex index,
						   typename std::enable_if<
						   std::is_compound<T>::value,
						   T>::type* value);
	template <typename T>
	static void atInternal(const mxArray* array,
						   const std::string& field,
						   mwIndex index,
						   T* value);
	template <typename T>
	static void setInternal(mxArray* array,
							mwIndex index,
							const typename std::enable_if<
							std::is_arithmetic<T>::value,
							T>::type& value);
	template <typename T>
	static void setInternal(mxArray* array,
							mwIndex index,
							const typename std::enable_if<
							std::is_compound<T>::value,
							T>::type& value);
	template <typename T>
	static void setInternal(mxArray* array,
							const std::string& field,
							mwIndex index,
							const T& value);
	template <typename T, typename R>
	static void assignTo(const mxArray* array, mwIndex index, R* value) {
		*value = *(reinterpret_cast<T*>(mxGetData(array)) + index);
	}
	template <typename T, typename R>
	static void assignTo(const mxArray* array, R* value) {
		T* data_pointer = reinterpret_cast<T*>(mxGetData(array));
		value->assign(data_pointer, data_pointer + mxGetNumberOfElements(array));
	}
	template <typename T>
	static void assignCellTo(const mxArray* array, mwIndex index, T* value) {
		const mxArray* element = mxGetCell(array, index);
		MEXPLUS_CHECK_NOTNULL(element);
		toInternal<T>(element, value);
	}
	template <typename T>
	static void assignCellTo(const mxArray* array, T* value) {
		for (int i = 0; i < mxGetNumberOfElements(array); ++i) {
			const mxArray* element = mxGetCell(array, i);
			MEXPLUS_CHECK_NOTNULL(element);
			value->push_back(to<typename T::value_type>(element));
		}
	}
	template <typename R, typename T>
	static void assignFrom(mxArray* array, mwIndex index, const T& value) {
		*(reinterpret_cast<R*>(mxGetData(array)) + index) = value;
	}

	/** Const pointer to the mxArray C object.
   */
	mxArray* array_;
	/** Flag to enable resource management.
   */
	bool owner_;
};

template <typename T>
void MxArray::toInternal(const mxArray* array, typename std::enable_if<
						 std::is_compound<T>::value &&
						 std::is_arithmetic<typename T::value_type>::value,
						 T>::type* value) {
	MEXPLUS_CHECK_NOTNULL(array);
	MEXPLUS_CHECK_NOTNULL(value);
	switch (mxGetClassID(array)) {
	case mxINT8_CLASS:    assignTo<int8_t, T>(array, value); break;
	case mxUINT8_CLASS:   assignTo<uint8_t, T>(array, value); break;
	case mxINT16_CLASS:   assignTo<int16_t, T>(array, value); break;
	case mxUINT16_CLASS:  assignTo<uint16_t, T>(array, value); break;
	case mxINT32_CLASS:   assignTo<int32_t, T>(array, value); break;
	case mxUINT32_CLASS:  assignTo<uint32_t, T>(array, value); break;
	case mxINT64_CLASS:   assignTo<int64_t, T>(array, value); break;
	case mxUINT64_CLASS:  assignTo<uint64_t, T>(array, value); break;
	case mxSINGLE_CLASS:  assignTo<float, T>(array, value); break;
	case mxDOUBLE_CLASS:  assignTo<double, T>(array, value); break;
	case mxLOGICAL_CLASS: assignTo<mxLogical, T>(array, value); break;
	case mxCHAR_CLASS:    assignTo<mxChar, T>(array, value); break;
	case mxCELL_CLASS:    assignCellTo<T>(array, value); break;
		// case mxSPARSE_CLASS:
	default:
		MEXPLUS_ERROR("Cannot convert %s.", mxGetClassName(array));
	}
}

template <typename T>
void MxArray::toInternal(const mxArray* array, typename std::enable_if<
						 std::is_compound<T>::value &&
						 !std::is_arithmetic<typename T::value_type>::value,
						 T>::type* value) {
	MEXPLUS_CHECK_NOTNULL(value);
	MEXPLUS_ASSERT(mxIsCell(array), "Expected a cell array.");
	for (int i = 0; i < mxGetNumberOfElements(array); ++i) {
		const mxArray* element = mxGetCell(array, i);
		value->push_back(to<typename T::value_type>(element));
	}
}

template <typename T>
void MxArray::atInternal(const mxArray* array,
						 mwIndex index,
						 typename std::enable_if<
						 std::is_arithmetic<T>::value, T>::type* value) {
	MEXPLUS_CHECK_NOTNULL(array);
	MEXPLUS_CHECK_NOTNULL(value);
	MEXPLUS_ASSERT(index < mxGetNumberOfElements(array),
				   "Index out of range: %u.",
				   index);
	switch (mxGetClassID(array)) {
	case mxINT8_CLASS: assignTo<int8_t, T>(array, index, value); break;
	case mxUINT8_CLASS: assignTo<uint8_t, T>(array, index, value); break;
	case mxINT16_CLASS: assignTo<int16_t, T>(array, index, value); break;
	case mxUINT16_CLASS: assignTo<uint16_t, T>(array, index, value); break;
	case mxINT32_CLASS: assignTo<int32_t, T>(array, index, value); break;
	case mxUINT32_CLASS: assignTo<uint32_t, T>(array, index, value); break;
	case mxINT64_CLASS: assignTo<int64_t, T>(array, index, value); break;
	case mxUINT64_CLASS: assignTo<uint64_t, T>(array, index, value); break;
	case mxSINGLE_CLASS: assignTo<float, T>(array, index, value); break;
	case mxDOUBLE_CLASS: assignTo<double, T>(array, index, value); break;
	case mxLOGICAL_CLASS: assignTo<mxLogical, T>(array, index, value); break;
	case mxCHAR_CLASS: assignTo<mxChar, T>(array, index, value); break;
	case mxCELL_CLASS: assignCellTo<T>(array, index, value); break;
		// case mxSPARSE_CLASS:
	default:
		MEXPLUS_ASSERT(true, "Cannot convert %s", mxGetClassName(array));
	}
}

template <typename T>
void MxArray::atInternal(const mxArray* array,
						 mwIndex index,
						 typename std::enable_if<
						 std::is_compound<T>::value, T>::type* value) {
	MEXPLUS_CHECK_NOTNULL(array);
	MEXPLUS_CHECK_NOTNULL(value);
	MEXPLUS_ASSERT(index < mxGetNumberOfElements(array),
				   "Index out of range: %u.",
				   index);
	MEXPLUS_ASSERT(mxIsCell(array), "Expected a cell array.");
	const mxArray* element = mxGetCell(array, index);
	toInternal<T>(element, value);
}

template <typename T>
void MxArray::atInternal(const mxArray* array,
						 const std::string& field,
						 mwIndex index,
						 T* value) {
	MEXPLUS_CHECK_NOTNULL(array);
	MEXPLUS_CHECK_NOTNULL(value);
	MEXPLUS_ASSERT(index < mxGetNumberOfElements(array),
				   "Index out of range: %u.",
				   index);
	MEXPLUS_ASSERT(mxIsStruct(array), "Expected a struct array.");
	const mxArray* element = mxGetField(array, index, field.c_str());
	MEXPLUS_ASSERT(element, "Invalid field name %s.", field.c_str());
	toInternal<T>(element, value);
}

template <typename T>
void MxArray::setInternal(mxArray* array,
						  mwIndex index,
						  const typename std::enable_if<
						  std::is_arithmetic<T>::value,
						  T>::type& value) {
	MEXPLUS_CHECK_NOTNULL(array);
	MEXPLUS_ASSERT(index < mxGetNumberOfElements(array),
				   "Index out of range: %u.",
				   index);
	switch (mxGetClassID(array)) {
	case mxINT8_CLASS: assignFrom<int8_t, T>(array, index, value); break;
	case mxUINT8_CLASS: assignFrom<uint8_t, T>(array, index, value); break;
	case mxINT16_CLASS: assignFrom<int16_t, T>(array, index, value); break;
	case mxUINT16_CLASS: assignFrom<uint16_t, T>(array, index, value); break;
	case mxINT32_CLASS: assignFrom<int32_t, T>(array, index, value); break;
	case mxUINT32_CLASS: assignFrom<uint32_t, T>(array, index, value); break;
	case mxINT64_CLASS: assignFrom<int64_t, T>(array, index, value); break;
	case mxUINT64_CLASS: assignFrom<uint64_t, T>(array, index, value); break;
	case mxSINGLE_CLASS: assignFrom<float, T>(array, index, value); break;
	case mxDOUBLE_CLASS: assignFrom<double, T>(array, index, value); break;
	case mxCHAR_CLASS: assignFrom<mxChar, T>(array, index, value); break;
	case mxLOGICAL_CLASS: assignFrom<mxLogical, T>(array, index, value); break;
	case mxCELL_CLASS: mxSetCell(array, index, from(value)); break;
	default:
		MEXPLUS_ERROR("Cannot assign to %s array.", mxGetClassName(array));
	}
}

template <typename T>
void MxArray::setInternal(mxArray* array,
						  mwIndex index,
						  const typename std::enable_if<
						  std::is_compound<T>::value,
						  T>::type& value) {
	MEXPLUS_CHECK_NOTNULL(array);
	MEXPLUS_ASSERT(index < mxGetNumberOfElements(array),
				   "Index out of range: %u.",
				   index);
	MEXPLUS_ASSERT(mxIsCell(array), "Expected a cell array.");
	mxSetCell(array, index, from(value));
}

template <typename T>
void MxArray::setInternal(mxArray* array,
						  const std::string& field,
						  mwIndex index,
						  const T& value) {
	MEXPLUS_CHECK_NOTNULL(array);
	MEXPLUS_ASSERT(index < mxGetNumberOfElements(array),
				   "Index out of range: %u.",
				   index);
	MEXPLUS_ASSERT(mxIsStruct(array), "Expected a struct array.");
	if (!mxGetField(array, index, field.c_str()))
		MEXPLUS_ASSERT(mxAddField(array, field.c_str()) >= 0,
					   "Failed to create a field '%s'",
					   field.c_str());
	mxSetField(array, index, field.c_str(), mexplus::from(value));
}

template <typename T>
mxArray* MxArray::Numeric(int rows, int columns) {
	typedef typename std::enable_if<
			std::is_same<typename MxTypes<T>::array_type, mxNumeric>::value,
			T>::type Scalar;
	mxArray* numeric = mxCreateNumericMatrix(rows,
											 columns,
											 MxTypes<Scalar>::class_id,
											 MxTypes<Scalar>::complexity);
	MEXPLUS_CHECK_NOTNULL(numeric);
	return numeric;
}

template <typename T>
T* MxArray::getData() const {
	MEXPLUS_CHECK_NOTNULL(array_);
	MEXPLUS_ASSERT(MxTypes<T>::class_id == classID(),
				   "Expected a %s array.",
				   typeid(T).name());
	return reinterpret_cast<T*>(mxGetData(array_));
}

template <typename T>
T MxArray::at(mwIndex row, mwIndex column) const {
	return at<T>(subscriptIndex(row, column));
}

template <typename T>
T MxArray::at(const std::vector<mwIndex>& subscripts) const {
	return at<T>(subscriptIndex(subscripts));
}

template <typename T>
void MxArray::set(mwIndex row, mwIndex column, const T& value) {
	set<T>(subscriptIndex(row, column), value);
}

template <typename T>
void MxArray::set(const std::vector<mwIndex>& subscripts, const T& value) {
	set<T>(subscriptIndex(subscripts), value);
}

} // namespace mexplus

#endif // __MEXPLUS_MXARRAY_H__
