/** MxTypes and other type traits for template.
 *
 * Kota Yamaguchi 2014 <kyamagu@cs.stonybrook.edu>
 */

#ifndef __MEXPLUS_MXTYPES_H__
#define __MEXPLUS_MXTYPES_H__

#include <matrix.h>
#include <type_traits>

namespace mexplus {

/** Traits for mxLogical-convertibles.
 */
template <typename T, typename U = T>
struct MxLogicalType : std::false_type {};

template <typename T>
struct MxLogicalType<T, typename std::enable_if<
    std::is_same<typename std::remove_cv<T>::type, bool>::value ||
    std::is_same<typename std::remove_cv<T>::type, mxLogical>::value,
    T>::type> :
    std::true_type {};

/** Traits for mxChar-convertibles.
 *
 * Treat them as an integer types when they are specified signed or unsigned,
 * because uint8_t is exactly unsigned char and there is no way to tell them
 * apart.
 */
template <typename T, typename U = T>
struct MxCharType : std::false_type {};

template <typename T>
struct MxCharType<T, typename std::enable_if<
    std::is_same<typename std::remove_cv<T>::type, char>::value ||
    // Visual Studio cannot distinguish these from uint.
    //std::is_same<typename std::remove_cv<T>::type, char16_t>::value ||
    //std::is_same<typename std::remove_cv<T>::type, char32_t>::value ||
    std::is_same<typename std::remove_cv<T>::type, mxChar>::value ||
    std::is_same<typename std::remove_cv<T>::type, wchar_t>::value,
    T>::type> :
    std::true_type {};

/** Traits for integer numerics.
 */
template <typename T, typename U = T>
struct MxIntType : std::false_type {};
template <typename T>
struct MxIntType<T, typename std::enable_if<
    std::is_integral<T>::value &&
    !MxLogicalType<T>::value &&
    !MxCharType<T>::value,
    T>::type> :
    std::true_type {};

typedef struct mxCell_tag {} mxCell;

typedef struct mxNumeric_tag {} mxNumeric;

/** Traits for mxArray.
 */
template <typename T, typename U = T>
struct MxTypes {
  typedef T type;
  typedef mxCell array_type;
  static const mxClassID class_id = mxUNKNOWN_CLASS;
  static const mxComplexity complexity = mxREAL;
};

template <typename T>
struct MxTypes<T, typename std::enable_if<MxCharType<T>::value, T>::type> {
  typedef T type;
  typedef mxChar array_type;
  static const mxClassID class_id = mxCHAR_CLASS;
  static const mxComplexity complexity = mxREAL;
};

template <typename T>
struct MxTypes<T, typename std::enable_if<MxLogicalType<T>::value, T>::type> {
  typedef T type;
  typedef mxLogical array_type;
  static const mxClassID class_id = mxLOGICAL_CLASS;
  static const mxComplexity complexity = mxREAL;
};

template <typename T>
struct MxTypes<T, typename std::enable_if<std::is_signed<T>::value &&
                                          MxIntType<T>::value &&
                                          sizeof(T) == 1, T>::type> {
  typedef T type;
  typedef mxNumeric array_type;
  static const mxClassID class_id = mxINT8_CLASS;
  static const mxComplexity complexity = mxREAL;
};

template <typename T>
struct MxTypes<T, typename std::enable_if<std::is_unsigned<T>::value &&
                                          MxIntType<T>::value &&
                                          sizeof(T) == 1, T>::type> {
  typedef T type;
  typedef mxNumeric array_type;
  static const mxClassID class_id = mxUINT8_CLASS;
  static const mxComplexity complexity = mxREAL;
};

template <typename T>
struct MxTypes<T, typename std::enable_if<std::is_signed<T>::value &&
                                          MxIntType<T>::value &&
                                          sizeof(T) == 2, T>::type> {
  typedef T type;
  typedef mxNumeric array_type;
  static const mxClassID class_id = mxINT16_CLASS;
  static const mxComplexity complexity = mxREAL;
};

template <typename T>
struct MxTypes<T, typename std::enable_if<MxIntType<T>::value &&
                                          std::is_unsigned<T>::value &&
                                          sizeof(T) == 2, T>::type> {
  typedef T type;
  typedef mxNumeric array_type;
  static const mxClassID class_id = mxUINT16_CLASS;
  static const mxComplexity complexity = mxREAL;
};

template <typename T>
struct MxTypes<T, typename std::enable_if<std::is_signed<T>::value &&
                                          MxIntType<T>::value &&
                                          sizeof(T) == 4, T>::type> {
  typedef T type;
  typedef mxNumeric array_type;
  static const mxClassID class_id = mxINT32_CLASS;
  static const mxComplexity complexity = mxREAL;
};

template <typename T>
struct MxTypes<T, typename std::enable_if<std::is_unsigned<T>::value &&
                                          MxIntType<T>::value &&
                                          sizeof(T) == 4, T>::type> {
  typedef T type;
  typedef mxNumeric array_type;
  static const mxClassID class_id = mxUINT32_CLASS;
  static const mxComplexity complexity = mxREAL;
};

template <typename T>
struct MxTypes<T, typename std::enable_if<std::is_signed<T>::value &&
                                          MxIntType<T>::value &&
                                          sizeof(T) == 8, T>::type> {
  typedef T type;
  typedef mxNumeric array_type;
  static const mxClassID class_id = mxINT64_CLASS;
  static const mxComplexity complexity = mxREAL;
};

template <typename T>
struct MxTypes<T, typename std::enable_if<std::is_unsigned<T>::value &&
                                          MxIntType<T>::value &&
                                          sizeof(T) == 8, T>::type> {
  typedef T type;
  typedef mxNumeric array_type;
  static const mxClassID class_id = mxUINT64_CLASS;
  static const mxComplexity complexity = mxREAL;
};

template <typename T>
struct MxTypes<T, typename std::enable_if<std::is_floating_point<T>::value &&
                                          sizeof(T) == 4, T>::type> {
  typedef T type;
  typedef mxNumeric array_type;
  static const mxClassID class_id = mxSINGLE_CLASS;
  static const mxComplexity complexity = mxREAL;
};

template <typename T>
struct MxTypes<T, typename std::enable_if<std::is_floating_point<T>::value &&
                                          sizeof(T) == 8, T>::type> {
  typedef T type;
  typedef mxNumeric array_type;
  static const mxClassID class_id = mxDOUBLE_CLASS;
  static const mxComplexity complexity = mxREAL;
};

} // namespace mexplus

#endif // __MEXPLUS_MXTYPES_H__
