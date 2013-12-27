/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef XSTYPEDEFS_H
#define XSTYPEDEFS_H

#include "xstypesconfig.h"

#ifndef XSENS_SINGLE_PRECISION
#include <stddef.h>
typedef double XsReal;	//!< Defines the floating point type used by the Xsens libraries
typedef size_t XsSize;	//!< XsSize must be unsigned number!
# ifndef PRINTF_SIZET_MODIFIER
#  if defined(XSENS_64BIT)
#    define PRINTF_SIZET_MODIFIER "l"
#  else
#    define PRINTF_SIZET_MODIFIER ""
#  endif
# endif // PRINTF_SIZET_MODIFIER
#else
typedef float XsReal;			//!< Defines the floating point type used by the Xsens libraries
typedef unsigned int XsSize;	//!< XsSize must be unsigned number!
#endif // XSENS_SINGLE_PRECISION


/*!	\addtogroup enums Global enumerations
	@{
*/
/*! \brief These flags define the behaviour of data contained by Xsens data structures
	\details Normally, the user should never need to use these directly.
*/
enum XsDataFlags {
	 XSDF_None = 0				//!< No flag set
	,XSDF_Managed = 1			//!< The contained data should be managed (freed) by the object, when false, the object assumes the memory is freed by some other process after its destruction
	,XSDF_FixedSize = 2			//!< The contained data points to a fixed-size buffer, this allows creation of dynamic objects on the stack without malloc/free overhead.
	,XSDF_Empty = 4				//!< The object contains undefined data / should be considered empty. Usually only relevant when XSDF_FixedSize is also set, as otherwise the data pointer will be NULL and empty-ness is implicit.
	,XSDF_DestructiveCopy = 8	//!< The contents of the object may be discarded when it is copied. This allows swapping the internals of a return value instead of copying. Note that the flag may be ignored if either the source or the destination do not allow swapping. Since this is a speed-optimization flag, there is no C-function for setting the flag. After the first copy operation, the flag is automatically reset (also for the copy) and the original object should be considered invalid. \note When set, the flag makes the source object non-const even if it was const! \note For some objects a destructiveCopy() function has been implemented, but it does nothing since the object is too small to benefit from the swap function.
};
/*! @} */
typedef enum XsDataFlags XsDataFlags;

#ifdef __cplusplus
extern "C" {
#endif

XSTYPES_DLL_API const char *XsDataFlags_toString(XsDataFlags f);

#ifdef __cplusplus
} // extern "C"
/*! \brief \copybrief XsDataFlags_toString \sa XsDataFlags_toString */
inline const char *toString(XsDataFlags s)
{
	return XsDataFlags_toString(s);
}
#else
// define BOOL, TRUE and FALSE
#ifndef BOOL
typedef int BOOL;
#endif

#ifndef TRUE
#define TRUE (1)
#endif

#ifndef FALSE
#define FALSE (0)
#endif
#endif // __cplusplus

#define XS_ENUM_TO_STR_CASE(value) case value: return #value;

#endif // file guard
