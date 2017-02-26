/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef XSBAUD_H
#define XSBAUD_H

#include "xstypesconfig.h"


/*!	\addtogroup enums Global enumerations
	@{
*/

#include "xsbaudcode.h"
#include "xsbaudrate.h"

/*! @} */

typedef enum XsBaudCode XsBaudCode;
typedef enum XsBaudRate XsBaudRate;

#ifdef __cplusplus
extern "C" {
#endif

XSTYPES_DLL_API XsBaudRate XsBaud_codeToRate(XsBaudCode baudcode);
XSTYPES_DLL_API XsBaudCode XsBaud_rateToCode(XsBaudRate baudrate);
XSTYPES_DLL_API int XsBaud_rateToNumeric(XsBaudRate baudrate);
XSTYPES_DLL_API XsBaudRate XsBaud_numericToRate(int numeric);

#ifdef __cplusplus
} // extern "C"

/*! \namespace XsBaud
	\brief Namespace for Baud rate and Baud code constants and conversions
*/
namespace XsBaud {
	/*! \copydoc XsBaud_codeToRate */
	inline XsBaudRate codeToRate(XsBaudCode baudcode)
	{
		return XsBaud_codeToRate(baudcode);
	}
	/*! \copydoc XsBaud_rateToCode */
	inline XsBaudCode rateToCode(XsBaudRate baudrate)
	{
		return XsBaud_rateToCode(baudrate);
	}
	/*! \copydoc XsBaud_rateToNumeric */
	inline int rateToNumeric(XsBaudRate baudrate)
	{
		return XsBaud_rateToNumeric(baudrate);
	}
	/*! \copydoc XsBaud_numericToRate*/
	inline XsBaudRate numericToRate(int numeric)
	{
		return XsBaud_numericToRate(numeric);
	}
}

#endif

#endif // file guard
