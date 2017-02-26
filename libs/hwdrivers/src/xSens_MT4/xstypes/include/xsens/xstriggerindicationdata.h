/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef XSTRIGGERINDICATIONDATA_H
#define XSTRIGGERINDICATIONDATA_H

#include "xstypesconfig.h"
#include "pstdint.h"

#ifdef __cplusplus
extern "C" {
#else
#define XSTRIGGERINDICATIONDATA_INITIALIZER	{ 0, 0, 0, 0 }
#endif

struct XsTriggerIndicationData;

XSTYPES_DLL_API void XsTriggerIndicationData_destruct(struct XsTriggerIndicationData* thisPtr);
XSTYPES_DLL_API int XsTriggerIndicationData_valid(const struct XsTriggerIndicationData* thisPtr);

#ifdef __cplusplus
} // extern "C"
#endif


/*! \brief Data for a trigger indication message */
struct XsTriggerIndicationData {
	uint8_t m_line;			//!< The line number
	uint8_t m_polarity;		//!< The polarity
	uint32_t m_timestamp;	//!< The timestamp
	uint16_t m_frameNumber;	//!< The frame number

#ifdef __cplusplus
	/*! Constructor
		\param[in] line Line
		\param[in] polarity Polarity
		\param[in] timestamp Timestamp
		\param[in] frameNumber Frame number
	*/
	explicit XsTriggerIndicationData(uint8_t line = 0, uint8_t polarity = 0, uint32_t timestamp = 0, uint16_t frameNumber = 0)
	  : m_line(line), m_polarity(polarity), m_timestamp(timestamp), m_frameNumber(frameNumber)
	{}

	/*! \brief \copybrief XsTriggerIndicationData_destruct */
	inline void clear()
	{
		XsTriggerIndicationData_destruct(this);
	}

	/*! \brief \copybrief XsTriggerIndicationData_valid */
	inline bool valid() const
	{
		return 0 != XsTriggerIndicationData_valid(this);
	}
#endif
};

typedef struct XsTriggerIndicationData XsTriggerIndicationData;

#endif // file guard
