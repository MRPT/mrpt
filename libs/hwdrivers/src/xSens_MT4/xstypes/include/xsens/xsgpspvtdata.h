/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#ifndef XSGPSPVTDATA_H
#define XSGPSPVTDATA_H

#include "xstypesconfig.h"
#include "pstdint.h"

#ifdef __cplusplus
extern "C"
{
#else
#define XSGPSPVTDATA_INITIALIZER                  \
	{                                             \
		0, 255, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 255 \
	}
#endif

	struct XsGpsPvtData;

	XSTYPES_DLL_API void XsGpsPvtData_destruct(struct XsGpsPvtData* thisPtr);
	XSTYPES_DLL_API int XsGpsPvtData_empty(const struct XsGpsPvtData* thisPtr);

#ifdef __cplusplus
}  // extern "C"
#endif

/*! \brief Data from the GPS unit of a legacy MTi-G.*/
struct XsGpsPvtData
{
	/** The pressure measurement in units of 2 Pascal, only valid if
	 * m_pressureAge is not 255 */
	uint16_t m_pressure;
	/** The age of the pressure measurement in packets. When it decreases
	 * relative to the previous packet, it indicates that new data is available.
	 */
	uint8_t m_pressureAge;
	/** Integer time of week in ms */
	uint32_t m_itow;
	/** Latitude in 1e-7 degrees */
	int32_t m_latitude;
	/** Longitude in 1e-7 degrees */
	int32_t m_longitude;
	/** Height in mm */
	int32_t m_height;
	/** Velocity towards north in cm/s */
	int32_t m_veln;
	/** Velocity towards east in cm/s */
	int32_t m_vele;
	/** Velocity towards down in cm/s */
	int32_t m_veld;
	/** Horizontal accuracy estimate, expected error standard deviation in mm */
	uint32_t m_hacc;
	/** Vertical accuracy estimate, expected error standard deviation in mm */
	uint32_t m_vacc;
	/** Speed accuracy estimate, expected error standard deviation in cm/s */
	uint32_t m_sacc;
	/** The age of the GPS measurement in packets. When it decreases relative to
	 * the previous packet, it indicates that new data is available. */
	uint8_t m_gpsAge;

#ifdef __cplusplus
	/*! \brief \copybrief XsGpsPvtData_destruct */
	inline void clear() { XsGpsPvtData_destruct(this); }
	/*! \brief \copybrief XsGpsPvtData_empty */
	inline bool empty() const { return 0 != XsGpsPvtData_empty(this); }
#endif
};

typedef struct XsGpsPvtData XsGpsPvtData;

#endif  // file guard
