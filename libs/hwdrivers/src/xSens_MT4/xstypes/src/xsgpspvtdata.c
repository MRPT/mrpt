#include "xsgpspvtdata.h"
#include <string.h>

/*! \addtogroup cinterface C Interface
	@{
*/

/*! \brief Destroy the %XsGpsPvtData object */
void XsGpsPvtData_destruct(XsGpsPvtData* thisPtr)
{
	memset(thisPtr, 0, sizeof(XsGpsPvtData));
	thisPtr->m_pressureAge = 255;
	thisPtr->m_gpsAge = 255;
}

/*! \brief Returns true if the object is empty (when it contains no valid data) */
int XsGpsPvtData_empty(const XsGpsPvtData* thisPtr)
{
	return thisPtr->m_pressureAge == 255 && thisPtr->m_gpsAge == 255;
}

/*! @} */
