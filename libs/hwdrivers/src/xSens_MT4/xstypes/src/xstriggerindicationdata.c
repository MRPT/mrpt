#include "xstriggerindicationdata.h"
#include <string.h>

/*! \addtogroup cinterface C Interface
	@{
*/

/*! \brief Destroy the %XsTriggerIndicationData object */
void XsTriggerIndicationData_destruct(XsTriggerIndicationData* thisPtr)
{
	memset(thisPtr, 0, sizeof(XsTriggerIndicationData));
}

/*! \brief Returns true if the object is valid (line and polarity may not be 0) */
int XsTriggerIndicationData_valid(const XsTriggerIndicationData* thisPtr)
{
	return thisPtr->m_line != 0 && thisPtr->m_polarity != 0;
}

/*! @} */
