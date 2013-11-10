#ifndef XSANALOGINDATA_H
#define XSANALOGINDATA_H

#include "pstdint.h"

/*! \brief Data from analog inputs from sensors. */
struct XsAnalogInData {
	uint16_t m_data; /*!< \brief The data */
#ifdef __cplusplus
	/*! \brief Construct a nulled analog data item */
	inline XsAnalogInData() : m_data(0)
	{}

	/*! \brief Construct analog-data with value \a data */
	inline XsAnalogInData(uint16_t data) : m_data(data)
	{}
#endif
};
typedef struct XsAnalogInData XsAnalogInData;

#endif // file guard
