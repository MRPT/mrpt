/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef XSOUTPUTCONFIGURATION_H
#define XSOUTPUTCONFIGURATION_H

#include "xstypesconfig.h"
#include "pstdint.h"
#include "xsdataidentifier.h"

#define XS_MAX_OUTPUTCONFIGURATIONS			(32)

#ifdef __cplusplus
extern "C" {
#else
#define XSOUTPUTCONFIGURATION_INITIALIZER		{ XDI_None, 0 }
#endif

struct XsOutputConfiguration;

XSTYPES_DLL_API void XsOutputConfiguration_swap(struct XsOutputConfiguration* a, struct XsOutputConfiguration* b);

#ifdef __cplusplus
} // extern "C"
#endif


/*! \brief Single data type output configuration
	\details This structure contains a single data type and the frequency at which it should be produced.
	If m_frequency is 0xFFFF and the %XsOutputConfiguration is used for input, the device will configure
	itself to its maximum frequency for the data type. If it is 0xFFFF and reported by the device,
	the data has no maximum frequency, but is sent along with appropriate packets (ie. packet counter)
*/
struct XsOutputConfiguration {
	XsDataIdentifier m_dataIdentifier;	//!< The data identifier
	uint16_t m_frequency;				//!< The frequency

#ifdef __cplusplus
	//! Constructor, initializes to an empty object
	XsOutputConfiguration()
		: m_dataIdentifier(XDI_None), m_frequency(0) {}

	//! Constructor, initializes to specified values
	XsOutputConfiguration(XsDataIdentifier di, uint16_t freq)
		: m_dataIdentifier(di), m_frequency(freq)
	{}

	//! Comparison operator
	bool operator == (const XsOutputConfiguration& other) const
	{
		return m_dataIdentifier == other.m_dataIdentifier && m_frequency == other.m_frequency;
	}
#endif
};
typedef struct XsOutputConfiguration XsOutputConfiguration;

#endif // file guard
