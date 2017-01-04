/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "obs-precomp.h"   // Precompiled headers

#include <mrpt/obs/CSinCosLookUpTableFor2DScans.h>
#include <mrpt/obs/CObservation2DRangeScan.h>

using namespace std;
using namespace mrpt::obs;
using namespace mrpt::utils;

/** Return two vectors with the cos and the sin of the angles for each of the
  * rays in a scan, computing them only the first time and returning a cached copy the rest.
  */
const CSinCosLookUpTableFor2DScans::TSinCosValues & CSinCosLookUpTableFor2DScans::getSinCosForScan(const CObservation2DRangeScan &scan) const
{
	T2DScanProperties scan_prop;
	scan.getScanProperties(scan_prop);
	return getSinCosForScan(scan_prop);
}

/** Return two vectors with the cos and the sin of the angles for each of the
  * rays in a scan, computing them only the first time and returning a cached copy the rest.
  */
const CSinCosLookUpTableFor2DScans::TSinCosValues & CSinCosLookUpTableFor2DScans::getSinCosForScan(const T2DScanProperties &scan_prop) const
{
	std::map<T2DScanProperties,TSinCosValues>::const_iterator it = m_cache.find(scan_prop);
	if (it!=m_cache.end())
	{ // Found in the cache:
		return it->second;
	}
	else
	{
		// If there're too many LUTs, something is wrong, so just free the memory:
		// (If you someday find someone with TWENTY *different* laser scanner models,
		//   please, accept my apologies... but send me a photo of them all!! ;-)
		if ( m_cache.size()>20 ) m_cache.clear();

		// Compute and insert in the cache:
		TSinCosValues &new_entry = m_cache[scan_prop];

		// Make sure the allocated memory at least have room for 4 extra double's at the end,
		//  for the case we use these buffers for SSE2 optimized code. If the final values are uninitialized it doesn't matter.
		new_entry.ccos.resize(scan_prop.nRays+4);
		new_entry.csin.resize(scan_prop.nRays+4);

		ASSERT_(scan_prop.nRays>=2)
		if (scan_prop.nRays>0)
		{
			double  Ang = (scan_prop.rightToLeft ? -0.5:+0.5) *scan_prop.aperture;
			const double dA = (scan_prop.rightToLeft ? 1.0:-1.0) * (scan_prop.aperture / (scan_prop.nRays-1));

			for (size_t i=0;i<scan_prop.nRays;i++)
			{
				new_entry.ccos[i] = cos( Ang );
				new_entry.csin[i] = sin( Ang );
				Ang+=dA;
			}
		}
		// return them:
		return new_entry;
	}
}

