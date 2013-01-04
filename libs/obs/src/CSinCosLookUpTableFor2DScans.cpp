/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                            |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
   +---------------------------------------------------------------------------+ */

#include <mrpt/obs.h>   // Precompiled headers

#include <mrpt/slam/CSinCosLookUpTableFor2DScans.h>

using namespace std;
using namespace mrpt::slam;
using namespace mrpt::utils;


/** Return two vectors with the cos and the sin of the angles for each of the
  * rays in a scan, computing them only the first time and returning a cached copy the rest.
  */
const CSinCosLookUpTableFor2DScans::TSinCosValues & CSinCosLookUpTableFor2DScans::getSinCosForScan(const CObservation2DRangeScan &scan)
{
	T2DScanProperties scan_prop;
	scan.getScanProperties(scan_prop);
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

		// Make sure the allocated memory at least have room for 4 more double's at the end, 
		//  for the case we use these buffers for SSE2 optimized code. This is the only way 
		//  I can figure out to emulate a STL-like "reserve":
		new_entry.ccos.resize(scan_prop.nRays+4);
		new_entry.csin.resize(scan_prop.nRays+4);
		new_entry.ccos.resize(scan_prop.nRays);
		new_entry.csin.resize(scan_prop.nRays);

		if (scan_prop.nRays>0)
		{
			double Ang, dA;
			if (scan_prop.rightToLeft)
			{
				Ang = - 0.5 * scan_prop.aperture;
				dA  = scan_prop.aperture / (scan_prop.nRays-1);
			}
			else
			{
				Ang = + 0.5 * scan_prop.aperture;
				dA  = - scan_prop.aperture / (scan_prop.nRays-1);
			}

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

