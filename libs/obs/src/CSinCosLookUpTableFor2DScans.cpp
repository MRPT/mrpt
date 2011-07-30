/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
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

