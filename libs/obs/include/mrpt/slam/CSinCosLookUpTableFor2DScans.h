/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                                 |
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
#ifndef CSinCosLookUpTableFor2DScans_H
#define CSinCosLookUpTableFor2DScans_H

#include <mrpt/slam/CObservation2DRangeScan.h>

namespace mrpt
{
namespace slam
{
	/** A smart look-up-table (LUT) of sin/cos values for 2D laser scans.
	  *  Refer to the main method CSinCosLookUpTableFor2DScans::getSinCosForScan()
	  *
	  *  This class is used in mrpt::slam::CPointsMap
	 * \ingroup mrpt_obs_grp
	  */
	class OBS_IMPEXP CSinCosLookUpTableFor2DScans
	{
	public:
		/** A pair of vectors with the cos and sin values. */
		struct TSinCosValues {
			mrpt::vector_float ccos, csin;
		};

		/** Return two vectors with the cos and the sin of the angles for each of the
		  * rays in a scan, computing them only the first time and returning a cached copy the rest.
		  *  Usage:
		  * \code
		  *   CSinCosLookUpTableFor2DScans cache;
		  *   ...
		  *   const CSinCosLookUpTableFor2DScans::TSinCosValues & sincos_vals = cache.getSinCosForScan( scan );
		  * \endcode
		  */
		const TSinCosValues & getSinCosForScan(const CObservation2DRangeScan &scan);

	private:
		std::map<T2DScanProperties,TSinCosValues>  m_cache; //!< The cache of known scans and their sin/cos tables.
	};


} // end NS slam
} // end NS mrpt

#endif
