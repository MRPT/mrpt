/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
   | Copyright (c) 2012, University of Almeria                                 |
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
#ifndef CObservationRawDAQ_H
#define CObservationRawDAQ_H

#include <mrpt/utils/CSerializable.h>
#include <mrpt/slam/CObservation.h>

namespace mrpt
{
namespace slam
{
	DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CObservationRawDAQ , CObservation, OBS_IMPEXP)

	/** Store raw data from a Data Acquisition (DAQ) device, such that input or output analog and digital channels, counters from encoders, etc. at one sampling instant.
	 *  All analog values are assumed to be volts. 
	 * \sa CObservation
	 * \ingroup mrpt_obs_grp
	 */
	class OBS_IMPEXP CObservationRawDAQ : public CObservation
	{
		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE( CObservationRawDAQ )
	 public:
		/** Constructor */
		inline CObservationRawDAQ(  ) { }
		/** Destructor */
		virtual ~CObservationRawDAQ()
		{ }

		std::vector<uint8_t>  AIN_8bits;  /** Readings from 8-bit analog input (ADCs) channels (vector length=channel count) in Volts. */
		std::vector<uint16_t> AIN_16bits; /** Readings from 16-bit analog input (ADCs) channels (vector length=channel count) in Volts. */
		std::vector<uint32_t> AIN_32bits; /** Readings from 16-bit analog input (ADCs) channels (vector length=channel count) in Volts. */
		std::vector<uint8_t>  AOUT_8bits;  /** Present output values for 8-bit analog output (DACs) channels (vector length=channel count) in Volts.*/
		std::vector<uint16_t> AOUT_16bits;  /** Present output values for 16-bit analog output (DACs) channels (vector length=channel count) in Volts.*/

		std::vector<uint8_t>  DIN;   /** Readings from digital inputs; each byte stores 8 digital inputs, or 8-bit port. */
		std::vector<uint8_t>  DOUT;  /** Present digital output values; each byte stores 8 digital inputs, or 8-bit port. */

		/** Not used in this class */
		void getSensorPose( CPose3D &out_sensorPose ) const { }
		/** Not used in this class */
		void setSensorPose( const CPose3D & ) { }

	}; // End of class def.

	} // End of namespace
} // End of namespace

#endif
