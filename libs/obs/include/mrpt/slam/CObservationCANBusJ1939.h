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
#ifndef CObservationCANBusJ1939_H
#define CObservationCANBusJ1939_H

#include <mrpt/utils/CSerializable.h>
#include <mrpt/slam/CObservation.h>

namespace mrpt
{
namespace slam
{
	DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CObservationCANBusJ1939 , CObservation, OBS_IMPEXP)

	/** This class stores a message from a CAN BUS with the protocol J1939
	 *
	 * \sa CObservation
	 * \ingroup mrpt_obs_grp
	 */
	class OBS_IMPEXP CObservationCANBusJ1939 : public CObservation
	{
		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE( CObservationCANBusJ1939 )

	 public:
		/** Constructor.
		*/
		CObservationCANBusJ1939(  ) :
			m_pgn(0), m_src_address(0), m_priority(0), m_pdu_format(0), m_pdu_spec(0), m_data_length(0)
		{ }

		/** Destructor
		*/
		virtual ~CObservationCANBusJ1939()
		{ }

		/** The Parameter Group Number within this frame */
		uint16_t m_pgn;

		/** The address of the source node within this frame */
		uint8_t m_src_address;

		/** The priority */
		uint8_t m_priority;

		/** PDU Format */
		uint8_t m_pdu_format;

		/** PDU Specific */
		uint8_t m_pdu_spec;

		/** Data length  */
		uint8_t m_data_length;

		/** The data within this frame (0-8 bytes) */
		std::vector<uint8_t> m_data;

		/** The ASCII frame */
		std::vector<char> m_raw_frame;

		/** Not used */
		void getSensorPose( CPose3D &out_sensorPose ) const { }
		void setSensorPose( const CPose3D & ) { }

	}; // End of class def.

	} // End of namespace
} // End of namespace

#endif
