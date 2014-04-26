/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "obs-precomp.h"   // Precompiled headers

#include <mrpt/slam/CObservationCANBusJ1939.h>
#include <mrpt/utils/CStream.h>

using namespace mrpt::slam;
using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace std;

// This must be added to any CSerializable class implementation file.
IMPLEMENTS_SERIALIZABLE(CObservationCANBusJ1939, CObservation,mrpt::slam)

/*---------------------------------------------------------------
  Implements the writing to a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
void  CObservationCANBusJ1939::writeToStream(CStream &out, int *version) const
{
	if (version)
		*version = 1;
	else
	{
		uint32_t i,n = m_data.size();
        out << m_pgn;
        out << m_src_address;
        out << m_priority;
        out << m_pdu_format;
        out << m_pdu_spec;
        out << m_data_length;
		out << n;

		for(i=0;i<n;i++)
			out << m_data[i];

		n = m_raw_frame.size();
		out << n;
		for(i=0;i<n;i++)
			out << uint8_t(m_raw_frame[i]);

		out << sensorLabel
		    << timestamp;
	}
}

/*---------------------------------------------------------------
  Implements the reading from a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
void  CObservationCANBusJ1939::readFromStream(CStream &in, int version)
{
	switch(version)
	{
	case 0:
	case 1:
		{
			uint32_t i,n;

			m_data.clear();
			m_raw_frame.clear();

			in >> m_pgn;
			in >> m_src_address;
			in >> m_priority;
			in >> m_pdu_format;
			in >> m_pdu_spec;
			in >> m_data_length;

			in >> n;
			m_data.resize(n);
			for(i=0;i<n;++i)
                in >> m_data[i];

            in >> n;
            m_raw_frame.resize(n);
            uint8_t aux;
			for(i=0;i<n;++i)
			{
			    in >> aux;
			    m_raw_frame[i] = char(aux);
			}

			in >> sensorLabel;
			in >> timestamp;

		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};

}
