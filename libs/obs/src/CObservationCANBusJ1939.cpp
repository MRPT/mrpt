/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "obs-precomp.h"   // Precompiled headers

#include <mrpt/obs/CObservationCANBusJ1939.h>
#include <mrpt/utils/CStream.h>

using namespace mrpt::obs;
using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace std;

// This must be added to any CSerializable class implementation file.
IMPLEMENTS_SERIALIZABLE(CObservationCANBusJ1939, CObservation,mrpt::obs)

/*---------------------------------------------------------------
  Implements the writing to a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
void  CObservationCANBusJ1939::writeToStream(mrpt::utils::CStream &out, int *version) const
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
void  CObservationCANBusJ1939::readFromStream(mrpt::utils::CStream &in, int version)
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

void CObservationCANBusJ1939::getDescriptionAsText(std::ostream &o) const
{
	CObservation::getDescriptionAsText(o);

	o << "Priority: " << format("0x%02X",m_priority) << " [Dec: " << int(m_priority) << "]" << endl;
	o << "Parameter Group Number (PGN): " << format("0x%04X",m_pgn) << " [Dec: " << int(m_pgn) << "]" << endl;
	o << "PDU Format: " << format("0x%02X",m_pdu_format) << " [Dec: " << int(m_pdu_format) << "]" << endl;
	o << "PDU Spec: " << format("0x%02X",m_pdu_spec) << " [Dec: " << int(m_pdu_spec) << "]" << endl;
	o << "Source address: " << format("0x%02X",m_src_address) << " [Dec: " << int(m_src_address) << "]" << endl;
	o << "Data length: " << format("0x%02X",m_data_length) << " [Dec: " << int(m_data_length) << "]" << endl;
	o << "Data: ";
	for(uint8_t k = 0; k < m_data.size(); ++k)
		o << format("0x%02X",m_data[k]) << " ";
	o << " [Dec: ";
	for(uint8_t k = 0; k < m_data.size(); ++k)
		o << int(m_data[k]) << " ";
	o << "]" << endl;

	o << "Raw frame: ";
	for(uint8_t k = 0; k < m_raw_frame.size(); ++k)
		o << m_raw_frame[k];
	o << endl;


}
