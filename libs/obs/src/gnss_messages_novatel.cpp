/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "obs-precomp.h"   // Precompiled headers

#include <mrpt/obs/gnss_messages_novatel.h>

using namespace std;
using namespace mrpt::obs::gnss;

nv_oem6_header_t::nv_oem6_header_t() :
	hdr_len(0),
	msg_id(0),
	msg_type(0),
	port_addr(0),
	msg_len(0),
	seq_number(0),
	idle_percent(0),
	time_status(0),
	week(0),
	ms_in_week(0),
	receiver_status(0),
	reserved(0),
	receiver_sw_version(0)
{
	synch[0]=synch[1]=synch[2]=0;
}

// ------------
Message_NV_OEM6_BESTPOS::content_t::content_t() :
	solution_stat(), position_type(),
	lat(),lon(),hgt(),
	undulation(), datum_id(),
	lat_sigma(), lon_sigma(), hgt_sigma(),
	diff_age(), sol_age(),
	num_sats_tracked(), num_sats_sol(), num_sats_sol_L1(), num_sats_sol_multi(),
	reserved(), ext_sol_stat(),
	galileo_beidou_mask(),gps_glonass_mask(),
	crc()
{
	base_station_id[0]=0;
}

void Message_NV_OEM6_BESTPOS::dumpToStream( mrpt::utils::CStream &out ) const
{
	out.printf("[Novatel OEM6 BESTPOS]\n");
	out.printf(" Solution status: %u\n", (unsigned)fields.solution_stat);
	out.printf(" Position type  : %u\n", (unsigned)fields.position_type);
	out.printf(" Longitude: %.09f deg  Latitude: %.09f deg  Height: %.03f m\n",
		fields.lon,
		fields.lat,
		fields.hgt );
}

