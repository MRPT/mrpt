
//  Copyright (c) 2003-2019 Xsens Technologies B.V. or subsidiaries worldwide.
//  All rights reserved.
//  
//  Redistribution and use in source and binary forms, with or without modification,
//  are permitted provided that the following conditions are met:
//  
//  1.	Redistributions of source code must retain the above copyright notice,
//  	this list of conditions, and the following disclaimer.
//  
//  2.	Redistributions in binary form must reproduce the above copyright notice,
//  	this list of conditions, and the following disclaimer in the documentation
//  	and/or other materials provided with the distribution.
//  
//  3.	Neither the names of the copyright holders nor the names of their contributors
//  	may be used to endorse or promote products derived from this software without
//  	specific prior written permission.
//  
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
//  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
//  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
//  THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
//  OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
//  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY OR
//  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.THE LAWS OF THE NETHERLANDS 
//  SHALL BE EXCLUSIVELY APPLICABLE AND ANY DISPUTES SHALL BE FINALLY SETTLED UNDER THE RULES 
//  OF ARBITRATION OF THE INTERNATIONAL CHAMBER OF COMMERCE IN THE HAGUE BY ONE OR MORE 
//  ARBITRATORS APPOINTED IN ACCORDANCE WITH SAID RULES.
//  

#ifndef PACKETSTAMPER_H
#define PACKETSTAMPER_H

#include <xstypes/pstdint.h>
#include <list>

struct XsDataPacket;

class PacketStamper
{
public:
	PacketStamper();
	void resetTosEstimation();

	static const int64_t AWINDABOUNDARY;
	static const int64_t MTSCBOUNDARY;
	static const int64_t SC8BOUNDARY;

	static int64_t calculateLargePacketCounter(int64_t frameCounter, int64_t lastCounter, int64_t boundary);
	static int64_t calculateLargeSampleTime(int64_t frameTime, int64_t lastTime);
	int64_t stampPacket(XsDataPacket& pack, XsDataPacket const& highest);

protected:
	/*! Holds a data point for the clock estimation algorithm */
	struct DataPair {
		int64_t m_pid;	//!< Packet ID of data item
		int64_t m_toa;	//!< Time Of Arrival of data item

		//! Returns true if the items are equal
		bool operator == (DataPair const& other) const
		{
			return m_pid == other.m_pid && m_toa == other.m_toa;
		}
	};
	DataPair m_latest;		//!< Latest known data (later data may arrive with a lower pid, which will not be put in this item)
	DataPair m_linearize;	//!< The very first item received, used to normalize to 0,0 so we have less computational issues with large numbers
	std::list<DataPair> m_dataPoints;	//!< The filtered history of interesting data items

	double m_toa0;	//!< The recomputed Time Of Arrival of PID 0
	double m_rate;	//!< The estimated clock rate per pid
	int m_rejectionCountdown;	//!< A countdown value that is used after the input sanity check rejects input to reject subsequent samples as well.

	void estimateTos(XsDataPacket& pack);
	int64_t estimateTosInternal(int64_t pid, int64_t toa);
	void estimateClockParameters();
	bool rejectOutlier();
};

#endif
