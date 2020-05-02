
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

#include <xstypes/xsdatapacket.h>
#include "packetstamper.h"

/*! \class PacketStamper
	\brief Supplies functionality for timestamping data packets.
	\details This class can analyze a data packet and create a proper packet id for it.
*/

//! \brief 32 bit MT Sample Counter boundary
const int64_t PacketStamper::AWINDABOUNDARY = 0x100000000LL;

//! \brief 16 bit MT Sample Counter boundary
const int64_t PacketStamper::MTSCBOUNDARY = 0x00010000LL;

//! \brief 8 bit Sample Counter boundary
const int64_t PacketStamper::SC8BOUNDARY = 0x00000100LL;

//! \brief Default constructor
PacketStamper::PacketStamper()
{
	resetTosEstimation();
}

//! \brief Reset the Time Of Sampling estimation parameters
void PacketStamper::resetTosEstimation()
{
	m_latest = DataPair{-1,0};
	m_rejectionCountdown = 0;
	m_dataPoints.clear();
}

/*! \brief Calculate the new large packet counter value based on \a frameCounter and the \a lastCounter
	\details Wraparound is at the given \a boundary
	\param[in] frameCounter The frame counter
	\param[in] lastCounter The last counter
	\param[in] boundary the boundary at which to assume a wrap-around
	\returns The computed packet counter value
	\note If lastCounter < 0, returns frameCounter
*/
int64_t PacketStamper::calculateLargePacketCounter(int64_t frameCounter, int64_t lastCounter, int64_t boundary)
{
	if (lastCounter < 0)
		return frameCounter;

	const int64_t lowMask = boundary - 1;
	const int64_t boundaryHalf = boundary / 2;

	int64_t low = lastCounter & lowMask;
	int64_t dt = frameCounter - low;
	if (dt < -boundaryHalf)
		return lastCounter + dt + boundary;	// positive wraparound
	if (dt < boundaryHalf)
		return lastCounter + dt;				// normal increment

	return lastCounter + dt - boundary;		// negative wraparound
}

/*! \brief Create 64 bit counter for a packet.
	\details Wrap when new XsDataPacket is too far away from the previous XsDataPacket in time.
	Use half cache size as reasonable time difference
	When infinite cache, simply wrap when new is lower than old
	\param pack The XsDataPacket that needs its 64-bit sample counter updated
	\param highestPacket The highest packet available for the current device, it will be updated if
		the new counter is higher than the stored value.
	\returns The computed counter for the packet.
*/
int64_t PacketStamper::stampPacket(XsDataPacket& pack, XsDataPacket const& highestPacket)
{
	pack.setTimeOfArrival(XsTimeStamp::now());
	int64_t newCounter, lastCounter = -1;

	if (!highestPacket.empty())
		lastCounter = highestPacket.packetId();

	if (pack.packetId() > 0)
		newCounter = pack.packetId();
	else if (pack.containsPacketCounter())
		newCounter = calculateLargePacketCounter(pack.packetCounter(), lastCounter, MTSCBOUNDARY);
	else if (pack.containsSampleTimeFine())
	{
		newCounter = lastCounter + 1;
		//if (pack.containsSampleTimeCoarse())
		//	newCounter = (int64_t) pack.sampleTime64();
		//else
		//	newCounter = calculateLargeSampleTime((int32_t) pack.sampleTimeFine(), lastCounter);
	}
	else if (pack.containsPacketCounter8())
		newCounter = calculateLargePacketCounter(pack.packetCounter8(), lastCounter, SC8BOUNDARY);
	else if (pack.containsAwindaSnapshot())
		newCounter = calculateLargePacketCounter(pack.awindaSnapshot().m_frameNumber, lastCounter, AWINDABOUNDARY);
	else
		newCounter = lastCounter + 1;

//	JLDEBUGG("XsensDeviceAPI", "%s [%08x] old = %I64d new = %I64d diff = %I64d", __FUNCTION__, did, lastCounter, newCounter, (newCounter-lastCounter));

	pack.setPacketId(newCounter);
	estimateTos(pack);

	return newCounter;
}

/*! \brief Calculate the new large sample time value based on \a frameTime and the \a lastTime
	\details Wraparound is at 864000000 (1 day @ 10kHz)
	\param[in] frameTime The frame time
	\param[in] lastTime The last time
	\returns The computed packet counter value
	\note If lastTime < 0, returns frameTime
*/
int64_t PacketStamper::calculateLargeSampleTime(int64_t frameTime, int64_t lastTime)
{
	if (lastTime < 0)
		return frameTime;

	int64_t low = lastTime % 864000000;
	int64_t dt = frameTime - low;
	if (dt < (-864000000/2))
		return lastTime + dt + 864000000;	// positive wraparound
	if (dt < (864000000/2))
		return lastTime + dt;				// normal increment

	return lastTime + dt - 864000000;		// negative wraparound
}

/*! \brief Estimate the time of sampling for the supplied packet \a pack and update it */
void PacketStamper::estimateTos(XsDataPacket& pack)
{
	if (pack.containsSampleTime64())
		pack.setEstimatedTimeOfSampling(XsTimeStamp((int64_t) pack.sampleTime64()));
	else
		pack.setEstimatedTimeOfSampling(estimateTosInternal(pack.packetId(), pack.timeOfArrival().msTime()));
}

/*! \brief Estimate the clock parameters based on the available data points
	\details Uses a least-square estimation to fit a line through the known dataset
	and then shifts the line down to ensure the TOA >= ETOS constraint holds
*/
void PacketStamper::estimateClockParameters()
{
	// now we need to find the most consistent rate by doing a least square best fit
	// which we then shift down to match the fastest toa
	double avgPid = 0.0;
	double avgToa = 0.0;

	// if we have enough data we exclude the last item from the averages since it is volatile
	auto last = *m_dataPoints.rbegin();
	bool popit = (m_dataPoints.size() >= 5);
	if (popit)
		m_dataPoints.pop_back();
	for (auto const& d : m_dataPoints)
	{
		avgPid += d.m_pid;
		avgToa += d.m_toa;
	}
	avgPid /= m_dataPoints.size();
	avgToa /= m_dataPoints.size();

	double fracTop = 0.0, fracBot = 0.0;
	for (auto const& d : m_dataPoints)
	{
		double dpid = d.m_pid - avgPid;
		double dtoa = d.m_toa - avgToa;
		fracTop += dpid*dtoa;
		fracBot += dpid*dpid;
	}
	m_rate = fracTop / fracBot;
	m_toa0 = avgToa - m_rate * avgPid;

	// put last item back
	if (popit)
		m_dataPoints.push_back(last);

	// shift down
	for (auto const& d : m_dataPoints)
	{
		double diff = d.m_pid * m_rate + m_toa0 - d.m_toa;
		if (diff > 0.0)
			m_toa0 -= diff;
	}
}

/*! \brief Remove the worst outlier from the known data points.
	\details Only items that are more than the estimated rate off from the current estimation
	are considered outliers
	\return true if anything was rejected, false otherwise
*/
bool PacketStamper::rejectOutlier()
{
	auto reject = m_dataPoints.end();
	double diffMin = 0.0;
	for (auto d = m_dataPoints.begin(); d != m_dataPoints.end(); ++d)
	{
		double diff = d->m_pid * m_rate + m_toa0 - d->m_toa;
		if (diff < -m_rate && diff < diffMin)
		{
			diffMin = diff;
			reject = d;
		}
	}
	if (reject != m_dataPoints.end())
	{
		m_dataPoints.erase(reject);
		return true;
	}
	return false;
}

/*! \brief Estimate the time of sampling for the supplied \a pid
	\details This function will estimate the time of samplinmg based on the supplied PID. If both
	PID and TOA are acceptable values, they will be used to update the estimation parameters.

	The current algorithm expects the values to be estimated to be constant for the lifetime of the
	object.

	\param pid The packet ID to estimate the time of sampling for.
	\param toa The time of arrival associated with this pid
	\return The estimated time of sampling.
*/
int64_t PacketStamper::estimateTosInternal(int64_t pid, int64_t toa)
{
	if (m_dataPoints.size() < 2)
	{
		if (m_dataPoints.empty())
		{
			m_linearize = DataPair{pid, toa};
			m_dataPoints.push_back(DataPair{0,0});
			m_toa0 = 0;
			m_rate = 0;
			m_rejectionCountdown = 0;
		}
		else if (pid > m_latest.m_pid && toa > m_latest.m_toa)
		{
			DataPair last = {pid - m_linearize.m_pid, toa - m_linearize.m_toa};
			auto first = *m_dataPoints.begin();
			m_toa0 = 0;
			m_rate = (double) (last.m_toa - first.m_toa) / (double) (last.m_pid - first.m_pid);
			m_dataPoints.push_back(last);
		}
		m_latest = DataPair{pid, toa};	// non-linearized values!
		return toa;
	}
	if (pid > m_latest.m_pid && toa > m_latest.m_toa)
	{
		// we might do an update
		while (pid - m_latest.m_pid == 1)	// 'while' so we can break out of this scope if necessary
		{
			// do sanity check on the data point before adding it
			bool enough = (m_dataPoints.size() >= 5 && toa - m_linearize.m_toa >= 1000);
			if (enough)
			{
				double toaPred = (pid - m_linearize.m_pid) * m_rate + m_toa0;
				if ((double) (toa - m_linearize.m_toa) - toaPred >= 2.0*m_rate)
				{
					// ignore this point, it doesn't match known information well enough
					// also ignore the next few points as they're likely also not very reliable
					m_rejectionCountdown = 5;
					break;
				}
			}
			if (m_rejectionCountdown > 0)
			{
				--m_rejectionCountdown;
				break;
			}

			// add data point to list
			m_dataPoints.push_back(DataPair {pid - m_linearize.m_pid, toa - m_linearize.m_toa});

			/* filter list, we remove any points that can't define the rate because they're above the
				toa line spanned by the neighbouring points
			*/
			if (enough)
			{
				auto next = m_dataPoints.begin();
				auto prev = next++;
				auto it = next++;
				while (next != m_dataPoints.end())
				{
					double rate = (double) (next->m_toa - prev->m_toa) / (double) (next->m_pid - prev->m_pid);
					double itoa = (it->m_pid - prev->m_pid) * rate + prev->m_toa;
					if ((double) it->m_toa >= itoa)
					{
						// useless data point, discard
						it = m_dataPoints.erase(it);
						if (it == m_dataPoints.end())
							break;
						prev = it;
						--prev;
					}
					else
					{
						// useful data point, keep it
						prev = it++;
						if (it == m_dataPoints.end())
							break;
					}
					next = it;
					++next;
				}
			}

			// forget too old data if we have enough data left afterwards
			estimateClockParameters();
			if (enough && m_dataPoints.size() >= 16)
			{
				bool reestimate = rejectOutlier();
				if (m_dataPoints.size() >= 16)
				{
					auto it = m_dataPoints.begin();
					++it;
					if ((toa - m_linearize.m_toa) - it->m_toa >= 30000)
					{
						m_dataPoints.pop_front();
						reestimate = true;
					}
				}
				if (reestimate)
					estimateClockParameters();
			}
			break;
		}
		m_latest = DataPair{pid, toa};
	}

	// estimate tos from last known values
	return std::min(toa, XsMath_doubleToInt64((pid - m_linearize.m_pid) * m_rate + m_toa0) + m_linearize.m_toa);
}
