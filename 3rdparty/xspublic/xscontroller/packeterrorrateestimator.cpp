
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

#include "packeterrorrateestimator.h"
#include "xscontrollerconfig.h"
#include <cmath>

#define PER_UPDATE_PERIOD_MILLISECONDS (10000)
#define INVALID_PACKET_RATE (-1)

/*!
 * \class PacketErrorRateEstimator
 * \brief Thread to periodically estimate packet error rate based on expected packet rate
 *
 * The Packet Error Rate (PER) is estimated based on the ratio of the number of
 * received packets compared to the number of expected packets within a time
 * window. The estimator operates as a thread that periodically updates the
 * PER estimate.
 */

/*! \brief Constructor */
PacketErrorRateEstimator::PacketErrorRateEstimator()
	: m_expectedPacketsPerSecond(INVALID_PACKET_RATE)
	, m_receivedPacketCount(0)
	, m_packetErrorRate(0)
{
}

/*! \brief Destructor */
PacketErrorRateEstimator::~PacketErrorRateEstimator()
{
}

/*!
 * \brief Set the expected packet reception rate in packets per second
 * \param packetsPerSecond The number of packets expected to be received per second
 */
void PacketErrorRateEstimator::setExpectedPacketsPerSecond(int16_t packetsPerSecond)
{
	xsens::Lock lock(&m_mutex);
	m_expectedPacketsPerSecond = packetsPerSecond;
}

/*!
 * \brief Indicate that a packet has been received
 * Should be called for every in order packet received by the device. Reception
 * of out of order packets, e.g. retransmissions, or late packets, should not
 * trigger a call to this function.
 */
void PacketErrorRateEstimator::packetReceived(void)
{
	xsens::Lock lock(&m_mutex);
	++m_receivedPacketCount;
}

/*!
 * \brief Return the currently estimated packet error rate
 * \return The packet error rate as a percentage
 */
uint8_t PacketErrorRateEstimator::packetErrorRate(void) const
{
	xsens::Lock lock(&m_mutex);
	return m_packetErrorRate;
}

/*!
 * \brief Initializes the estimation parameters
 */
void PacketErrorRateEstimator::initFunction(void)
{
	xsNameThisThread("Packet Error Rate Estimator");
	m_previousUpdateTime = XsTimeStamp::nowMs();
}

/*!
 * \brief Updates the packet error rate estimate periodically
 */
int32_t PacketErrorRateEstimator::innerFunction(void)
{
	xsens::Lock lock(&m_mutex);

	const int64_t now = XsTimeStamp::nowMs();

	if (m_expectedPacketsPerSecond > 0)
	{
		const int64_t msSinceLastUpdate = now - m_previousUpdateTime;

		if (msSinceLastUpdate > 0)
		{
			const float expectedPackets = (float)floor(m_expectedPacketsPerSecond * (msSinceLastUpdate / 1000));
			float packetDeliveryRate = 100.0f * (m_receivedPacketCount / expectedPackets);

			if (packetDeliveryRate > 100.0f)
				packetDeliveryRate = 100.0f;

			m_packetErrorRate = 100 - (uint8_t)floor(packetDeliveryRate);
			JLDEBUGG("Received " << m_receivedPacketCount << " packets in " << msSinceLastUpdate << " ms. Delivery rate: " << (int) (packetDeliveryRate) << "%. Error rate: " << (int)m_packetErrorRate << "%.");
		}
		else
		{
			m_packetErrorRate = 0;
			JLDEBUGG("Received " << m_receivedPacketCount << " packets in " << msSinceLastUpdate << " ms. Delivery rate unknown (100%). Error rate unknown (" << (int)m_packetErrorRate << "%).");
		}
	}

	m_receivedPacketCount = 0;
	m_previousUpdateTime = now;

	return PER_UPDATE_PERIOD_MILLISECONDS;
}

