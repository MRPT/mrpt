/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */


#include "hwdrivers-precomp.h"   // Precompiled headers

#include <mrpt/hwdrivers/CIbeoLuxETH.h> // Precompiled headers

#include <bitset>

#define APPERTURE           4.712385    // in radian <=> 270°

using namespace mrpt;
using namespace mrpt::system;
using namespace mrpt::poses;
using namespace mrpt::utils;
using namespace mrpt::hwdrivers;
using namespace mrpt::obs;
using namespace std;

// TODO: Use enum instead
const unsigned char SearchForAF	= 0;
const unsigned char SearchForFE	= 1;
const unsigned char SearchForC0	= 2;
const unsigned char SearchForC2	= 3;
const unsigned char PacketFound	= 4;
const unsigned char SaveData	= 5;

IMPLEMENTS_GENERIC_SENSOR(CIbeoLuxETH,mrpt::hwdrivers)

CIbeoLuxETH::CIbeoLuxETH(string _ip, unsigned int _port):
        m_ip(_ip),
        m_port(_port),
		m_sensorPose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        m_maxRange(200.0),
        m_beamApperture(.25*M_PI/180.0),
        vwinkel(0.0)
{
}

CIbeoLuxETH::~CIbeoLuxETH()
{
	m_run = false;
	mrpt::system::joinThread(dataCollectionThread);
	// Wait a little for the thread to come down
	// Don't ask why, it just works
	//TODO: Try without the delay
	sleep(10);
}

void CIbeoLuxETH::dataCollection()
{
	unsigned char state = SearchForAF;
	unsigned char msgIn[1], Header[20], ScanListHeader[44], ScanPointData[10];
	unsigned int datatype, /*scannumber,*/ numScanpoints, angleTicks, SPlayer, SPdistance; // SPecho;
	int SPHangle;
	unsigned char msg[32];

	// Start TCP-connection to laserscanner
	m_client.connect(m_ip, m_port);

	// Send filter command
	makeCommandHeader(msg);
	makeTypeCommand(msg);
	m_client.writeAsync(&msg[0], 32);

	// Send start command
	makeCommandHeader(msg);
	makeStartCommand(msg);
	m_client.writeAsync(&msg[0], 28);

	while(m_run)
	{
		switch(state)
		{
			case SearchForAF:
				m_client.readAsync(msgIn, 1, 100, 10);
				if (msgIn[0] == 0xAF) state = SearchForFE;
				break;
			case SearchForFE:
				m_client.readAsync(msgIn, 1, 100, 10);
				if (msgIn[0] == 0xFE) state = SearchForC0; else state = SearchForAF;
				break;
			case SearchForC0:
				m_client.readAsync(msgIn, 1, 100, 10);
				if (msgIn[0] == 0xC0) state = SearchForC2; else state = SearchForAF;
				break;
			case SearchForC2:
				m_client.readAsync(msgIn, 1, 100, 10);
				if (msgIn[0] == 0xC2) state = PacketFound; else state = SearchForAF;
				break;
			case PacketFound:
				m_client.readAsync(Header, 20, 100, 10);
				datatype = Header[10] * 0x100 + Header[11];
				switch(datatype)
				{
					case 0x2030:
						// do nothing
						state = SearchForAF;
						break;
					case 0x2221:
						// do nothing
						state = SearchForAF;
						break;
					case 0x2805:
						// do nothing
						state = SearchForAF;
						break;
					case 0x2020:
						// do nothing
						state = SearchForAF;
						break;
					case 0x2202:
						state = SaveData;
						break;
					default:
						std::cerr << "UNKNOWN packet of type " << hex << datatype << " received!!\n";
						state = SearchForAF;
				}
				break;
			case SaveData:
				// Create new observation object pointer
				CObservation3DRangeScanPtr newObs = CObservation3DRangeScan::Create();
				newObs->hasPoints3D = true;
				newObs->maxRange = 200.00;

				m_client.readAsync(ScanListHeader, 44, 10, 10);
				/*scannumber =
				ScanListHeader[1] * 0x100 + ScanListHeader[0]; */
				numScanpoints = ScanListHeader[29] * 0x100 + ScanListHeader[28];
				angleTicks = ScanListHeader[23] * 0x100 + ScanListHeader[22];

				for (unsigned int i = 0; i < numScanpoints; ++i)
				{
					bool dropPacket = false;

					m_client.readAsync(ScanPointData, 10, 10, 10);
					SPlayer = ScanPointData[0] & 0x0F; // two lower bits denote layer
					// SPecho  = ScanPointData[0] >> 4; // two higher bits denote echo
					SPHangle = (char)ScanPointData[3] * 0x100 + ScanPointData[2]; // signed INT16 here
					SPdistance = ScanPointData[5] * 0x100 + ScanPointData[4];

					// Sanity checks
					if (SPlayer > 4)
					{
						dropPacket = true;
						//std::cerr << "Invalid layer: " << SPlayer << " should be element of [0,3] Scanpoint dropped.\n";
					}
					if ((SPHangle < -((int)angleTicks)/2) || (SPHangle > (int)angleTicks/2))
					{
						dropPacket = true;
						//std::cerr << "Invalid horizontal angle: " << (int)-angleTicks/2 << "< " << SPHangle << " <" << angleTicks/2 << " Scanpoint dropped.\n";
					}
					if ((SPdistance < 30) || (SPdistance > 20000))
					{
						dropPacket = true;
						//std::cerr << "Invalid distance: 30< " << SPdistance << " <20000 Scanpoint dropped.\n";
					}


					if (!dropPacket)
					{
						//TODO: Process point information correctly
						CPoint3D cartesianPoint = convertToCartesian(
							convertLayerToRad(SPlayer), // vertikal coord of scanner
							convertTicksToHRad(SPHangle, angleTicks), // horizontal coord of scanner
							SPdistance);

						// write scanpoint data to observation object
						newObs->points3D_x.push_back(cartesianPoint.x());
						newObs->points3D_y.push_back(cartesianPoint.y());
						newObs->points3D_z.push_back(cartesianPoint.z());
					}
				} // for

				// return observation to framework
				appendObservation( newObs );

				state = SearchForAF;
				break; // SaveData
		}	// Switch
	}	// While

	// Send stop command
	makeCommandHeader(msg);
	makeStopCommand(msg);
	m_client.writeAsync(&msg[0], 28);

	m_client.close();
}	// dataCollection

CPoint3D CIbeoLuxETH::convertToCartesian(float vrad, float hrad, float distance)
{
	float x, y, z;
	float rho, phi, theta;

	// Convert from laserscanner coordinate system to spherical coordinates
	rho = distance/100; // cm to meter
	phi = -hrad+(M_PI/2); // start with 0 pointing straight up
	theta = vrad+M_PI; // 0 is straight ahead, going clockwise for 2 Pi

	x = rho * sin (phi) * cos (theta);
	y = rho * sin (phi) * sin (theta);
	z = rho * cos (phi);

	CPoint3D point(x, y, z);
	return point;
}

double CIbeoLuxETH::convertTicksToHRad(int hticks, int hticksPerRotation)
{
	return M_PI*2 * hticks / hticksPerRotation;
}

double CIbeoLuxETH::convertLayerToRad(int scanlayer)
{
	double vangle;

	switch(scanlayer)
	{
		case 0:
			vangle = -0.02094395103;
			break;
		case 1:
			vangle = -0.006981317009;
			break;
		case 2:
			vangle =  0.006981317009;
			break;
		case 3:
			vangle =  0.02094395103;
			break;
		default:
			vangle = 0;
			std::cerr << "Layer: " << scanlayer << "! Returning " << vangle << " as angle.\n";
			break;
	}

	return vangle;
}

void CIbeoLuxETH::loadConfig_sensorSpecific( const mrpt::utils::CConfigFileBase &configSource,
                             const std::string	  &iniSection )
{
	float pose_x, pose_y, pose_z, pose_yaw, pose_pitch, pose_roll;
    bool faillNotFound = false;
    pose_x = configSource.read_float(iniSection,"pose_x",0,faillNotFound);
    pose_y = configSource.read_float(iniSection,"pose_y",0,faillNotFound);
    pose_z = configSource.read_float(iniSection,"pose_z",0,faillNotFound);
    pose_yaw = configSource.read_float(iniSection,"pose_yaw",0,faillNotFound);
    pose_pitch = configSource.read_float(iniSection,"pose_pitch",0,faillNotFound);
    pose_roll = configSource.read_float(iniSection,"pose_roll",0,faillNotFound);

    m_sensorPose = CPose3D( pose_x, pose_y, pose_z,
        DEG2RAD( pose_yaw ),DEG2RAD( pose_pitch ), DEG2RAD( pose_roll ));

}

void CIbeoLuxETH::makeCommandHeader(unsigned char* buffer)
{
	// Header - all big endian
	buffer[0] = 0xAF; // magic word
	buffer[1] = 0xFE;
	buffer[2] = 0xC0;
	buffer[3] = 0xC2;
	buffer[4] = 0x00; // Size of previous message, here just left to null
	buffer[5] = 0x00;
	buffer[6] = 0x00;
	buffer[7] = 0x00;
	buffer[8] = 0x00; // Size of data block
	buffer[9] = 0x00;
	buffer[10] = 0x00;
	buffer[11] = 0x00;	// to be set by the command function
	buffer[12] = 0x00;	// Reserved + source Id
	buffer[13] = 0x78;	// source ID of 0x78 as observed
	buffer[14] = 0x20;	// Data Type - 2010 = command
	buffer[15] = 0x10;
	buffer[16] = 0x00;	// 4* ntpp time (s) + 4* fractions of a second
	buffer[17] = 0x00;
	buffer[18] = 0x00;
	buffer[19] = 0x00;
	buffer[20] = 0x00;
	buffer[21] = 0x00;
	buffer[22] = 0x00;
	buffer[23] = 0x00;
}

void CIbeoLuxETH::makeStartCommand(unsigned char* buffer)
{
	// Header - all big endian
	buffer[11] = 0x04;	// Size of data block
	// Data Block - all little endian
	buffer[24] = 0x20;	// Start Measure 0x0020
	buffer[25] = 0x00;
	buffer[26] = 0x00;	// Reserved, but obligatory
	buffer[27] = 0x00;
}

void CIbeoLuxETH::makeStopCommand(unsigned char* buffer)
{
	// Header - all big endian
	buffer[11] = 0x04;	// Size of data block
	// Data Block - all little endian
	buffer[24] = 0x21;	// Stop Measure 0x0021
	buffer[25] = 0x00;
	buffer[26] = 0x00;	// Reserved, but obligatory
	buffer[27] = 0x00;
}

void CIbeoLuxETH::makeTypeCommand(unsigned char* buffer)
{
	// Header - all big endian
	buffer[11] = 0x08;	// Size of data block
	// Data Block - big endian (for filter command!)
	buffer[24] = 0x00;	// Command Type - 0005 = set datatype filter
	buffer[25] = 0x05;
	buffer[26] = 0x00;	// Data type filter length
	buffer[27] = 0x02;
	buffer[28] = 0x22;	// start value
	buffer[29] = 0x00;
	buffer[30] = 0x22;	// end value
	buffer[31] = 0x10;
}

/** This method can or cannot be implemented in the derived class, depending on the need for it.
*  \exception This method must throw an exception with a descriptive message if some critical error is found.
*/
void CIbeoLuxETH::initialize()
{
	m_run = true;

	// Start a thread to collect and interpret scandata
	//std::cout << "Start dataCollectionThread\n";

	// boost threads:
	//boost::thread dataCollectionThread(boost::bind(&CIbeoLuxETH::dataCollection,this));

	dataCollectionThread = createThreadFromObjectMethod(this, &CIbeoLuxETH::dataCollection);
}

void CIbeoLuxETH::doProcess()
{
	// nothing is done here
	// data is collected in the dataCollection thread
}
