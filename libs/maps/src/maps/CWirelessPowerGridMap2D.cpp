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

#include <mrpt/maps.h>  // Precompiled header



#include <mrpt/slam/CWirelessPowerGridMap2D.h>
#include <mrpt/slam/CObservationWirelessPower.h>
#include <mrpt/system/os.h>
#include <mrpt/math/utils.h>
#include <mrpt/utils/CTicTac.h>
#include <mrpt/utils/color_maps.h>

#include <mrpt/opengl.h>
#include <numeric>

using namespace mrpt;
using namespace mrpt::slam;
using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace std;

IMPLEMENTS_SERIALIZABLE(CWirelessPowerGridMap2D, CRandomFieldGridMap2D,mrpt::slam)

/*---------------------------------------------------------------
						Constructor
  ---------------------------------------------------------------*/
CWirelessPowerGridMap2D::CWirelessPowerGridMap2D(
	TMapRepresentation	mapType,
	float		x_min,
	float		x_max,
	float		y_min,
	float		y_max,
	float		resolution ) :
		CRandomFieldGridMap2D(mapType, x_min,x_max,y_min,y_max,resolution ),
		insertionOptions()
{
	// Set the grid to initial values (and adjusts the KF covariance matrix!)
	//  Also, calling clear() is mandatory to end initialization of our base class (read note in CRandomFieldGridMap2D::CRandomFieldGridMap2D)
	CMetricMap::clear();
}

CWirelessPowerGridMap2D::~CWirelessPowerGridMap2D()
{
}

/*---------------------------------------------------------------
						clear
  ---------------------------------------------------------------*/
void  CWirelessPowerGridMap2D::internal_clear()
{
	// Just do the generic clear:
	CRandomFieldGridMap2D::internal_clear();

	// Anything else special for this derived class?
	// ...
}


/*---------------------------------------------------------------
						insertObservation
  ---------------------------------------------------------------*/
bool  CWirelessPowerGridMap2D::internal_insertObservation(
	const CObservation	*obs,
	const CPose3D			*robotPose )
{
	MRPT_START

	CPose2D		robotPose2D;
	CPose3D		robotPose3D;

	if (robotPose)
	{
		robotPose2D = CPose2D(*robotPose);
		robotPose3D = (*robotPose);
	}
	else
	{
		// Default values are (0,0,0)
	}

	if ( IS_CLASS(obs, CObservationWirelessPower ))
	{
		/********************************************************************
					OBSERVATION TYPE: CObservationWirelessPower
		********************************************************************/
		const CObservationWirelessPower	*o = static_cast<const CObservationWirelessPower*>( obs );
		float						sensorReading;

		//Cover all robot e-noses (m_readings)

		// Get index to differentiate between enoses --> insertionoptions.enose_id
//		for (std::vector<CObservationWirelessPower::TObservationENose>::const_iterator it = o->m_readings.begin(); it!=o->m_readings.end();it+=1)

//		ASSERT_(o->m_readings.size() > insertionOptions.enose_id);
//		const CObservationWirelessPower::TObservationENose *it = &o->m_readings[insertionOptions.enose_id];
//		{
			// Compute the 3D sensor pose in world coordinates:
			CPose2D		sensorPose = CPose2D(robotPose3D + o->sensorPoseOnRobot );

			// Compute the sensor reading value (Volts):
		/*	if (insertionOptions.sensorType==0x0000){	//compute the mean
				sensorReading = math::mean( it->readingsVoltage );
			}
			else
			{
				// Look for the correct sensor type
				size_t i;
				for (i=0; i<it->sensorTypes.size(); i++)
				{
					if (it->sensorTypes.at(i) == int(insertionOptions.sensorType) )
						break;
				}

				if (i<it->sensorTypes.size()){
					sensorReading = it->readingsVoltage[i];
				}
				else
				{
					//Sensor especified not found, compute default mean value
					sensorReading = math::mean( it->readingsVoltage );
				}
			}*/
			sensorReading = o->power;

			// Conversion Voltage(V)-->1/Resistance(1/Ohms)
			//sensorReading = 1/ (5 * insertionOptions.VoltageDivider_Res /sensorReading - insertionOptions.VoltageDivider_Res);

			// Normalization:
			sensorReading = (sensorReading - insertionOptions.R_min) /( insertionOptions.R_max - insertionOptions.R_min );

			// Update the gross estimates of mean/vars for the whole reading history (see IROS2009 paper):
			m_average_normreadings_mean = (sensorReading + m_average_normreadings_count*m_average_normreadings_mean)/(1+m_average_normreadings_count);
			m_average_normreadings_var  = (square(sensorReading - m_average_normreadings_mean) + m_average_normreadings_count*m_average_normreadings_var) /(1+m_average_normreadings_count);
			m_average_normreadings_count++;

#if 0
			cout << "[DEBUG] m_average_normreadings_count: " << m_average_normreadings_count << " -> mean: " << m_average_normreadings_mean << " var: " <<  m_average_normreadings_var  << endl;
#endif

			// Finally, do the actual map update with that value:
			switch (m_mapType)
			{
				case mrKernelDM:           insertObservation_KernelDM_DMV(sensorReading,sensorPose, false); break;
				case mrKernelDMV:          insertObservation_KernelDM_DMV(sensorReading,sensorPose, true); break;
				case mrKalmanFilter:       insertObservation_KF(sensorReading,sensorPose); break;
				case mrKalmanApproximate:  insertObservation_KF2(sensorReading,sensorPose);
			};

	//	} // for each e-nose obs.

		return true;	// Done!

	} // end if "CObservationWirelessPower"

	return false;

	MRPT_END
}


/*---------------------------------------------------------------
						computeObservationLikelihood
  ---------------------------------------------------------------*/
double	 CWirelessPowerGridMap2D::computeObservationLikelihood(
	const CObservation		*obs,
	const CPose3D			&takenFrom )
{
	MRPT_UNUSED_PARAM(obs);MRPT_UNUSED_PARAM(takenFrom);

    THROW_EXCEPTION("Not implemented yet!");
}

/*---------------------------------------------------------------
  Implements the writing to a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
void  CWirelessPowerGridMap2D::writeToStream(CStream &out, int *version) const
{
	if (version)
		*version = 3;
	else
	{
		uint32_t	n;

		// Save the dimensions of the grid:
		out << m_x_min << m_x_max << m_y_min << m_y_max;
		out << m_resolution;
		out << static_cast<uint32_t>(m_size_x) << static_cast<uint32_t>(m_size_y);

		// To assure compatibility: The size of each cell:
		n = static_cast<uint32_t>(sizeof( TRandomFieldCell ));
		out << n;

		// Save the map contents:
		n = static_cast<uint32_t>(m_map.size());
		out << n;

		// Save the "m_map": This requires special handling for big endian systems:
#if MRPT_IS_BIG_ENDIAN
		for (uint32_t i=0;i<n;i++)
		{
			out << m_map[i].kf_mean << m_map[i].dm_mean << m_map[i].dmv_var_mean;
		}
#else
		// Little endian: just write all at once:
		out.WriteBuffer( &m_map[0], sizeof(m_map[0])*m_map.size() );  // TODO: Do this endianness safe!!
#endif


		// Version 1: Save the insertion options:
		out << uint8_t(m_mapType)
			<< m_cov
			<< m_stackedCov;

		out << insertionOptions.sigma
			<< insertionOptions.cutoffRadius
			<< insertionOptions.R_min
			<< insertionOptions.R_max
			<< insertionOptions.KF_covSigma
			<< insertionOptions.KF_initialCellStd
			<< insertionOptions.KF_observationModelNoise
			<< insertionOptions.KF_defaultCellMeanValue
			<< insertionOptions.KF_W_size;

		// New in v3:
		out << m_average_normreadings_mean << m_average_normreadings_var << uint64_t(m_average_normreadings_count);

	}
}

// Aux struct used below (must be at global scope for STL):
struct TOldCellTypeInVersion1
{
	float	mean, std;
	float	w,wr;
};

/*---------------------------------------------------------------
  Implements the reading from a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
void  CWirelessPowerGridMap2D::readFromStream(CStream &in, int version)
{
	switch(version)
	{
	case 0:
	case 1:
	case 2:
	case 3:
		{
			uint32_t	n,i,j;

			// Load the dimensions of the grid:
			in >> m_x_min >> m_x_max >> m_y_min >> m_y_max;
			in >> m_resolution;
			in >> i >> j;
			m_size_x = i;
			m_size_y = j;

			// To assure compatibility: The size of each cell:
			in >> n;

			if (version<2)
			{	// Converter from old versions <=1
				ASSERT_( n == static_cast<uint32_t>( sizeof( TOldCellTypeInVersion1 ) ));
				// Load the map contents in an aux struct:
				in >> n;
				vector<TOldCellTypeInVersion1>  old_map(n);
				in.ReadBuffer( &old_map[0], sizeof(old_map[0])*old_map.size() );

				// Convert to newer format:
				m_map.resize(n);
				for (size_t k=0;k<n;k++)
				{
					m_map[k].kf_mean = (old_map[k].w!=0) ? old_map[k].wr : old_map[k].mean;
					m_map[k].kf_std  = (old_map[k].w!=0) ? old_map[k].w  : old_map[k].std;
				}
			}
			else
			{
				ASSERT_EQUAL_( n , static_cast<uint32_t>( sizeof( TRandomFieldCell ) ));
				// Load the map contents:
				in >> n;
				m_map.resize(n);

				// Read the note in writeToStream()
#if MRPT_IS_BIG_ENDIAN
				for (uint32_t i=0;i<n;i++)
					in >> m_map[i].kf_mean >> m_map[i].dm_mean >> m_map[i].dmv_var_mean;
#else
				// Little endian: just read all at once:
				in.ReadBuffer( &m_map[0], sizeof(m_map[0])*m_map.size() );
#endif
			}

			// Version 1: Insertion options:
			if (version>=1)
			{
				uint8_t	i;
				in  >> i;
				m_mapType = TMapRepresentation(i);

				in	>> m_cov
					>> m_stackedCov;

				in  >> insertionOptions.sigma
					>> insertionOptions.cutoffRadius
					>> insertionOptions.R_min
					>> insertionOptions.R_max
					>> insertionOptions.KF_covSigma
					>> insertionOptions.KF_initialCellStd
					>> insertionOptions.KF_observationModelNoise
					>> insertionOptions.KF_defaultCellMeanValue
					>> insertionOptions.KF_W_size;
			}

			if (version>=3)
			{
				uint64_t N;
				in >> m_average_normreadings_mean >> m_average_normreadings_var >> N;
				m_average_normreadings_count = N;
			}

			m_hasToRecoverMeanAndCov = true;
		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};

}

/*---------------------------------------------------------------
					TInsertionOptions
 ---------------------------------------------------------------*/
CWirelessPowerGridMap2D::TInsertionOptions::TInsertionOptions()
{
}

/*---------------------------------------------------------------
					dumpToTextStream
  ---------------------------------------------------------------*/
void  CWirelessPowerGridMap2D::TInsertionOptions::dumpToTextStream(CStream	&out) const
{
	out.printf("\n----------- [CWirelessPowerGridMap2D::TInsertionOptions] ------------ \n\n");
	internal_dumpToTextStream_common(out);  // Common params to all random fields maps:

	out.printf("\n");
}

/*---------------------------------------------------------------
					loadFromConfigFile
  ---------------------------------------------------------------*/
void  CWirelessPowerGridMap2D::TInsertionOptions::loadFromConfigFile(
	const mrpt::utils::CConfigFileBase  &iniFile,
	const std::string &section)
{
	// Common data fields for all random fields maps:
	internal_loadFromConfigFile_common(iniFile,section);
}



/*---------------------------------------------------------------
					getAsBitmapFile
 ---------------------------------------------------------------*/
void  CWirelessPowerGridMap2D::getAsBitmapFile(mrpt::utils::CImage &out_img) const
{
	MRPT_START

	// Nothing special in this derived class:
	CRandomFieldGridMap2D::getAsBitmapFile(out_img);

	MRPT_END
}


/*---------------------------------------------------------------
					saveMetricMapRepresentationToFile
  ---------------------------------------------------------------*/
void  CWirelessPowerGridMap2D::saveMetricMapRepresentationToFile(
	const std::string	&filNamePrefix ) const
{
	MRPT_START

	// Nothing special in this derived class:
	CRandomFieldGridMap2D::saveMetricMapRepresentationToFile(filNamePrefix);

	MRPT_END
}

/*---------------------------------------------------------------
					saveAsMatlab3DGraph
  ---------------------------------------------------------------*/
void  CWirelessPowerGridMap2D::saveAsMatlab3DGraph(const std::string  &filName) const
{
	MRPT_START

	// Nothing special in this derived class:
	CRandomFieldGridMap2D::saveAsMatlab3DGraph(filName);

	MRPT_END
}

/*---------------------------------------------------------------
						getAs3DObject
---------------------------------------------------------------*/
void  CWirelessPowerGridMap2D::getAs3DObject( mrpt::opengl::CSetOfObjectsPtr	&outObj ) const
{
	MRPT_START
	if (m_disableSaveAs3DObject)
		return;

	CRandomFieldGridMap2D::getAs3DObject(outObj);

	MRPT_END
}
