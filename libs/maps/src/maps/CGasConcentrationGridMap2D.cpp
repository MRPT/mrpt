/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */

#include <mrpt/maps.h>  // Precompiled header



#include <mrpt/slam/CGasConcentrationGridMap2D.h>
#include <mrpt/slam/CObservationGasSensors.h>
#include <mrpt/system/os.h>
#include <mrpt/math/utils.h>
#include <mrpt/utils/CTicTac.h>
#include <mrpt/utils/color_maps.h>

#include <mrpt/opengl.h>
#include <numeric>
#include <ios>
#include <sstream>

using namespace mrpt;
using namespace mrpt::slam;
using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace std;

IMPLEMENTS_SERIALIZABLE(CGasConcentrationGridMap2D, CRandomFieldGridMap2D,mrpt::slam)

/*---------------------------------------------------------------
						Constructor
  ---------------------------------------------------------------*/
CGasConcentrationGridMap2D::CGasConcentrationGridMap2D(
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

CGasConcentrationGridMap2D::~CGasConcentrationGridMap2D()
{	
}

/*---------------------------------------------------------------
						clear
  ---------------------------------------------------------------*/
void  CGasConcentrationGridMap2D::internal_clear()
{
	// Just do the generic clear:
	CRandomFieldGridMap2D::internal_clear();

	// Anything else special for this derived class?
	// ...
}


/*---------------------------------------------------------------
						insertObservation
  ---------------------------------------------------------------*/
bool  CGasConcentrationGridMap2D::internal_insertObservation(
	const CObservation	*obs,
	const CPose3D		*robotPose )
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

	if ( IS_CLASS(obs, CObservationGasSensors ))
	{
		/********************************************************************
					OBSERVATION TYPE: CObservationGasSensors
		********************************************************************/
		const CObservationGasSensors	*o = static_cast<const CObservationGasSensors*>( obs );
		
		if (o->sensorLabel == insertionOptions.sensorLabel)
		{
			/** Correct sensorLabel, process the observation */
			
			float sensorReading;
			// Get index to differentiate between enoses --> insertionoptions.enose_id
			//for (std::vector<CObservationGasSensors::TObservationENose>::const_iterator it = o->m_readings.begin(); it!=o->m_readings.end();it+=1)

			ASSERT_(o->m_readings.size() > insertionOptions.enose_id);
			const CObservationGasSensors::TObservationENose *it = &o->m_readings[insertionOptions.enose_id];
			{
				// Compute the 3D sensor pose in world coordinates:
				CPose2D		sensorPose( CPose3D(robotPose2D) + it->eNosePoseOnTheRobot );

				// Compute the sensor reading value (Volts):
				if (insertionOptions.sensorType==0x0000){	//compute the mean
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
				}

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

			} // Selected e-noseID obs.

			return true;	// Done!

		} //endif correct "sensorLabel"

	} // end if "CObservationGasSensors"

	return false;

	MRPT_END
}

/*---------------------------------------------------------------
						computeObservationLikelihood
  ---------------------------------------------------------------*/
double	 CGasConcentrationGridMap2D::computeObservationLikelihood(
	const CObservation		*obs,
	const CPose3D			&takenFrom )
{
	MRPT_UNUSED_PARAM(obs);MRPT_UNUSED_PARAM(takenFrom);

    THROW_EXCEPTION("Not implemented yet!");
}

/*---------------------------------------------------------------
  Implements the writing to a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
void  CGasConcentrationGridMap2D::writeToStream(CStream &out, int *version) const
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
		n = static_cast<uint32_t>(sizeof( TGasConcentrationCell ));
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
void  CGasConcentrationGridMap2D::readFromStream(CStream &in, int version)
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
				ASSERT_EQUAL_( n , static_cast<uint32_t>( sizeof( TGasConcentrationCell ) ));
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
CGasConcentrationGridMap2D::TInsertionOptions::TInsertionOptions() :
	sensorType				( 0x0000 ),		//By default use the mean between all e-nose sensors
	enose_id				( 0 )			//By default use the first enose

{
}

/*---------------------------------------------------------------
					dumpToTextStream
  ---------------------------------------------------------------*/
void  CGasConcentrationGridMap2D::TInsertionOptions::dumpToTextStream(CStream	&out) const
{
	out.printf("\n----------- [CGasConcentrationGridMap2D::TInsertionOptions] ------------ \n\n");
	internal_dumpToTextStream_common(out);  // Common params to all random fields maps:

	out.printf("sensorType								= %u\n", (unsigned)sensorType);
	out.printf("enose_id								= %u\n", (unsigned)enose_id);

	out.printf("\n");
}

/*---------------------------------------------------------------
					loadFromConfigFile
  ---------------------------------------------------------------*/
void  CGasConcentrationGridMap2D::TInsertionOptions::loadFromConfigFile(
	const mrpt::utils::CConfigFileBase  &iniFile,
	const std::string &section)
{
	// Common data fields for all random fields maps:
	internal_loadFromConfigFile_common(iniFile,section);
	
	// Specific data fields for gasGridMaps
	sensorLabel	= iniFile.read_string(section.c_str(),"sensorLabel","Full_MCEnose");
	enose_id	= iniFile.read_int(section.c_str(),"enoseID",enose_id);
	{
		//Read sensor type in hexadecimal
		std::string sensorType_str = iniFile.read_string(section.c_str(),"sensorType","-1");
		int tmpSensorType;
		stringstream convert ( sensorType_str );
		convert>> std::hex >> tmpSensorType;

		if (tmpSensorType>=0)
		{
			// Valid number found:
			sensorType = tmpSensorType;
		}
		else
		{ // fall back to old name, or default to current value:
			sensorType = iniFile.read_int(section.c_str(),"KF_sensorType",sensorType);			
		}
	}
	
}


/*---------------------------------------------------------------
					getAsBitmapFile
 ---------------------------------------------------------------*/
void  CGasConcentrationGridMap2D::getAsBitmapFile(mrpt::utils::CImage &out_img) const
{
	MRPT_START

	// Nothing special in this derived class:
	CRandomFieldGridMap2D::getAsBitmapFile(out_img);

	MRPT_END
}


/*---------------------------------------------------------------
					saveMetricMapRepresentationToFile
  ---------------------------------------------------------------*/
void  CGasConcentrationGridMap2D::saveMetricMapRepresentationToFile(
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
void  CGasConcentrationGridMap2D::saveAsMatlab3DGraph(const std::string  &filName) const
{
	MRPT_START

	// Nothing special in this derived class:
	CRandomFieldGridMap2D::saveAsMatlab3DGraph(filName);

	MRPT_END
}

/*---------------------------------------------------------------
						getAs3DObject
---------------------------------------------------------------*/
void  CGasConcentrationGridMap2D::getAs3DObject( mrpt::opengl::CSetOfObjectsPtr	&outObj ) const
{
	MRPT_START
	if (m_disableSaveAs3DObject)
		return;

	CRandomFieldGridMap2D::getAs3DObject(outObj);

	MRPT_END
}