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

#include <mrpt/slam/CReflectivityGridMap2D.h>
#include <mrpt/slam/CObservationReflectivity.h>

#include <mrpt/opengl/CTexturedPlane.h>


using namespace mrpt;
using namespace mrpt::slam;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace mrpt::utils;
using namespace mrpt::system;
using namespace std;

IMPLEMENTS_SERIALIZABLE(CReflectivityGridMap2D, CMetricMap,mrpt::slam)


// Lookup tables for log-odds
CLogOddsGridMapLUT<CReflectivityGridMap2D::cell_t>  CReflectivityGridMap2D::m_logodd_lut;

/*---------------------------------------------------------------
						Constructor
  ---------------------------------------------------------------*/
CReflectivityGridMap2D::CReflectivityGridMap2D(
	float		x_min,
	float		x_max,
	float		y_min,
	float		y_max,
	float		resolution ) :
		CDynamicGrid<int8_t>( x_min,x_max,y_min,y_max,resolution ),
		insertionOptions()
{
	internal_clear();
}

/*---------------------------------------------------------------
						clear
  ---------------------------------------------------------------*/
void  CReflectivityGridMap2D::internal_clear()
{
	fill( m_logodd_lut.p2l(0.5) );
}

/*---------------------------------------------------------------
						isEmpty
  ---------------------------------------------------------------*/
bool  CReflectivityGridMap2D::isEmpty() const
{
	return false;
}

/*---------------------------------------------------------------
						insertObservation
  ---------------------------------------------------------------*/
bool  CReflectivityGridMap2D::internal_insertObservation(
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

	if ( IS_CLASS(obs, CObservationReflectivity ) )
	{
		/********************************************************************
					OBSERVATION TYPE: CObservationReflectivity
		********************************************************************/
		const CObservationReflectivity	*o = static_cast<const CObservationReflectivity*>( obs );

		CPose3D sensor_pose;
		sensor_pose.composeFrom(robotPose3D, o->sensorPose);

		// log-odd increment due to the observation:
		const cell_t logodd_observation = m_logodd_lut.p2l( o->reflectivityLevel );

		// Update cell, with saturation:
		cell_t *cell = cellByPos(sensor_pose.x(),sensor_pose.y());
		if (!cell)
		{
			// We need to resize the grid!
			const float new_x_min = std::min( m_x_min, (float)sensor_pose.x());
			const float new_y_min = std::min( m_y_min, (float)sensor_pose.y());
			const float new_x_max = std::min( m_x_max, (float)sensor_pose.x());
			const float new_y_max = std::min( m_y_max, (float)sensor_pose.y());

			const int8_t default_cell = m_logodd_lut.p2l(0.5);
			resize(new_x_min,new_x_max, new_y_min,new_y_max, default_cell , 2.0 /* addit. margin */ );

			// Now we should get the cell:
			cell = cellByPos(sensor_pose.x(),sensor_pose.y());

			ASSERTMSG_(cell!=NULL,"cell==NULL even after resizing grid!?")
		}

		const int cell_old = static_cast<int>(*cell);
		int cell_new = cell_old + static_cast<int>(logodd_observation);
		keep_min(cell_new, static_cast<int>( CELLTYPE_MAX)-1);
		keep_max(cell_new, static_cast<int>( CELLTYPE_MIN)+1);

		*cell = static_cast<cell_t>(cell_new);

		return true;	// Done!
	} // end if "CObservationGasSensors"

	return false;

	MRPT_END
}


/*---------------------------------------------------------------
						computeObservationLikelihood
  ---------------------------------------------------------------*/
double	 CReflectivityGridMap2D::computeObservationLikelihood(
	const CObservation		*obs,
	const CPose3D			&takenFrom )
{
	MRPT_START

	if ( IS_CLASS(obs, CObservationReflectivity ) )
	{
		/********************************************************************
					OBSERVATION TYPE: CObservationReflectivity
		********************************************************************/
		const CObservationReflectivity	*o = static_cast<const CObservationReflectivity*>( obs );

		CPose3D sensor_pose;
		sensor_pose.composeFrom(takenFrom, o->sensorPose);

		cell_t *cell = cellByPos(sensor_pose.x(),sensor_pose.y());
		if (!cell)
			return 0; // out of the map..
		else
		{
			ASSERT_ABOVEEQ_(o->reflectivityLevel,0)
			ASSERT_BELOWEQ_(o->reflectivityLevel,1)
			return -0.5*square( ( m_logodd_lut.p2l(*cell) - o->reflectivityLevel )/o->sensorStdNoise);
		}
	}
	else
		return 0;

	MRPT_END
}

/*---------------------------------------------------------------
  Implements the writing to a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
void  CReflectivityGridMap2D::writeToStream(CStream &out, int *version) const
{
	if (version)
		*version = 0;
	else
	{
		// Save the dimensions of the grid:
		out << m_x_min << m_x_max << m_y_min << m_y_max;
		out << m_resolution;
		out << static_cast<uint32_t>(m_size_x) << static_cast<uint32_t>(m_size_y);

		// Map cells:
		const uint32_t n = static_cast<uint32_t>(m_map.size());
		out.WriteBuffer(&m_map[0],n);

		// Save the insertion options:
		// out << insertionOptions.maxOccupancyUpdateCertainty;

	}
}

/*---------------------------------------------------------------
  Implements the reading from a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
void  CReflectivityGridMap2D::readFromStream(CStream &in, int version)
{
	switch(version)
	{
	case 0:
		{
			uint32_t	n,i,j;

			// Load the dimensions of the grid:
			in >> m_x_min >> m_x_max >> m_y_min >> m_y_max;
			in >> m_resolution;
			in >> i >> j;
			m_size_x = i;
			m_size_y = j;

			// Map cells:
			in >> n;
			m_map.resize(n);
			in.ReadBuffer(&m_map[0],n);

			// Load the insertion options:
			// in >> insertionOptions.maxOccupancyUpdateCertainty;

		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};

}

/*---------------------------------------------------------------
					TInsertionOptions
 ---------------------------------------------------------------*/
CReflectivityGridMap2D::TInsertionOptions::TInsertionOptions()
{
}

/*---------------------------------------------------------------
					dumpToTextStream
  ---------------------------------------------------------------*/
void  CReflectivityGridMap2D::TInsertionOptions::dumpToTextStream(CStream	&out) const
{
	out.printf("\n----------- [CReflectivityGridMap2D::TInsertionOptions] ------------ \n\n");

	//LOADABLEOPTS_DUMP_VAR(maxOccupancyUpdateCertainty, float)

	out.printf("\n");
}

/*---------------------------------------------------------------
					loadFromConfigFile
  ---------------------------------------------------------------*/
void  CReflectivityGridMap2D::TInsertionOptions::loadFromConfigFile(
	const mrpt::utils::CConfigFileBase  &iniFile,
	const std::string &section)
{
	//MRPT_LOAD_CONFIG_VAR( maxOccupancyUpdateCertainty,	float, iniFile, section )
}

/*---------------------------------------------------------------
					saveMetricMapRepresentationToFile
  ---------------------------------------------------------------*/
void  CReflectivityGridMap2D::saveMetricMapRepresentationToFile(
	const std::string	&filNamePrefix ) const
{
	std::string		fil;

	// Text matrix:
	saveToTextFile( filNamePrefix + std::string("_probability.txt") );
}

/*---------------------------------------------------------------
					getAsImage
  ---------------------------------------------------------------*/
void  CReflectivityGridMap2D::getAsImage(
	utils::CImage	&img,
	bool verticalFlip,
	bool forceRGB ) const
{
	if (!forceRGB)
	{	// 8bit gray-scale
		img.resize(m_size_x,m_size_y,1,true);
		const cell_t	*srcPtr = &m_map[0];
		unsigned char	*destPtr;
		for (unsigned int y=0;y<m_size_y;y++)
		{
			if (!verticalFlip)
					destPtr = img(0,m_size_y-1-y);
			else 	destPtr = img(0,y);
			for (unsigned int x=0;x<m_size_x;x++)
			{
				*destPtr++ = m_logodd_lut.l2p_255(*srcPtr++);
			}
		}
	}
	else
	{	// 24bit RGB:
		img.resize(m_size_x,m_size_y,3,true);
		const cell_t	*srcPtr = &m_map[0];
		unsigned char	*destPtr;
		for (unsigned int y=0;y<m_size_y;y++)
		{
			if (!verticalFlip)
					destPtr = img(0,m_size_y-1-y);
			else 	destPtr = img(0,y);
			for (unsigned int x=0;x<m_size_x;x++)
			{
				uint8_t c = m_logodd_lut.l2p_255(*srcPtr++);
				*destPtr++ = c;
				*destPtr++ = c;
				*destPtr++ = c;
			}
		}
	}
}


/*---------------------------------------------------------------
						getAs3DObject
---------------------------------------------------------------*/
void  CReflectivityGridMap2D::getAs3DObject( mrpt::opengl::CSetOfObjectsPtr	&outSetOfObj ) const
{
	if (m_disableSaveAs3DObject)
		return;

	MRPT_START

	opengl::CTexturedPlanePtr	outObj = opengl::CTexturedPlane::Create();

	outObj->setPlaneCorners(m_x_min,m_x_max,m_y_min,m_y_max);

	// Create the color & transparecy (alpha) images:
	CImage			imgColor(m_size_x,m_size_y,1);
	CImage			imgTrans(m_size_x,m_size_y,1);

	const cell_t		*srcPtr = &m_map[0];
	unsigned char		*destPtr_color;
	unsigned char		*destPtr_trans;

	for (unsigned int y=0;y<m_size_y;y++)
	{
		destPtr_color = imgColor(0,y);
		destPtr_trans = imgTrans(0,y);
		for (unsigned int x=0;x<m_size_x;x++)
		{
			uint8_t  cell255 = m_logodd_lut.l2p_255(*srcPtr++);
			*destPtr_color++ = cell255;

			int8_t   auxC = (int8_t)((signed short)cell255)-128;
			*destPtr_trans++ = auxC>0 ? (auxC << 1) : ((-auxC) << 1);
		}
	}

	outObj->assignImage_fast( imgColor,imgTrans );
	outSetOfObj->insert( outObj );

	MRPT_END
}

/*---------------------------------------------------------------
  Computes the ratio in [0,1] of correspondences between "this" and the "otherMap" map, whose 6D pose relative to "this" is "otherMapPose"
 *   In the case of a multi-metric map, this returns the average between the maps. This method always return 0 for grid maps.
 * \param  otherMap					  [IN] The other map to compute the matching with.
 * \param  otherMapPose				  [IN] The 6D pose of the other map as seen from "this".
 * \param  minDistForCorr			  [IN] The minimum distance between 2 non-probabilistic map elements for counting them as a correspondence.
 * \param  minMahaDistForCorr		  [IN] The minimum Mahalanobis distance between 2 probabilistic map elements for counting them as a correspondence.
 *
 * \return The matching ratio [0,1]
 * \sa computeMatchingWith2D
  --------------------------------------------------------------- */
float  CReflectivityGridMap2D::compute3DMatchingRatio(
		const CMetricMap						*otherMap,
		const CPose3D							&otherMapPose,
		float									minDistForCorr,
		float									minMahaDistForCorr
		) const
{
	MRPT_UNUSED_PARAM(otherMap);
	MRPT_UNUSED_PARAM(otherMapPose);
	MRPT_UNUSED_PARAM(minDistForCorr);
	MRPT_UNUSED_PARAM(minMahaDistForCorr);

	return 0;
}

