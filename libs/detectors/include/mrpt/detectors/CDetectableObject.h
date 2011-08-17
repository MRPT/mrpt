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

#ifndef CDetectableObject_H
#define CDetectableObject_H

#include <mrpt/utils/CSerializable.h>
#include <mrpt/slam/CObservation.h>

#include <mrpt/detectors/link_pragmas.h>

namespace mrpt
{
	namespace detectors
	{
		using namespace mrpt::slam;

		DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CDetectableObject, mrpt::utils::CSerializable, DETECTORS_IMPEXP )

		/** Base class that contains common atributes and functions of detectable objects.
		  * It was initially thought for detected objects in images from cams, but it's easily
		  * expandable to other source types (f.i. scanners).
		  * \ingroup mrpt_detectors_grp
		  */
		class DETECTORS_IMPEXP CDetectableObject: public mrpt::utils::CSerializable
		{
			DEFINE_VIRTUAL_SERIALIZABLE( CDetectableObject )

		public:

			std::string	m_id; //!< Must be an unique id for each detectable object

			CObservationPtr	obs; //!< Observation wich contain the deteted object

			inline void setObservation( CObservationPtr newObs ){	obs = newObs;	};

		}; // End of class


		DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CDetectable2D, mrpt::detectors::CDetectableObject, DETECTORS_IMPEXP )

		class DETECTORS_IMPEXP CDetectable2D: public CDetectableObject
		{
			DEFINE_SERIALIZABLE( CDetectable2D )

		public:

			float m_x, m_y; //!< 2D Coordinates of detected object
			float m_height, m_width; //!< Size of detected object

			/** Extra constructor */
			CDetectable2D( const int &x = 0, const int &y = 0, const int &height = 0, const int &width = 0 )
				: m_x(x), m_y(y), m_height(height), m_width(width) 
			{};

			/** Copy pointer content constructor */
			CDetectable2D( const CDetectable2D *d )
			{	
				*this = *d;
			};
			
			/** Compute distance between centers of two detectable 2D objects.
			  * \return calculated distance.
			  */
			inline double distanceTo( const CDetectable2D &d2 )
			{	
				// Calculate objects centers
				double c_x1 = ( m_x + m_width/2 );
				double c_x2 = ( d2.m_x + d2.m_width/2 );
				double c_y1 = ( m_y + m_height/2 ) ;
				double c_y2 = ( d2.m_y + d2.m_height/2 ) ;

				return sqrt( pow( c_x1 - c_x2, 2 ) + pow( c_y1 - c_y2, 2 ) );
			};

		};

		
		DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CDetectable3D, mrpt::detectors::CDetectable2D, DETECTORS_IMPEXP )

		class DETECTORS_IMPEXP CDetectable3D: public CDetectable2D
		{
			DEFINE_SERIALIZABLE( CDetectable3D )

		public:

			CDetectable3D(){};

			CDetectable3D( const CDetectable2DPtr &object2d )
				: CDetectable2D( object2d.pointer() ), m_z(0)
			{ };

			/** Copy pointer content constructor */
			CDetectable3D( const CDetectable3D *d )
			{	
				*this = *d;
			};

				
			float		m_z; //!< Z coordinate of detected object

		}; // End of class
	}

}

#endif
