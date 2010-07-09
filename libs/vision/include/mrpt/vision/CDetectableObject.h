/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                   http://mrpt.sourceforge.net/                            |
   |                                                                           |
   |   Copyright (C) 2005-2010  University of Malaga                           |
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

namespace mrpt
{
	namespace vision
	{
		DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CDetectableObject, mrpt::utils::CSerializable, VISION_IMPEXP )

		class VISION_IMPEXP CDetectableObject: public mrpt::utils::CSerializable
		{
			DEFINE_VIRTUAL_SERIALIZABLE( CDetectableObject )

		public:

			std::string	m_id; //!< A unique id for each detectable object

		}; // End of class


		DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CDetectable2D, mrpt::vision::CDetectableObject, VISION_IMPEXP )

		class VISION_IMPEXP CDetectable2D: public CDetectableObject
		{
			DEFINE_SERIALIZABLE( CDetectable2D )

		public:

			int m_x, m_y; //!< 2D Coordinates of detected object
			int m_height, m_width; //!< Size of detected object

			/** Default constructor */
			CDetectable2D() {};

			/** Extra constructor */
			CDetectable2D( const int &x, const int &y, const int &height, const int &width ): m_x(x), m_y(y), m_height(height), m_width(width) {};

			/** Copy constructor */
			CDetectable2D( const CDetectable2D &d2 )
			{	
				m_x = d2.m_x;
				m_y = d2.m_y;
				m_height = d2.m_height;
				m_width = d2.m_width;
			};

			/** Copy constructor */
			CDetectable2D( const CDetectable2D *d2 )
			{	
				m_x = d2->m_x;
				m_y = d2->m_y;
				m_height = d2->m_height;
				m_width = d2->m_width;
			};
			
			/** Compute distance between centers of two detectable 2D objects.
			  * \return calculated distance.
			  */
			inline double distanceTo( const CDetectable2D &d2 )
			{
				double mean_x1 = ( m_x + m_width/2 );
				double mean_x2 = ( d2.m_x + d2.m_width/2 );
				double mean_y1 = ( m_y + m_height/2 ) ;
				double mean_y2 = ( d2.m_y + d2.m_height/2 ) ;
				return sqrt( pow( mean_x1 - mean_x2, 2 ) + pow( mean_y1 - mean_y2, 2 ) );
			};

		};

		
		DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CDetectable3D, mrpt::vision::CDetectable2D, VISION_IMPEXP )

		class VISION_IMPEXP CDetectable3D: public CDetectable2D
		{
			DEFINE_SERIALIZABLE( CDetectable3D )

		public:

			int		m_z; //!< Z coordinate of detected object

		}; // End of class
	}

}

#endif