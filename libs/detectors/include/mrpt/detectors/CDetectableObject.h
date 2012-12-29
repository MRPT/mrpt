/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                                 |
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
