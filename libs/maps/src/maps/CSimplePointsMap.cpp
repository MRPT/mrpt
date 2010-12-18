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

#include <mrpt/maps.h>  // Precompiled header

#include <mrpt/math/CPolygon.h>
#include <mrpt/slam/CSimplePointsMap.h>
#include <mrpt/slam/CObservationRange.h>

using namespace mrpt;
using namespace mrpt::slam;
using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace mrpt::math;

IMPLEMENTS_SERIALIZABLE(CSimplePointsMap, CPointsMap,mrpt::slam)

/*---------------------------------------------------------------
						Constructor
  ---------------------------------------------------------------*/
CSimplePointsMap::CSimplePointsMap()
{
	reserve( 400 );
}

/*---------------------------------------------------------------
						Destructor
  ---------------------------------------------------------------*/
CSimplePointsMap::~CSimplePointsMap()
{
}

/*---------------------------------------------------------------
						Copy constructor
  ---------------------------------------------------------------*/
void  CSimplePointsMap::copyFrom(const CPointsMap &obj)
{
	MRPT_START;

	if (this==&obj)
		return;

	x = obj.x;
	y = obj.y;
	z = obj.z;
	pointWeight = obj.pointWeight;

	m_largestDistanceFromOriginIsUpdated = obj.m_largestDistanceFromOriginIsUpdated;
	m_largestDistanceFromOrigin = obj.m_largestDistanceFromOrigin;

	kdtree_mark_as_outdated();

	MRPT_END;
}

/*---------------------------------------------------------------
					LoadFromRangeScan
 Transform the range scan into a set of cartessian coordinated
   points, leaving a given min. distance between them.
  ---------------------------------------------------------------*/
void  CSimplePointsMap::loadFromRangeScan(
	const CObservation2DRangeScan		&rangeScan,
	const CPose3D						*robotPose )
{
	int	i;
	CPose3D			sensorPose3D;

	mark_as_modified();

	// If robot pose is supplied, compute sensor pose relative to it.
	if (!robotPose)
			sensorPose3D = rangeScan.sensorPose;
	else	sensorPose3D = (*robotPose) + rangeScan.sensorPose;


	if (!insertionOptions.addToExistingPointsMap)
	{
		x.clear();
		y.clear();
		z.clear();
		pointWeight.clear();
	}

	int		sizeRangeScan = rangeScan.scan.size();

	// For a great gain in efficiency:
	if ( x.size()+2*sizeRangeScan > x.capacity() )
		reserve( x.size() + (x.size()>>2) + 3*sizeRangeScan );

	// --------------------------------------------------------------------------
	//		GENERAL CASE OF SCAN WITH ARBITRARY 3D ORIENTATION
	// --------------------------------------------------------------------------
	{
		CMatrixDouble33	ROT;
		sensorPose3D.getRotationMatrix(ROT);

		MRPT_CHECK_NORMAL_NUMBER( sensorPose3D.x() )
		MRPT_CHECK_NORMAL_NUMBER( sensorPose3D.y() )
		MRPT_CHECK_NORMAL_NUMBER( sensorPose3D.z() )

		// For quicker access:
		float		m00 = ROT.get_unsafe(0,0);
		float		m01 = ROT.get_unsafe(0,1);
		float		m03 = sensorPose3D.x();
		float		m10 = ROT.get_unsafe(1,0);
		float		m11 = ROT.get_unsafe(1,1);
		float		m13 = sensorPose3D.y();
		float		m20 = ROT.get_unsafe(2,0);
		float		m21 = ROT.get_unsafe(2,1);
		float		m23 = sensorPose3D.z();

		float		lx_1,ly_1,lz_1,lx,ly,lz;		// Punto anterior y actual:
		float		lx_2,ly_2,lz_2;				// Punto antes del anterior

		// Initial last point:
		lx_1 = -100; ly_1 = -100; lz_1 = -100;
		lx_2 = -100; ly_2 = -100; lz_2 = -100;

		// ------------------------------------------------------
		//		Pass range scan to a set of 2D points:
		// ------------------------------------------------------
		vector<float>		scan_x,scan_y;
		double		Ang, dA;
		if (rangeScan.rightToLeft)
		{
			Ang = - 0.5 * rangeScan.aperture;
			dA  = rangeScan.aperture / (sizeRangeScan-1);
		}
		else
		{
			Ang = + 0.5 * rangeScan.aperture;
			dA  = - rangeScan.aperture / (sizeRangeScan-1);
		}

		scan_x.resize( sizeRangeScan );
		scan_y.resize( sizeRangeScan );

		MRPT_TODO("Possible optization: use vector_float with precomputed cos/sin table.")

		vector<float>::iterator		 scan_x_it, scan_y_it;
		vector<float>::const_iterator scan_it;

		for ( scan_x_it=scan_x.begin(),
			  scan_y_it=scan_y.begin(),
			  scan_it=rangeScan.scan.begin();
				scan_it!=rangeScan.scan.end();
			  scan_x_it++, scan_y_it++,scan_it++)
		{
			*scan_x_it = *scan_it * cos(  Ang );
			*scan_y_it = *scan_it * sin(  Ang );
			Ang+=dA;
		}

		float  minDistSqrBetweenLaserPoints = square( insertionOptions.minDistBetweenLaserPoints );

		// If the user doesn't want a minimum distance:
		if (insertionOptions.minDistBetweenLaserPoints<0)
			minDistSqrBetweenLaserPoints = -1;

		// ----------------------------------------------------------------
		//   Transform these points into 3D using the pose transformation:
		// ----------------------------------------------------------------
		bool	lastPointWasValid = true;
		bool	thisIsTheFirst = true;
		bool  	lastPointWasInserted = false;
		float	changeInDirection = 0;

		for (i=0;i<sizeRangeScan;i++)
		{
			// Punto actual del scan:
			if ( rangeScan.validRange[i] ) //(rangeScan.scan[i]< rangeScan.maxRange )
			{
				lx = m00*scan_x[i] + m01*scan_y[i] + m03;
				ly = m10*scan_x[i] + m11*scan_y[i] + m13;
				lz = m20*scan_x[i] + m21*scan_y[i] + m23;

				lastPointWasInserted = false;

				// Add if distance > minimum only:
				float d2 = (square(lx-lx_1) + square(ly-ly_1) + square(lz-lz_1) );
				if ( thisIsTheFirst || (lastPointWasValid && (d2 > minDistSqrBetweenLaserPoints)) )
				{
					thisIsTheFirst = false;
					// Si quieren que interpolemos tb. los puntos lejanos, hacerlo:
					if (insertionOptions.also_interpolate && i>1)
					{
						float d = sqrt( d2 );

						if ((lx!=lx_1 || ly!=ly_1) && (lx_1!=lx_2 || ly_1!=ly_2) )
								changeInDirection = atan2(ly-ly_1,lx-lx_1)-atan2(ly_1-ly_2,lx_1-lx_2);
						else	changeInDirection = 0;

						// Conditions to really interpolate the points:
						if (d>=2*insertionOptions.minDistBetweenLaserPoints &&
							d<insertionOptions.maxDistForInterpolatePoints &&
							fabs(changeInDirection)<DEG2RAD(5) )
						{
							int nInterpol = round(d / (2*sqrt(minDistSqrBetweenLaserPoints)));

							for (int q=1;q<nInterpol;q++)
							{
								float i_x = lx_1 + q*(lx-lx_1)/nInterpol;
								float i_y = ly_1 + q*(ly-ly_1)/nInterpol;
								float i_z = lz_1 + q*(lz-lz_1)/nInterpol;

								x.push_back( i_x );
								y.push_back( i_y );
								z.push_back( i_z );
								pointWeight.push_back( 1 );
							}
						} // End of interpolate:
					}

					x.push_back( lx );
					y.push_back( ly );
					z.push_back( lz );
					pointWeight.push_back( 1 );

					lastPointWasInserted = true;

					lx_2 = lx_1;
					ly_2 = ly_1;
					lz_2 = lz_1;

					lx_1 = lx;
					ly_1 = ly;
					lz_1 = lz;
				}

			}

			// Save for next iteration:
			lastPointWasValid = rangeScan.validRange[i] != 0;
		}

		// The last point
		if (lastPointWasValid && !lastPointWasInserted)
		{
			x.push_back( lx );
			y.push_back( ly );
			z.push_back( lz );
			pointWeight.push_back( 1 );
		}
	}
}

/*---------------------------------------------------------------
					LoadFromRangeScan
 Enter the set of cartessian coordinated points from the
   3D range scan into the map.
  ---------------------------------------------------------------*/
void  CSimplePointsMap::loadFromRangeScan(
	const CObservation3DRangeScan		&rangeScan,
	const CPose3D						*robotPose )
{
	CPose3D			sensorPose3D;

	mark_as_modified();

	// If robot pose is supplied, compute sensor pose relative to it.
	if (!robotPose)
			sensorPose3D = rangeScan.sensorPose;
	else	sensorPose3D = (*robotPose) + rangeScan.sensorPose;


	if (!insertionOptions.addToExistingPointsMap)
	{
		x.clear();
		y.clear();
		z.clear();
		pointWeight.clear();
	}

	if (!rangeScan.hasPoints3D)
		return; // Nothing to do!

	const size_t sizeRangeScan = rangeScan.points3D_x.size();

	// For a great gain in efficiency:
	if ( x.size()+sizeRangeScan> x.capacity() )
		reserve( size_t(x.size() + 1.1*sizeRangeScan) );

	// --------------------------------------------------------------------------
	//		GENERAL CASE OF SCAN WITH ARBITRARY 3D ORIENTATION
	// --------------------------------------------------------------------------
	{
		CMatrixDouble33	ROT;
		sensorPose3D.getRotationMatrix(ROT);

		MRPT_CHECK_NORMAL_NUMBER( sensorPose3D.x() )
		MRPT_CHECK_NORMAL_NUMBER( sensorPose3D.y() )
		MRPT_CHECK_NORMAL_NUMBER( sensorPose3D.z() )

		// For quicker access:
		float		m00 = ROT.get_unsafe(0,0);
		float		m01 = ROT.get_unsafe(0,1);
		float		m02 = ROT.get_unsafe(0,2);
		float		m03 = sensorPose3D.x();
		float		m10 = ROT.get_unsafe(1,0);
		float		m11 = ROT.get_unsafe(1,1);
		float		m12 = ROT.get_unsafe(1,2);
		float		m13 = sensorPose3D.y();
		float		m20 = ROT.get_unsafe(2,0);
		float		m21 = ROT.get_unsafe(2,1);
		float		m22 = ROT.get_unsafe(2,2);
		float		m23 = sensorPose3D.z();

		float		lx_1,ly_1,lz_1,lx,ly,lz;	// Last and current point.

		// Initial last point:
		lx_1 = -100; ly_1 = -100; lz_1 = -100;

		float  minDistSqrBetweenLaserPoints = square( insertionOptions.minDistBetweenLaserPoints );

		// If the user doesn't want a minimum distance:
		if (insertionOptions.minDistBetweenLaserPoints<0)
			minDistSqrBetweenLaserPoints = -1;

		// -----------------------------------------------------
		//   Transform 3D points using the pose transformation:
		// -----------------------------------------------------
		bool	lastPointWasValid = true;
		bool	thisIsTheFirst = true;
		bool  	lastPointWasInserted = false;
//		float	changeInDirection = 0;

		for (size_t i=0;i<sizeRangeScan;i++)
		{
			// Valid point?
			if ( rangeScan.points3D_x[i]!=0 && rangeScan.points3D_y[i]!=0 )
			{
				float scan_x = rangeScan.points3D_x[i];
				float scan_y = rangeScan.points3D_y[i];
				float scan_z = rangeScan.points3D_z[i];

				lx = m00*scan_x + m01*scan_y + m02*scan_z + m03;
				ly = m10*scan_x + m11*scan_y + m12*scan_z + m13;
				lz = m20*scan_x + m21*scan_y + m22*scan_z + m23;

				lastPointWasInserted = false;

				// Add if distance > minimum only:
				float d2 = (square(lx-lx_1) + square(ly-ly_1) + square(lz-lz_1) );
				if ( thisIsTheFirst || (lastPointWasValid && (d2 > minDistSqrBetweenLaserPoints)) )
				{
					thisIsTheFirst = false;

					x.push_back( lx );
					y.push_back( ly );
					z.push_back( lz );
					pointWeight.push_back( 1 );

					lastPointWasInserted = true;

					lx_1 = lx;
					ly_1 = ly;
					lz_1 = lz;
				}

				lastPointWasValid  = true;
			}
			else
			{
				lastPointWasValid  = false;
			}
		}

		// The last point
		if (lastPointWasValid && !lastPointWasInserted)
		{
			x.push_back( lx );
			y.push_back( ly );
			z.push_back( lz );
			pointWeight.push_back( 1 );
		}
	}
}

/*---------------------------------------------------------------
					load2D_from_text_file
  Load from a text file. In each line there are a point coordinates.
    Returns false if any error occured, true elsewere.
  ---------------------------------------------------------------*/
bool  CSimplePointsMap::load2D_from_text_file(std::string file)
{
	MRPT_START;

	mark_as_modified();

	FILE	*f=os::fopen(file.c_str(),"rt");
	if (!f) return false;

	char		str[500];
	char		*ptr,*ptr1,*ptr2;

	// Clear current map:
	x.clear();
	y.clear();
	z.clear();
	pointWeight.clear();

	while (!feof(f))
	{
		// Read one string line:
		str[0] = 0;
		if (!fgets(str,sizeof(str),f)) break;

		// Find the first digit:
		ptr=str;
		while (ptr[0] && (ptr[0]==' ' || ptr[0]=='\t' || ptr[0]=='\r' || ptr[0]=='\n'))
			ptr++;

		// And try to parse it:
		float	xx = strtod(ptr,&ptr1);
		if (ptr1!=ptr)
		{
			float	yy = strtod(ptr1,&ptr2);
			if (ptr2!=ptr1)
			{
				x.push_back(xx);
				y.push_back(yy);
				z.push_back(0);
				pointWeight.push_back(1);
			}
		}
	}

	os::fclose(f);
	return true;

	MRPT_END;
}


/*---------------------------------------------------------------
					load3D_from_text_file
  Load from a text file. In each line there are a point coordinates.
    Returns false if any error occured, true elsewere.
  ---------------------------------------------------------------*/
bool  CSimplePointsMap::load3D_from_text_file(std::string file)
{
	MRPT_START;

	mark_as_modified();

	FILE	*f=os::fopen(file.c_str(),"rt");
	if (!f) return false;

	char		str[100];
	char		*ptr,*ptr1,*ptr2,*ptr3;

	// Clear current map:
	x.clear();
	y.clear();
	z.clear();
	pointWeight.clear();

	while (!feof(f))
	{
		// Read one string line:
		str[0] = 0;
		if (!fgets(str,sizeof(str),f)) break;

		// Find the first digit:
		ptr=str;
		while (ptr[0] && (ptr[0]==' ' || ptr[0]=='\t' || ptr[0]=='\r' || ptr[0]=='\n'))
			ptr++;

		// And try to parse it:
		float	xx = strtod(ptr,&ptr1);
		if (ptr1!=str)
		{
			float	yy = strtod(ptr1,&ptr2);
			if (ptr2!=ptr1)
			{
				float	zz = strtod(ptr2,&ptr3);
				if (ptr3!=ptr2)
				{
					x.push_back(xx);
					y.push_back(yy);
					z.push_back(zz);
					pointWeight.push_back(1);
				}
			}
		}
	}

	os::fclose(f);
	return true;

	MRPT_END;
}


/*---------------------------------------------------------------
					writeToStream
   Implements the writing to a CStream capability of
     CSerializable objects
  ---------------------------------------------------------------*/
void  CSimplePointsMap::writeToStream(CStream &out, int *version) const
{
	if (version)
		*version = 6;
	else
	{
		uint32_t n = x.size();

		// First, write the number of points:
		out << n;

		if (n>0)
		{
			out.WriteBufferFixEndianness(&x[0],n);
			out.WriteBufferFixEndianness(&y[0],n);
			out.WriteBufferFixEndianness(&z[0],n);
			out.WriteBufferFixEndianness(&pointWeight[0],n);
		}

		// version 2: options saved too
		out	<< insertionOptions.minDistBetweenLaserPoints
			<< insertionOptions.addToExistingPointsMap
			<< insertionOptions.also_interpolate
			<< insertionOptions.disableDeletion
			<< insertionOptions.fuseWithExisting
			<< insertionOptions.isPlanarMap
			// << insertionOptions.matchStaticPointsOnly  // Removed in v6
			<< insertionOptions.maxDistForInterpolatePoints;

		// Insertion as 3D:
		out << m_disableSaveAs3DObject;

		// Added in version 3:
		out << insertionOptions.horizontalTolerance;

		// Added in version 5:
		likelihoodOptions.writeToStream(out);
	}
}

/*---------------------------------------------------------------
					readFromStream
   Implements the reading from a CStream capability of
      CSerializable objects
  ---------------------------------------------------------------*/
void  CSimplePointsMap::readFromStream(CStream &in, int version)
{
	switch(version)
	{
	case 0:
	case 1:
	case 2:
	case 3:
	case 4:
	case 5:
	case 6:
		{
			mark_as_modified();

			// Read the number of points:
			uint32_t n;
			in >> n;

			x.resize(n);
			y.resize(n);
			z.resize(n);
			pointWeight.resize(n,1);	// Default value=1

			if (n>0)
			{
				in.ReadBufferFixEndianness(&x[0],n);
				in.ReadBufferFixEndianness(&y[0],n);
				in.ReadBufferFixEndianness(&z[0],n);

				// Version 1: weights are also stored:
				// Version 4: Type becomes long int -> uint32_t for portability!!
				if (version>=1)
				{
					if (version>=4)
							in.ReadBufferFixEndianness(&pointWeight[0],n);
					else	in.ReadBufferFixEndianness((unsigned long*)(&pointWeight[0]),n);
				}
			}

			if (version>=2)
			{
				// version 2: options saved too
				in 	>> insertionOptions.minDistBetweenLaserPoints
					>> insertionOptions.addToExistingPointsMap
					>> insertionOptions.also_interpolate
					>> insertionOptions.disableDeletion
					>> insertionOptions.fuseWithExisting
					>> insertionOptions.isPlanarMap;

				if (version<6)
				{
					bool old_matchStaticPointsOnly;
					in >> old_matchStaticPointsOnly;
				}

				in >> insertionOptions.maxDistForInterpolatePoints;

				in >> m_disableSaveAs3DObject;
			}

			if (version>=3)
			{
				in >> insertionOptions.horizontalTolerance;
			}

			if (version>=5) // version 5: added likelihoodOptions
				likelihoodOptions.readFromStream(in);

		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};

}

/*---------------------------------------------------------------
					Clear
  ---------------------------------------------------------------*/
void  CSimplePointsMap::internal_clear()
{
	// This swap() thing is the only way to really deallocate the memory.
	{ vector<float> empt;  x.swap(empt); }
	{ vector<float> empt;  y.swap(empt); }
	{ vector<float> empt;  z.swap(empt); }

	mark_as_modified();
}

/*---------------------------------------------------------------
					setPoint
			Store a points coordinates:
  ---------------------------------------------------------------*/
void  CSimplePointsMap::setPoint(size_t index,CPoint2D &p)
{
	if (index>=this->x.size())
		THROW_EXCEPTION("Index out of bounds");

	this->x[index] = p.x();
	this->y[index] = p.y();
	this->z[index] = 0;

	mark_as_modified();
}
void  CSimplePointsMap::setPoint(size_t index,CPoint3D &p)
{
	if (index>=this->x.size())
		THROW_EXCEPTION("Index out of bounds");

	this->x[index] = p.x();
	this->y[index] = p.y();
	this->z[index] = p.z();
	mark_as_modified();
}
void  CSimplePointsMap::setPoint(size_t index,float x,float y)
{
	if (index>=this->x.size())
		THROW_EXCEPTION("Index out of bounds");

	this->x[index] = x;
	this->y[index] = y;
	this->z[index] = 0;
	mark_as_modified();
}
void  CSimplePointsMap::setPoint(size_t index,float x,float y,float z)
{
	if (index>=this->x.size())
		THROW_EXCEPTION("Index out of bounds");

	this->x[index] = x;
	this->y[index] = y;
	this->z[index] = z;
	mark_as_modified();
}

/*---------------------------------------------------------------
Insert the contents of another map into this one, fusing the previous content with the new one.
 This means that points very close to existing ones will be "fused", rather than "added". This prevents
 the unbounded increase in size of these class of maps.
 ---------------------------------------------------------------*/
void  CSimplePointsMap::fuseWith(
			CPointsMap			*otherMap,
			float				minDistForFuse,
			std::vector<bool>	*notFusedPoints)
{
	TMatchingPairList	correspondences;
	TPoint3D			a,b;
	float				corrRatio;
	const CPose2D		nullPose(0,0,0);

	mark_as_modified();

	//const size_t nThis  =     this->getPointsCount();
	const size_t nOther = otherMap->getPointsCount();

	// Find correspondences between this map and the other one:
	// ------------------------------------------------------------
	computeMatchingWith2D( otherMap,			// The other map
						   nullPose,	// The other map's pose
						   minDistForFuse,	// Max. dist. for correspondence
						   0,
						   nullPose,
						   correspondences,
						   corrRatio );

	// Initially, all set to "true" -> "not fused".
	if (notFusedPoints)
	{
		notFusedPoints->clear();
		notFusedPoints->reserve( x.size() + nOther );
		notFusedPoints->resize( x.size(), true );
	}

	// Speeds-up possible memory reallocations:
	reserve( x.size() + nOther );

	// Merge matched points from both maps:
	//  AND add new points which have been not matched:
	// -------------------------------------------------
	for (size_t i=0;i<nOther;i++)
	{
		const unsigned long	w_a = otherMap->getPoint(i,a);	// Get "local" point into "a"

		// Find closest correspondence of "a":
		int			closestCorr = -1;
		float		minDist	= std::numeric_limits<float>::max();
		for (TMatchingPairList::const_iterator corrsIt = correspondences.begin(); corrsIt!=correspondences.end(); ++corrsIt)
		{
			if (corrsIt->other_idx==i)
			{
				float	dist = square( corrsIt->other_x - corrsIt->this_x ) +
							   square( corrsIt->other_y - corrsIt->this_y ) +
							   square( corrsIt->other_z - corrsIt->this_z );
				if (dist<minDist)
				{
					minDist = dist;
					closestCorr = corrsIt->this_idx;
				}
			}
		} // End of for each correspondence...


		if (closestCorr!=-1)
		{	// Merge:		FUSION
			unsigned long w_b = getPoint(closestCorr,b);

			ASSERT_((w_a+w_b)>0);

			float	 F = 1.0f/(w_a+w_b);

			x[closestCorr]=F*(w_a*a.x+w_b*b.x);
			y[closestCorr]=F*(w_a*a.y+w_b*b.y);
			z[closestCorr]=F*(w_a*a.z+w_b*b.z);

			pointWeight[closestCorr] = w_a+w_b;

			// Append to fused points list
			if (notFusedPoints)
				(*notFusedPoints)[closestCorr] = false;
		}
		else
		{	// New point:	ADDITION
			x.push_back( a.x );
			y.push_back( a.y );
			z.push_back( a.z );
			pointWeight.push_back( 1 );
			if (notFusedPoints)
				(*notFusedPoints).push_back(false);
		}
	}


}

/*---------------------------------------------------------------
						insertPoint
 ---------------------------------------------------------------*/
void  CSimplePointsMap::insertPoint( float x, float y, float z )
{
	this->x.push_back(x);
	this->y.push_back(y);
	this->z.push_back(z);
	pointWeight.push_back(1);

	mark_as_modified();
}

/*---------------------------------------------------------------
						applyDeletionMask
 ---------------------------------------------------------------*/
void  CSimplePointsMap::applyDeletionMask( std::vector<bool> &mask )
{
	ASSERT_( getPointsCount()==mask.size() );

	size_t	i,j,n;

	// Remove marked points:
	n = mask.size();
	for (i=0,j=0;i<n;i++)
	{
		if (!mask[i])
		{
			x[j]				= x[i];
			y[j]				= y[i];
			z[j]				= z[i];
			pointWeight[j++]	= pointWeight[i];
		}
	}

	// Set new correct size:
	x.resize(j);
	y.resize(j);
	z.resize(j);
	pointWeight.resize(j);

	mark_as_modified();
}

/*---------------------------------------------------------------
					insertObservation

Insert the observation information into this map.
 ---------------------------------------------------------------*/
bool  CSimplePointsMap::internal_insertObservation(
	const CObservation	*obs,
    const CPose3D			*robotPose)
{
	MRPT_START;

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

	if (IS_CLASS(obs,CObservation2DRangeScan))
	{
		/********************************************************************

					OBSERVATION TYPE: CObservation2DRangeScan

			********************************************************************/
		mark_as_modified();

		const CObservation2DRangeScan *o = static_cast<const CObservation2DRangeScan *>(obs);
		// Insert only HORIZONTAL scans??
		bool	reallyInsertIt;

		if (insertionOptions.isPlanarMap)
			 reallyInsertIt = o->isPlanarScan( insertionOptions.horizontalTolerance );
		else reallyInsertIt = true;

		if (reallyInsertIt)
		{
			CSimplePointsMap	auxMap;
			size_t					i,n;
			CPose3D				sensorPose3D = robotPose3D + o->sensorPose;
			CPose2D				sensorPose2D( sensorPose3D );
			CPolygon			pol;
			const float			*xs,*ys,*zs;
			float				x,y;
			vector_int			fusedPointIndices;
			std::vector<bool>	checkForDeletion;

			// 1) Fuse into the points map or add directly?
			// ----------------------------------------------
			if (insertionOptions.fuseWithExisting)
			{
				// Fuse:
				auxMap.insertionOptions = insertionOptions;
				auxMap.insertionOptions.addToExistingPointsMap = false;

				auxMap.loadFromRangeScan(
					*o,						// The laser range scan observation
					&robotPose3D			// The robot pose
					);

				fuseWith(	&auxMap,					// Fuse with this map
							insertionOptions.minDistBetweenLaserPoints,	// Min dist.
							&checkForDeletion		// Set to "false" if a point in "map" has been fused.
							);
			}
			else
			{
				// Don't fuse: Simply add
				insertionOptions.addToExistingPointsMap = true;
				loadFromRangeScan(
					*o,						// The laser range scan observation
					&robotPose3D				// The robot pose
					);

				// Don't build this vector if is not used later!
				if (!insertionOptions.disableDeletion)
				{
					n = getPointsCount();
					checkForDeletion.resize(n);
					for (i=0;i<n;i++) checkForDeletion[i] = true;
				}
			}

			if (! insertionOptions.disableDeletion )
			{
				// 2) Delete points in newly added free
				//      region, thus dynamic areas:
				// --------------------------------------
				// Load scan as a polygon:
				auxMap.getPointsBuffer( n, xs,ys,zs);
				pol.setAllVertices( n, xs, ys );

				// Check for deletion of points in "map"
				n = getPointsCount();
				for (i=0;i<n;i++)
				{
					if ( checkForDeletion[i] )		// Default to true, unless a fused point, which must be kept.
					{
						getPoint(i,x,y);
						if ( !pol.PointIntoPolygon( x,y) )
							checkForDeletion[i] = false;	// Out of polygon, don't delete
					}
				}

				// Create a new points list just with non-deleted points.
				// ----------------------------------------------------------
				applyDeletionMask( checkForDeletion );
			}


			return true;
		}
		// A planar map and a non-horizontal scan.
		else return false;
	}
	else
	if (IS_CLASS(obs,CObservation3DRangeScan))
	{
		/********************************************************************

					OBSERVATION TYPE: CObservation3DRangeScan

			********************************************************************/
		mark_as_modified();

		const CObservation3DRangeScan *o = static_cast<const CObservation3DRangeScan *>(obs);
		// Insert only HORIZONTAL scans??
		bool	reallyInsertIt;

		if (insertionOptions.isPlanarMap)
			 reallyInsertIt = false; // Don't insert 3D range observation into planar map
		else reallyInsertIt = true;

		if (reallyInsertIt)
		{
			CSimplePointsMap	auxMap;

			// 1) Fuse into the points map or add directly?
			// ----------------------------------------------
			if (insertionOptions.fuseWithExisting)
			{
				// Fuse:
				auxMap.insertionOptions = insertionOptions;
				auxMap.insertionOptions.addToExistingPointsMap = false;

				auxMap.loadFromRangeScan(
					*o,						// The laser range scan observation
					&robotPose3D			// The robot pose
					);

				fuseWith(	&auxMap,					// Fuse with this map
							insertionOptions.minDistBetweenLaserPoints,	// Min dist.
							NULL			// rather than &checkForDeletion which we don't need for 3D observations
							);
			}
			else
			{
				// Don't fuse: Simply add
				insertionOptions.addToExistingPointsMap = true;
				loadFromRangeScan(
					*o,						// The laser range scan observation
					&robotPose3D			// The robot pose
					);
			}

			// This could be implemented to check whether existing points fall into empty-space 3D polygon
			// but performance for standard Swissranger scans (176*144 points) may be too sluggish?
			//if (! insertionOptions.disableDeletion )
			//{
			//	// 2) Delete points in newly added free
			//	//      region, thus dynamic areas
			//  // --------------------------------------
			//}


			return true;
		}
		// A planar map and a non-horizontal scan.
		else return false;
	}
	else
	if ( IS_CLASS(obs,CObservationRange))
	{
		/********************************************************************
					OBSERVATION TYPE: CObservationRange  (IRs, Sonars, etc.)
		 ********************************************************************/
		mark_as_modified();

		const CObservationRange* o = static_cast<const CObservationRange*>(obs);

		const double aper_2 = 0.5*o->sensorConeApperture;

		this->reserve( this->size() + o->sensedData.size()*30 );  // faster push_back's.

		for (CObservationRange::const_iterator it=o->begin();it!=o->end();++it)
		{
			const CPose3D sensorPose = robotPose3D + CPose3D(it->sensorPose);
			const double rang = it->sensedDistance;

			if (rang<=0 || rang<o->minSensorDistance || rang>o->maxSensorDistance)
				continue;

			// Insert a few points with a given maximum separation between them:
			const double arc_len = o->sensorConeApperture*rang;
			const unsigned int nSteps = round(1+arc_len/0.05);
			const double Aa = o->sensorConeApperture/double(nSteps);
			TPoint3D loc, glob;

			for (double a1=-aper_2;a1<aper_2;a1+=Aa)
			{
				for (double a2=-aper_2;a2<aper_2;a2+=Aa)
				{
					loc.x = cos(a1)*cos(a2)*rang;
					loc.y = cos(a1)*sin(a2)*rang;
					loc.z = sin(a1)*rang;
					sensorPose.composePoint(loc,glob);

					x.push_back( glob.x );
					y.push_back( glob.y );
					z.push_back( glob.z );
					pointWeight.push_back( 1 );
				}
			}
		}
		return true;
	}
	else
	{
		/********************************************************************
					OBSERVATION TYPE: Unknown
		********************************************************************/
		return false;
	}

	MRPT_END;
}


/*---------------------------------------------------------------
					insertAnotherMap
 ---------------------------------------------------------------*/
void  CSimplePointsMap::insertAnotherMap(	CPointsMap			*otherMap,
											const CPose2D		&otherPose)
{
	size_t							N_this = size();
	size_t							N_other = otherMap->size();
	vector<float>::iterator					xs,ys,zs;
	std::vector<uint32_t>::iterator	ws;
	CPoint3D								p,pp;

	mark_as_modified();

	// Reserve:
	reserve( N_this + N_other );

	for (xs=otherMap->x.begin(), ys=otherMap->y.begin(), zs=otherMap->z.begin(), ws=otherMap->pointWeight.begin();
				xs!=otherMap->x.end();
		 xs++,ys++,zs++,ws++)
	{
		// Load the next point:
		p.x( *xs );
		p.y( *ys );
		p.z( *zs );

		// Translation:
		pp = otherPose + p;

		// Add to this map:
		x.push_back( pp.x() );
		y.push_back( pp.y() );
		z.push_back( pp.z() );
		pointWeight.push_back( *ws );
	}
}

/*---------------------------------------------------------------
					auxParticleFilterCleanUp
 ---------------------------------------------------------------*/
void  CSimplePointsMap::auxParticleFilterCleanUp()
{

}

/*---------------------------------------------------------------
					auxParticleFilterCleanUp
 ---------------------------------------------------------------*/
void CSimplePointsMap::reserve(size_t newLength)
{
	x.reserve( newLength );
	y.reserve( newLength );
	z.reserve( newLength );
	pointWeight.reserve( newLength );
}
