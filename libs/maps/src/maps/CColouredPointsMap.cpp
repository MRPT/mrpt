/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                   http://mrpt.sourceforge.net/                            |
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

#include <mrpt/slam/CColouredPointsMap.h>
#include <mrpt/slam/CSimplePointsMap.h>

#include <mrpt/utils/color_maps.h>
#include <mrpt/math/CPolygon.h>

#include <mrpt/opengl/CPointCloudColoured.h>

//#include <mrpt/vision/utils.h>
//#include <mrpt/vision/pinhole.h>

using namespace mrpt;
using namespace mrpt::slam;
using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace mrpt::math;
//using namespace mrpt::vision;
//using namespace mrpt::vision::pinhole;

IMPLEMENTS_SERIALIZABLE(CColouredPointsMap, CPointsMap,mrpt::slam)

/*---------------------------------------------------------------
						Constructor
  ---------------------------------------------------------------*/
CColouredPointsMap::CColouredPointsMap() :
	z_min(-10),
	z_max(10),
	bFilterByHeight(false)
{
}

/*---------------------------------------------------------------
						Destructor
  ---------------------------------------------------------------*/
CColouredPointsMap::~CColouredPointsMap()
{
}

/*---------------------------------------------------------------
						Copy constructor
  ---------------------------------------------------------------*/
void  CColouredPointsMap::copyFrom(const CPointsMap &obj)
{
	MRPT_START;

	if (this==&obj)
		return;

	x = obj.x;
	y = obj.y;
	z = obj.z;
	pointWeight = obj.pointWeight;
	m_color_R.assign(x.size(),1);
	m_color_G.assign(x.size(),1);
	m_color_B.assign(x.size(),1);
	m_min_dist.assign(x.size(),1e4);

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
void  CColouredPointsMap::loadFromRangeScan(
	const CObservation2DRangeScan		&rangeScan,
	const CPose3D						*robotPose )
{
	CPose3D		sensorPose3D;

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
		m_color_R.clear();
		m_color_G.clear();
		m_color_B.clear();
		m_min_dist.clear();
	}

	const int sizeRangeScan = rangeScan.scan.size();

	// For a great gain in efficiency:
	if ( x.size()+2*sizeRangeScan > x.capacity() )
	{
		reserve( (size_t) (x.size() * 1.2f) + 3*sizeRangeScan );
	}

	// --------------------------------------------------------------------------
	//  SPECIAL CASE OF HORIZONTAL SCAN: QUICKER IMPLEMENTATION
	// --------------------------------------------------------------------------
	CMatrixDouble44	HM;
	sensorPose3D.getHomogeneousMatrix(HM);

	// --------------------------------------------------------------------------
	//		GENERAL CASE OF SCAN WITH ARBITRARY 3D ORIENTATION
	// --------------------------------------------------------------------------
	{
		// For quicker access:
		float		m00 = HM.get_unsafe(0,0);
		float		m01 = HM.get_unsafe(0,1);
		float		m03 = HM.get_unsafe(0,3);
		float		m10 = HM.get_unsafe(1,0);
		float		m11 = HM.get_unsafe(1,1);
		float		m13 = HM.get_unsafe(1,3);
		float		m20 = HM.get_unsafe(2,0);
		float		m21 = HM.get_unsafe(2,1);
		float		m23 = HM.get_unsafe(2,3);

		float		lx_1,ly_1,lz_1,lx,ly,lz;		// Punto anterior y actual:
		float		lx_2,ly_2,lz_2;				// Punto antes del anterior

		// Initial last point:
		lx_1 = -100; ly_1 = -100; lz_1 = -100;
		lx_2 = -100; ly_2 = -100; lz_2 = -100;

		// ------------------------------------------------------
		//		Pass range scan to a set of 2D points:
		// ------------------------------------------------------
		Eigen::Array<float,Eigen::Dynamic,1>  scan_x(sizeRangeScan), scan_y(sizeRangeScan);

		// Use old method (1) or the faster cos/sin look-up-tables (0)?
#if 0
		// Old method (slower)
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

		for (int i=0;i<sizeRangeScan;i++)
		{
			scan_x[i] = rangeScan.scan[i] * cos( Ang );
			scan_y[i] = rangeScan.scan[i] * sin( Ang );
			Ang+=dA;
		}
#else
		// New faster method:
		const CSinCosLookUpTableFor2DScans::TSinCosValues & sincos_vals = m_scans_sincos_cache.getSinCosForScan(rangeScan);
		{
			const mrpt::vector_float scan_vals( rangeScan.scan ); // Convert from the std::vector
			// Vectorized (optimized) scalar multiplications:
			scan_x = scan_vals.array() * sincos_vals.ccos.array();
			scan_y = scan_vals.array() * sincos_vals.csin.array();
		}
#endif

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

		ASSERT_(colorScheme.z_max!=colorScheme.z_min);
		const float Az_1_color = 1.0/(colorScheme.z_max-colorScheme.z_min);
		float	pR=1,pG=1,pB=1;

		for (int i=0;i<sizeRangeScan;i++)
		{
			// Punto actual del scan:
			if ( rangeScan.validRange[i] )
			{
				const float rel_z = m20*scan_x[i] + m21*scan_y[i];

				lx = m00*scan_x[i] + m01*scan_y[i] + m03;
				ly = m10*scan_x[i] + m11*scan_y[i] + m13;
				lz = rel_z + m23;

				// Compute color:
				switch( colorScheme.scheme )
				{
				//case cmFromHeightRelativeToSensor:
				case cmFromHeightRelativeToSensorJet:
				case cmFromHeightRelativeToSensorGray:
					{
						float q = (rel_z-colorScheme.z_min)*Az_1_color;
						if (q<0) q=0; else if (q>1) q=1;

						if (colorScheme.scheme==cmFromHeightRelativeToSensorGray)
						{
							pR=pG=pB=q;
						}
						else
						{
							mrpt::utils::jet2rgb(q, pR,pG,pB);
						}
					}
					break;
				case cmFromIntensityImage:
					{
						// In 2D scans we don't have this channel.
						pR=pG=pB=1.0;
					}
					break;
				default:
					THROW_EXCEPTION("Unknown color scheme");
				}


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
								if( !bFilterByHeight || ( i_z >= z_min && i_z <= z_max ) )
								{
									x.push_back( i_x );
									y.push_back( i_y );
									z.push_back( i_z );
									pointWeight.push_back( 1 );
									m_color_R.push_back(pR);
									m_color_G.push_back(pG);
									m_color_B.push_back(pB);
									m_min_dist.push_back(1e4);
								} // end if
							} // end for
						} // End of interpolate:
					}

					if( !bFilterByHeight || (lz >= z_min && lz <= z_max ) )
					{
						x.push_back( lx );
						y.push_back( ly );
						z.push_back( lz );
						pointWeight.push_back( 1 );
						m_color_R.push_back(pR);
						m_color_G.push_back(pG);
						m_color_B.push_back(pB);
						m_min_dist.push_back(1e4);

						lastPointWasInserted = true;

						lx_2 = lx_1;
						ly_2 = ly_1;
						lz_2 = lz_1;

						lx_1 = lx;
						ly_1 = ly;
						lz_1 = lz;
					}
				}

			}

			// Save for next iteration:
			lastPointWasValid = rangeScan.validRange[i] != 0;

		}

		// The last point
		if (lastPointWasValid && !lastPointWasInserted)
		{
			if( !bFilterByHeight || (lz >= z_min && lz <= z_max ) )
			{
				x.push_back( lx );
				y.push_back( ly );
				z.push_back( lz );
				pointWeight.push_back( 1 );
				m_color_R.push_back(pR);
				m_color_G.push_back(pG);
				m_color_B.push_back(pB);
				m_min_dist.push_back(1e4);
			}
		}
	}
}

/*---------------------------------------------------------------
					LoadFromRangeScan
 Transform the range scan into a set of cartessian coordinated
   points, leaving a given min. distance between them.
  ---------------------------------------------------------------*/
void  CColouredPointsMap::loadFromRangeScan(
	const CObservation3DRangeScan		&rangeScan,
	const CPose3D						*robotPose )
{
	CPose3D		sensorPose3D;

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
		m_color_R.clear();
		m_color_G.clear();
		m_color_B.clear();
		m_min_dist.clear();
	}

	if (!rangeScan.hasPoints3D)
		return; // Nothing to do!

	const size_t sizeRangeScan = rangeScan.points3D_x.size();


	bool hasValidIntensityImage = false;
	size_t imgW=0, imgH=0;

	if (rangeScan.hasIntensityImage)
	{
		// assure the size matches?
		if (sizeRangeScan == rangeScan.intensityImage.getWidth() * rangeScan.intensityImage.getHeight() )
		{
			hasValidIntensityImage = true;
			imgW = rangeScan.intensityImage.getWidth();
			imgH = rangeScan.intensityImage.getHeight();
		}
	}


	// For a great gain in efficiency:
	if ( x.size()+sizeRangeScan> x.capacity() )
		reserve( size_t(x.size() + 1.1*sizeRangeScan) );

	// --------------------------------------------------------------------------
	//  SPECIAL CASE OF HORIZONTAL SCAN: QUICKER IMPLEMENTATION
	// --------------------------------------------------------------------------
	CMatrixDouble44	HM;
	sensorPose3D.getHomogeneousMatrix(HM);

	// --------------------------------------------------------------------------
	//		GENERAL CASE OF SCAN WITH ARBITRARY 3D ORIENTATION
	// --------------------------------------------------------------------------
	{
		// For quicker access:
		float		m00 = HM.get_unsafe(0,0);
		float		m01 = HM.get_unsafe(0,1);
		float		m02 = HM.get_unsafe(0,2);
		float		m03 = HM.get_unsafe(0,3);
		float		m10 = HM.get_unsafe(1,0);
		float		m11 = HM.get_unsafe(1,1);
		float		m12 = HM.get_unsafe(1,2);
		float		m13 = HM.get_unsafe(1,3);
		float		m20 = HM.get_unsafe(2,0);
		float		m21 = HM.get_unsafe(2,1);
		float		m22 = HM.get_unsafe(2,2);
		float		m23 = HM.get_unsafe(2,3);

		float		lx_1,ly_1,lz_1,lx,ly,lz;		// Punto anterior y actual:

		// Initial last point:
		lx_1 = -100; ly_1 = -100; lz_1 = -100;

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

		ASSERT_(colorScheme.z_max!=colorScheme.z_min);
		const float Az_1_color = 1.0/(colorScheme.z_max-colorScheme.z_min);
		float	pR=1,pG=1,pB=1;

		const float K_8u = 1.0f/255;

		const bool hasColorIntensityImg = hasValidIntensityImage && rangeScan.intensityImage.isColor();

		// running indices for the image pixels for the gray levels:
		// If both range & intensity images coincide (e.g. SwissRanger), then we can just
		// assign 3D points to image pixels one-to-one, but that's not the case if
		//
		const bool simple_3d_to_color_relation = (std::abs(rangeScan.relativePoseIntensityWRTDepth.norm())<1e-5);
		unsigned int img_idx_x = 0, img_idx_y = 0;

		// Will be used just if simple_3d_to_color_relation=false
		const float cx = rangeScan.cameraParamsIntensity.cx();
		const float cy = rangeScan.cameraParamsIntensity.cy();
		const float fx = rangeScan.cameraParamsIntensity.fx();
		const float fy = rangeScan.cameraParamsIntensity.fy();

		for (size_t i=0;i<sizeRangeScan;i++)
		{
			// Valid point?
			if ( rangeScan.points3D_x[i]!=0 && rangeScan.points3D_y[i]!=0 )
			{
				const float scan_x = rangeScan.points3D_x[i];
				const float scan_y = rangeScan.points3D_y[i];
				const float scan_z = rangeScan.points3D_z[i];

				const float rel_z = m20*scan_x + m21*scan_y + m22*scan_z;

				lx = m00*scan_x + m01*scan_y + m02*scan_z + m03;
				ly = m10*scan_x + m11*scan_y + m12*scan_z + m13;
				lz = rel_z + m23;

				// Compute color:
				switch( colorScheme.scheme )
				{
				//case cmFromHeightRelativeToSensor:
				case cmFromHeightRelativeToSensorJet:
				case cmFromHeightRelativeToSensorGray:
					{
						float q = (rel_z-colorScheme.z_min)*Az_1_color;
						if (q<0) q=0; else if (q>1) q=1;

						if (colorScheme.scheme==cmFromHeightRelativeToSensorGray)
						{
							pR=pG=pB=q;
						}
						else
						{
							mrpt::utils::jet2rgb(q, pR,pG,pB);
						}
					}
					break;
				case cmFromIntensityImage:
					{
						// Do we have to project the 3D point into the image plane??
						bool hasValidColor = false;
						if (simple_3d_to_color_relation)
						{
							hasValidColor=true;
						}
						else
						{
							// Do projection:
							TPoint3D  pt; // pt_wrt_colorcam;
							rangeScan.relativePoseIntensityWRTDepth.inverseComposePoint(
								scan_x,scan_y,scan_z,
								pt.x,pt.y,pt.z);

							// Project to image plane:
							if (pt.z)
							{
								img_idx_x = cx + fx * pt.x/pt.z;
								img_idx_y = cy + fy * pt.y/pt.z;

								hasValidColor=
									img_idx_x>=0 && img_idx_x<imgW &&
									img_idx_y>=0 && img_idx_y<imgH;
							}
						}

						if (hasValidColor && hasColorIntensityImg)
						{
							const uint8_t *c= rangeScan.intensityImage.get_unsafe(img_idx_x, img_idx_y, 0);
							pR= c[2] * K_8u;
							pG= c[1] * K_8u;
							pB= c[0] * K_8u;
						}
						else
						if (hasValidColor && hasValidIntensityImage)
						{
							uint8_t c= *rangeScan.intensityImage.get_unsafe(img_idx_x, img_idx_y, 0);
							pR=pG=pB= c * K_8u;
						}
						else
						{
							pR=pG=pB=1.0;
						}
					}
					break;
				default:
					THROW_EXCEPTION("Unknown color scheme");
				}

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
					m_color_R.push_back(pR);
					m_color_G.push_back(pG);
					m_color_B.push_back(pB);
					m_min_dist.push_back(1e4);

					lastPointWasInserted = true;

					lx_1 = lx;
					ly_1 = ly;
					lz_1 = lz;
				}

				lastPointWasValid = true;
			}
			else
			{
				lastPointWasValid = false;
			}

			// Advance the color pointer (just for simple cases, e.g. SwissRanger, not for Kinect)
			if (simple_3d_to_color_relation && hasValidIntensityImage)
			{
				if (++img_idx_x>=imgW)
				{
					img_idx_y++;
					img_idx_x=0;
				}
			}

		}

		// The last point
		if (lastPointWasValid && !lastPointWasInserted)
		{
			x.push_back( lx );
			y.push_back( ly );
			z.push_back( lz );
			pointWeight.push_back( 1 );
			m_color_R.push_back(pR);
			m_color_G.push_back(pG);
			m_color_B.push_back(pB);
			m_min_dist.push_back(1e4);
		}
	}
}


/*---------------------------------------------------------------
					load2D_from_text_file
  Load from a text file. In each line there are a point coordinates.
    Returns false if any error occured, true elsewere.
  ---------------------------------------------------------------*/
bool  CColouredPointsMap::load2D_from_text_file(std::string file)
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
	m_color_R.clear();
	m_color_G.clear();
	m_color_B.clear();
	m_min_dist.clear();

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

	m_color_R.assign(x.size(),1);
	m_color_G.assign(x.size(),1);
	m_color_B.assign(x.size(),1);
	m_min_dist.assign(x.size(),1e4);

	return true;

	MRPT_END;
}


/*---------------------------------------------------------------
					load3D_from_text_file
  Load from a text file. In each line there are a point coordinates.
    Returns false if any error occured, true elsewere.
  ---------------------------------------------------------------*/
bool  CColouredPointsMap::load3D_from_text_file(std::string file)
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
	m_color_R.clear();
	m_color_G.clear();
	m_color_B.clear();
	m_min_dist.clear();

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

	m_color_R.assign(x.size(),1);
	m_color_G.assign(x.size(),1);
	m_color_B.assign(x.size(),1);
	m_min_dist.assign(x.size(),1e4);

	return true;

	MRPT_END;
}


/*---------------------------------------------------------------
					writeToStream
   Implements the writing to a CStream capability of
     CSerializable objects
  ---------------------------------------------------------------*/
void  CColouredPointsMap::writeToStream(CStream &out, int *version) const
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
			//  << insertionOptions.matchStaticPointsOnly  // Removed in version 6
			<< insertionOptions.maxDistForInterpolatePoints;

		// Insertion as 3D:
		out << m_disableSaveAs3DObject;

		// Added in version 3:
		out << insertionOptions.horizontalTolerance;

		// V4:
		out << m_color_R << m_color_G << m_color_B << m_min_dist;

		// V5:
		likelihoodOptions.writeToStream(out);
	}
}

/*---------------------------------------------------------------
					readFromStream
   Implements the reading from a CStream capability of
      CSerializable objects
  ---------------------------------------------------------------*/
void  CColouredPointsMap::readFromStream(CStream &in, int version)
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

			if (version>=4)  // Color data
			{
				in >> m_color_R >> m_color_G >> m_color_B >> m_min_dist;
			}
			else
			{
				m_color_R.assign(x.size(),1.0f);
				m_color_G.assign(x.size(),1.0f);
				m_color_B.assign(x.size(),1.0f);
				m_min_dist.assign(x.size(),2000.0f);
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
void  CColouredPointsMap::internal_clear()
{
	// This swap() thing is the only way to really deallocate the memory.
	{ vector<float> empt;  x.swap(empt); }
	{ vector<float> empt;  y.swap(empt); }
	{ vector<float> empt;  z.swap(empt); }

	{ vector<float> empt;  m_color_R.swap(empt); }
	{ vector<float> empt;  m_color_G.swap(empt); }
	{ vector<float> empt;  m_color_B.swap(empt); }

	{ vector<float> empt;  m_min_dist.swap(empt); }

	mark_as_modified();
}

/*---------------------------------------------------------------
					setPoint
			Store a points coordinates:
  ---------------------------------------------------------------*/
void  CColouredPointsMap::setPoint(size_t index,CPoint2D &p)
{
	if (index>=this->x.size())
		THROW_EXCEPTION("Index out of bounds");

	this->x[index] = p.x();
	this->y[index] = p.y();
	this->z[index] = 0;

	mark_as_modified();
}
void  CColouredPointsMap::setPoint(size_t index,CPoint3D &p)
{
	if (index>=this->x.size())
		THROW_EXCEPTION("Index out of bounds");

	this->x[index] = p.x();
	this->y[index] = p.y();
	this->z[index] = p.z();
	mark_as_modified();
}
void  CColouredPointsMap::setPoint(size_t index,float x,float y)
{
	if (index>=this->x.size())
		THROW_EXCEPTION("Index out of bounds");

	this->x[index] = x;
	this->y[index] = y;
	this->z[index] = 0;
	mark_as_modified();
}
void  CColouredPointsMap::setPoint(size_t index,float x,float y,float z)
{
	if (index>=this->x.size())
		THROW_EXCEPTION("Index out of bounds");

	this->x[index] = x;
	this->y[index] = y;
	this->z[index] = z;
	mark_as_modified();
}

/** Changes a given point from map. First index is 0.
 * \exception Throws std::exception on index out of bound.
 */
void  CColouredPointsMap::setPoint(size_t index,float x, float y, float z, float R, float G, float B)
{
	if (index>=this->x.size())
		THROW_EXCEPTION("Index out of bounds");
	this->x[index] = x;
	this->y[index] = y;
	this->z[index] = z;
	this->m_color_R[index]=R;
	this->m_color_G[index]=G;
	this->m_color_B[index]=B;
	mark_as_modified();
}

/** Changes just the color of a given point from the map. First index is 0.
 * \exception Throws std::exception on index out of bound.
 */
void  CColouredPointsMap::setPointColor(size_t index,float R, float G, float B)
{
	if (index>=this->x.size())
		THROW_EXCEPTION("Index out of bounds");
	this->m_color_R[index]=R;
	this->m_color_G[index]=G;
	this->m_color_B[index]=B;
	// mark_as_modified();  // No need to rebuild KD-trees, etc...
}


/*---------------------------------------------------------------
Insert the contents of another map into this one, fusing the previous content with the new one.
 This means that points very close to existing ones will be "fused", rather than "added". This prevents
 the unbounded increase in size of these class of maps.
 ---------------------------------------------------------------*/
void  CColouredPointsMap::fuseWith(
			CPointsMap			*otherMap,
			float				minDistForFuse,
			std::vector<bool>	*notFusedPoints)
{
	TMatchingPairList			correspondences;
	TMatchingPairList::iterator	corrsIt;
	size_t						nThis,nOther,i;
	CPoint3D					a,b;
	float						corrRatio;
	static CPose2D				nullPose(0,0,0);

	mark_as_modified();

	nThis  =     this->getPointsCount();
	nOther = otherMap->getPointsCount();

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
	for (i=0;i<nOther;i++)
	{
		unsigned long	w_a = otherMap->getPoint(i,a);	// Get "local" point into "a"

		// Find closest correspondence of "a":
		int				closestCorr = -1;
		float			minDist	= 1e6;
		for (corrsIt = correspondences.begin(); corrsIt!=correspondences.end(); corrsIt++)
		{
			if (corrsIt->other_idx==i)
			{
				float	dist = square( corrsIt->other_x - corrsIt->this_x ) +
							   square( corrsIt->other_y - corrsIt->this_y ) +
							   square( corrsIt->other_z - corrsIt->this_z );
				if (minDist<dist)
				{
					minDist = dist;
					closestCorr = corrsIt->this_idx;
				}
			}
		} // End of for each correspondence...


		if (closestCorr!=-1)
		{	// Merge:		FUSION
			unsigned long w_b = CPointsMap::getPoint(closestCorr,b);

			ASSERT_((w_a+w_b)>0);

			float	 F = 1.0f/(w_a+w_b);

			x[closestCorr]=F*(w_a*a.x()+w_b*b.x());
			y[closestCorr]=F*(w_a*a.y()+w_b*b.y());
			z[closestCorr]=F*(w_a*a.z()+w_b*b.z());

			pointWeight[closestCorr] = w_a+w_b;

			// Append to fused points list
			if (notFusedPoints)
				(*notFusedPoints)[closestCorr] = false;
		}
		else
		{	// New point:	ADDITION
			x.push_back( a.x() );
			y.push_back( a.y() );
			z.push_back( a.z() );
			pointWeight.push_back( 1 );
			m_color_R.push_back(1);
			m_color_G.push_back(1);
			m_color_B.push_back(1);

			if (notFusedPoints) notFusedPoints->push_back(false);
		}
	}


}

/*---------------------------------------------------------------
						insertPoint
 ---------------------------------------------------------------*/
void  CColouredPointsMap::insertPoint( float x, float y, float z )
{
	this->x.push_back(x);
	this->y.push_back(y);
	this->z.push_back(z);
	pointWeight.push_back(1);
	m_color_R.push_back(1);
	m_color_G.push_back(1);
	m_color_B.push_back(1);

	mark_as_modified();
}

/*---------------------------------------------------------------
						insertPoint
 ---------------------------------------------------------------*/
void  CColouredPointsMap::insertPoint( float x, float y, float z, float R, float G, float B )
{
	this->x.push_back(x);
	this->y.push_back(y);
	this->z.push_back(z);
	pointWeight.push_back(1);
	m_color_R.push_back(R);
	m_color_G.push_back(G);
	m_color_B.push_back(B);

	mark_as_modified();
}

/*---------------------------------------------------------------
						insertPoint
 ---------------------------------------------------------------*/
void  CColouredPointsMap::insertPoint( CPoint3D p )
{
	x.push_back(p.x());
	y.push_back(p.y());
	z.push_back(p.z());
	pointWeight.push_back(1);
	m_color_R.push_back(1);
	m_color_G.push_back(1);
	m_color_B.push_back(1);

	mark_as_modified();
}

/*---------------------------------------------------------------
						applyDeletionMask
 ---------------------------------------------------------------*/
void  CColouredPointsMap::applyDeletionMask( std::vector<bool> &mask )
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
			m_color_R[j]		= m_color_R[i];
			m_color_G[j]		= m_color_G[i];
			m_color_B[j]		= m_color_B[i];
			pointWeight[j++]	= pointWeight[i];
		}
	}

	// Set new correct size:
	x.resize(j);
	y.resize(j);
	z.resize(j);
	pointWeight.resize(j);
	m_color_R.resize(j);
	m_color_G.resize(j);
	m_color_B.resize(j);


	mark_as_modified();
}

/*---------------------------------------------------------------
					insertObservation

Insert the observation information into this map.
 ---------------------------------------------------------------*/
bool  CColouredPointsMap::internal_insertObservation(
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

	if ( IS_CLASS(obs, CObservation2DRangeScan) )
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
			CColouredPointsMap	auxMap;
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
						CPointsMap::getPoint(i,x,y);
						if ( !pol.PointIntoPolygon( x,y) )
							checkForDeletion[i] = false;	// Out of polygon, don't delete
					}
				}

				// Create a new points list just with non-deleted points.
				// ----------------------------------------------------------
				applyDeletionMask( checkForDeletion );
			}

			//if( bFilterByHeight )
			//    filterByHeight( z_min, z_max );

			return true;
		}
		// A planar map and a non-horizontal scan.
		else return false;
	}
	else
	if ( IS_CLASS(obs,CObservation3DRangeScan) )
	{
		/********************************************************************

					OBSERVATION TYPE: CObservation3DRangeScan

			********************************************************************/
		mark_as_modified();

		const CObservation3DRangeScan *o = static_cast<const CObservation3DRangeScan *>(obs);
		// Insert only HORIZONTAL scans??
		bool	reallyInsertIt = true;

		if (insertionOptions.isPlanarMap)
			 reallyInsertIt = false; // Don't insert 3D scans in a planar map!

		if (reallyInsertIt)
		{
			CColouredPointsMap	auxMap;
			CPose3D				sensorPose3D = robotPose3D + o->sensorPose;
			CPose2D				sensorPose2D( sensorPose3D );
			CPolygon			pol;

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
					&robotPose3D				// The robot pose
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
	{
		return false;
	}

	MRPT_END;
}

/*---------------------------------------------------------------
					reserve
 ---------------------------------------------------------------*/
void CColouredPointsMap::reserve(size_t newLength)
{
	x.reserve( newLength );
	y.reserve( newLength );
	z.reserve( newLength );
	m_color_R.reserve( newLength );
	m_color_G.reserve( newLength );
	m_color_B.reserve( newLength );
	m_min_dist.reserve( newLength );
}

/*---------------------------------------------------------------
					getAs3DObject
 ---------------------------------------------------------------*/
void CColouredPointsMap::getAs3DObject( mrpt::opengl::CSetOfObjectsPtr	&outObj ) const
{
	ASSERT_(outObj);

	if (m_disableSaveAs3DObject)
		return;

	opengl::CPointCloudColouredPtr  obj = opengl::CPointCloudColoured::Create();

	obj->loadFromPointsMap(this);
	obj->setColor(1,1,1,  1.0);

	obj->setPointSize( mrpt::global_settings::POINTSMAPS_3DOBJECT_POINTSIZE );

	outObj->insert(obj);
}

/*---------------------------------------------------------------
					TColourOptions
 ---------------------------------------------------------------*/
CColouredPointsMap::TColourOptions::TColourOptions() :
	scheme(cmFromHeightRelativeToSensor),
	z_min(-10),
	z_max(10),
	d_max(5)
{
}

/*---------------------------------------------------------------
					TColourOptions
 ---------------------------------------------------------------*/
void CColouredPointsMap::TColourOptions::loadFromConfigFile(
	const mrpt::utils::CConfigFileBase  &source,
	const std::string &section)
{
	MRPT_LOAD_CONFIG_VAR_CAST( scheme, int, TColouringMethod,   source,section )
	MRPT_LOAD_CONFIG_VAR(z_min, float,		source,section)
	MRPT_LOAD_CONFIG_VAR(z_max, float,		source,section)
	MRPT_LOAD_CONFIG_VAR(d_max, float,		source,section)
}

/*---------------------------------------------------------------
					TColourOptions
 ---------------------------------------------------------------*/
void  CColouredPointsMap::TColourOptions::dumpToTextStream(CStream	&out) const
{
	out.printf("\n----------- [CColouredPointsMap::TColourOptions] ------------ \n\n");

	out.printf("scheme                                  = %u\n",	(unsigned)scheme);
	out.printf("z_min                                   = %f\n",	z_min );
	out.printf("z_max                                   = %f\n",	z_max );
	out.printf("d_max                                   = %f\n",	d_max );
}

/*---------------------------------------------------------------
					getPoint
 ---------------------------------------------------------------*/
void  CColouredPointsMap::getPoint( size_t index, float &x, float &y, float &z, float &R, float &G, float &B ) const
{
	if (index >= this->x.size())
		THROW_EXCEPTION("Index out of bounds");

	x = this->x[index];
	y = this->y[index];
	z = this->z[index];

	R = m_color_R[index];
	G = m_color_G[index];
	B = m_color_B[index];
}

/** Retrieves a point color (colors range is [0,1])
  */
void  CColouredPointsMap::getPointColor( size_t index, float &R, float &G, float &B ) const
{
	if (index >= this->x.size())
		THROW_EXCEPTION("Index out of bounds");

	R = m_color_R[index];
	G = m_color_G[index];
	B = m_color_B[index];
}

// This function is duplicated from "mrpt::vision::pinhole::projectPoint_with_distortion", to avoid
//  a dependency on mrpt-vision.
void aux_projectPoint_with_distortion(
	const mrpt::math::TPoint3D  &P,
	const mrpt::utils::TCamera  &params,
	mrpt::utils::TPixelCoordf  &pixel,
	bool accept_points_behind
	)
{
	// Pinhole model:
	const double x = P.x/P.z;
	const double y = P.y/P.z;

	// Radial distortion:
	const double r2 = square(x)+square(y);
	const double r4 = square(r2);
	const double r6 = r2*r4;

	pixel.x = params.cx() + params.fx() *(  x*(1+params.dist[0]*r2+params.dist[1]*r4+params.dist[4]*r6) + 2*params.dist[2]*x*y+params.dist[3]*(r2+2*square(x))  );
	pixel.y = params.cy() + params.fy() *(  y*(1+params.dist[0]*r2+params.dist[1]*r4+params.dist[4]*r6) + 2*params.dist[3]*x*y+params.dist[2]*(r2+2*square(y))  );
}


/*---------------------------------------------------------------
					getPoint
 ---------------------------------------------------------------*/
bool CColouredPointsMap::colourFromObservation( const CObservationImage &obs, const CPose3D &robotPose )
{
	// Check if image is not grayscale
	ASSERT_( obs.image.isColor() );

	CPose3D cameraPoseR;					// Get Camera Pose on the Robot(B) (CPose3D)
	CPose3D cameraPoseW;					// World Camera Pose

	obs.getSensorPose( cameraPoseR );
	cameraPoseW = robotPose + cameraPoseR;	// Camera Global Coordinates

	// Image Information
	unsigned int imgW = obs.image.getWidth();
	unsigned int imgH = obs.image.getHeight();

	std::vector<float>::iterator	itx,ity,itz,itr,itg,itb;

	// Projection related variables
	std::vector<TPixelCoordf>			projectedPoints;	// The set of projected points in the image
	std::vector<TPixelCoordf>::iterator itProPoints;		// Iterator for projectedPoints
	std::vector<int>					p_idx;
	std::vector<float>					p_dist;
	std::vector<unsigned int>			p_proj;

	// Get the N closest points
	kdTreeNClosestPoint2DIdx( cameraPoseW.x(), cameraPoseW.y(),					// query point
							   200000,										// number of points to search
							   p_idx, p_dist );								// indexes and distances of the returned points

	// Fill p3D vector
	for( size_t k = 0; k < p_idx.size(); k++ )
	{
		float d = sqrt(p_dist[k]);
		int idx = p_idx[k];
		if( d < colorScheme.d_max && d < m_min_dist[idx] )
		{
			TPixelCoordf px;
			aux_projectPoint_with_distortion(TPoint3D( x[idx], y[idx], z[idx] ),  obs.cameraParams, px, true);
			projectedPoints.push_back(px);
			p_proj.push_back(k);
		} // end if
	} // end for

	// Get channel order
	unsigned int chR,chG,chB;
	if( obs.image.getChannelsOrder()[0] == 'B' ) { chR = 2; chG = 1;	chB = 0; }
	else { chR = 0; chG = 1; chB = 2; }

	unsigned int n_proj = 0;
	const float factor = 1.0/255;	// Normalize pixels:

	// Get the colour of the projected points
	size_t k;
	for( itProPoints = projectedPoints.begin(), k = 0;
		 itProPoints != projectedPoints.end();
		 itProPoints++, k++ )
	{
		// Only get the points projected inside the image
		if( itProPoints->x >= 0 && itProPoints->x < imgW && itProPoints->y > 0 && itProPoints->y < imgH )
		{
			unsigned int ii = p_idx[p_proj[k]];
			uint8_t * p = obs.image( (unsigned int)itProPoints->x, (unsigned int)itProPoints->y );

			m_color_R[ii]	= p[chR]*factor; // R
			m_color_G[ii]	= p[chG]*factor; // G
			m_color_B[ii]	= p[chB]*factor; // B
			m_min_dist[ii]	= p_dist[p_proj[k]];

			n_proj++;
		}
	} // end for

	return true;
} // end colourFromObservation


void CColouredPointsMap::resetPointsMinDist( float defValue )
{
	m_min_dist.assign(x.size(),defValue);
}


bool  CColouredPointsMap::save3D_and_colour_to_text_file(const std::string &file) const
{
	FILE	*f=os::fopen(file.c_str(),"wt");
	if (!f) return false;

	for (unsigned int i=0;i<x.size();i++)
		os::fprintf(f,"%f %f %f %d %d %d\n",x[i],y[i],z[i],(uint8_t)(255*m_color_R[i]),(uint8_t)(255*m_color_G[i]),(uint8_t)(255*m_color_B[i]));
	//	os::fprintf(f,"%f %f %f %f %f %f %f\n",x[i],y[i],z[i],m_color_R[i],m_color_G[i],m_color_B[i],m_min_dist[i]);

	os::fclose(f);
	return true;

}

// ================================ PLY files import & export virtual methods ================================

/** In a base class, reserve memory to prepare subsequent calls to PLY_import_set_vertex */
void CColouredPointsMap::PLY_import_set_vertex_count(const size_t N)
{
	x.resize( N );
	y.resize( N );
	z.resize( N );
	m_color_R.resize( N );
	m_color_G.resize( N );
	m_color_B.resize( N );
	m_min_dist.resize( N );
}

/** In a base class, will be called after PLY_import_set_vertex_count() once for each loaded point. 
  *  \param pt_color Will be NULL if the loaded file does not provide color info.
  */
void CColouredPointsMap::PLY_import_set_vertex(const size_t idx, const mrpt::math::TPoint3Df &pt, const mrpt::utils::TColorf *pt_color)
{
	if (pt_color)
			this->setPoint(idx,pt.x,pt.y,pt.z,pt_color->R,pt_color->G,pt_color->B);
	else	this->setPoint(idx,pt.x,pt.y,pt.z);
}

/** In a base class, will be called after PLY_export_get_vertex_count() once for each exported point. 
  *  \param pt_color Will be NULL if the loaded file does not provide color info.
  */
void CColouredPointsMap::PLY_export_get_vertex(
	const size_t idx, 
	mrpt::math::TPoint3Df &pt, 
	bool &pt_has_color,
	mrpt::utils::TColorf &pt_color) const
{
	pt_has_color=true;
	
	pt.x = x[idx];
	pt.y = y[idx];
	pt.z = z[idx];

	pt_color.R = m_color_R[idx];
	pt_color.G = m_color_G[idx];
	pt_color.B = m_color_B[idx];
}
