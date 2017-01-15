/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef cpointsmap_crtp_common_H
#define cpointsmap_crtp_common_H

#include <mrpt/utils/round.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/obs/CObservation2DRangeScan.h>

namespace mrpt
{
namespace maps
{
namespace detail
{

	template <class Derived>
	struct loadFromRangeImpl
	{
		static inline void  templ_loadFromRangeScan(
			Derived &obj,
			const mrpt::obs::CObservation2DRangeScan		&rangeScan,
			const mrpt::poses::CPose3D			*robotPose )
		{
			using namespace mrpt::poses;
			using mrpt::utils::square;
			using mrpt::utils::DEG2RAD;
			obj.mark_as_modified();

			// If robot pose is supplied, compute sensor pose relative to it.
			CPose3D sensorPose3D(UNINITIALIZED_POSE);
			if (!robotPose)
					sensorPose3D = rangeScan.sensorPose;
			else	sensorPose3D.composeFrom(*robotPose, rangeScan.sensorPose);

			// Insert vs. load and replace:
			if (!obj.insertionOptions.addToExistingPointsMap)
				obj.resize(0); // Resize to 0 instead of clear() so the std::vector<> memory is not actually deadllocated and can be reused.

			const int sizeRangeScan = rangeScan.scan.size();

			if (!sizeRangeScan)
				return; // Nothing to do.

			// For a great gain in efficiency:
			if ( obj.x.size()+sizeRangeScan > obj.x.capacity() )
			{
				obj.reserve( (size_t) (obj.x.size() * 1.2f) + 3*sizeRangeScan );
			}

			// GENERAL CASE OF SCAN WITH ARBITRARY 3D ORIENTATION:
			//  Specialize a bit the equations since we know that z=0 always for the scan in local coordinates:
			mrpt::maps::CPointsMap::TLaserRange2DInsertContext  lric(rangeScan);
			sensorPose3D.getHomogeneousMatrix(lric.HM);

			// For quicker access as "float" numbers:
			float		m00 = lric.HM.get_unsafe(0,0);
			float		m01 = lric.HM.get_unsafe(0,1);
			float		m03 = lric.HM.get_unsafe(0,3);
			float		m10 = lric.HM.get_unsafe(1,0);
			float		m11 = lric.HM.get_unsafe(1,1);
			float		m13 = lric.HM.get_unsafe(1,3);
			float		m20 = lric.HM.get_unsafe(2,0);
			float		m21 = lric.HM.get_unsafe(2,1);
			float		m23 = lric.HM.get_unsafe(2,3);

			float		lx_1,ly_1,lz_1,lx=0,ly=0,lz=0; // Punto anterior y actual:
			float		lx_2,ly_2;				 // Punto antes del anterior

			// Initial last point:
			lx_1 = -100; ly_1 = -100; lz_1 = -100;
			lx_2 = -100; ly_2 = -100;

			// Minimum distance between points to reduce high density scans:
			const bool   useMinDist = obj.insertionOptions.minDistBetweenLaserPoints>0;
			const float  minDistSqrBetweenLaserPoints = square( obj.insertionOptions.minDistBetweenLaserPoints );

			// ----------------------------------------------------------------
			//   Transform these points into 3D using the pose transformation:
			// ----------------------------------------------------------------
			bool	lastPointWasValid = true;
			bool	thisIsTheFirst = true;
			bool  	lastPointWasInserted = false;

			// Initialize extra stuff in derived class:
			pointmap_traits<Derived>::internal_loadFromRangeScan2D_init(obj, lric);

			// Resize now for efficiency, if there're invalid or filtered points, buffers
			//  will be reduced at the end:
			const size_t nPointsAtStart = obj.size();
			size_t nextPtIdx = nPointsAtStart;

			{
				const size_t expectedMaxSize = nPointsAtStart+(sizeRangeScan* (obj.insertionOptions.also_interpolate ? 3:1) );
				obj.x.resize( expectedMaxSize );
				obj.y.resize( expectedMaxSize );
				obj.z.resize( expectedMaxSize );
			}

			// ------------------------------------------------------
			//		Pass range scan to a set of 2D points:
			// ------------------------------------------------------
			// Use a LUT to convert ranges -> (x,y) ; Automatically computed upon first usage.
			const mrpt::obs::CSinCosLookUpTableFor2DScans::TSinCosValues & sincos_vals = obj.m_scans_sincos_cache.getSinCosForScan(rangeScan);

			// Build list of points in global coordinates:
			Eigen::Array<float,Eigen::Dynamic,1>  scan_gx(sizeRangeScan+3), scan_gy(sizeRangeScan+3),scan_gz(sizeRangeScan+3);  // The +3 is to assure there's room for "nPackets*4"
			{
		#if MRPT_HAS_SSE2
				// Number of 4-floats:
				size_t nPackets = sizeRangeScan/4;
				if ( (sizeRangeScan & 0x03)!=0) nPackets++;

				// We want to implement:
				//   scan_gx = m00*scan_x+m01*scan_y+m03;
				//   scan_gy = m10*scan_x+m11*scan_y+m13;
				//   scan_gz = m20*scan_x+m21*scan_y+m23;
				//
				//  With: scan_x = ccos*range
				//        scan_y = csin*range
				//
				const __m128 m00_4val = _mm_set1_ps(m00); // load 4 copies of the same value
				const __m128 m01_4val = _mm_set1_ps(m01);
				const __m128 m03_4val = _mm_set1_ps(m03);

				const __m128 m10_4val = _mm_set1_ps(m10);
				const __m128 m11_4val = _mm_set1_ps(m11);
				const __m128 m13_4val = _mm_set1_ps(m13);

				const __m128 m20_4val = _mm_set1_ps(m20);
				const __m128 m21_4val = _mm_set1_ps(m21);
				const __m128 m23_4val = _mm_set1_ps(m23);

				// Make sure the input std::vector<> has room enough for reads of 4-float at a time:
				// Invalid reads should not be a problem, but just for safety...
				// JLBC: OCT/2016: rangeScan.scan() is now, by design, ensured to hold vectors of 4*N capacity, so there is no need to call reserve() here.

				const float *ptr_in_scan = &rangeScan.scan[0];
				const float *ptr_in_cos  = &sincos_vals.ccos[0];
				const float *ptr_in_sin  = &sincos_vals.csin[0];

				float *ptr_out_x    = &scan_gx[0];
				float *ptr_out_y    = &scan_gy[0];
				float *ptr_out_z    = &scan_gz[0];

				for( ; nPackets; nPackets--, ptr_in_scan+=4, ptr_in_cos+=4, ptr_in_sin+=4,  ptr_out_x+=4, ptr_out_y+=4, ptr_out_z+=4 )
				{
					const __m128 scan_4vals = _mm_loadu_ps(ptr_in_scan);  // *Unaligned* load

					const __m128 xs = _mm_mul_ps(scan_4vals, _mm_load_ps(ptr_in_cos) );
					const __m128 ys = _mm_mul_ps(scan_4vals, _mm_load_ps(ptr_in_sin) );

					_mm_store_ps(ptr_out_x, _mm_add_ps(m03_4val, _mm_add_ps( _mm_mul_ps(xs,m00_4val), _mm_mul_ps(ys,m01_4val) ) ) );
					_mm_store_ps(ptr_out_y, _mm_add_ps(m13_4val, _mm_add_ps( _mm_mul_ps(xs,m10_4val), _mm_mul_ps(ys,m11_4val) ) ) );
					_mm_store_ps(ptr_out_z, _mm_add_ps(m23_4val, _mm_add_ps( _mm_mul_ps(xs,m20_4val), _mm_mul_ps(ys,m21_4val) ) ) );
				}
		#else  // MRPT_HAS_SSE2
				// The "+3" is to assure the buffer has room for the SSE2 method which works with 4-tuples of floats.
				Eigen::Array<float,Eigen::Dynamic,1>  scan_x(sizeRangeScan+3), scan_y(sizeRangeScan+3);

				// Convert from the std::vector format:
				const Eigen::Map<Eigen::Matrix<float,Eigen::Dynamic,1> > scan_vals( const_cast<float*>(&rangeScan.scan[0]),rangeScan.scan.size(),1 );
				// SinCos table allocates N+4 floats for the convenience of SSE2: Map to make it appears it has the correct size:
				const Eigen::Map<Eigen::Matrix<float,Eigen::Dynamic,1> > ccos( const_cast<float*>(&sincos_vals.ccos[0]),rangeScan.scan.size(),1 );
				const Eigen::Map<Eigen::Matrix<float,Eigen::Dynamic,1> > csin( const_cast<float*>(&sincos_vals.csin[0]),rangeScan.scan.size(),1 );

				// Vectorized (optimized) scalar multiplications:
				scan_x = scan_vals.array() * ccos.array();
				scan_y = scan_vals.array() * csin.array();

				// To global:
				// Non (manually) vectorized version:
				scan_gx = m00*scan_x+m01*scan_y+m03;
				scan_gy = m10*scan_x+m11*scan_y+m13;
				scan_gz = m20*scan_x+m21*scan_y+m23;
		#endif // MRPT_HAS_SSE2
			}


			for (int i=0;i<sizeRangeScan;i++)
			{
				if ( rangeScan.validRange[i] )
				{
					lx = scan_gx[i];
					ly = scan_gy[i];
					lz = scan_gz[i];

					// Specialized work in derived classes:
					pointmap_traits<Derived>::internal_loadFromRangeScan2D_prepareOneRange(obj,lx,ly,lz,lric);

					lastPointWasInserted = false;

					// Add if distance > minimum only:
					bool pt_pass_min_dist = true;
					float d2 = 0;
					if (useMinDist || obj.insertionOptions.also_interpolate)
					{
						if (!lastPointWasValid)
								pt_pass_min_dist = false;
						else
						{
							d2 = (square(lx-lx_1) + square(ly-ly_1) + square(lz-lz_1) );
							pt_pass_min_dist = (d2 > minDistSqrBetweenLaserPoints);
						}
					}

					if ( thisIsTheFirst || pt_pass_min_dist )
					{
						thisIsTheFirst = false;
						// Si quieren que interpolemos tb. los puntos lejanos, hacerlo:

						if (obj.insertionOptions.also_interpolate && i>1)
						{
							float changeInDirection;
							const float d = std::sqrt( d2 );

							if ((lx!=lx_1 || ly!=ly_1) && (lx_1!=lx_2 || ly_1!=ly_2) )
									changeInDirection = atan2(ly-ly_1,lx-lx_1)-atan2(ly_1-ly_2,lx_1-lx_2);
							else	changeInDirection = 0;

							// Conditions to really interpolate the points:
							if (d>=2*obj.insertionOptions.minDistBetweenLaserPoints &&
								d<obj.insertionOptions.maxDistForInterpolatePoints &&
								fabs(changeInDirection)<DEG2RAD(5) )
							{
								int nInterpol = mrpt::utils::round(d / (2*sqrt(minDistSqrBetweenLaserPoints)));

								for (int q=1;q<nInterpol;q++)
								{
									float i_x = lx_1 + q*(lx-lx_1)/nInterpol;
									float i_y = ly_1 + q*(ly-ly_1)/nInterpol;
									float i_z = lz_1 + q*(lz-lz_1)/nInterpol;
									if( !obj.m_heightfilter_enabled || ( i_z >= obj.m_heightfilter_z_min && i_z <= obj.m_heightfilter_z_max ) )
									{
										obj.x.push_back( i_x );
										obj.y.push_back( i_y );
										obj.z.push_back( i_z );
										// Allow derived classes to add any other information to that point:
										pointmap_traits<Derived>::internal_loadFromRangeScan2D_postPushBack(obj,lric);
									} // end if
								} // end for
							} // End of interpolate:
						}

						if( !obj.m_heightfilter_enabled || (lz >= obj.m_heightfilter_z_min && lz <= obj.m_heightfilter_z_max ) )
						{
							obj.x[nextPtIdx] = lx;
							obj.y[nextPtIdx] = ly;
							obj.z[nextPtIdx] = lz;
							nextPtIdx++;

							// Allow derived classes to add any other information to that point:
							pointmap_traits<Derived>::internal_loadFromRangeScan2D_postPushBack(obj,lric);

							lastPointWasInserted = true;
							if (useMinDist)
							{
								lx_2 = lx_1;
								ly_2 = ly_1;

								lx_1 = lx;
								ly_1 = ly;
								lz_1 = lz;
							}

						}
					}
				}

				// Save for next iteration:
				lastPointWasValid = rangeScan.validRange[i] != 0;
			}

			// The last point
			if (lastPointWasValid && !lastPointWasInserted)
			{
				if( !obj.m_heightfilter_enabled || (lz >= obj.m_heightfilter_z_min && lz <= obj.m_heightfilter_z_max ) )
				{
					obj.x[nextPtIdx] = lx;
					obj.y[nextPtIdx] = ly;
					obj.z[nextPtIdx] = lz;
					nextPtIdx++;
					// Allow derived classes to add any other information to that point:
					pointmap_traits<Derived>::internal_loadFromRangeScan2D_postPushBack(obj,lric);
				}
			}

			// Adjust size:
			obj.x.resize( nextPtIdx );
			obj.y.resize( nextPtIdx );
			obj.z.resize( nextPtIdx );
		}

		static inline void  templ_loadFromRangeScan(
			Derived &obj,
			const mrpt::obs::CObservation3DRangeScan		&rangeScan,
			const mrpt::poses::CPose3D						*robotPose )
		{
			using namespace mrpt::poses;
			using mrpt::utils::square;
			obj.mark_as_modified();

			// If robot pose is supplied, compute sensor pose relative to it.
			CPose3D sensorPose3D(UNINITIALIZED_POSE);
			if (!robotPose)
					sensorPose3D = rangeScan.sensorPose;
			else	sensorPose3D.composeFrom(*robotPose, rangeScan.sensorPose);

			// Insert vs. load and replace:
			if (!obj.insertionOptions.addToExistingPointsMap)
				obj.resize(0); // Resize to 0 instead of clear() so the std::vector<> memory is not actually deadllocated and can be reused.

			if (!rangeScan.hasPoints3D)
				return; // Nothing to do!

			const size_t sizeRangeScan = rangeScan.points3D_x.size();

			// For a great gain in efficiency:
			if ( obj.x.size()+sizeRangeScan> obj.x.capacity() )
				obj.reserve( size_t(obj.x.size() + 1.1*sizeRangeScan) );


			// GENERAL CASE OF SCAN WITH ARBITRARY 3D ORIENTATION:
			// --------------------------------------------------------------------------
			mrpt::maps::CPointsMap::TLaserRange3DInsertContext  lric(rangeScan);
			sensorPose3D.getHomogeneousMatrix(lric.HM);
			// For quicker access to values as "float" instead of "doubles":
			float		m00 = lric.HM.get_unsafe(0,0);
			float		m01 = lric.HM.get_unsafe(0,1);
			float		m02 = lric.HM.get_unsafe(0,2);
			float		m03 = lric.HM.get_unsafe(0,3);
			float		m10 = lric.HM.get_unsafe(1,0);
			float		m11 = lric.HM.get_unsafe(1,1);
			float		m12 = lric.HM.get_unsafe(1,2);
			float		m13 = lric.HM.get_unsafe(1,3);
			float		m20 = lric.HM.get_unsafe(2,0);
			float		m21 = lric.HM.get_unsafe(2,1);
			float		m22 = lric.HM.get_unsafe(2,2);
			float		m23 = lric.HM.get_unsafe(2,3);

			float		lx_1,ly_1,lz_1,lx=0,ly=0,lz=0;		// Punto anterior y actual:

			// Initial last point:
			lx_1 = -100; ly_1 = -100; lz_1 = -100;

			float  minDistSqrBetweenLaserPoints = square( obj.insertionOptions.minDistBetweenLaserPoints );

			// If the user doesn't want a minimum distance:
			if (obj.insertionOptions.minDistBetweenLaserPoints<=0)
				minDistSqrBetweenLaserPoints = -1;

			// ----------------------------------------------------------------
			//   Transform these points into 3D using the pose transformation:
			// ----------------------------------------------------------------
			bool	lastPointWasValid = true;
			bool	thisIsTheFirst = true;
			bool  	lastPointWasInserted = false;

			// Initialize extra stuff in derived class:
			pointmap_traits<Derived>::internal_loadFromRangeScan3D_init(obj,lric);

			for (size_t i=0;i<sizeRangeScan;i++)
			{
				// Valid point?
				if ( rangeScan.points3D_x[i]!=0 || rangeScan.points3D_y[i]!=0 || rangeScan.points3D_z[i]!=0 || obj.insertionOptions.insertInvalidPoints)
				{
					lric.scan_x = rangeScan.points3D_x[i];
					lric.scan_y = rangeScan.points3D_y[i];
					lric.scan_z = rangeScan.points3D_z[i];

					lx = m00*lric.scan_x + m01*lric.scan_y + m02*lric.scan_z + m03;
					ly = m10*lric.scan_x + m11*lric.scan_y + m12*lric.scan_z + m13;
					lz = m20*lric.scan_x + m21*lric.scan_y + m22*lric.scan_z + m23;

					// Specialized work in derived classes:
					pointmap_traits<Derived>::internal_loadFromRangeScan3D_prepareOneRange(obj,lx,ly,lz,lric);

					lastPointWasInserted = false;

					// Add if distance > minimum only:
					float d2 = (square(lx-lx_1) + square(ly-ly_1) + square(lz-lz_1) );
					if ( thisIsTheFirst || (lastPointWasValid && (d2 > minDistSqrBetweenLaserPoints)) )
					{
						thisIsTheFirst = false;

						obj.x.push_back( lx );
						obj.y.push_back( ly );
						obj.z.push_back( lz );
						// Allow derived classes to add any other information to that point:
						pointmap_traits<Derived>::internal_loadFromRangeScan3D_postPushBack(obj,lric);

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

				pointmap_traits<Derived>::internal_loadFromRangeScan3D_postOneRange(obj,lric);
			}

			// The last point
			if (lastPointWasValid && !lastPointWasInserted)
			{
				if (lx!=0 || ly!=0 || lz!=0)
				{
					obj.x.push_back( lx );
					obj.y.push_back( ly );
					obj.z.push_back( lz );
					// Allow derived classes to add any other information to that point:
					pointmap_traits<Derived>::internal_loadFromRangeScan3D_postPushBack(obj,lric);
				}
			}
		}

	};

} // end NS
} // end NS
} // end NS

#endif


