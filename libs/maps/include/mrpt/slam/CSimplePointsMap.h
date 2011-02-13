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
#ifndef CSimplePointsMap_H
#define CSimplePointsMap_H

#include <mrpt/slam/CPointsMap.h>
#include <mrpt/slam/CObservation2DRangeScan.h>
#include <mrpt/slam/CObservation3DRangeScan.h>
#include <mrpt/utils/CSerializable.h>
#include <mrpt/math/CMatrix.h>

#include <mrpt/maps/link_pragmas.h>

namespace mrpt
{
	namespace slam
	{

		DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CSimplePointsMap , CPointsMap, MAPS_IMPEXP )

		/** A cloud of points in 2D or 3D, which can be built from a sequence of laser scans.
		 *    This class stores the coordinates (x,y,z) and a "weight", or counter of how many times that point has been seen, used only if points fusion is enabled in the options structure.
		 * \sa CMetricMap, CPoint, mrpt::utils::CSerializable
		 */
		class MAPS_IMPEXP CSimplePointsMap : public CPointsMap
		{
			// This must be added to any CSerializable derived class:
			DEFINE_SERIALIZABLE( CSimplePointsMap )
		 public:

			 /** Destructor
			   */
			 virtual ~CSimplePointsMap();

			 /** Default constructor
			  */
			 CSimplePointsMap();

			 /** Copy operator
			  */
			 void  copyFrom(const CPointsMap &obj);

			/** Transform the range scan into a set of cartesian coordinated
			  *	 points. The options in "insertionOptions" are considered in this method.
			  * \param rangeScan The scan to be inserted into this map
			  * \param robotPose The robot 3D pose, default to (0,0,0|0deg,0deg,0deg). It is used to compute the sensor pose relative to the robot actual pose. Recall sensor pose is embeded in the observation class.
			  *
			  *   NOTE: Only ranges marked as "valid=true" in the observation will be inserted
			  *
			  * \sa CObservation2DRangeScan
			  */
			void  loadFromRangeScan(
					const CObservation2DRangeScan &rangeScan,
					const CPose3D				  *robotPose = NULL );

			/** Enter the set of cartesian coordinated points from the 3D range scan into
			  *	 the map. The options in "insertionOptions" are considered in this method.
			  * \param rangeScan The 3D scan to be inserted into this map
			  * \param robotPose The robot 3D pose, default to (0,0,0|0deg,0deg,0deg). It is used to compute the sensor pose relative to the robot actual pose. Recall sensor pose is embeded in the observation class.
			  *
			  *   NOTE: Only ranges marked as "valid=true" in the observation will be inserted
			  *
			  * \sa CObservation3DRangeScan
			  */
			void  loadFromRangeScan(
					const CObservation3DRangeScan &rangeScan,
					const CPose3D				  *robotPose = NULL );

			/** Load from a text file. In each line there are a point coordinates.
			 *   Returns false if any error occured, true elsewere.
			 */
			bool  load2D_from_text_file(std::string file);

			/** Load from a text file. In each line there are a point coordinates.
			 *   Returns false if any error occured, true elsewere.
			 */
			bool  load3D_from_text_file(std::string file);


			/** Insert the contents of another map into this one, fusing the previous content with the new one.
			 *    This means that points very close to existing ones will be "fused", rather than "added". This prevents
			 *     the unbounded increase in size of these class of maps.
			 *		NOTICE that "otherMap" is neither translated nor rotated here, so if this is desired it must done
			 *		 before calling this method.
			 * \param otherMap The other map whose points are to be inserted into this one.
			 * \param minDistForFuse Minimum distance (in meters) between two points, each one in a map, to be considered the same one and be fused rather than added.
			 * \param notFusedPoints If a pointer is supplied, this list will contain at output a list with a "bool" value per point in "this" map. This will be false/true according to that point having been fused or not.
			 * \sa insertAnotherMap
			 */
			void  fuseWith(	CPointsMap			*otherMap,
										float				minDistForFuse  = 0.02f,
										std::vector<bool>	*notFusedPoints = NULL);

			/** Insert the contents of another map into this one, without fusing close points.
			 * \param otherMap The other map whose points are to be inserted into this one.
			 * \param otherPose The pose of the other map in the coordinates of THIS map
			 * \sa fuseWith
			 */
			void  insertAnotherMap(
										CPointsMap			*otherMap,
										const CPose2D		&otherPose);

			/** Changes a given point from map, as a 2D point. First index is 0.
			 * \exception Throws std::exception on index out of bound.
			 */
			virtual void  setPoint(size_t index,CPoint2D &p);

			/** Changes a given point from map, as a 3D point. First index is 0.
			 * \exception Throws std::exception on index out of bound.
			 */
			virtual void  setPoint(size_t index,CPoint3D &p);

			/** Changes a given point from map. First index is 0.
			 * \exception Throws std::exception on index out of bound.
			 */
			virtual void  setPoint(size_t index,float x, float y);

			/** Changes a given point from map. First index is 0.
			 * \exception Throws std::exception on index out of bound.
			 */
			virtual void  setPoint(size_t index,float x, float y, float z);

			/** Provides a way to insert individual points into the map:
			  */
			void  insertPoint( float x, float y, float z = 0 );

			/** Provides a way to insert individual points into the map:
			  */
			void  insertPoint( const mrpt::math::TPoint3D &new_pnt ) {
				this->insertPoint(new_pnt.x,new_pnt.y,new_pnt.z);
			}

			/** Remove from the map the points marked in a bool's array as "true".
			  *
			  * \exception std::exception If mask size is not equal to points count.
			  */
			void  applyDeletionMask( std::vector<bool> &mask );

			/** Reserves memory for a given number of points: the size of the map does not change, it only reserves the memory.
			  *  This is useful for situations where it is approximately known the final size of the map. This method is more
			  *  efficient than constantly increasing the size of the buffers. Refer to the STL C++ library's "reserve" methods.
			  */
			void reserve(size_t newLength);

			/** Set all the points at once from vectors with X,Y and Z coordinates (if Z is not provided, it will be set to all zeros).
			  * \tparam VECTOR can be mrpt::vector_float or std::vector<float> or any other column or row Eigen::Matrix.
			  */
			template <typename VECTOR>
			inline void setAllPointsTemplate(const VECTOR &X,const VECTOR &Y,const VECTOR &Z = VECTOR())
			{
				const size_t N = X.size();
				ASSERT_EQUAL_(X.size(),Y.size())
				ASSERT_(Z.size()==0 || Z.size()==X.size())
				x.resize(N); y.resize(N); z.resize(N);
				const bool z_valid = Z.empty();
				if (z_valid) for (size_t i=0;i<N;i++) { this->x[i]=X[i]; this->y[i]=Y[i]; this->z[i]=Z[i]; }
				else         for (size_t i=0;i<N;i++) { this->x[i]=X[i]; this->y[i]=Y[i]; this->z[i]=0; }
				pointWeight.assign(N,1);
				mark_as_modified();
			}

			/** Set all the points at once from vectors with X,Y and Z coordinates. \sa getAllPoints */
			virtual void setAllPoints(const std::vector<float> &X,const std::vector<float> &Y,const std::vector<float> &Z)
			{
				setAllPointsTemplate(X,Y,Z);
			}

			/** Set all the points at once from vectors with X and Y coordinates (Z=0). \sa getAllPoints */
			virtual void setAllPoints(const std::vector<float> &X,const std::vector<float> &Y)
			{
				setAllPointsTemplate(X,Y);
			}

			/** If the map is a simple points map or it's a multi-metric map that contains EXACTLY one simple points map, return it. 
				* Otherwise, return NULL
				*/
			virtual const CSimplePointsMap * getAsSimplePointsMap() const { return this; }
			virtual       CSimplePointsMap * getAsSimplePointsMap()       { return this; }

		protected:
			/** Clear the map, erasing all the points.
			 */
			virtual void  internal_clear();

			 /** Insert the observation information into this map. This method must be implemented
			  *    in derived classes.
			  * \param obs The observation
			  * \param robotPose The 3D pose of the robot mobile base in the map reference system, or NULL (default) if you want to use CPose2D(0,0,deg)
			  *
			  * \sa CObservation::insertObservationInto
			  */
			bool  internal_insertObservation( const CObservation *obs, const CPose3D *robotPose = NULL );

		}; // End of class def.

	} // End of namespace
} // End of namespace

#endif
