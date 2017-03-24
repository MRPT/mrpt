/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef  KDTreeCapable_H
#define  KDTreeCapable_H

#include <mrpt/utils/utils_defs.h>

// nanoflann library:
#include <mrpt/otherlibs/nanoflann/nanoflann.hpp>
#include <mrpt/math/lightweight_geom_data.h>

namespace mrpt
{
	namespace math
	{
		/** \addtogroup kdtree_grp KD-Trees
		  *  \ingroup mrpt_base_grp
		  *  @{ */


		/** A generic adaptor class for providing Nearest Neighbor (NN) lookup via the `nanoflann` library.
		 *   This makes use of the CRTP design pattern.
		 *
		 *  Derived classes must be aware of the need to call "kdtree_mark_as_outdated()" when the data points
		 *   change to mark the cached KD-tree (an "index") as invalid, and also implement the following interface
		 *   (note that these are not virtual functions due to the usage of CRTP):
		 *
		 *  \code
		 *   // Must return the number of data points
		 *   inline size_t kdtree_get_point_count() const { ... }
		 *
		 *   // Returns the distance between the vector "p1[0:size-1]" and the data point with index "idx_p2" stored in the class:
		 *   inline float kdtree_distance(const float *p1, const size_t idx_p2,size_t size) const { ... }
		 *
		 *   // Returns the dim'th component of the idx'th point in the class:
		 *   inline num_t kdtree_get_pt(const size_t idx, int dim) const { ... }
		 *
		 *   // Optional bounding-box computation: return false to default to a standard bbox computation loop.
		 *   //   Return true if the BBOX was already computed by the class and returned in "bb" so it can be avoided to redo it again.
		 *   //   Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3 for point clouds)
		 *   template <class BBOX>
		 *   bool kdtree_get_bbox(BBOX &bb) const
		 *   {
		 *      bb[0].low = ...; bb[0].high = ...;  // "x" limits
		 *      return true;
		 *   }
		 *
		 *  \endcode
		 *
		 * The KD-tree index will be built on demand only upon call of any of the query methods provided by this class.
		 *
		 *  Notice that there is only ONE internal cached KD-tree, so if a method to query a 2D point is called,
		 *  then another method for 3D points, then again the 2D method, three KD-trees will be built. So, try
		 *  to group all the calls for a given dimensionality together or build different class instances for
		 *  queries of each dimensionality, etc.
		 *
		 *  \sa See some of the derived classes for example implementations. See also the documentation of nanoflann
		 * \ingroup mrpt_base_grp
		 */
		template <class Derived, typename num_t = float, typename metric_t = nanoflann::L2_Simple_Adaptor<num_t,Derived> >
		class KDTreeCapable
		{
		public:
			// Types ---------------
			typedef KDTreeCapable<Derived,num_t,metric_t>                 self_t;
			// ---------------------

			/// Constructor
			inline KDTreeCapable() : m_kdtree_is_uptodate(false)  { }

			/// Destructor (nothing needed to do here)
			inline ~KDTreeCapable() { }

			/// CRTP helper method
			inline const Derived& derived() const { return *static_cast<const Derived*>(this); }
			/// CRTP helper method
			inline       Derived& derived()       { return *static_cast<Derived*>(this); }

			struct TKDTreeSearchParams
			{
				TKDTreeSearchParams() :
					leaf_max_size(10)
				{
				}
				size_t leaf_max_size; //!< Max points per leaf
			};

			TKDTreeSearchParams  kdtree_search_params; //!< Parameters to tune the ANN searches

			/** @name Public utility methods to query the KD-tree
				@{ */

			/** KD Tree-based search for the closest point (only ONE) to some given 2D coordinates.
			  *  This method automatically build the "m_kdtree_data" structure when:
			  *		- It is called for the first time
			  *		- The map has changed
			  *		- The KD-tree was build for 3D.
			  *
			  * \param x0  The X coordinate of the query.
			  * \param y0  The Y coordinate of the query.
			  * \param out_x The X coordinate of the found closest correspondence.
			  * \param out_y The Y coordinate of the found closest correspondence.
			  * \param out_dist_sqr The square distance between the query and the returned point.
			  *
			  * \return The index of the closest point in the map array.
			  *  \sa kdTreeClosestPoint3D, kdTreeTwoClosestPoint2D
			  */
			inline size_t kdTreeClosestPoint2D(
				float   x0,
				float   y0,
				float   	  &out_x,
				float   	  &out_y,
				float		  &out_dist_sqr
				) const
			{
				MRPT_START
				rebuild_kdTree_2D(); // First: Create the 2D KD-Tree if required
				if ( !m_kdtree2d_data.m_num_points ) THROW_EXCEPTION("There are no points in the KD-tree.")

				const size_t knn = 1; // Number of points to retrieve
				size_t ret_index;
				nanoflann::KNNResultSet<num_t> resultSet(knn);
				resultSet.init(&ret_index, &out_dist_sqr );

				m_kdtree2d_data.query_point[0] = x0;
				m_kdtree2d_data.query_point[1] = y0;
		        m_kdtree2d_data.index->findNeighbors(resultSet, &m_kdtree2d_data.query_point[0], nanoflann::SearchParams());

				// Copy output to user vars:
				out_x = derived().kdtree_get_pt(ret_index,0);
				out_y = derived().kdtree_get_pt(ret_index,1);

				return ret_index;
				MRPT_END
			}

			/// \overload
			inline size_t kdTreeClosestPoint2D(
				float   x0,
				float   y0,
				float   &out_dist_sqr ) const
			{
				MRPT_START
				rebuild_kdTree_2D(); // First: Create the 2D KD-Tree if required
				if ( !m_kdtree2d_data.m_num_points ) THROW_EXCEPTION("There are no points in the KD-tree.")

				const size_t knn = 1; // Number of points to retrieve
				size_t ret_index;
				nanoflann::KNNResultSet<num_t> resultSet(knn);
				resultSet.init(&ret_index, &out_dist_sqr );

				m_kdtree2d_data.query_point[0] = x0;
				m_kdtree2d_data.query_point[1] = y0;
		        m_kdtree2d_data.index->findNeighbors(resultSet, &m_kdtree2d_data.query_point[0], nanoflann::SearchParams());

				return ret_index;
				MRPT_END
			}

			/// \overload
			inline size_t kdTreeClosestPoint2D(const TPoint2D &p0,TPoint2D &pOut,float &outDistSqr) const	{
				float dmy1,dmy2;
				size_t res=kdTreeClosestPoint2D(static_cast<float>(p0.x),static_cast<float>(p0.y),dmy1,dmy2,outDistSqr);
				pOut.x=dmy1;
				pOut.y=dmy2;
				return res;
			}

			/** Like kdTreeClosestPoint2D, but just return the square error from some point to its closest neighbor.
			  */
			inline float kdTreeClosestPoint2DsqrError(
				float   x0,
				float   y0 ) const
			{
				float closerx,closery,closer_dist;
				kdTreeClosestPoint2D(x0,y0, closerx,closery,closer_dist);
				return closer_dist;
			}

			inline float kdTreeClosestPoint2DsqrError(const TPoint2D &p0) const	{
				return kdTreeClosestPoint2DsqrError(static_cast<float>(p0.x),static_cast<float>(p0.y));
			}

			/** KD Tree-based search for the TWO closest point to some given 2D coordinates.
			  *  This method automatically build the "m_kdtree_data" structure when:
			  *		- It is called for the first time
			  *		- The map has changed
			  *		- The KD-tree was build for 3D.
			  *
			  * \param x0  The X coordinate of the query.
			  * \param y0  The Y coordinate of the query.
			  * \param out_x1 The X coordinate of the first correspondence.
			  * \param out_y1 The Y coordinate of the first correspondence.
			  * \param out_x2 The X coordinate of the second correspondence.
			  * \param out_y2 The Y coordinate of the second correspondence.
			  * \param out_dist_sqr1 The square distance between the query and the first returned point.
			  * \param out_dist_sqr2 The square distance between the query and the second returned point.
			  *
			  *  \sa kdTreeClosestPoint2D
			  */
			inline void kdTreeTwoClosestPoint2D(
				float   x0,
				float   y0,
				float   	  &out_x1,
				float   	  &out_y1,
				float   	  &out_x2,
				float   	  &out_y2,
				float		  &out_dist_sqr1,
				float		  &out_dist_sqr2 ) const
			{
				MRPT_START
				rebuild_kdTree_2D(); // First: Create the 2D KD-Tree if required
				if ( !m_kdtree2d_data.m_num_points ) THROW_EXCEPTION("There are no points in the KD-tree.")

				const size_t knn = 2; // Number of points to retrieve
				size_t  ret_indexes[2];
				float   ret_sqdist[2];
				nanoflann::KNNResultSet<num_t> resultSet(knn);
				resultSet.init(&ret_indexes[0], &ret_sqdist[0] );

				m_kdtree2d_data.query_point[0] = x0;
				m_kdtree2d_data.query_point[1] = y0;
		        m_kdtree2d_data.index->findNeighbors(resultSet, &m_kdtree2d_data.query_point[0], nanoflann::SearchParams());

				// Copy output to user vars:
				out_x1 = derived().kdtree_get_pt(ret_indexes[0],0);
				out_y1 = derived().kdtree_get_pt(ret_indexes[0],1);
				out_dist_sqr1 = ret_sqdist[0];

				out_x2 = derived().kdtree_get_pt(ret_indexes[1],0);
				out_y2 = derived().kdtree_get_pt(ret_indexes[1],1);
				out_dist_sqr2 = ret_sqdist[0];

				MRPT_END
			}


			inline void kdTreeTwoClosestPoint2D(const TPoint2D &p0,TPoint2D &pOut1,TPoint2D &pOut2,float &outDistSqr1,float &outDistSqr2) const	{
				float dmy1,dmy2,dmy3,dmy4;
				kdTreeTwoClosestPoint2D(p0.x,p0.y,dmy1,dmy2,dmy3,dmy4,outDistSqr1,outDistSqr2);
				pOut1.x=static_cast<double>(dmy1);
				pOut1.y=static_cast<double>(dmy2);
				pOut2.x=static_cast<double>(dmy3);
				pOut2.y=static_cast<double>(dmy4);
			}

			/** KD Tree-based search for the N closest point to some given 2D coordinates.
			  *  This method automatically build the "m_kdtree_data" structure when:
			  *		- It is called for the first time
			  *		- The map has changed
			  *		- The KD-tree was build for 3D.
			  *
			  * \param x0  The X coordinate of the query.
			  * \param y0  The Y coordinate of the query.
			  * \param N The number of closest points to search.
			  * \param out_x The vector containing the X coordinates of the correspondences.
			  * \param out_y The vector containing the Y coordinates of the correspondences.
			  * \param out_dist_sqr The vector containing the square distance between the query and the returned points.
			  *
			  * \return The list of indices
			  *  \sa kdTreeClosestPoint2D
			  *  \sa kdTreeTwoClosestPoint2D
			  */
			inline
			std::vector<size_t> kdTreeNClosestPoint2D(
				float			x0,
				float			y0,
				size_t  knn,
				std::vector<float>  &out_x,
				std::vector<float>  &out_y,
				std::vector<float>  &out_dist_sqr ) const
			{
				MRPT_START
				rebuild_kdTree_2D(); // First: Create the 2D KD-Tree if required
				if ( !m_kdtree2d_data.m_num_points ) THROW_EXCEPTION("There are no points in the KD-tree.")

				std::vector<size_t> ret_indexes(knn);
				out_x.resize(knn);
				out_y.resize(knn);
				out_dist_sqr.resize(knn);

				nanoflann::KNNResultSet<num_t> resultSet(knn);
				resultSet.init(&ret_indexes[0], &out_dist_sqr[0] );

				m_kdtree2d_data.query_point[0] = x0;
				m_kdtree2d_data.query_point[1] = y0;
		        m_kdtree2d_data.index->findNeighbors(resultSet, &m_kdtree2d_data.query_point[0], nanoflann::SearchParams());

				for (size_t i=0;i<knn;i++)
				{
					out_x[i] = derived().kdtree_get_pt(ret_indexes[i],0);
					out_y[i] = derived().kdtree_get_pt(ret_indexes[i],1);
				}
				return ret_indexes;
				MRPT_END
			}

			inline std::vector<size_t> kdTreeNClosestPoint2D(const TPoint2D &p0,size_t N,std::vector<TPoint2D> &pOut,std::vector<float> &outDistSqr) const	{
				std::vector<float> dmy1,dmy2;
				std::vector<size_t> res=kdTreeNClosestPoint2D(static_cast<float>(p0.x),static_cast<float>(p0.y),N,dmy1,dmy2,outDistSqr);
				pOut.resize(dmy1.size());
				for (size_t i=0;i<dmy1.size();i++)	{
					pOut[i].x=static_cast<double>(dmy1[i]);
					pOut[i].y=static_cast<double>(dmy2[i]);
				}
				return res;
			}

			/** KD Tree-based search for the N closest point to some given 2D coordinates and returns their indexes.
			  *  This method automatically build the "m_kdtree_data" structure when:
			  *		- It is called for the first time
			  *		- The map has changed
			  *		- The KD-tree was build for 3D.
			  *
			  * \param x0  The X coordinate of the query.
			  * \param y0  The Y coordinate of the query.
			  * \param N The number of closest points to search.
			  * \param out_idx The indexes of the found closest correspondence.
			  * \param out_dist_sqr The square distance between the query and the returned point.
			  *
			  *  \sa kdTreeClosestPoint2D
			  */
			inline void kdTreeNClosestPoint2DIdx(
				float			x0,
				float			y0,
				size_t  knn,
				std::vector<size_t>	&out_idx,
				std::vector<float>  &out_dist_sqr ) const
			{
				MRPT_START
				rebuild_kdTree_2D(); // First: Create the 2D KD-Tree if required
				if ( !m_kdtree2d_data.m_num_points ) THROW_EXCEPTION("There are no points in the KD-tree.")

				out_idx.resize(knn);
				out_dist_sqr.resize(knn);
				nanoflann::KNNResultSet<num_t> resultSet(knn);
				resultSet.init(&out_idx[0], &out_dist_sqr[0] );

				m_kdtree2d_data.query_point[0] = x0;
				m_kdtree2d_data.query_point[1] = y0;
		        m_kdtree2d_data.index->findNeighbors(resultSet, &m_kdtree2d_data.query_point[0], nanoflann::SearchParams());
				MRPT_END
			}

			inline void kdTreeNClosestPoint2DIdx(const TPoint2D &p0,size_t N,std::vector<size_t> &outIdx,std::vector<float> &outDistSqr) const	{
				return kdTreeNClosestPoint2DIdx(static_cast<float>(p0.x),static_cast<float>(p0.y),N,outIdx,outDistSqr);
			}

			/** KD Tree-based search for the closest point (only ONE) to some given 3D coordinates.
			  *  This method automatically build the "m_kdtree_data" structure when:
			  *		- It is called for the first time
			  *		- The map has changed
			  *		- The KD-tree was build for 2D.
			  *
			  * \param x0  The X coordinate of the query.
			  * \param y0  The Y coordinate of the query.
			  * \param z0  The Z coordinate of the query.
			  * \param out_x The X coordinate of the found closest correspondence.
			  * \param out_y The Y coordinate of the found closest correspondence.
			  * \param out_z The Z coordinate of the found closest correspondence.
			  * \param out_dist_sqr The square distance between the query and the returned point.
			  *
			  * \return The index of the closest point in the map array.
			  *  \sa kdTreeClosestPoint2D
			  */
			inline size_t kdTreeClosestPoint3D(
				float   x0,
				float   y0,
				float   z0,
				float   	  &out_x,
				float   	  &out_y,
				float   	  &out_z,
				float		  &out_dist_sqr
				) const
			{
				MRPT_START
				rebuild_kdTree_3D(); // First: Create the 3D KD-Tree if required
				if ( !m_kdtree3d_data.m_num_points ) THROW_EXCEPTION("There are no points in the KD-tree.")

				const size_t knn = 1; // Number of points to retrieve
				size_t ret_index;
				nanoflann::KNNResultSet<num_t> resultSet(knn);
				resultSet.init(&ret_index, &out_dist_sqr );

				m_kdtree3d_data.query_point[0] = x0;
				m_kdtree3d_data.query_point[1] = y0;
				m_kdtree3d_data.query_point[2] = z0;
		        m_kdtree3d_data.index->findNeighbors(resultSet, &m_kdtree3d_data.query_point[0], nanoflann::SearchParams());

				// Copy output to user vars:
				out_x = derived().kdtree_get_pt(ret_index,0);
				out_y = derived().kdtree_get_pt(ret_index,1);
				out_z = derived().kdtree_get_pt(ret_index,2);

				return ret_index;
				MRPT_END
			}

			/// \overload
			inline size_t kdTreeClosestPoint3D(
				float   x0,
				float   y0,
				float   z0,
				float		  &out_dist_sqr
				) const
			{
				MRPT_START
				rebuild_kdTree_3D(); // First: Create the 3D KD-Tree if required
				if ( !m_kdtree3d_data.m_num_points ) THROW_EXCEPTION("There are no points in the KD-tree.")

				const size_t knn = 1; // Number of points to retrieve
				size_t ret_index;
				nanoflann::KNNResultSet<num_t> resultSet(knn);
				resultSet.init(&ret_index, &out_dist_sqr );

				m_kdtree3d_data.query_point[0] = x0;
				m_kdtree3d_data.query_point[1] = y0;
				m_kdtree3d_data.query_point[2] = z0;
		        m_kdtree3d_data.index->findNeighbors(resultSet, &m_kdtree3d_data.query_point[0], nanoflann::SearchParams());

				return ret_index;
				MRPT_END
			}

			/// \overload
			inline size_t kdTreeClosestPoint3D(const TPoint3D &p0,TPoint3D &pOut,float &outDistSqr) const	{
				float dmy1,dmy2,dmy3;
				size_t res=kdTreeClosestPoint3D(static_cast<float>(p0.x),static_cast<float>(p0.y),static_cast<float>(p0.z),dmy1,dmy2,dmy3,outDistSqr);
				pOut.x=static_cast<double>(dmy1);
				pOut.y=static_cast<double>(dmy2);
				pOut.z=static_cast<double>(dmy3);
				return res;
			}

			/** KD Tree-based search for the N closest points to some given 3D coordinates.
			  *  This method automatically build the "m_kdtree_data" structure when:
			  *		- It is called for the first time
			  *		- The map has changed
			  *		- The KD-tree was build for 2D.
			  *
			  * \param x0  The X coordinate of the query.
			  * \param y0  The Y coordinate of the query.
			  * \param z0  The Z coordinate of the query.
			  * \param N The number of closest points to search.
			  * \param out_x The vector containing the X coordinates of the correspondences.
			  * \param out_y The vector containing the Y coordinates of the correspondences.
			  * \param out_z The vector containing the Z coordinates of the correspondences.
			  * \param out_dist_sqr The vector containing the square distance between the query and the returned points.
			  *
			  *  \sa kdTreeNClosestPoint2D
			  */
			inline void kdTreeNClosestPoint3D(
				float			x0,
				float			y0,
				float			z0,
				size_t  knn,
				std::vector<float>  &out_x,
				std::vector<float>  &out_y,
				std::vector<float>  &out_z,
				std::vector<float>  &out_dist_sqr ) const
			{
				MRPT_START
				rebuild_kdTree_3D(); // First: Create the 3D KD-Tree if required
				if ( !m_kdtree3d_data.m_num_points ) THROW_EXCEPTION("There are no points in the KD-tree.")

				std::vector<size_t> ret_indexes(knn);
				out_x.resize(knn);
				out_y.resize(knn);
				out_z.resize(knn);
				out_dist_sqr.resize(knn);

				nanoflann::KNNResultSet<num_t> resultSet(knn);
				resultSet.init(&ret_indexes[0], &out_dist_sqr[0] );

				m_kdtree3d_data.query_point[0] = x0;
				m_kdtree3d_data.query_point[1] = y0;
				m_kdtree3d_data.query_point[2] = z0;
				m_kdtree3d_data.index->findNeighbors(resultSet, &m_kdtree3d_data.query_point[0], nanoflann::SearchParams());

				for (size_t i=0;i<knn;i++)
				{
					out_x[i] = derived().kdtree_get_pt(ret_indexes[i],0);
					out_y[i] = derived().kdtree_get_pt(ret_indexes[i],1);
					out_z[i] = derived().kdtree_get_pt(ret_indexes[i],2);
				}
				MRPT_END
			}

            /** KD Tree-based search for the N closest points to some given 3D coordinates.
			  *  This method automatically build the "m_kdtree_data" structure when:
			  *		- It is called for the first time
			  *		- The map has changed
			  *		- The KD-tree was build for 2D.
			  *
			  * \param x0  The X coordinate of the query.
			  * \param y0  The Y coordinate of the query.
			  * \param z0  The Z coordinate of the query.
			  * \param N The number of closest points to search.
			  * \param out_x The vector containing the X coordinates of the correspondences.
			  * \param out_y The vector containing the Y coordinates of the correspondences.
			  * \param out_z The vector containing the Z coordinates of the correspondences.
			  * \param out_idx The vector containing the indexes of the correspondences.
			  * \param out_dist_sqr The vector containing the square distance between the query and the returned points.
			  *
			  *  \sa kdTreeNClosestPoint2D
			  */
			inline void kdTreeNClosestPoint3DWithIdx(
				float			x0,
				float			y0,
				float			z0,
				size_t  knn,
				std::vector<float>  &out_x,
				std::vector<float>  &out_y,
				std::vector<float>  &out_z,
				std::vector<size_t>  &out_idx,
				std::vector<float>  &out_dist_sqr ) const
			{
				MRPT_START
				rebuild_kdTree_3D(); // First: Create the 3D KD-Tree if required
				if ( !m_kdtree3d_data.m_num_points ) THROW_EXCEPTION("There are no points in the KD-tree.")

				out_x.resize(knn);
				out_y.resize(knn);
				out_z.resize(knn);
				out_idx.resize(knn);
				out_dist_sqr.resize(knn);

				nanoflann::KNNResultSet<num_t> resultSet(knn);
				resultSet.init(&out_idx[0], &out_dist_sqr[0] );

				m_kdtree3d_data.query_point[0] = x0;
				m_kdtree3d_data.query_point[1] = y0;
				m_kdtree3d_data.query_point[2] = z0;
				m_kdtree3d_data.index->findNeighbors(resultSet, &m_kdtree3d_data.query_point[0], nanoflann::SearchParams());

				for (size_t i=0;i<knn;i++)
				{
					out_x[i] = derived().kdtree_get_pt(out_idx[i],0);
					out_y[i] = derived().kdtree_get_pt(out_idx[i],1);
					out_z[i] = derived().kdtree_get_pt(out_idx[i],2);
				}
				MRPT_END
			}

			inline void kdTreeNClosestPoint3D(const TPoint3D &p0,size_t N,std::vector<TPoint3D> &pOut,std::vector<float> &outDistSqr) const	{
				std::vector<float> dmy1,dmy2,dmy3;
				kdTreeNClosestPoint3D(static_cast<float>(p0.x),static_cast<float>(p0.y),static_cast<float>(p0.z),N,dmy1,dmy2,dmy3,outDistSqr);
				pOut.resize(dmy1.size());
				for (size_t i=0;i<dmy1.size();i++)	{
					pOut[i].x=static_cast<double>(dmy1[i]);
					pOut[i].y=static_cast<double>(dmy2[i]);
					pOut[i].z=static_cast<double>(dmy3[i]);
				}
			}

			/** KD Tree-based search for all the points within a given radius of some 3D point.
			  *  This method automatically build the "m_kdtree_data" structure when:
			  *		- It is called for the first time
			  *		- The map has changed
			  *		- The KD-tree was build for 2D.
			  *
			  * \param x0  The X coordinate of the query.
			  * \param y0  The Y coordinate of the query.
			  * \param z0  The Z coordinate of the query.
			  * \param maxRadiusSqr The square of the desired search radius.
			  * \param out_indices_dist The output list, with pairs of indeces/squared distances for the found correspondences.
			  * \return Number of found points.
			  *
			  *  \sa kdTreeRadiusSearch2D, kdTreeNClosestPoint3DIdx
			  */
			inline size_t kdTreeRadiusSearch3D(
				const num_t x0, const num_t y0, const num_t z0,
				const num_t maxRadiusSqr,
				std::vector<std::pair<size_t,num_t> >& out_indices_dist ) const
			{
				MRPT_START
				rebuild_kdTree_3D(); // First: Create the 3D KD-Tree if required
				out_indices_dist.clear();
				if ( m_kdtree3d_data.m_num_points!=0 )
				{
					const num_t xyz[3] = {x0,y0,z0};
					m_kdtree3d_data.index->radiusSearch(&xyz[0], maxRadiusSqr, out_indices_dist, nanoflann::SearchParams() );
				}
				return out_indices_dist.size();
				MRPT_END
			}

			/** KD Tree-based search for all the points within a given radius of some 2D point.
			  *  This method automatically build the "m_kdtree_data" structure when:
			  *		- It is called for the first time
			  *		- The map has changed
			  *		- The KD-tree was build for 3D.
			  *
			  * \param x0  The X coordinate of the query.
			  * \param y0  The Y coordinate of the query.
			  * \param maxRadiusSqr The square of the desired search radius.
			  * \param out_indices_dist The output list, with pairs of indeces/squared distances for the found correspondences.
			  * \return Number of found points.
			  *
			  *  \sa kdTreeRadiusSearch3D, kdTreeNClosestPoint2DIdx
			  */
			inline size_t kdTreeRadiusSearch2D(
				const num_t x0, const num_t y0,
				const num_t maxRadiusSqr,
				std::vector<std::pair<size_t,num_t> >& out_indices_dist ) const
			{
				MRPT_START
				rebuild_kdTree_2D(); // First: Create the 2D KD-Tree if required
				out_indices_dist.clear();
				if ( m_kdtree2d_data.m_num_points!=0 )
				{
					const num_t xyz[2] = {x0,y0};
					m_kdtree2d_data.index->radiusSearch(&xyz[0], maxRadiusSqr, out_indices_dist, nanoflann::SearchParams() );
				}
				return out_indices_dist.size();
				MRPT_END
			}

			/** KD Tree-based search for the N closest point to some given 3D coordinates and returns their indexes.
			  *  This method automatically build the "m_kdtree_data" structure when:
			  *		- It is called for the first time
			  *		- The map has changed
			  *		- The KD-tree was build for 2D.
			  *
			  * \param x0  The X coordinate of the query.
			  * \param y0  The Y coordinate of the query.
			  * \param z0  The Z coordinate of the query.
			  * \param N The number of closest points to search.
			  * \param out_idx The indexes of the found closest correspondence.
			  * \param out_dist_sqr The square distance between the query and the returned point.
			  *
			  *  \sa kdTreeClosestPoint2D,  kdTreeRadiusSearch3D
			  */
			inline void kdTreeNClosestPoint3DIdx(
				float			x0,
				float			y0,
				float			z0,
				size_t  knn,
				std::vector<size_t>	&out_idx,
				std::vector<float>  &out_dist_sqr ) const
			{
				MRPT_START
				rebuild_kdTree_3D(); // First: Create the 3D KD-Tree if required
				if ( !m_kdtree3d_data.m_num_points ) THROW_EXCEPTION("There are no points in the KD-tree.")

				out_idx.resize(knn);
				out_dist_sqr.resize(knn);
				nanoflann::KNNResultSet<num_t> resultSet(knn);
				resultSet.init(&out_idx[0], &out_dist_sqr[0] );

				m_kdtree3d_data.query_point[0] = x0;
				m_kdtree3d_data.query_point[1] = y0;
				m_kdtree3d_data.query_point[2] = z0;
				m_kdtree3d_data.index->findNeighbors(resultSet, &m_kdtree3d_data.query_point[0], nanoflann::SearchParams());
				MRPT_END
			}

			inline void kdTreeNClosestPoint3DIdx(const TPoint3D &p0,size_t N,std::vector<size_t> &outIdx,std::vector<float> &outDistSqr) const	{
				kdTreeNClosestPoint3DIdx(static_cast<float>(p0.x),static_cast<float>(p0.y),static_cast<float>(p0.z),N,outIdx,outDistSqr);
			}

			/* @} */

		protected:
			/** To be called by child classes when KD tree data changes. */
			inline void kdtree_mark_as_outdated() const { m_kdtree_is_uptodate = false; }

		private:
			/** Internal structure with the KD-tree representation (mainly used to avoid copying pointers with the = operator) */
			template <int _DIM = -1>
			struct TKDTreeDataHolder
			{
				/** Init the pointer to NULL. */
				inline TKDTreeDataHolder() : index(NULL),m_dim(_DIM), m_num_points(0) { }

				/** Copy constructor: It actually does NOT copy the kd-tree, a new object will be created if required!   */
				inline TKDTreeDataHolder(const TKDTreeDataHolder &)  : index(NULL),m_dim(_DIM), m_num_points(0) { }

				/** Copy operator: It actually does NOT copy the kd-tree, a new object will be created if required!  */
				inline TKDTreeDataHolder& operator =(const TKDTreeDataHolder &o) {
					if (&o!=this) clear();
					return *this;
				}

				/** Free memory (if allocated) */
				inline ~TKDTreeDataHolder() { clear(); }

				/** Free memory (if allocated)  */
				inline void clear()	{ mrpt::utils::delete_safe( index ); }

				typedef nanoflann::KDTreeSingleIndexAdaptor<metric_t,Derived, _DIM> kdtree_index_t;

				kdtree_index_t *index;  //!< NULL or the up-to-date index

				std::vector<num_t> query_point;
				size_t           m_dim;         //!< Dimensionality. typ: 2,3
				size_t           m_num_points;
			};

			mutable TKDTreeDataHolder<2>  m_kdtree2d_data;
			mutable TKDTreeDataHolder<3>  m_kdtree3d_data;
			mutable TKDTreeDataHolder<>   m_kdtreeNd_data;
			mutable bool                  m_kdtree_is_uptodate; //!< whether the KD tree needs to be rebuilt or not.

			/// Rebuild, if needed the KD-tree for 2D (nDims=2), 3D (nDims=3), ... asking the child class for the data points.
			void rebuild_kdTree_2D() const
			{
				typedef typename TKDTreeDataHolder<2>::kdtree_index_t  tree2d_t;

				if (!m_kdtree_is_uptodate) { m_kdtree2d_data.clear(); m_kdtree3d_data.clear(); m_kdtreeNd_data.clear(); }

				if (!m_kdtree2d_data.index)
				{
					// Erase previous tree:
					m_kdtree2d_data.clear();
					// And build new index:
					const size_t N = derived().kdtree_get_point_count();
					m_kdtree2d_data.m_num_points = N;
					m_kdtree2d_data.m_dim        = 2;
					m_kdtree2d_data.query_point.resize(2);
					if (N)
					{
						m_kdtree2d_data.index = new tree2d_t(2, derived(),  nanoflann::KDTreeSingleIndexAdaptorParams(kdtree_search_params.leaf_max_size) );
						m_kdtree2d_data.index->buildIndex();
					}
					m_kdtree_is_uptodate = true;
				}
			}

			/// Rebuild, if needed the KD-tree for 2D (nDims=2), 3D (nDims=3), ... asking the child class for the data points.
			void rebuild_kdTree_3D() const
			{
				typedef typename TKDTreeDataHolder<3>::kdtree_index_t  tree3d_t;

				if (!m_kdtree_is_uptodate) { m_kdtree2d_data.clear(); m_kdtree3d_data.clear(); m_kdtreeNd_data.clear(); }

				if (!m_kdtree3d_data.index)
				{
					// Erase previous tree:
					m_kdtree3d_data.clear();
					// And build new index:
					const size_t N = derived().kdtree_get_point_count();
					m_kdtree3d_data.m_num_points = N;
					m_kdtree3d_data.m_dim        = 3;
					m_kdtree3d_data.query_point.resize(3);
					if (N)
					{
						m_kdtree3d_data.index = new tree3d_t(3, derived(),  nanoflann::KDTreeSingleIndexAdaptorParams(kdtree_search_params.leaf_max_size) );
						m_kdtree3d_data.index->buildIndex();
					}
					m_kdtree_is_uptodate = true;
				}
			}

		};  // end of KDTreeCapable

		/**  @} */  // end of grouping

	} // End of namespace
} // End of namespace
#endif
