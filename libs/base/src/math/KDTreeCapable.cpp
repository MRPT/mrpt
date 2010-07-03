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

#include <mrpt/base.h>  // Precompiled headers


#include <mrpt/math/KDTreeCapable.h>

using namespace mrpt;
using namespace mrpt::math;
using namespace mrpt::utils;


/** Auxiliary class, instantiated once globally with the idea of avoiding one memory leak of the ANN library if "annClose" is not called.
  */
class CAuxANNCloser
{
public:
	CAuxANNCloser() {  }
	~CAuxANNCloser() { annClose(); }
	void deleteKDTree( ANNkd_tree *kdtree )  //!< Some foolish method just to assure the linker does not blank this class ;-)
	{
		delete kdtree;
	}
};
CAuxANNCloser	theANNcloser;  //!< The single global instance of CAuxANNCloser


/*---------------------------------------------------------------
		ctor
 ---------------------------------------------------------------*/
KDTreeCapable::KDTreeCapable() :
	m_KDTreeDataIsUpToDate	(false)
{
}


/*---------------------------------------------------------------
		dtor
 ---------------------------------------------------------------*/
KDTreeCapable::~KDTreeCapable()
{
}


/*---------------------------------------------------------------
			rebuild_kdTree
---------------------------------------------------------------*/
void KDTreeCapable::rebuild_kdTree(size_t nDims) const
{
	if ( !KDTreeData.m_pDataTree  ||
	     !m_KDTreeDataIsUpToDate  ||
	      KDTreeData.m_nDim != nDims )
	{
		// Erase previous tree:
		KDTreeData.clear();

		// Create new KD-Tree
		const size_t N = this->kdtree_get_point_count();
		KDTreeData.m_nTreeSize = N;
		KDTreeData.m_nDim      = nDims;

		//allocate memory for query point and results
		KDTreeData.m_QueryPoint = annAllocPt(  KDTreeData.m_nDim );

		//allocate memory for data points
		KDTreeData.m_DataPoints = annAllocPts(KDTreeData.m_nTreeSize,KDTreeData.m_nDim);

		// Create the list of points in ANN's format:
		this->kdtree_fill_point_data(KDTreeData.m_DataPoints,nDims);

		//allocate memory for tree and build it
		KDTreeData.m_pDataTree=new ANNkd_tree(
			KDTreeData.m_DataPoints,
			KDTreeData.m_nTreeSize,
			KDTreeData.m_nDim );

		m_KDTreeDataIsUpToDate = true;
	}
}

/*---------------------------------------------------------------
						TKDTreeData
---------------------------------------------------------------*/
KDTreeCapable::TKDTreeData::TKDTreeData() :
	m_pDataTree  ( NULL ),
	m_DataPoints ( NULL ),
	m_QueryPoint ( NULL ),
	m_nTreeSize(0),
	m_nDim(0),
	m_nk(0)
{
}

/*---------------------------------------------------------------
				copy TKDTreeData
---------------------------------------------------------------*/
KDTreeCapable::TKDTreeData::TKDTreeData(const TKDTreeData &o) :
	m_pDataTree  ( NULL ),
	m_DataPoints ( NULL ),
	m_QueryPoint ( NULL ),
	m_nTreeSize(0),
	m_nDim(0),
	m_nk(0)
{
}

/*---------------------------------------------------------------
					TKDTreeData::operator =
---------------------------------------------------------------*/
KDTreeCapable::TKDTreeData& KDTreeCapable::TKDTreeData::operator =(const KDTreeCapable::TKDTreeData &o)
{
	if (this!=&o)
		clear();
	return *this;
}

/*---------------------------------------------------------------
						Destructor
---------------------------------------------------------------*/
KDTreeCapable::TKDTreeData::~TKDTreeData()
{
	clear();
}

/*---------------------------------------------------------------
						clear
---------------------------------------------------------------*/
void KDTreeCapable::TKDTreeData::clear()
{
	if(m_pDataTree!=NULL)
	{
		theANNcloser.deleteKDTree( m_pDataTree );
		m_pDataTree = NULL;
	}

	if(	m_DataPoints)
	{
		annDeallocPts(m_DataPoints);
		m_DataPoints = NULL;
	}

	if(m_QueryPoint!=NULL)
	{
		annDeallocPt(m_QueryPoint);
		m_QueryPoint = NULL;
	}
}


/*---------------------------------------------------------------
 KD Tree-based search for the closest point (only ONE) to some given 2D coordinates.
  *  This method automatically build the "KDTreeData" structure when:
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
  *  \sa kdTreeClosestPoint3D
---------------------------------------------------------------*/
size_t KDTreeCapable::kdTreeClosestPoint2D(
	float   x0,
	float   y0,
	float   	  &out_x,
	float   	  &out_y,
	float		  &out_dist_sqr ) const
{
	// First: Create the KD-Tree if required:
	rebuild_kdTree(2); // 2d

	if ( !KDTreeData.m_nTreeSize ) THROW_EXCEPTION("There are no points in the KD-tree.")

    KDTreeData.m_QueryPoint[0]=x0;
    KDTreeData.m_QueryPoint[1]=y0;

	ANNidx my_NearNeighbourIndex;

    KDTreeData.m_pDataTree->annkSearch(
		KDTreeData.m_QueryPoint,
		1,		// We want just ONE point
		&my_NearNeighbourIndex,
		&out_dist_sqr
		);

	// Copy output to user vars:
	size_t  idx = my_NearNeighbourIndex;
	out_x = KDTreeData.m_DataPoints[idx][0];
	out_y = KDTreeData.m_DataPoints[idx][1];

	return idx;
}

/*---------------------------------------------------------------
				kdTreeClosestPoint2DsqrError
---------------------------------------------------------------*/
float KDTreeCapable::kdTreeClosestPoint2DsqrError(
	float   x0,
	float   y0 ) const
{
	// First: Create the KD-Tree if required:
	rebuild_kdTree(2); // 2d

	if ( !KDTreeData.m_nTreeSize ) THROW_EXCEPTION("There are no points in the KD-tree.")

    KDTreeData.m_QueryPoint[0]=x0;
    KDTreeData.m_QueryPoint[1]=y0;

   	ANNidx my_NearNeighbourIndex;

	float  closestSqrErr;
    KDTreeData.m_pDataTree->annkSearch(
		KDTreeData.m_QueryPoint,
		1,		// We want just ONE point
		&my_NearNeighbourIndex,
		&closestSqrErr
		);

	return closestSqrErr;
}

/*---------------------------------------------------------------
				kdTreeTwoClosestPoint2D
---------------------------------------------------------------*/
void KDTreeCapable::kdTreeTwoClosestPoint2D(
	float   x0,
	float   y0,
	float   &out_x1,
	float   &out_y1,
	float   &out_x2,
	float   &out_y2,
	float	&out_dist_sqr1,
	float	&out_dist_sqr2 ) const
{
	// First: Create the KD-Tree if required:
	rebuild_kdTree(2); // 2d

	if ( KDTreeData.m_nTreeSize<2 ) THROW_EXCEPTION("Map must have at least 2 points.")

	ANNdist 		nearNeighbourDistances[2];
	ANNidx			nearNeighbourIndices[2];

    KDTreeData.m_QueryPoint[0]=x0;
    KDTreeData.m_QueryPoint[1]=y0;

    KDTreeData.m_pDataTree->annkSearch(
		KDTreeData.m_QueryPoint,
		2,		// We want TWO points
		nearNeighbourIndices,
		nearNeighbourDistances
		);

	// Copy output to user vars:
	out_dist_sqr1 = nearNeighbourDistances[0];
	out_dist_sqr2 = nearNeighbourDistances[1];

	size_t  idx1 = nearNeighbourIndices[0];
	out_x1 = KDTreeData.m_DataPoints[idx1][0];
	out_y1 = KDTreeData.m_DataPoints[idx1][1];

	size_t  idx2 = nearNeighbourIndices[1];
	out_x2 = KDTreeData.m_DataPoints[idx2][0];
	out_y2 = KDTreeData.m_DataPoints[idx2][1];
}

/*---------------------------------------------------------------
				kdTreeNClosestPoint2D
---------------------------------------------------------------*/
std::vector<size_t>  KDTreeCapable::kdTreeNClosestPoint2D(
	float			x0,
	float			y0,
	size_t  N,
	std::vector<float>  &out_x,
	std::vector<float>  &out_y,
	std::vector<float>  &out_dist_sqr ) const
{
	ASSERT_(N>0)

	// First: Create the KD-Tree if required:
	rebuild_kdTree(2); // 2d

	if ( KDTreeData.m_nTreeSize<N )
		THROW_EXCEPTION( format("KD-tree must have at least %u points.",(unsigned int)N) )

    KDTreeData.m_QueryPoint[0]=x0;
    KDTreeData.m_QueryPoint[1]=y0;

	std::vector<ANNdist> 	my_NearNeighbourDistances( N );
	std::vector<ANNidx> 	my_NearNeighbourIndices( N );

    KDTreeData.m_pDataTree->annkSearch(
		KDTreeData.m_QueryPoint,
		N,		// We want N points
		&my_NearNeighbourIndices[0],
		&my_NearNeighbourDistances[0]
		);

	std::vector<size_t> indices(N);

	out_x.resize(N);
	out_y.resize(N);
	out_dist_sqr.resize(N);

	// Copy output to user vars:
	for( size_t k = 0; k < N; k++ )
	{
		const size_t  idx = my_NearNeighbourIndices[k];
		indices[k]		= idx;
		out_x[k] = KDTreeData.m_DataPoints[idx][0];
		out_y[k] = KDTreeData.m_DataPoints[idx][1];
		out_dist_sqr[k] = my_NearNeighbourDistances[k];
	}

	return indices;
}

/*---------------------------------------------------------------
				kdTreeNClosestPoint3DIdx
---------------------------------------------------------------*/
void KDTreeCapable::kdTreeNClosestPoint2DIdx(
	float			x0,
	float			y0,
	size_t  N,
	std::vector<int>	&out_idx,
	std::vector<float>  &out_dist_sqr ) const
{
	ASSERT_(N>0)

	// First: Create the KD-Tree if required:
	rebuild_kdTree(2); // 2d

	if ( KDTreeData.m_nTreeSize<N )
		THROW_EXCEPTION( format("KD-tree must have at least %u points.",(unsigned int)N) )

    KDTreeData.m_QueryPoint[0]=x0;
    KDTreeData.m_QueryPoint[1]=y0;

	std::vector<ANNdist> my_NearNeighbourDistances( N );
	std::vector<ANNidx> my_NearNeighbourIndices( N );

	KDTreeData.m_pDataTree->annkSearch(
		KDTreeData.m_QueryPoint,
		N,		// We want N points
		&my_NearNeighbourIndices[0],
		&my_NearNeighbourDistances[0]
		);

	out_idx.resize(N);
	out_dist_sqr.resize(N);

	// Copy output to user vars:
	for( size_t k = 0; k < N; k++ )
	{
		out_idx[k]		= (int)my_NearNeighbourIndices[k];
		out_dist_sqr[k] = my_NearNeighbourDistances[k];
	}

}

/*---------------------------------------------------------------
 KD Tree-based search for the closest point (only ONE) to some given 3D coordinates.
*  This method automatically build the "KDTreeData" structure when:
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
---------------------------------------------------------------*/
size_t KDTreeCapable::kdTreeClosestPoint3D(
	float   x0,
	float   y0,
	float   z0,
	float   &out_x,
	float   &out_y,
	float   &out_z,
	float	&out_dist_sqr ) const
{
	// First: Create the KD-Tree if required:
	rebuild_kdTree(3); // 3d

	if ( !KDTreeData.m_nTreeSize) THROW_EXCEPTION("KD-tree must have at least 1 point")

    KDTreeData.m_QueryPoint[0]=x0;
    KDTreeData.m_QueryPoint[1]=y0;
    KDTreeData.m_QueryPoint[2]=z0;


	ANNidx my_NearNeighbourIndex;

    KDTreeData.m_pDataTree->annkSearch(
		KDTreeData.m_QueryPoint,
		1,		// We want just ONE point
		&my_NearNeighbourIndex,
		&out_dist_sqr
		);

	// Copy output to user vars:
	size_t  idx = my_NearNeighbourIndex;
	out_x = KDTreeData.m_DataPoints[idx][0];
	out_y = KDTreeData.m_DataPoints[idx][1];
	out_z = KDTreeData.m_DataPoints[idx][2];

	return idx;
}

/*---------------------------------------------------------------
				kdTreeNClosestPoint3D
---------------------------------------------------------------*/
void KDTreeCapable::kdTreeNClosestPoint3D(
	float			x0,
	float			y0,
	float			z0,
	size_t  N,
	std::vector<float>  &out_x,
	std::vector<float>  &out_y,
	std::vector<float>  &out_z,
	std::vector<float>  &out_dist_sqr ) const
{
	ASSERT_(N>0)

	// First: Create the KD-Tree if required:
	rebuild_kdTree(3); // 3d

	if ( !KDTreeData.m_nTreeSize) THROW_EXCEPTION(format("KD-tree must have at least %u point",(unsigned int)N ))


    KDTreeData.m_QueryPoint[0]=x0;
    KDTreeData.m_QueryPoint[1]=y0;
	KDTreeData.m_QueryPoint[2]=z0;

	std::vector<ANNdist> my_NearNeighbourDistances( N );
	std::vector<ANNidx> my_NearNeighbourIndices( N );

    KDTreeData.m_pDataTree->annkSearch(
		KDTreeData.m_QueryPoint,
		N,		// We want N points
		&my_NearNeighbourIndices[0],
		&my_NearNeighbourDistances[0]
		);

	out_x.resize(N);
	out_y.resize(N);
	out_z.resize(N);
	out_dist_sqr.resize(N);

	// Copy output to user vars:
	for( size_t k = 0; k < N; k++ )
	{
		const size_t  idx		= my_NearNeighbourIndices[k];
		out_x[k]		= KDTreeData.m_DataPoints[idx][0];
		out_y[k]		= KDTreeData.m_DataPoints[idx][1];
		out_z[k]		= KDTreeData.m_DataPoints[idx][2];
		out_dist_sqr[k] = my_NearNeighbourDistances[k];
	}
}

/*---------------------------------------------------------------
				kdTreeNClosestPoint3DIdx
---------------------------------------------------------------*/
void KDTreeCapable::kdTreeNClosestPoint3DIdx(
	float			x0,
	float			y0,
	float			z0,
	size_t  N,
	std::vector<int>	&out_idx,
	std::vector<float>  &out_dist_sqr ) const
{
	ASSERT_(N>0)

	// First: Create the KD-Tree if required:
	rebuild_kdTree(3); // 3d

	if ( !KDTreeData.m_nTreeSize) THROW_EXCEPTION(format("KD-tree must have at least %u point",(unsigned int)N ))


    KDTreeData.m_QueryPoint[0]=x0;
    KDTreeData.m_QueryPoint[1]=y0;
	KDTreeData.m_QueryPoint[2]=z0;

	std::vector<ANNdist> my_NearNeighbourDistances( N );
	std::vector<ANNidx> my_NearNeighbourIndices( N );

    //tictac.Tic();
	KDTreeData.m_pDataTree->annkSearch(
		KDTreeData.m_QueryPoint,
		N,		// We want N points
		&my_NearNeighbourIndices[0],
		&my_NearNeighbourDistances[0]
		);

	out_idx.resize(N);
	out_dist_sqr.resize(N);

	// Copy output to user vars:
	for( size_t k = 0; k < N; k++ )
	{
		out_idx[k]		= (int)my_NearNeighbourIndices[k];
		out_dist_sqr[k] = my_NearNeighbourDistances[k];
	}
}
