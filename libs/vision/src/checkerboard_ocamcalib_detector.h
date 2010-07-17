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

#ifndef INTERNAL_CHECKERBOARD_INCL_H
#define INTERNAL_CHECKERBOARD_INCL_H

#include "do_opencv_includes.h"

#include <cmath>
#include <cstdio>

#if MRPT_HAS_OPENCV

// Definition Contour Struct
struct CvContourEx
{
    CV_CONTOUR_FIELDS()
    int counter;
};

// Definition Corner Struct
struct CvCBCorner;
typedef stlplus::smart_ptr<CvCBCorner> CvCBCornerPtr;

struct CvCBCorner
{
	CvCBCorner() : row(-1000),column(-1000), count(0)
	{}

    CvPoint2D32f	pt;				// X and y coordinates
	int				row;			// Row and column of the corner
	int				column;			// in the found pattern
	bool			needsNeighbor;	// Does the corner require a neighbor?
    int				count;			// number of corner neighbors
    CvCBCornerPtr	neighbors[4];	// pointer to all corner neighbors
};


// Definition Quadrangle Struct
// This structure stores information about the chessboard quadrange
struct CvCBQuad;
typedef stlplus::smart_ptr<CvCBQuad>  CvCBQuadPtr;

struct CvCBQuad
{
	CvCBQuad() : count(0),group_idx(0),edge_len(0),labeled(false)
	{}

    int				count;							// Number of quad neihbors
    int				group_idx;						// Quad group ID
    float			edge_len;						// Smallest side length^2
    CvCBCornerPtr	corners[4];				//CvCBCorner *corners[4];				// Coordinates of quad corners
    CvCBQuadPtr		neighbors[4];		// Pointers of quad neighbors
	bool labeled;						// Has this corner been labeled?
};


//===========================================================================
// PUBLIC FUNCTION PROTOTYPES
//===========================================================================
// Return: -1: errors, 0: not found, 1: found OK
int cvFindChessboardCorners3( const mrpt::utils::CImage & img_, CvSize pattern_size, std::vector<CvPoint2D32f> &out_corners);


//===========================================================================
// INTERNAL FUNCTION PROTOTYPES
//===========================================================================
int icvGenerateQuads( std::vector<CvCBQuadPtr> &quads, vector<CvCBCornerPtr> &corners,
                             const mrpt::utils::CImage &img, int flags, int dilation,
							 bool firstRun );

void mrFindQuadNeighbors2( std::vector<CvCBQuadPtr> &quads, int quad_count, int dilation);

int mrAugmentBestRun( std::vector<CvCBQuadPtr> &new_quads, int new_dilation,
							 std::vector<CvCBQuadPtr> &old_quads, int old_dilation );

int icvFindConnectedQuads( std::vector<CvCBQuadPtr> &quads, int quad_count, std::vector<CvCBQuadPtr> &quad_group,
								  int group_idx,
                                  int dilation );

void mrLabelQuadGroup( std::vector<CvCBQuadPtr> &quad_group, int count, CvSize pattern_size,
							  bool firstRun );

int icvCleanFoundConnectedQuads( int quad_count, std::vector<CvCBQuadPtr> &quads, CvSize pattern_size );

// Return 1 on success in finding all the quads, 0 on didn't, -1 on error.
int myQuads2Points( const std::vector<CvCBQuadPtr> &output_quads, const CvSize &pattern_size, std::vector<CvPoint2D32f> &out_corners);


#endif // MRPT_HAS_OPENCV

#endif 

