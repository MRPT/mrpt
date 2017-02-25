/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef INTERNAL_CHECKERBOARD_INCL_H
#define INTERNAL_CHECKERBOARD_INCL_H

// Universal include for all versions of OpenCV
#include <mrpt/otherlibs/do_opencv_includes.h> 

#include <cmath>
#include <cstdio>

#if MRPT_HAS_OPENCV

// Debug visualizations...
//Ming #define VIS 1
#define VIS 0

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
	CvCBQuad() : count(0),group_idx(0),edge_len(0),labeled(false),area(0.0), area_ratio(1.0)
	{}

    int				count;							// Number of quad neihbors
    int				group_idx;						// Quad group ID
    float			edge_len;						// Smallest side length^2
    CvCBCornerPtr	corners[4];				//CvCBCorner *corners[4];				// Coordinates of quad corners
    CvCBQuadPtr		neighbors[4];		// Pointers of quad neighbors
	bool labeled;						// Has this corner been labeled?
	double          area, area_ratio; 
};


//===========================================================================
// PUBLIC FUNCTION PROTOTYPES
//===========================================================================
// Return: -1: errors, 0: not found, 1: found OK
int cvFindChessboardCorners3(
	const mrpt::utils::CImage & img_,
	CvSize pattern_size,
	std::vector<CvPoint2D32f> &out_corners);

// Return: true: found OK
bool find_chessboard_corners_multiple(
	const mrpt::utils::CImage & img_,
	CvSize pattern_size,
	std::vector< std::vector<CvPoint2D32f> > &out_corners);


//===========================================================================
// INTERNAL FUNCTION PROTOTYPES
//===========================================================================
int icvGenerateQuads( std::vector<CvCBQuadPtr> &quads, std::vector<CvCBCornerPtr> &corners,
                             const mrpt::utils::CImage &img, int flags, int dilation,
							 bool firstRun );

void mrFindQuadNeighbors2( std::vector<CvCBQuadPtr> &quads, int dilation);

int mrAugmentBestRun( std::vector<CvCBQuadPtr> &new_quads, int new_dilation,
							 std::vector<CvCBQuadPtr> &old_quads, int old_dilation );

void icvFindConnectedQuads(
	std::vector<CvCBQuadPtr> &in_quads,
	std::vector<CvCBQuadPtr> &out_quad_group,
	const int group_idx,
    const int dilation );

void mrLabelQuadGroup( std::vector<CvCBQuadPtr> &quad_group,  const CvSize &pattern_size, bool firstRun );

// Remove quads' extra quads until reached the expected number of quads.
void icvCleanFoundConnectedQuads( std::vector<CvCBQuadPtr> &quads, const CvSize &pattern_size );

// JL: Return 1 on success in finding all the quads, 0 on didn't, -1 on error.
int myQuads2Points( const std::vector<CvCBQuadPtr> &output_quads, const CvSize &pattern_size, std::vector<CvPoint2D32f> &out_corners);

// JL: Make unique all the (smart pointers-pointed) objects in the list and neighbors lists.
void quadListMakeUnique( std::vector<CvCBQuadPtr> &quads);

// JL: Refactored code from within cvFindChessboardCorners3() and alternative algorithm:
bool do_special_dilation(mrpt::utils::CImage &thresh_img, const int dilations,
	IplConvKernel *kernel_cross,
	IplConvKernel *kernel_rect,
	IplConvKernel *kernel_diag1,
	IplConvKernel *kernel_diag2,
	IplConvKernel *kernel_horz,
	IplConvKernel *kernel_vert
	);


#endif // MRPT_HAS_OPENCV

#endif

