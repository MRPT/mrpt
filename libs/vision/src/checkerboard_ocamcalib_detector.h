/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#pragma once

// Universal include for all versions of OpenCV
#include <mrpt/otherlibs/do_opencv_includes.h>
#include <mrpt/img/CImage.h>

#include <cmath>
#include <cstdio>

#include <memory>

#if MRPT_HAS_OPENCV

// Debug visualizations...
// Ming #define VIS 1
#define VIS 0

// Definition Contour Struct
struct CvContourEx
{
	CV_CONTOUR_FIELDS()
	int counter;
};

// Definition Corner Struct
struct CvCBCorner;

struct CvCBCorner
{
	using Ptr = std::shared_ptr<CvCBCorner>;
	CvCBCorner() = default;
	CvPoint2D32f pt;  // X and y coordinates
	int row{-1000};  // Row and column of the corner
	int column{-1000};  // in the found pattern
	bool needsNeighbor;  // Does the corner require a neighbor?
	int count{0};  // number of corner neighbors
	CvCBCorner::Ptr neighbors[4];  // pointer to all corner neighbors
};

// Definition Quadrangle Struct
// This structure stores information about the chessboard quadrange
struct CvCBQuad;

struct CvCBQuad
{
	using Ptr = std::shared_ptr<CvCBQuad>;
	CvCBQuad() = default;

	int count{0};  // Number of quad neihbors
	int group_idx{0};  // Quad group ID
	float edge_len{0};  // Smallest side length^2
	CvCBCorner::Ptr corners[4];  // CvCBCorner *corners[4];				//
	// Coordinates of quad corners
	CvCBQuad::Ptr neighbors[4];  // Pointers of quad neighbors
	bool labeled{false};  // Has this corner been labeled?
	double area{0.0}, area_ratio{1.0};
};

//===========================================================================
// PUBLIC FUNCTION PROTOTYPES
//===========================================================================
// Return: -1: errors, 0: not found, 1: found OK
int cvFindChessboardCorners3(
	const mrpt::img::CImage& img_, CvSize pattern_size,
	std::vector<CvPoint2D32f>& out_corners);

// Return: true: found OK
bool find_chessboard_corners_multiple(
	const mrpt::img::CImage& img_, CvSize pattern_size,
	std::vector<std::vector<CvPoint2D32f>>& out_corners);

//===========================================================================
// INTERNAL FUNCTION PROTOTYPES
//===========================================================================
int icvGenerateQuads(
	std::vector<CvCBQuad::Ptr>& quads, std::vector<CvCBCorner::Ptr>& corners,
	const mrpt::img::CImage& img, int flags, int dilation, bool firstRun);

void mrFindQuadNeighbors2(std::vector<CvCBQuad::Ptr>& quads, int dilation);

int mrAugmentBestRun(
	std::vector<CvCBQuad::Ptr>& new_quads, int new_dilation,
	std::vector<CvCBQuad::Ptr>& old_quads, int old_dilation);

void icvFindConnectedQuads(
	std::vector<CvCBQuad::Ptr>& in_quads,
	std::vector<CvCBQuad::Ptr>& out_quad_group, const int group_idx,
	const int dilation);

void mrLabelQuadGroup(
	std::vector<CvCBQuad::Ptr>& quad_group, const CvSize& pattern_size,
	bool firstRun);

// Remove quads' extra quads until reached the expected number of quads.
void icvCleanFoundConnectedQuads(
	std::vector<CvCBQuad::Ptr>& quads, const CvSize& pattern_size);

// JL: Return 1 on success in finding all the quads, 0 on didn't, -1 on error.
int myQuads2Points(
	const std::vector<CvCBQuad::Ptr>& output_quads, const CvSize& pattern_size,
	std::vector<CvPoint2D32f>& out_corners);

// JL: Make unique all the (smart pointers-pointed) objects in the list and
// neighbors lists.
void quadListMakeUnique(std::vector<CvCBQuad::Ptr>& quads);

// JL: Refactored code from within cvFindChessboardCorners3() and alternative
// algorithm:
bool do_special_dilation(
	mrpt::img::CImage& thresh_img, const int dilations,
	IplConvKernel* kernel_cross, IplConvKernel* kernel_rect,
	IplConvKernel* kernel_diag1, IplConvKernel* kernel_diag2,
	IplConvKernel* kernel_horz, IplConvKernel* kernel_vert);

#endif  // MRPT_HAS_OPENCV
