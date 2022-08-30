/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "maps-precomp.h"  // Precomp header
//
#include <mrpt/core/round.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/opengl/CTexturedPlane.h>
#include <mrpt/system/os.h>

using namespace mrpt;
using namespace mrpt::maps;
using namespace mrpt::obs;
using namespace mrpt::poses;
using namespace mrpt::img;
using namespace std;

/*---------------------------------------------------------------
					getAsImage
  ---------------------------------------------------------------*/
void COccupancyGridMap2D::getAsImage(
	CImage& img, bool verticalFlip, bool forceRGB, bool tricolor) const
{
	if (!tricolor)
	{
		if (!forceRGB)
		{  // 8bit gray-scale
			img.resize(m_size_x, m_size_y, mrpt::img::CH_GRAY);
			const cellType* srcPtr = &m_map[0];
			unsigned char* destPtr;
			for (unsigned int y = 0; y < m_size_y; y++)
			{
				if (!verticalFlip) destPtr = img(0, m_size_y - 1 - y);
				else
					destPtr = img(0, y);
				for (unsigned int x = 0; x < m_size_x; x++)
				{
					*destPtr++ = l2p_255(*srcPtr++);
				}
			}
		}
		else
		{  // 24bit RGB:
			img.resize(m_size_x, m_size_y, mrpt::img::CH_RGB);
			const cellType* srcPtr = &m_map[0];
			unsigned char* destPtr;
			for (unsigned int y = 0; y < m_size_y; y++)
			{
				if (!verticalFlip) destPtr = img(0, m_size_y - 1 - y);
				else
					destPtr = img(0, y);
				for (unsigned int x = 0; x < m_size_x; x++)
				{
					uint8_t c = l2p_255(*srcPtr++);
					*destPtr++ = c;
					*destPtr++ = c;
					*destPtr++ = c;
				}
			}
		}
	}
	else
	{
		// TRICOLOR: 0, 0.5, 1
		if (!forceRGB)
		{  // 8bit gray-scale
			img.resize(m_size_x, m_size_y, mrpt::img::CH_GRAY);
			const cellType* srcPtr = &m_map[0];
			unsigned char* destPtr;
			for (unsigned int y = 0; y < m_size_y; y++)
			{
				if (!verticalFlip) destPtr = img(0, m_size_y - 1 - y);
				else
					destPtr = img(0, y);
				for (unsigned int x = 0; x < m_size_x; x++)
				{
					uint8_t c = l2p_255(*srcPtr++);
					if (c < 120) c = 0;
					else if (c > 136)
						c = 255;
					else
						c = 127;
					*destPtr++ = c;
				}
			}
		}
		else
		{  // 24bit RGB:
			img.resize(m_size_x, m_size_y, mrpt::img::CH_RGB);
			const cellType* srcPtr = &m_map[0];
			unsigned char* destPtr;
			for (unsigned int y = 0; y < m_size_y; y++)
			{
				if (!verticalFlip) destPtr = img(0, m_size_y - 1 - y);
				else
					destPtr = img(0, y);
				for (unsigned int x = 0; x < m_size_x; x++)
				{
					uint8_t c = l2p_255(*srcPtr++);
					if (c < 120) c = 0;
					else if (c > 136)
						c = 255;
					else
						c = 127;

					*destPtr++ = c;
					*destPtr++ = c;
					*destPtr++ = c;
				}
			}
		}
	}
}

/*---------------------------------------------------------------
					getAsImageFiltered
  ---------------------------------------------------------------*/
void COccupancyGridMap2D::getAsImageFiltered(
	CImage& img, bool verticalFlip, bool forceRGB) const
{
	getAsImage(img, verticalFlip, forceRGB);

	// Do filtering to improve the noisy peaks in grids:
	if (insertionOptions.CFD_features_gaussian_size != 0)
		img.filterGaussian(
			img, round(insertionOptions.CFD_features_gaussian_size));
	if (insertionOptions.CFD_features_median_size != 0)
		img.filterMedian(img, round(insertionOptions.CFD_features_median_size));
}

void COccupancyGridMap2D::getVisualizationInto(
	mrpt::opengl::CSetOfObjects& o) const
{
	if (!genericMapParams.enableSaveAs3DObject) return;

	MRPT_START

	auto outObj = mrpt::opengl::CTexturedPlane::Create();

	outObj->setPlaneCorners(m_xMin, m_xMax, m_yMin, m_yMax);

	outObj->setLocation(0, 0, insertionOptions.mapAltitude);

	// Create the color & transparecy (alpha) images:
	CImage imgColor(m_size_x, m_size_y, mrpt::img::CH_GRAY);
	CImage imgTrans(m_size_x, m_size_y, mrpt::img::CH_GRAY);

	const cellType* srcPtr = &m_map[0];

	for (unsigned int y = 0; y < m_size_y; y++)
	{
		unsigned char* destPtr_color = imgColor(0, y);
		unsigned char* destPtr_trans = imgTrans(0, y);
		for (unsigned int x = 0; x < m_size_x; x++)
		{
			uint8_t cell255 = l2p_255(*srcPtr++);
			*destPtr_color++ = cell255;

			int8_t auxC = (int8_t)((signed short)cell255) - 127;
			*destPtr_trans++ = auxC > 0 ? (auxC << 1) : ((-auxC) << 1);
		}
	}

	outObj->assignImage(imgColor, imgTrans);
	o.insert(outObj);

	MRPT_END
}

/** Get a point cloud with all (border) occupied cells as points */
void COccupancyGridMap2D::getAsPointCloud(
	mrpt::maps::CSimplePointsMap& pm, const float occup_threshold) const
{
	pm.clear();
	pm.reserve(1000);

	// for all rows in the gridmap
	for (size_t i = 1; i + 1 < m_size_x; i++)
	{
		// for all columns in the gridmap
		for (size_t j = 1; j + 1 < m_size_y; j++)
		{
			// if there is an obstacle and *it is a borderline*:
			bool is_surrounded = true;
			for (int di = -1; di <= 1 && is_surrounded; di++)
				for (int dj = -1; dj <= 1 && is_surrounded; dj++)
					if ((di != 0 || dj != 0) &&
						getCell(i + di, j + dj) > occup_threshold)
						is_surrounded = false;

			if (getCell(i, j) < occup_threshold && !is_surrounded)
				pm.insertPoint(idx2x(i), idx2y(j));
		}
	}

	// Now, all outermost cells, without the surrounded condition:
	for (size_t i = 0; i < m_size_x; i++)
	{
		if (getCell(i, 0) < occup_threshold) pm.insertPoint(idx2x(i), idx2y(0));
		if (getCell(i, m_size_y - 1) < occup_threshold)
			pm.insertPoint(idx2x(i), idx2y(m_size_y - 1));
	}
	for (size_t j = 1; j + 1 < m_size_y; j++)
	{
		if (getCell(0, j) < occup_threshold) pm.insertPoint(idx2x(0), idx2y(j));
		if (getCell(m_size_x - 1, j) < occup_threshold)
			pm.insertPoint(idx2x(m_size_x - 1), idx2y(j));
	}
}
