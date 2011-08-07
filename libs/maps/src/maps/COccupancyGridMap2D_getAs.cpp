/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
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


#include <mrpt/slam/COccupancyGridMap2D.h>
#include <mrpt/opengl.h>

using namespace mrpt;
using namespace mrpt::slam;
using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace std;



/*---------------------------------------------------------------
					getAsImage
  ---------------------------------------------------------------*/
void  COccupancyGridMap2D::getAsImage(
	utils::CImage	&img,
	bool verticalFlip,
	bool forceRGB,
	bool tricolor ) const
{
	if (!tricolor)
	{
		if (!forceRGB)
		{	// 8bit gray-scale
			img.resize(size_x,size_y,1,true); //verticalFlip);
			const cellType	*srcPtr = &map[0];
			unsigned char	*destPtr;
			for (unsigned int y=0;y<size_y;y++)
			{
				if (!verticalFlip)
						destPtr = img(0,size_y-1-y);
				else 	destPtr = img(0,y);
				for (unsigned int x=0;x<size_x;x++)
				{
					*destPtr++ = l2p_255(*srcPtr++);
				}
			}
		}
		else
		{	// 24bit RGB:
			img.resize(size_x,size_y,3,true); //verticalFlip);
			const cellType	*srcPtr = &map[0];
			unsigned char	*destPtr;
			for (unsigned int y=0;y<size_y;y++)
			{
				if (!verticalFlip)
						destPtr = img(0,size_y-1-y);
				else 	destPtr = img(0,y);
				for (unsigned int x=0;x<size_x;x++)
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
		{	// 8bit gray-scale
			img.resize(size_x,size_y,1,true); //verticalFlip);
			const cellType	*srcPtr = &map[0];
			unsigned char	*destPtr;
			for (unsigned int y=0;y<size_y;y++)
			{
				if (!verticalFlip)
						destPtr = img(0,size_y-1-y);
				else 	destPtr = img(0,y);
				for (unsigned int x=0;x<size_x;x++)
				{
					uint8_t c = l2p_255(*srcPtr++);
					if (c<120)
						c=0;
					else if (c>136)
						c=255;
					else c = 127;
					*destPtr++ = c;
				}
			}
		}
		else
		{	// 24bit RGB:
			img.resize(size_x,size_y,3,true); //verticalFlip);
			const cellType	*srcPtr = &map[0];
			unsigned char	*destPtr;
			for (unsigned int y=0;y<size_y;y++)
			{
				if (!verticalFlip)
						destPtr = img(0,size_y-1-y);
				else 	destPtr = img(0,y);
				for (unsigned int x=0;x<size_x;x++)
				{
					uint8_t c = l2p_255(*srcPtr++);
					if (c<120)
						c=0;
					else if (c>136)
						c=255;
					else c = 127;

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
void  COccupancyGridMap2D::getAsImageFiltered(
	utils::CImage	&img,
	bool verticalFlip,
	bool forceRGB ) const
{
	getAsImage(img,verticalFlip,forceRGB);

	// Do filtering to improve the noisy peaks in grids:
	// ----------------------------------------------------
#if 0
	CTicTac  t;
#endif
	if (insertionOptions.CFD_features_gaussian_size!=0) 	img.filterGaussianInPlace( round( insertionOptions.CFD_features_gaussian_size ) );
	if (insertionOptions.CFD_features_median_size!=0) 		img.filterMedianInPlace( round( insertionOptions.CFD_features_median_size ) );
#if 0
	cout << "[COccupancyGridMap2D::getAsImageFiltered] Filtered in: " << t.Tac()*1000 << " ms" << endl;
#endif
}



/*---------------------------------------------------------------
				getAs3DObject
  ---------------------------------------------------------------*/
void  COccupancyGridMap2D::getAs3DObject(mrpt::opengl::CSetOfObjectsPtr	&outSetOfObj ) const
{
	if (m_disableSaveAs3DObject)
		return;

	MRPT_START

	opengl::CTexturedPlanePtr	outObj = opengl::CTexturedPlane::Create();

	outObj->setPlaneCorners(x_min,x_max,y_min,y_max);

	outObj->setLocation(0,0, insertionOptions.mapAltitude );

	// Create the color & transparecy (alpha) images:
	CImage			imgColor(size_x,size_y,1);
	CImage			imgTrans(size_x,size_y,1);


	unsigned int x,y;

	const cellType		*srcPtr = &map[0];
	unsigned char		*destPtr_color;
	unsigned char		*destPtr_trans;

	for (y=0;y<size_y;y++)
	{
		destPtr_color = imgColor(0,y);
		destPtr_trans = imgTrans(0,y);
		for (x=0;x<size_x;x++)
		{
			uint8_t  cell255 = l2p_255(*srcPtr++);
			*destPtr_color++ = cell255;

			int8_t   auxC = (int8_t)((signed short)cell255)-128;
			*destPtr_trans++ = auxC>0 ? (auxC << 1) : ((-auxC) << 1);
		}
	}

	outObj->assignImage_fast( imgColor,imgTrans );
	outSetOfObj->insert( outObj );

	MRPT_END
}

