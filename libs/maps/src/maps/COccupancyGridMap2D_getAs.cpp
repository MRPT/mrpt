/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                                 |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
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


	const cellType		*srcPtr = &map[0];
	
	for (unsigned int y=0;y<size_y;y++)
	{
		unsigned char *destPtr_color = imgColor(0,y);
		unsigned char *destPtr_trans = imgTrans(0,y);
		for (unsigned int x=0;x<size_x;x++)
		{
			uint8_t  cell255 = l2p_255(*srcPtr++);
			*destPtr_color++ = cell255;

			int8_t   auxC = (int8_t)((signed short)cell255)-127;
			*destPtr_trans++ = auxC>0 ? (auxC << 1) : ((-auxC) << 1);
		}
	}

	outObj->assignImage_fast( imgColor,imgTrans );
	outSetOfObj->insert( outObj );

	MRPT_END
}

