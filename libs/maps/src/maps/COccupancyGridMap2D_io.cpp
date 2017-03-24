/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "maps-precomp.h" // Precomp header

#include <mrpt/system/os.h>
#include <mrpt/math/CMatrix.h>
#include <mrpt/utils/CStream.h>
#include <mrpt/utils/CEnhancedMetaFile.h>
#include <mrpt/utils/round.h> // round()
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/random.h>

using namespace mrpt;
using namespace mrpt::maps;
using namespace mrpt::math;
using namespace mrpt::obs;
using namespace mrpt::random;
using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace mrpt::system;
using namespace std;


/*---------------------------------------------------------------
					saveAsBitmapFile
  ---------------------------------------------------------------*/
bool  COccupancyGridMap2D::saveAsBitmapFile(const std::string &file) const
{
	MRPT_START

	CImage			img;
	getAsImage(img);
	return img.saveToFile(file);

	MRPT_END
}




/*---------------------------------------------------------------
					writeToStream
	Implements the writing to a CStream capability of
	  CSerializable objects
  ---------------------------------------------------------------*/
void  COccupancyGridMap2D::writeToStream(mrpt::utils::CStream &out, int *version) const
{
	if (version)
		*version = 6;
	else
	{
		// Version 3: Change to log-odds. The only change is in the loader, when translating
		//   from older versions.

		// Version 2: Save OCCUPANCY_GRIDMAP_CELL_SIZE_8BITS/16BITS
#ifdef OCCUPANCY_GRIDMAP_CELL_SIZE_8BITS
		out << uint8_t(8);
#else
		out << uint8_t(16);
#endif

		out << size_x << size_y << x_min << x_max << y_min << y_max << resolution;
		ASSERT_(size_x*size_y==map.size());

#ifdef OCCUPANCY_GRIDMAP_CELL_SIZE_8BITS
		out.WriteBuffer(&map[0], sizeof(map[0])*size_x*size_y);
#else
		out.WriteBufferFixEndianness(&map[0], size_x*size_y);
#endif

		// insertionOptions:
		out <<	insertionOptions.mapAltitude
			<<	insertionOptions.useMapAltitude
			<<	insertionOptions.maxDistanceInsertion
			<<	insertionOptions.maxOccupancyUpdateCertainty
			<<	insertionOptions.considerInvalidRangesAsFreeSpace
			<<	insertionOptions.decimation
			<<	insertionOptions.horizontalTolerance;

		// Likelihood:
		out	<<	(int32_t)likelihoodOptions.likelihoodMethod
			<<	likelihoodOptions.LF_stdHit
			<<	likelihoodOptions.LF_zHit
			<<	likelihoodOptions.LF_zRandom
			<<	likelihoodOptions.LF_maxRange
			<<	likelihoodOptions.LF_decimation
			<<	likelihoodOptions.LF_maxCorrsDistance
			<<	likelihoodOptions.LF_alternateAverageMethod
			<<	likelihoodOptions.MI_exponent
			<<	likelihoodOptions.MI_skip_rays
			<<	likelihoodOptions.MI_ratio_max_distance
			<<	likelihoodOptions.rayTracing_useDistanceFilter
			<<	likelihoodOptions.rayTracing_decimation
			<<	likelihoodOptions.rayTracing_stdHit
			<<	likelihoodOptions.consensus_takeEachRange
			<<	likelihoodOptions.consensus_pow
			<<	likelihoodOptions.OWA_weights
			<<	likelihoodOptions.enableLikelihoodCache;

		// Insertion as 3D:
		out << genericMapParams; // v6

		// Version 4:
		out << insertionOptions.CFD_features_gaussian_size
		    << insertionOptions.CFD_features_median_size;

		// Version: 5;
		out << insertionOptions.wideningBeamsWithDistance;

	}
}

/*---------------------------------------------------------------
					readFromStream
  ---------------------------------------------------------------*/
void  COccupancyGridMap2D::readFromStream(mrpt::utils::CStream &in, int version)
{
	m_is_empty = false;

	switch(version)
	{
	case 0:
	case 1:
	case 2:
	case 3:
	case 4:
	case 5:
	case 6:
		{
#			ifdef OCCUPANCY_GRIDMAP_CELL_SIZE_8BITS
				const uint8_t	MyBitsPerCell = 8;
#			else
				const uint8_t	MyBitsPerCell = 16;
#			endif

			uint8_t		bitsPerCellStream;

			// Version 2: OCCUPANCY_GRIDMAP_CELL_SIZE_8BITS/16BITS
			if (version>=2)
					in >> bitsPerCellStream;
			else	bitsPerCellStream = MyBitsPerCell;  // Old versinons: hope it's the same...

			uint32_t        new_size_x,new_size_y;
			float           new_x_min,new_x_max,new_y_min,new_y_max;
			float			new_resolution;

			//resetFeaturesCache();

			in >> new_size_x >> new_size_y >> new_x_min >> new_x_max >> new_y_min >> new_y_max >> new_resolution;

			setSize(new_x_min,new_x_max,new_y_min,new_y_max,new_resolution,0.5);

			ASSERT_(size_x*size_y==map.size());

			if (bitsPerCellStream==MyBitsPerCell)
			{
				// Perfect:
			#ifdef OCCUPANCY_GRIDMAP_CELL_SIZE_8BITS
				in.ReadBuffer(&map[0], sizeof(map[0])*map.size());
			#else
				in.ReadBufferFixEndianness(&map[0], map.size());
			#endif
			}
			else
			{
				// We must do a conversion...
#			ifdef OCCUPANCY_GRIDMAP_CELL_SIZE_8BITS
				// We are 8-bit, stream is 16-bit
				ASSERT_(bitsPerCellStream==16);
				std::vector<uint16_t>    auxMap( map.size() );
				in.ReadBuffer(&auxMap[0], sizeof(auxMap[0])*auxMap.size());

				size_t  i, N = map.size();
				uint8_t         *ptrTrg = (uint8_t*)&map[0];
				const uint16_t  *ptrSrc = (const uint16_t*)&auxMap[0];
				for (i=0;i<N;i++)
					*ptrTrg++ = (*ptrSrc++) >> 8;
#			else
				// We are 16-bit, stream is 8-bit
				ASSERT_(bitsPerCellStream==8);
				std::vector<uint8_t>    auxMap( map.size() );
				in.ReadBuffer(&auxMap[0], sizeof(auxMap[0])*auxMap.size());

				size_t  i, N = map.size();
				uint16_t       *ptrTrg = (uint16_t*)&map[0];
				const uint8_t  *ptrSrc = (const uint8_t*)&auxMap[0];
				for (i=0;i<N;i++)
					*ptrTrg++ = (*ptrSrc++) << 8;
#			endif
			}

			// If we are converting an old dump, convert from probabilities to log-odds:
			if (version<3)
			{
				size_t  i, N = map.size();
				cellType  *ptr = &map[0];
				for (i=0;i<N;i++)
				{
					double p = cellTypeUnsigned(*ptr) * (1.0f/0xFF);
					if (p<0)
						p=0;
					if (p>1)
						p=1;
					*ptr++ = p2l( p );
				}
			}

			// For the precomputed likelihood trick:
			precomputedLikelihoodToBeRecomputed = true;

			if (version>=1)
			{
				// insertionOptions:
				in  >>	insertionOptions.mapAltitude
					>>	insertionOptions.useMapAltitude
					>>	insertionOptions.maxDistanceInsertion
					>>	insertionOptions.maxOccupancyUpdateCertainty
					>>	insertionOptions.considerInvalidRangesAsFreeSpace
					>>	insertionOptions.decimation
					>>	insertionOptions.horizontalTolerance;

				// Likelihood:
				int32_t		i;
				in 	>>	i; likelihoodOptions.likelihoodMethod = static_cast<TLikelihoodMethod>(i);
				in	>>	likelihoodOptions.LF_stdHit
					>>	likelihoodOptions.LF_zHit
					>>	likelihoodOptions.LF_zRandom
					>>	likelihoodOptions.LF_maxRange
					>>	likelihoodOptions.LF_decimation
					>>	likelihoodOptions.LF_maxCorrsDistance
					>>	likelihoodOptions.LF_alternateAverageMethod
					>>	likelihoodOptions.MI_exponent
					>>	likelihoodOptions.MI_skip_rays
					>>	likelihoodOptions.MI_ratio_max_distance
					>>	likelihoodOptions.rayTracing_useDistanceFilter
					>>	likelihoodOptions.rayTracing_decimation
					>>	likelihoodOptions.rayTracing_stdHit
					>>	likelihoodOptions.consensus_takeEachRange
					>>	likelihoodOptions.consensus_pow
					>>	likelihoodOptions.OWA_weights
					>>	likelihoodOptions.enableLikelihoodCache;

				// Insertion as 3D:
				if (version>=6)
					in >> genericMapParams;
				else 
				{
					bool disableSaveAs3DObject;
					in >> disableSaveAs3DObject;
					genericMapParams.enableSaveAs3DObject = !disableSaveAs3DObject;
				}
			}

			if (version>=4)
			{
				in >> insertionOptions.CFD_features_gaussian_size
				   >> insertionOptions.CFD_features_median_size;
			}

			if (version>=5)
			{
				in >> insertionOptions.wideningBeamsWithDistance;
			}

		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};
}

/*---------------------------------------------------------------
					loadFromBitmapFile
 Load a 8-bits, black & white bitmap file as a grid map. It will be loaded such as coordinates (0,0) falls just in the middle of map.
\param file The file to be loaded.
\param resolution The size of a pixel (cell), in meters. Recall cells are always squared, so just a dimension is needed.
\return False on any error.
 ---------------------------------------------------------------*/
bool  COccupancyGridMap2D::loadFromBitmapFile(
	const std::string	&file,
	float			resolution,
	float			xCentralPixel,
	float			yCentralPixel)
{
	MRPT_START

	CImage		imgFl;
	if (!imgFl.loadFromFile(file,0))
		return false;

	m_is_empty = false;
	return loadFromBitmap(imgFl,resolution, xCentralPixel, yCentralPixel);

	MRPT_END
}

/*---------------------------------------------------------------
					loadFromBitmap
 ---------------------------------------------------------------*/
bool  COccupancyGridMap2D::loadFromBitmap(const mrpt::utils::CImage &imgFl, float resolution, float xCentralPixel, float yCentralPixel)
{
	MRPT_START

	// For the precomputed likelihood trick:
	precomputedLikelihoodToBeRecomputed = true;

	size_t bmpWidth = imgFl.getWidth();
	size_t bmpHeight = imgFl.getHeight();

	if (size_x!=bmpWidth || size_y!=bmpHeight)
	{
		// Middle of bitmap?
		if (xCentralPixel<-1 || yCentralPixel<=-1)
		{
			xCentralPixel = imgFl.getWidth() / 2.0f;
			yCentralPixel = imgFl.getHeight() / 2.0f;
		}

		// Resize grid:
		float new_x_max = (imgFl.getWidth() - xCentralPixel) * resolution;
		float new_x_min = - xCentralPixel * resolution;
		float new_y_max = (imgFl.getHeight() - yCentralPixel) * resolution;
		float new_y_min = - yCentralPixel * resolution;

		setSize(new_x_min,new_x_max,new_y_min,new_y_max,resolution);
	}

	// And load cells content:
	for (size_t x=0;x<bmpWidth;x++)
		for (size_t y=0;y<bmpHeight;y++)
		{
			float f = imgFl.getAsFloat(x,bmpHeight-1-y);
			f = std::max(0.01f,f);
			f = std::min(0.99f,f);
			setCell(x,y,f);
		}

	m_is_empty = false;
	return true;

	MRPT_END
}



/*---------------------------------------------------------------
				saveAsBitmapTwoMapsWithCorrespondences
  ---------------------------------------------------------------*/
bool  COccupancyGridMap2D::saveAsBitmapTwoMapsWithCorrespondences(
	const std::string						&fileName,
	const COccupancyGridMap2D				*m1,
	const COccupancyGridMap2D				*m2,
	const TMatchingPairList		&corrs)
{
	MRPT_START

	CImage			img1,img2;
	CImage			img(10,10,3,true);
	unsigned int	i,n , Ay1, Ay2;
	unsigned int	px, py;

	// The individual maps:
	// ---------------------------------------------
	m1->getAsImage( img1, false );
	m2->getAsImage( img2, false );
	unsigned int lx1 = img1.getWidth();
	unsigned int ly1 = img1.getHeight();

	unsigned int lx2 = img2.getWidth();
	unsigned int ly2 = img2.getHeight();

	// The map with the lowest height has to be vertically aligned:
	if (ly1>ly2)
	{
		Ay1 = 0;
		Ay2 = (ly1-ly2)/2;
	}
	else
	{
		Ay2 = 0;
		Ay1 = (ly2-ly1)/2;
	}


	// Compute the size of the composite image:
	// ---------------------------------------------
	img.resize(lx1 + lx2 + 1, max(ly1,ly2), 3, true );
	img.filledRectangle(0,0,img.getWidth()-1,img.getHeight()-1, TColor::black );	// background: black
	img.drawImage(0,Ay1,img1);
	img.drawImage(lx1+1,Ay2,img2);

	// Draw the features:
	// ---------------------------------------------
	n = corrs.size();
	TColor  lineColor = TColor::black;
	for (i=0;i<n;i++)
	{
		// In M1:
		px = m1->x2idx( corrs[i].this_x );
		py = Ay1+ly1-1- m1->y2idx( corrs[i].this_y );
		img.rectangle(px-10,py-10,px+10,py+10,lineColor);
		img.rectangle(px-11,py-11,px+11,py+11,lineColor);

		// In M2:
		px = lx1+1 + m2->x2idx( corrs[i].other_x );
		py = Ay2+ly2-1- m2->y2idx( corrs[i].other_y );
		img.rectangle(px-10,py-10,px+10,py+10,lineColor);
		img.rectangle(px-11,py-11,px+11,py+11,lineColor);
	}

	// Draw the correspondences as lines:
	// ---------------------------------------------
	for (i=0;i<n;i++)
	{
		lineColor = TColor(
			static_cast<long>(randomGenerator.drawUniform(0,255.0f)),
			static_cast<long>(randomGenerator.drawUniform(0,255.0f)),
			static_cast<long>(randomGenerator.drawUniform(0,255.0f)) );

		img.line(
			m1->x2idx( corrs[i].this_x ),
//				lx1+1+ m1->x2idx( corrs[i].this_x ),
			Ay1+ly1-1- m1->y2idx( corrs[i].this_y ),
			lx1+1+ m2->x2idx( corrs[i].other_x ),
			Ay2+ly2-1-m2->y2idx( corrs[i].other_y ),
            lineColor);
	} // i

	return img.saveToFile(fileName.c_str() );

	MRPT_END
}

/*---------------------------------------------------------------
				saveAsEMFTwoMapsWithCorrespondences
  ---------------------------------------------------------------*/
bool  COccupancyGridMap2D::saveAsEMFTwoMapsWithCorrespondences(
	const std::string						&fileName,
	const COccupancyGridMap2D				*m1,
	const COccupancyGridMap2D				*m2,
	const TMatchingPairList		&corrs)
{
	MRPT_START

	CEnhancedMetaFile				emf(fileName,1);
	CImage			img1,img2;
	TColor			lineColor;
	unsigned int	i, Ay1, Ay2;
	unsigned int	px, py;


	lineColor = TColor::red;

	// The individual maps:
	// ---------------------------------------------
#ifdef MRPT_OS_WINDOWS
	m1->getAsImage( img1, true );
	m2->getAsImage( img2, true );
#else
	m1->getAsImage( img1, false );		// Linux: emulated EMF is different.
	m2->getAsImage( img2, false );
#endif
	unsigned int lx1 = img1.getWidth();
	unsigned int ly1 = img1.getHeight();
	//unsigned int lx2 = img2.getWidth();
	unsigned int ly2 = img2.getHeight();

	// The map with the lowest height has to be vertically aligned:
	if (ly1>ly2)
	{
		Ay1 = 0;
		Ay2 = (ly1-ly2)/2;
	}
	else
	{
		Ay2 = 0;
		Ay1 = (ly2-ly1)/2;
	}


	// Draw the pair of maps:
	// ---------------------------------------------
	emf.drawImage(0,Ay1,img1);
	emf.drawImage(lx1+1,Ay2,img2);

	// Draw the features:
	// ---------------------------------------------
	const unsigned int n = corrs.size();
	lineColor = TColor::black;
	for (i=0;i<n;i++)
	{
		// In M1:
		px = m1->x2idx( corrs[i].this_x );
		py = Ay1+ly1-1- m1->y2idx( corrs[i].this_y );
		emf.rectangle(px-10,py-10,px+10,py+10,lineColor);
		emf.rectangle(px-11,py-11,px+11,py+11,lineColor);

		// In M2:
		px = lx1+1 + m2->x2idx( corrs[i].other_x );
		py = Ay2+ly2-1- m2->y2idx( corrs[i].other_y );
		emf.rectangle(px-10,py-10,px+10,py+10,lineColor);
		emf.rectangle(px-11,py-11,px+11,py+11,lineColor);
	}

/** /
	// Draw the correspondences as lines:
	// ---------------------------------------------
	for (i=0;i<n;i++)
	{
		lineColor =
			((unsigned long)RandomUni(0,255.0f)) +
			(((unsigned long)RandomUni(0,255.0f)) << 8 ) +
			(((unsigned long)RandomUni(0,255.0f)) << 16 );

		emf.line(
			m1->x2idx( corrs[i].this_x ),
			Ay1+ly1-1- m1->y2idx( corrs[i].this_y ),
			lx1+1+ m2->x2idx( corrs[i].other_x ),
			Ay2+ly2-1-m2->y2idx( corrs[i].other_y ),
            lineColor);
	} // i
/ **/

	// Draw the correspondences as text labels:
	// ---------------------------------------------
	char	str[100];
	for (i=0;i<n;i++)
	{
		os::sprintf(str,100,"%i",i);

		emf.textOut(
			m1->x2idx( corrs[i].this_x ) - 10 ,
			Ay1+ly1-1- m1->y2idx( corrs[i].this_y ) - 25,
			str, TColor::black );

		emf.textOut(
			lx1+1+ m2->x2idx( corrs[i].other_x ) - 10,
			Ay2+ly2-1-m2->y2idx( corrs[i].other_y ) - 25,
			str,TColor::black );
	} // i

	return true;

	MRPT_END
}


/*---------------------------------------------------------------
					auxParticleFilterCleanUp
 ---------------------------------------------------------------*/
void  COccupancyGridMap2D::saveMetricMapRepresentationToFile(
	const std::string	&filNamePrefix
	) const
{
	std::string		fil( filNamePrefix + std::string(".png") );
	saveAsBitmapFile( fil  );

	fil = filNamePrefix + std::string("_limits.txt");
	CMatrix LIMITS(1,4);
	LIMITS(0,0) = x_min;
	LIMITS(0,1) = x_max;
	LIMITS(0,2) = y_min;
	LIMITS(0,3) = y_max;
	LIMITS.saveToTextFile( fil, MATRIX_FORMAT_FIXED, false /* add mrpt header */, "% Grid limits: [x_min x_max y_min y_max]\n" );
}


