/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "vision-precomp.h"   // Precompiled headers

#include <mrpt/vision/CFeatureExtraction.h>
#include <mrpt/utils/CTicTac.h>
#include <mrpt/utils/CStream.h>

using namespace mrpt;
using namespace mrpt::vision;
using namespace mrpt::utils;
using namespace mrpt::system;
using namespace std;


/************************************************************************************************
*								Constructor	        									        *
************************************************************************************************/
CFeatureExtraction::CFeatureExtraction()
{
}

/************************************************************************************************
*								Destructor          									        *
************************************************************************************************/
CFeatureExtraction::~CFeatureExtraction()
{
}

struct sort_pred {
	bool operator()(const std::vector<unsigned int> &left, const std::vector<unsigned int> &right) {
        return left[1] < right[1];
    }
};

/************************************************************************************************
*								extractFeatures  									        *
************************************************************************************************/
void  CFeatureExtraction::detectFeatures(
		const CImage			& img,
		CFeatureList			& feats,
		const unsigned int		init_ID,
		const unsigned int		nDesiredFeatures,
		const TImageROI			& ROI) const
{
	switch( options.featsType )
	{
		case featHarris:
			MRPT_TODO("Refactor: check if OpenCV's tile method can be directly called to save space here?")
			if( options.harrisOptions.tile_image )
			{
				mrpt::utils::CTicTac tictac;

				if( !( ROI.xMax == 0 && ROI.xMin == 0 && ROI.yMax == 0 && ROI.yMin == 0 ) )	// ROI must be not active for this option
					std::cout << "Warning: Image ROI is not taken into account, as harrisOptions.tile is set to YES" << std::endl;

				TImageROI newROI;

				unsigned int wd	= img.getWidth();
				unsigned int hg	= img.getHeight();

				unsigned int tt	= 0;											// Total number of features detected in the whole image
				std::vector<std::vector<unsigned int> > tam( 8 );
				std::vector<CFeatureList> aux_feats( 8 );						// 2x4 tiles into the image -> 8 sets of features

				for( unsigned int k = 0; k < 4; k++ )							// Search over the 2x4 tiled image
				{
					// Resize the inner vector
					tam[k].resize(2);
					tam[k+4].resize(2);

					// First row
					newROI.xMin = k*wd/4.f;
					newROI.yMin = 0;
					newROI.xMax = wd/4.f + k*wd/4.f - 1;
					newROI.yMax = hg/2.f - 1;

					tictac.Tic();
					extractFeaturesKLT( img, aux_feats[k], init_ID, nDesiredFeatures, newROI );
					cout << "Tiempo en extraer una tile: " << tictac.Tac()*1000.0f << endl;

					tam[k][0] = k;
					tam[k][1] = aux_feats[k].size();

					// Second row
					newROI.xMin = k*wd/4;
					newROI.yMin = hg/2;
					newROI.xMax = wd/4 + k*wd/4 - 1;
					newROI.yMax = hg-1;

					tictac.Tic();
					extractFeaturesKLT( img, aux_feats[k+4], init_ID, nDesiredFeatures, newROI );
					cout << "Tiempo en extraer una tile: " << tictac.Tac()*1000.0f << endl;

					tam[k+4][0] = k+4;
					tam[k+4][1] = aux_feats[k+4].size();

					tt += aux_feats[k].size() + aux_feats[k+4].size();
				}

				// Merge all the features
				unsigned int new_nDesiredFeatures = nDesiredFeatures <= 0 ? 300 : nDesiredFeatures;
				unsigned int o_n_per_tile = floor( new_nDesiredFeatures/8.0f );
				feats.clear();
				if( tt > new_nDesiredFeatures )						// We have found too many features, we have to select them
				{
					// Order the size vector
					std::sort(tam.begin(), tam.end(), sort_pred());

					if( tam[0][1] > o_n_per_tile ) // The smallest subset
					{
						// Everything goes right -> Get o_n_per_tile features from each tile.
						for( unsigned int m = 0; m < 8; m++ )
							for( unsigned int k = 0; k < o_n_per_tile; k++ )
								feats.push_back( aux_feats[m][k] );
					}
					else
					{
						std::vector<std::vector<unsigned int> >::iterator itVector;
						unsigned int n_per_tile	= o_n_per_tile;

						for( itVector = tam.begin(); itVector != tam.end(); itVector++ )
						{
							if( (*itVector)[1] < n_per_tile )	// Size of the subset
							{
								// We have to distribute the features among the tiles
								for(unsigned int k = 0; k < (*itVector)[1]; k++)
								{
									feats.push_back( aux_feats[(*itVector)[0]][k] );
									n_per_tile += (n_per_tile - (*itVector)[1]);
								} // end for
							} // end if
							else
							{
								for(unsigned int k = 0; k < n_per_tile; k++)
								{
									feats.push_back( aux_feats[(*itVector)[0]][k] );
									n_per_tile = o_n_per_tile;
								} // end for
							} // end else
						} // end for 'itVector'
					} // end else
				} // end if tt > nDesiredFeatures
				else	// We have found less features than the desired
				{
					CFeatureList::iterator itList;
					for( unsigned int m = 0; m < 8; m++ )
						for( itList = aux_feats[m].begin(); itList != aux_feats[m].end(); itList++ )
							feats.push_back( *itList );;
				}

			} // end if

			else
				extractFeaturesKLT(img, feats, init_ID, nDesiredFeatures, ROI);
			break;

		case featKLT:
			extractFeaturesKLT(img, feats, init_ID, nDesiredFeatures, ROI);
			break;

		case featSIFT:
			extractFeaturesSIFT(img, feats, init_ID, nDesiredFeatures, ROI);
			break;

		case featBCD:
			extractFeaturesBCD(img, feats, init_ID, nDesiredFeatures, ROI);
			break;

		case featSURF:
			extractFeaturesSURF(img, feats, init_ID, nDesiredFeatures, ROI);
			break;

		case featFAST:
			extractFeaturesFAST(img, feats, init_ID, nDesiredFeatures, ROI);
			break;

		case featFASTER9:
			extractFeaturesFASTER_N(9,img, feats, init_ID, nDesiredFeatures, ROI);
			break;
		case featFASTER10:
			extractFeaturesFASTER_N(10,img, feats, init_ID, nDesiredFeatures, ROI);
			break;
		case featFASTER12:
			extractFeaturesFASTER_N(12,img, feats, init_ID, nDesiredFeatures, ROI);
			break;

		case featORB:
			extractFeaturesORB( img, feats, init_ID, nDesiredFeatures, ROI );
			break;

		default:
			THROW_EXCEPTION("options.method has an invalid value!");
			break;
	}
}

/************************************************************************************************
 								computeDescriptors
************************************************************************************************/
void  CFeatureExtraction::computeDescriptors(
	const CImage	&in_img,
	CFeatureList		&inout_features,
	TDescriptorType		in_descriptor_list) const
{
	MRPT_START

	int nDescComputed = 0;

	if ((in_descriptor_list & descSIFT) != 0)
	{
		this->internal_computeSiftDescriptors(in_img,inout_features);
		++nDescComputed;
	}
	if ((in_descriptor_list & descSURF) != 0)
	{
		this->internal_computeSurfDescriptors(in_img,inout_features);
		++nDescComputed;
	}
	if ((in_descriptor_list & descSpinImages) != 0)
	{
		this->internal_computeSpinImageDescriptors(in_img,inout_features);
		++nDescComputed;
	}
	if ((in_descriptor_list & descPolarImages) != 0)
	{
		this->internal_computePolarImageDescriptors(in_img,inout_features);
		++nDescComputed;
	}
	if ((in_descriptor_list & descLogPolarImages) != 0)
	{
		this->internal_computeLogPolarImageDescriptors(in_img,inout_features);
		++nDescComputed;
	}
	if ((in_descriptor_list & descORB) != 0)
	{
		this->internal_computeORBDescriptors(in_img,inout_features);
		++nDescComputed;
	}

	if (!nDescComputed)
		THROW_EXCEPTION_CUSTOM_MSG1("No known descriptor value found in in_descriptor_list=%u",(unsigned)in_descriptor_list)

	MRPT_END
}

/************************************************************************************************
*								extractFeaturesBCD  									        *
************************************************************************************************/
void  CFeatureExtraction::extractFeaturesBCD(
		const CImage		&img,
		CFeatureList			&feats,
		unsigned int			init_ID,
		unsigned int			nDesiredFeatures,
		const TImageROI			&ROI) const
{
	MRPT_UNUSED_PARAM(img);
	MRPT_UNUSED_PARAM(feats);
	MRPT_UNUSED_PARAM(init_ID);
	MRPT_UNUSED_PARAM(nDesiredFeatures);
	MRPT_UNUSED_PARAM(ROI);

	THROW_EXCEPTION("Not implemented yet!");
} // end extractFeaturesBCD



/*------------------------------------------------------------
					TOptions()
-------------------------------------------------------------*/
CFeatureExtraction::TOptions::TOptions(const TFeatureType _featsType) :
	featsType	( _featsType)	// Default Method: Kanade-Lucas-Tomasi
{
	// General options
	patchSize		= 21;					// Patch size
	FIND_SUBPIXEL	= true;					// Find subpixel
	useMask         = false;                // Use mask for finding features
	addNewFeatures  = false;                // Add to existing feature list

	// Harris Options
	harrisOptions.k				= 0.04f;
	harrisOptions.radius		= 3;		//15;
	harrisOptions.threshold		= 0.005f;	//0.01f; The lower this is, more features will be found
	harrisOptions.sigma			= 3.0f;
	harrisOptions.min_distance	= 5;		//10;
	harrisOptions.tile_image	= false;

	// KLT Options
	KLTOptions.min_distance		= 5;		//10;
	KLTOptions.threshold		= 0.1f;		//0.005 ; 0.01f;
	KLTOptions.radius			= 15;		//3;
	KLTOptions.tile_image		= false;

	// SIFT Options
	SIFTOptions.implementation	= Hess;		// Default implementation: Hess

	// SURF Options

	// BCD Options

	// FAST:
	FASTOptions.threshold				= 20;
	FASTOptions.nonmax_suppression 		= true;
	FASTOptions.use_KLT_response		= false;
	FASTOptions.min_distance 			= 5;

	// ORB:
	ORBOptions.extract_patch			= false;
	ORBOptions.min_distance				= 0;
	ORBOptions.n_levels					= 8;
	ORBOptions.scale_factor				= 1.2f;

	// SpinImages Options:
	SpinImagesOptions.hist_size_distance  = 10;
	SpinImagesOptions.hist_size_intensity = 10;
	SpinImagesOptions.radius              = 20;
	SpinImagesOptions.std_dist            = 0.4f;
	SpinImagesOptions.std_intensity       = 10;

	// TPolarImagesOptions
	PolarImagesOptions.bins_angle		= 8;
	PolarImagesOptions.bins_distance	= 6;
	PolarImagesOptions.radius			= 20;

	// LogPolarImagesOptions
	LogPolarImagesOptions.radius		= 30;
	LogPolarImagesOptions.num_angles	= 16; // Log-Polar image patch will have dimensions WxH, with:  W=num_angles,  H= rho_scale * log(radius)
	LogPolarImagesOptions.rho_scale		= 5;

}

/*---------------------------------------------------------------
					dumpToTextStream
  ---------------------------------------------------------------*/
void CFeatureExtraction::TOptions::dumpToTextStream(mrpt::utils::CStream	&out) const
{
	out.printf("\n----------- [CFeatureExtraction::TOptions] ------------ \n\n");

	LOADABLEOPTS_DUMP_VAR(featsType,int)
	LOADABLEOPTS_DUMP_VAR(patchSize, int)
	LOADABLEOPTS_DUMP_VAR(FIND_SUBPIXEL, bool)
	LOADABLEOPTS_DUMP_VAR(useMask, bool)
	LOADABLEOPTS_DUMP_VAR(addNewFeatures, bool)

	LOADABLEOPTS_DUMP_VAR(harrisOptions.k,double)
	LOADABLEOPTS_DUMP_VAR(harrisOptions.radius,int)
	LOADABLEOPTS_DUMP_VAR(harrisOptions.threshold,float)
	LOADABLEOPTS_DUMP_VAR(harrisOptions.sigma,float)
	LOADABLEOPTS_DUMP_VAR(harrisOptions.min_distance,float)

	LOADABLEOPTS_DUMP_VAR(KLTOptions.min_distance,float)
	LOADABLEOPTS_DUMP_VAR(KLTOptions.threshold,float)
	LOADABLEOPTS_DUMP_VAR(KLTOptions.radius,int)

	LOADABLEOPTS_DUMP_VAR(SIFTOptions.implementation,int)

	LOADABLEOPTS_DUMP_VAR(SURFOptions.rotation_invariant,bool)
	LOADABLEOPTS_DUMP_VAR(SURFOptions.hessianThreshold,int)
	LOADABLEOPTS_DUMP_VAR(SURFOptions.nOctaves,int)
	LOADABLEOPTS_DUMP_VAR(SURFOptions.nLayersPerOctave,int)

	LOADABLEOPTS_DUMP_VAR(FASTOptions.threshold,int)
	LOADABLEOPTS_DUMP_VAR(FASTOptions.nonmax_suppression,bool)
	LOADABLEOPTS_DUMP_VAR(FASTOptions.min_distance,float)
	LOADABLEOPTS_DUMP_VAR(FASTOptions.use_KLT_response,bool)

	LOADABLEOPTS_DUMP_VAR(ORBOptions.scale_factor,float)
	LOADABLEOPTS_DUMP_VAR(ORBOptions.min_distance,int)
	LOADABLEOPTS_DUMP_VAR(ORBOptions.n_levels,int)
	LOADABLEOPTS_DUMP_VAR(ORBOptions.extract_patch,bool)

	LOADABLEOPTS_DUMP_VAR(SpinImagesOptions.hist_size_distance,int)
	LOADABLEOPTS_DUMP_VAR(SpinImagesOptions.hist_size_intensity,int)
	LOADABLEOPTS_DUMP_VAR(SpinImagesOptions.radius,int)
	LOADABLEOPTS_DUMP_VAR(SpinImagesOptions.std_dist,float)
	LOADABLEOPTS_DUMP_VAR(SpinImagesOptions.std_intensity,float)

	LOADABLEOPTS_DUMP_VAR(PolarImagesOptions.bins_angle,int)
	LOADABLEOPTS_DUMP_VAR(PolarImagesOptions.bins_distance,int)
	LOADABLEOPTS_DUMP_VAR(PolarImagesOptions.radius,int)

	LOADABLEOPTS_DUMP_VAR(LogPolarImagesOptions.radius,int)
	LOADABLEOPTS_DUMP_VAR(LogPolarImagesOptions.num_angles,int)
	LOADABLEOPTS_DUMP_VAR(LogPolarImagesOptions.rho_scale,double)

	out.printf("\n");
}

/*---------------------------------------------------------------
					loadFromConfigFile
  ---------------------------------------------------------------*/
void CFeatureExtraction::TOptions::loadFromConfigFile(
	const mrpt::utils::CConfigFileBase  &iniFile,
	const std::string &section)
{
	MRPT_LOAD_CONFIG_VAR_CAST(featsType, int, TFeatureType, iniFile,section)
	MRPT_LOAD_CONFIG_VAR(patchSize, int,  iniFile, section)
	MRPT_LOAD_CONFIG_VAR(FIND_SUBPIXEL, bool,  iniFile, section)
	MRPT_LOAD_CONFIG_VAR(useMask, bool,  iniFile, section)
	MRPT_LOAD_CONFIG_VAR(addNewFeatures, bool,  iniFile, section)

	//string sect = section;
	MRPT_LOAD_CONFIG_VAR(harrisOptions.k,double,  iniFile,section)
	MRPT_LOAD_CONFIG_VAR(harrisOptions.radius,int,  iniFile,section)
	MRPT_LOAD_CONFIG_VAR(harrisOptions.threshold,float,  iniFile,section)
	MRPT_LOAD_CONFIG_VAR(harrisOptions.sigma,float,  iniFile,section)
	MRPT_LOAD_CONFIG_VAR(harrisOptions.min_distance,float,  iniFile,section)

	MRPT_LOAD_CONFIG_VAR(KLTOptions.min_distance,float,  iniFile,section)
	MRPT_LOAD_CONFIG_VAR(KLTOptions.threshold,float,  iniFile,section)
	MRPT_LOAD_CONFIG_VAR(KLTOptions.radius,int,  iniFile,section)

	MRPT_LOAD_CONFIG_VAR_CAST(SIFTOptions.implementation,int,TSIFTImplementation, iniFile,section)
	MRPT_LOAD_CONFIG_VAR(SIFTOptions.threshold,double,  iniFile,section)
	MRPT_LOAD_CONFIG_VAR(SIFTOptions.edgeThreshold,double,  iniFile,section)

	MRPT_LOAD_CONFIG_VAR(SURFOptions.rotation_invariant,bool,  iniFile,section)
	MRPT_LOAD_CONFIG_VAR(SURFOptions.hessianThreshold,int,  iniFile,section)
	MRPT_LOAD_CONFIG_VAR(SURFOptions.nOctaves,int,  iniFile,section)
	MRPT_LOAD_CONFIG_VAR(SURFOptions.nLayersPerOctave,int,  iniFile,section)

	MRPT_LOAD_CONFIG_VAR(FASTOptions.threshold,int,  iniFile,section)
	MRPT_LOAD_CONFIG_VAR(FASTOptions.nonmax_suppression,bool,  iniFile,section)
	MRPT_LOAD_CONFIG_VAR(FASTOptions.min_distance,float,  iniFile,section)
	MRPT_LOAD_CONFIG_VAR(FASTOptions.use_KLT_response,bool,  iniFile,section)

	MRPT_LOAD_CONFIG_VAR(ORBOptions.extract_patch,bool,  iniFile,section)
	MRPT_LOAD_CONFIG_VAR(ORBOptions.min_distance,int,  iniFile,section)
	MRPT_LOAD_CONFIG_VAR(ORBOptions.n_levels,int,  iniFile,section)
	MRPT_LOAD_CONFIG_VAR(ORBOptions.scale_factor,float,  iniFile,section)

	MRPT_LOAD_CONFIG_VAR(SpinImagesOptions.hist_size_distance,int,  iniFile,section)
	MRPT_LOAD_CONFIG_VAR(SpinImagesOptions.hist_size_intensity,int,  iniFile,section)
	MRPT_LOAD_CONFIG_VAR(SpinImagesOptions.radius,int,  iniFile,section)
	MRPT_LOAD_CONFIG_VAR(SpinImagesOptions.std_dist,float,  iniFile,section)
	MRPT_LOAD_CONFIG_VAR(SpinImagesOptions.std_intensity,float,  iniFile,section)

	MRPT_LOAD_CONFIG_VAR(PolarImagesOptions.bins_angle,int,  iniFile,section)
	MRPT_LOAD_CONFIG_VAR(PolarImagesOptions.bins_distance,int,  iniFile,section)
	MRPT_LOAD_CONFIG_VAR(PolarImagesOptions.radius,int,  iniFile,section)

	MRPT_LOAD_CONFIG_VAR(LogPolarImagesOptions.radius,int,  iniFile,section)
	MRPT_LOAD_CONFIG_VAR(LogPolarImagesOptions.num_angles,int,  iniFile,section)
	MRPT_LOAD_CONFIG_VAR(LogPolarImagesOptions.rho_scale,double,  iniFile,section)

}

