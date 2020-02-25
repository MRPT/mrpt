/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          https://www.mrpt.org/                            |
   |                                                                           |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file        |
   | See: https://www.mrpt.org/Authors - All rights reserved.                  |
   | Released under BSD License. See details in https://www.mrpt.org/License   |
   +---------------------------------------------------------------------------+
   */

#include "vision-precomp.h"  // Precompiled headers

#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/CTicTac.h>
#include <mrpt/vision/CFeatureExtraction.h>

using namespace mrpt;
using namespace mrpt::img;
using namespace mrpt::vision;
using namespace mrpt::img;
using namespace mrpt::system;
using namespace std;

struct sort_pred
{
	bool operator()(
		const std::vector<unsigned int>& left,
		const std::vector<unsigned int>& right)
	{
		return left[1] < right[1];
	}
};

/************************************************************************************************
 *								extractFeatures *
 ************************************************************************************************/
void CFeatureExtraction::detectFeatures(
	const CImage& img, CFeatureList& feats, const unsigned int init_ID,
	const unsigned int nDesiredFeatures, const TImageROI& ROI)
{
	CTimeLoggerEntry tle(profiler, "detectFeatures");

	switch (options.featsType)
	{
		case featHarris:
			extractFeaturesKLT(img, feats, init_ID, nDesiredFeatures, ROI);
			break;

		case featKLT:
			extractFeaturesKLT(img, feats, init_ID, nDesiredFeatures, ROI);
			break;

		case featSIFT:
			extractFeaturesSIFT(img, feats, init_ID, nDesiredFeatures, ROI);
			break;

		case featSURF:
			extractFeaturesSURF(img, feats, init_ID, nDesiredFeatures, ROI);
			break;

		case featFAST:
			extractFeaturesFAST(img, feats, init_ID, nDesiredFeatures);
			break;

		case featFASTER9:
			extractFeaturesFASTER_N(
				9, img, feats, init_ID, nDesiredFeatures, ROI);
			break;
		case featFASTER10:
			extractFeaturesFASTER_N(
				10, img, feats, init_ID, nDesiredFeatures, ROI);
			break;
		case featFASTER12:
			extractFeaturesFASTER_N(
				12, img, feats, init_ID, nDesiredFeatures, ROI);
			break;

		case featORB:
			extractFeaturesORB(img, feats, init_ID, nDesiredFeatures, ROI);
			break;

		// # added by Raghavender Sahdev
		case featAKAZE:
			extractFeaturesAKAZE(img, feats, init_ID, nDesiredFeatures, ROI);
			break;
		case featLSD:
			extractFeaturesLSD(img, feats, init_ID, nDesiredFeatures, ROI);
			break;

		default:
			THROW_EXCEPTION("options.method has an invalid value!");
			break;
	}
}

/************************************************************************************************
								computeDescriptors
************************************************************************************************/
void CFeatureExtraction::computeDescriptors(
	const CImage& in_img, CFeatureList& inout_features,
	TDescriptorType in_descriptor_list)
{
	MRPT_START
	CTimeLoggerEntry tle(profiler, "computeDescriptors");

	int nDescComputed = 0;

	if ((in_descriptor_list & descSIFT) != 0)
	{
		this->internal_computeSiftDescriptors(in_img, inout_features);
		++nDescComputed;
	}
	if ((in_descriptor_list & descSURF) != 0)
	{
		this->internal_computeSurfDescriptors(in_img, inout_features);
		++nDescComputed;
	}
	if ((in_descriptor_list & descSpinImages) != 0)
	{
		this->internal_computeSpinImageDescriptors(in_img, inout_features);
		++nDescComputed;
	}
	if ((in_descriptor_list & descPolarImages) != 0)
	{
		this->internal_computePolarImageDescriptors(in_img, inout_features);
		++nDescComputed;
	}
	if ((in_descriptor_list & descLogPolarImages) != 0)
	{
		this->internal_computeLogPolarImageDescriptors(in_img, inout_features);
		++nDescComputed;
	}
	if ((in_descriptor_list & descORB) != 0)
	{
		this->internal_computeORBDescriptors(in_img, inout_features);
		++nDescComputed;
	}
	// # added by Raghavender Sahdev
	if ((in_descriptor_list & descBLD) != 0)
	{
		this->internal_computeBLDLineDescriptors(in_img, inout_features);
		++nDescComputed;
	}
	if ((in_descriptor_list & descLATCH) != 0)
	{
		this->internal_computeLATCHDescriptors(in_img, inout_features);
		++nDescComputed;
	}
	if (!nDescComputed)
		THROW_EXCEPTION_FMT(
			"No known descriptor value found in in_descriptor_list=%u",
			(unsigned)in_descriptor_list);

	MRPT_END
}

void CFeatureExtraction::TOptions::dumpToTextStream(std::ostream& out) const
{
	out << "\n----------- [CFeatureExtraction::TOptions] ------------ \n\n";

	LOADABLEOPTS_DUMP_VAR(featsType, int)
	LOADABLEOPTS_DUMP_VAR(patchSize, int)
	LOADABLEOPTS_DUMP_VAR(FIND_SUBPIXEL, bool)
	LOADABLEOPTS_DUMP_VAR(useMask, bool)
	LOADABLEOPTS_DUMP_VAR(addNewFeatures, bool)

	LOADABLEOPTS_DUMP_VAR(harrisOptions.k, double)
	LOADABLEOPTS_DUMP_VAR(harrisOptions.radius, int)
	LOADABLEOPTS_DUMP_VAR(harrisOptions.threshold, float)
	LOADABLEOPTS_DUMP_VAR(harrisOptions.sigma, float)
	LOADABLEOPTS_DUMP_VAR(harrisOptions.min_distance, float)

	LOADABLEOPTS_DUMP_VAR(KLTOptions.min_distance, float)
	LOADABLEOPTS_DUMP_VAR(KLTOptions.threshold, float)
	LOADABLEOPTS_DUMP_VAR(KLTOptions.radius, int)

	LOADABLEOPTS_DUMP_VAR(SIFTOptions.implementation, int)

	LOADABLEOPTS_DUMP_VAR(SURFOptions.rotation_invariant, bool)
	LOADABLEOPTS_DUMP_VAR(SURFOptions.hessianThreshold, int)
	LOADABLEOPTS_DUMP_VAR(SURFOptions.nOctaves, int)
	LOADABLEOPTS_DUMP_VAR(SURFOptions.nLayersPerOctave, int)

	LOADABLEOPTS_DUMP_VAR(FASTOptions.threshold, int)
	LOADABLEOPTS_DUMP_VAR(FASTOptions.nonmax_suppression, bool)
	LOADABLEOPTS_DUMP_VAR(FASTOptions.min_distance, float)
	LOADABLEOPTS_DUMP_VAR(FASTOptions.use_KLT_response, bool)

	LOADABLEOPTS_DUMP_VAR(ORBOptions.scale_factor, float)
	LOADABLEOPTS_DUMP_VAR(ORBOptions.min_distance, int)
	LOADABLEOPTS_DUMP_VAR(ORBOptions.n_levels, int)
	LOADABLEOPTS_DUMP_VAR(ORBOptions.extract_patch, bool)

	LOADABLEOPTS_DUMP_VAR(SpinImagesOptions.hist_size_distance, int)
	LOADABLEOPTS_DUMP_VAR(SpinImagesOptions.hist_size_intensity, int)
	LOADABLEOPTS_DUMP_VAR(SpinImagesOptions.radius, int)
	LOADABLEOPTS_DUMP_VAR(SpinImagesOptions.std_dist, float)
	LOADABLEOPTS_DUMP_VAR(SpinImagesOptions.std_intensity, float)

	LOADABLEOPTS_DUMP_VAR(PolarImagesOptions.bins_angle, int)
	LOADABLEOPTS_DUMP_VAR(PolarImagesOptions.bins_distance, int)
	LOADABLEOPTS_DUMP_VAR(PolarImagesOptions.radius, int)

	LOADABLEOPTS_DUMP_VAR(LogPolarImagesOptions.radius, int)
	LOADABLEOPTS_DUMP_VAR(LogPolarImagesOptions.num_angles, int)
	LOADABLEOPTS_DUMP_VAR(LogPolarImagesOptions.rho_scale, double)

	// # added by Raghavender Sahdev
	LOADABLEOPTS_DUMP_VAR(AKAZEOptions.descriptor_type, int)
	LOADABLEOPTS_DUMP_VAR(AKAZEOptions.descriptor_size, int)
	LOADABLEOPTS_DUMP_VAR(AKAZEOptions.descriptor_channels, int)
	LOADABLEOPTS_DUMP_VAR(AKAZEOptions.threshold, float)
	LOADABLEOPTS_DUMP_VAR(AKAZEOptions.nOctaves, int)
	LOADABLEOPTS_DUMP_VAR(AKAZEOptions.nOctaveLayers, int)
	LOADABLEOPTS_DUMP_VAR(AKAZEOptions.diffusivity, int)

	LOADABLEOPTS_DUMP_VAR(LSDOptions.nOctaves, int)
	LOADABLEOPTS_DUMP_VAR(LSDOptions.scale, int)

	LOADABLEOPTS_DUMP_VAR(BLDOptions.numOfOctave, int)
	LOADABLEOPTS_DUMP_VAR(BLDOptions.reductionRatio, int)
	LOADABLEOPTS_DUMP_VAR(BLDOptions.widthOfBand, int)

	LOADABLEOPTS_DUMP_VAR(LATCHOptions.bytes, int)
	LOADABLEOPTS_DUMP_VAR(LATCHOptions.half_ssd_size, int)
	LOADABLEOPTS_DUMP_VAR(LATCHOptions.rotationInvariance, bool)

	out << "\n";
}

/*---------------------------------------------------------------
					loadFromConfigFile
  ---------------------------------------------------------------*/
void CFeatureExtraction::TOptions::loadFromConfigFile(
	const mrpt::config::CConfigFileBase& iniFile, const std::string& section)
{
	featsType = iniFile.read_enum(section, "featsType", featsType);

	MRPT_LOAD_CONFIG_VAR(patchSize, int, iniFile, section)
	MRPT_LOAD_CONFIG_VAR(FIND_SUBPIXEL, bool, iniFile, section)
	MRPT_LOAD_CONFIG_VAR(useMask, bool, iniFile, section)
	MRPT_LOAD_CONFIG_VAR(addNewFeatures, bool, iniFile, section)

	// string sect = section;
	MRPT_LOAD_CONFIG_VAR(harrisOptions.k, double, iniFile, section)
	MRPT_LOAD_CONFIG_VAR(harrisOptions.radius, int, iniFile, section)
	MRPT_LOAD_CONFIG_VAR(harrisOptions.threshold, float, iniFile, section)
	MRPT_LOAD_CONFIG_VAR(harrisOptions.sigma, float, iniFile, section)
	MRPT_LOAD_CONFIG_VAR(harrisOptions.min_distance, float, iniFile, section)

	MRPT_LOAD_CONFIG_VAR(KLTOptions.min_distance, float, iniFile, section)
	MRPT_LOAD_CONFIG_VAR(KLTOptions.threshold, float, iniFile, section)
	MRPT_LOAD_CONFIG_VAR(KLTOptions.radius, int, iniFile, section)

	MRPT_LOAD_CONFIG_VAR_CAST(
		SIFTOptions.implementation, int, TSIFTImplementation, iniFile, section)
	MRPT_LOAD_CONFIG_VAR(SIFTOptions.threshold, double, iniFile, section)
	MRPT_LOAD_CONFIG_VAR(SIFTOptions.edgeThreshold, double, iniFile, section)

	MRPT_LOAD_CONFIG_VAR(SURFOptions.rotation_invariant, bool, iniFile, section)
	MRPT_LOAD_CONFIG_VAR(SURFOptions.hessianThreshold, int, iniFile, section)
	MRPT_LOAD_CONFIG_VAR(SURFOptions.nOctaves, int, iniFile, section)
	MRPT_LOAD_CONFIG_VAR(SURFOptions.nLayersPerOctave, int, iniFile, section)

	MRPT_LOAD_CONFIG_VAR(FASTOptions.threshold, int, iniFile, section)
	MRPT_LOAD_CONFIG_VAR(FASTOptions.nonmax_suppression, bool, iniFile, section)
	MRPT_LOAD_CONFIG_VAR(FASTOptions.min_distance, int, iniFile, section)
	MRPT_LOAD_CONFIG_VAR(FASTOptions.use_KLT_response, bool, iniFile, section)

	MRPT_LOAD_CONFIG_VAR(ORBOptions.extract_patch, bool, iniFile, section)
	MRPT_LOAD_CONFIG_VAR(ORBOptions.min_distance, int, iniFile, section)
	MRPT_LOAD_CONFIG_VAR(ORBOptions.n_levels, int, iniFile, section)
	MRPT_LOAD_CONFIG_VAR(ORBOptions.scale_factor, float, iniFile, section)

	MRPT_LOAD_CONFIG_VAR(
		SpinImagesOptions.hist_size_distance, int, iniFile, section)
	MRPT_LOAD_CONFIG_VAR(
		SpinImagesOptions.hist_size_intensity, int, iniFile, section)
	MRPT_LOAD_CONFIG_VAR(SpinImagesOptions.radius, int, iniFile, section)
	MRPT_LOAD_CONFIG_VAR(SpinImagesOptions.std_dist, float, iniFile, section)
	MRPT_LOAD_CONFIG_VAR(
		SpinImagesOptions.std_intensity, float, iniFile, section)

	MRPT_LOAD_CONFIG_VAR(PolarImagesOptions.bins_angle, int, iniFile, section)
	MRPT_LOAD_CONFIG_VAR(
		PolarImagesOptions.bins_distance, int, iniFile, section)
	MRPT_LOAD_CONFIG_VAR(PolarImagesOptions.radius, int, iniFile, section)

	MRPT_LOAD_CONFIG_VAR(LogPolarImagesOptions.radius, int, iniFile, section)
	MRPT_LOAD_CONFIG_VAR(
		LogPolarImagesOptions.num_angles, int, iniFile, section)
	MRPT_LOAD_CONFIG_VAR(
		LogPolarImagesOptions.rho_scale, double, iniFile, section)

	// #added by Raghavender Sahdev
	MRPT_LOAD_CONFIG_VAR(AKAZEOptions.descriptor_type, int, iniFile, section)
	MRPT_LOAD_CONFIG_VAR(AKAZEOptions.descriptor_size, int, iniFile, section)
	MRPT_LOAD_CONFIG_VAR(
		AKAZEOptions.descriptor_channels, int, iniFile, section)
	MRPT_LOAD_CONFIG_VAR(AKAZEOptions.threshold, float, iniFile, section)
	MRPT_LOAD_CONFIG_VAR(AKAZEOptions.nOctaves, int, iniFile, section)
	MRPT_LOAD_CONFIG_VAR(AKAZEOptions.nOctaveLayers, int, iniFile, section)
	MRPT_LOAD_CONFIG_VAR(AKAZEOptions.diffusivity, int, iniFile, section)

	MRPT_LOAD_CONFIG_VAR(LSDOptions.nOctaves, int, iniFile, section)
	MRPT_LOAD_CONFIG_VAR(LSDOptions.scale, int, iniFile, section)

	MRPT_LOAD_CONFIG_VAR(BLDOptions.numOfOctave, int, iniFile, section)
	MRPT_LOAD_CONFIG_VAR(BLDOptions.widthOfBand, int, iniFile, section)
	MRPT_LOAD_CONFIG_VAR(BLDOptions.reductionRatio, int, iniFile, section)

	MRPT_LOAD_CONFIG_VAR(LATCHOptions.bytes, int, iniFile, section)
	MRPT_LOAD_CONFIG_VAR(LATCHOptions.half_ssd_size, int, iniFile, section)
	MRPT_LOAD_CONFIG_VAR(
		LATCHOptions.rotationInvariance, bool, iniFile, section)
}
