/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          https://www.mrpt.org/                            |
   |                                                                           |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file        |
   | See: https://www.mrpt.org/Authors - All rights reserved.                  |
   | Released under BSD License. See details in https://www.mrpt.org/License   |
   +---------------------------------------------------------------------------+
   */

#include "vision-precomp.h"  // Precompiled headers

#include <mrpt/io/CFileInputStream.h>
#include <mrpt/io/CFileOutputStream.h>
#include <mrpt/io/CTextFileLinesParser.h>
#include <mrpt/math/data_utils.h>
#include <mrpt/serialization/optional_serialization.h>
#include <mrpt/serialization/stl_serialization.h>
#include <mrpt/system/os.h>
#include <mrpt/vision/CFeature.h>
#include <mrpt/vision/types.h>
#include <mrpt/vision/utils.h>
#include <iostream>

using namespace mrpt;
using namespace mrpt::vision;
using namespace mrpt::img;
using namespace mrpt::math;
using namespace mrpt::system;
using namespace mrpt::io;
using namespace std;

IMPLEMENTS_SERIALIZABLE(CFeature, CSerializable, mrpt::vision)

// --------------------------------------------------
//			loadFromConfigFile
// --------------------------------------------------
/**  Load all the params from a config source, in the format described in
 * saveToConfigFile()
 */
void TMultiResDescMatchOptions::loadFromConfigFile(
	const mrpt::config::CConfigFileBase& cfg, const std::string& section)
{
	useOriFilter = cfg.read_bool(section, "useOriFilter", true, false);
	oriThreshold = cfg.read_double(section, "oriThreshold", 0.2, false);
	lastSeenThreshold = cfg.read_int(section, "lastSeenThreshold", 10, false);
	timesSeenThreshold = cfg.read_int(section, "timesSeenThreshold", 5, false);
	minFeaturesToFind = cfg.read_int(section, "minFeaturesToFind", 5, false);
	minFeaturesToBeLost =
		cfg.read_int(section, "minFeaturesToBeLost", 5, false);

	useDepthFilter = cfg.read_bool(section, "useDepthFilter", true, false);

	matchingThreshold =
		cfg.read_double(section, "matchingThreshold", 1e4, false);
	matchingRatioThreshold =
		cfg.read_double(section, "matchingRatioThreshold", 0.5, false);

	lowScl1 = cfg.read_int(section, "lowScl1", 0, false);
	lowScl2 = cfg.read_int(section, "lowScl1", 0, false);
	highScl1 = cfg.read_int(section, "highScl1", 6, false);
	highScl2 = cfg.read_int(section, "highScl2", 6, false);

	searchAreaSize = cfg.read_uint64_t(section, "searchAreaSize", 20, false);
}

// --------------------------------------------------
//			saveToConfigFile
// --------------------------------------------------
void TMultiResDescMatchOptions::saveToConfigFile(
	mrpt::config::CConfigFileBase& cfg, const std::string& section) const
{
	if (useOriFilter)
	{
		cfg.write(section, "useOriFilter", "true");
		cfg.write(section, "oriThreshold", oriThreshold);
	}
	else
		cfg.write(section, "useOriFilter", "false");

	if (useDepthFilter)
		cfg.write(section, "useDepthFilter", "true");
	else
		cfg.write(section, "useDepthFilter", "false");

	cfg.write(section, "matchingThreshold", matchingThreshold);
	cfg.write(section, "matchingRatioThreshold", matchingRatioThreshold);
	cfg.write(section, "lowScl1", lowScl1);
	cfg.write(section, "lowScl2", lowScl2);
	cfg.write(section, "highScl1", highScl1);
	cfg.write(section, "highScl2", highScl2);

	cfg.write(section, "searchAreaSize", searchAreaSize);
	cfg.write(section, "lastSeenThreshold", lastSeenThreshold);
	cfg.write(section, "timesSeenThreshold", timesSeenThreshold);
	cfg.write(section, "minFeaturesToFind", minFeaturesToFind);
	cfg.write(section, "minFeaturesToBeLost", minFeaturesToBeLost);
}  // end-saveToConfigFile

// --------------------------------------------------
//			dumpToTextStream
// --------------------------------------------------
void TMultiResDescMatchOptions::dumpToTextStream(std::ostream& out) const
{
	out << "\n----------- [vision::TMultiResDescMatchOptions] ------------ \n";
	out << "Use orientation filter?:        ";
	if (useOriFilter)
	{
		out << "Yes\n";
		out << mrpt::format(
			"· Orientation threshold:        %.1f deg\n",
			RAD2DEG(oriThreshold));
	}
	else
		out << "No\n";
	out << "Use depth filter?:              ";
	if (useDepthFilter)
		out << "Yes\n";
	else
	{
		out << "No\n";
		out << mrpt::format("Lowest scale in list1:          %d\n", lowScl1);
		out << mrpt::format("Highest scale in list1:         %d\n", highScl1);
		out << mrpt::format("Lowest scale in list2:          %d\n", lowScl2);
		out << mrpt::format("Highest scale in list2:         %d\n", highScl2);
	}
	out << mrpt::format(
		"#frames last seen threshold:    %d\n", lastSeenThreshold);
	out << mrpt::format(
		"#frames to be stable threshold: %d\n", timesSeenThreshold);
	out << mrpt::format(
		"min. # features in system:      %d\n", minFeaturesToFind);
	out << mrpt::format(
		"min. # features to be lost:     %d\n", minFeaturesToBeLost);
	out << mrpt::format(
		"Matching threshold:             %.2f\n", matchingThreshold);
	out << mrpt::format(
		"Matching ratio threshold:       %.2f\n", matchingRatioThreshold);
	out << mrpt::format(
		"Size of the search window:      %d px\n", searchAreaSize);
	out << "-------------------------------------------------------- \n";
}  // end-dumpToTextStream

// --------------------------------------------------
//			loadFromConfigFile
// --------------------------------------------------
/**  Load all the params from a config source, in the format described in
 * saveToConfigFile()
 */
void TMultiResDescOptions::loadFromConfigFile(
	const mrpt::config::CConfigFileBase& cfg, const std::string& section)
{
	basePSize = cfg.read_uint64_t(section, "basePSize", 23, false);
	comLScl = cfg.read_int(section, "comLScl", 0, false);
	comHScl = cfg.read_int(section, "comHScl", 6, false);
	sg1 = cfg.read_double(section, "sg1", 0.5, false);
	sg2 = cfg.read_double(section, "sg2", 7.5, false);
	sg3 = cfg.read_double(section, "sg3", 8.0, false);
	computeDepth = cfg.read_bool(section, "computeDepth", true, false);
	blurImage = cfg.read_bool(section, "blurImage", true, false);
	fx = cfg.read_double(section, "fx", 0.0, false);
	cx = cfg.read_double(section, "cx", 0.0, false);
	cy = cfg.read_double(section, "cy", 0.0, false);
	baseline = cfg.read_double(section, "baseline", 0.0, false);
	computeHashCoeffs =
		cfg.read_bool(section, "computeHashCoeffs", false, false);

	cfg.read_vector(section, "scales", vector<double>(), scales, false);
	if (scales.size() < 1)
	{
		scales.resize(7);
		scales[0] = 0.5;
		scales[1] = 0.8;
		scales[2] = 1.0;
		scales[3] = 1.2;
		scales[4] = 1.5;
		scales[5] = 1.8;
		scales[6] = 2.0;
	}  // end-if
}

// --------------------------------------------------
//			saveToConfigFile
// --------------------------------------------------
void TMultiResDescOptions::saveToConfigFile(
	mrpt::config::CConfigFileBase& cfg, const std::string& section) const
{
	cfg.write(section, "basePSize", basePSize);
	cfg.write(section, "comLScl", comLScl);
	cfg.write(section, "comHScl", comHScl);
	cfg.write(section, "sg1", sg1);
	cfg.write(section, "sg2", sg2);
	cfg.write(section, "sg3", sg3);

	cfg.write(section, "computeDepth", computeDepth ? "true" : "false");
	cfg.write(section, "blurImage", blurImage ? "true" : "false");
	cfg.write(section, "fx", fx);
	cfg.write(section, "cx", cx);
	cfg.write(section, "cy", cy);
	cfg.write(section, "baseline", baseline);
	cfg.write(
		section, "computeHashCoeffs", computeHashCoeffs ? "true" : "false");

	char buf[300];
	for (double scale : scales)
		mrpt::system::os::sprintf(buf, 300, "%.2f ", scale);
	cfg.write(section, "scales", buf);
}  // end-saveToConfigFile

// --------------------------------------------------
//			dumpToTextStream
// --------------------------------------------------
void TMultiResDescOptions::dumpToTextStream(std::ostream& out) const
{
	out << "\n----------- [vision::TMultiResDescOptions] ------------ \n";
	out << mrpt::format("Base patch size:                %d px\n", basePSize);
	out << mrpt::format("Lowest scale to compute:        %d\n", comLScl);
	out << mrpt::format("Highest scale to compute:       %d\n", comHScl);
	out << mrpt::format("Image smoothing sigma:          %.2f px\n", sg1);
	out << mrpt::format("Orientation histogram sigma:    %.2f\n", sg2);
	out << mrpt::format("Descriptor histogram sigma:     %.2f\n", sg3);
	out << "Compute depth:                  ";
	if (computeDepth)
	{
		out << "Yes\n";
		out << mrpt::format("Focal length:                   %.2f px\n", fx);
		out << mrpt::format("Principal point (cx):           %.2f px\n", cx);
		out << mrpt::format("Principal point (cy):           %.2f px\n", cy);
		out << mrpt::format(
			"Baseline:                       %.2f m\n", baseline);
	}
	else
		out << "No\n";

	out << "Compute Hash Coeffs:            ";
	if (computeHashCoeffs)
		out << "Yes\n";
	else
		out << "No\n";

	out << "Blur image previously:          ";
	if (blurImage)
		out << "Yes\n";
	else
		out << "No\n";

	out << "Scales:                         ";
	for (double scale : scales) out << mrpt::format("%.2f ", scale);
	out << "\n";
	out << "-------------------------------------------------------- \n";
}  // end-dumpToTextStream

void CFeature::dumpToTextStream(std::ostream& out) const
{
	out << "\n----------- [vision::CFeature] ------------ \n";
	out << mrpt::format(
		"Feature ID:                     %d\n", (int)keypoint.ID);
	out << mrpt::format(
		"Coordinates:                    (%.2f,%.2f) px\n", keypoint.pt.x,
		keypoint.pt.y);
	out << mrpt::format("PatchSize:                      %d\n", patchSize);
	out << "Type:                           ";
	out << mrpt::typemeta::TEnumType<decltype(type)>::value2name(type) << "\n";
	out << "Status:                         ";
	switch (track_status)
	{
		case 0:
			out << "Idle\n";
			break;
		case 1:
			out << "[KLT] Out of bounds [KLT]\n";
			break;
		case 5:
			out << "[KLT] Tracked\n";
			break;
		case 10:
			out << "[KLT] Lost\n";
			break;
	}

	out << mrpt::format("Response:                       %.2f\n", response);
	out << mrpt::format("Main orientation:               %.2f\n", orientation);
	out << mrpt::format(
		"Initial Depth:                  %.2f m\n", initialDepth);
	out << mrpt::format("Depth:                          %.2f m\n", depth);
	out << mrpt::format(
		"3D point:                       (%.2f,%.2f,%.2f) m\n", p3D.x, p3D.y,
		p3D.z);
	out << "Is point feature?:              ";
	isPointFeature() ? out << "Yes\n" : out << "No\n";

	out << "Has SIFT descriptor?:           ";
	descriptors.hasDescriptorSIFT() ? out << "Yes\n" : out << "No\n";
	out << "Has SURF descriptor?:           ";
	descriptors.hasDescriptorSURF() ? out << "Yes\n" : out << "No\n";
	out << "Has Spin image descriptor?:     ";
	descriptors.hasDescriptorSpinImg() ? out << "Yes\n" : out << "No\n";
	out << "Has Polar descriptor?:          ";
	descriptors.hasDescriptorPolarImg() ? out << "Yes\n" : out << "No\n";
	out << "Has Log Polar descriptor?:      ";
	descriptors.hasDescriptorLogPolarImg() ? out << "Yes\n" : out << "No\n";
	out << "Has ORB descriptor?:			";
	descriptors.hasDescriptorORB() ? out << "Yes\n" : out << "No\n";
	//# added by Raghavender Sahdev
	out << "Has BLD descriptor?:			";
	descriptors.hasDescriptorBLD() ? out << "Yes\n" : out << "No\n";
	out << "Has LATCH descriptor?:			";
	descriptors.hasDescriptorLATCH() ? out << "Yes\n" : out << "No\n";

}  // end dumpToTextStream

void CFeature::dumpToConsole() const { dumpToTextStream(std::cout); }
uint8_t CFeature::serializeGetVersion() const { return 2; }
void CFeature::serializeTo(mrpt::serialization::CArchive& out) const
{
	// The coordinates:
	out << keypoint.pt.x << keypoint.pt.y << keypoint.ID << patch << patchSize
		<< (uint32_t)type << (uint32_t)keypoint.track_status
		<< keypoint.response << orientation << keypoint.octave << user_flags
		<< depth << initialDepth << p3D << descriptors.SIFT << descriptors.SURF
		<< descriptors.SpinImg << descriptors.SpinImg_range_rows
		<< descriptors.PolarImg << descriptors.LogPolarImg
		<< descriptors.polarImgsNoRotation << descriptors.ORB << descriptors.BLD
		<< descriptors.LATCH;
}

void CFeature::serializeFrom(mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 0:
		case 1:
		case 2:
		{
			// The coordinates:
			uint32_t aux_type, aux_KLTS;
			in >> keypoint.pt.x >> keypoint.pt.y >> keypoint.ID >> patch >>
				patchSize >> aux_type >> aux_KLTS >> keypoint.response >>
				orientation >> keypoint.octave >> user_flags;
			if (version > 0)
			{
				in >> depth >> initialDepth >> p3D;
			}
			in >> descriptors.SIFT >> descriptors.SURF >> descriptors.SpinImg >>
				descriptors.SpinImg_range_rows >> descriptors.PolarImg >>
				descriptors.LogPolarImg >> descriptors.polarImgsNoRotation >>
				descriptors.BLD >> descriptors.LATCH;
			if (version > 1) in >> descriptors.ORB;

			type = (TKeyPointMethod)aux_type;
			track_status = (TFeatureTrackStatus)aux_KLTS;
		}
		break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
	};
}

/****************************************************
				Class CFEATURE
*****************************************************/

// Return false only for Blob detectors (SIFT, SURF)
bool CFeature::isPointFeature() const
{
	return type == featSIFT || type == featSURF;
}

// --------------------------------------------------
//			patchCorrelationTo
// --------------------------------------------------
float CFeature::patchCorrelationTo(const CFeature& oFeature) const
{
	MRPT_START
	ASSERT_(patch);

	ASSERT_(patch->getWidth() == oFeature.patch->getWidth());
	ASSERT_(patch->getHeight() == oFeature.patch->getHeight());
	ASSERT_(patch->getHeight() > 0 && patch->getWidth() > 0);
	size_t x_max, y_max;
	double max_val;
	mrpt::vision::openCV_cross_correlation(
		*patch, *oFeature.patch, x_max, y_max, max_val);

	// Value as "distance" in the range [0,1], best = 0
	return d2f(0.5 - 0.5 * max_val);

	MRPT_END
}

// --------------------------------------------------
//			descriptorDistanceTo
// --------------------------------------------------
float CFeature::descriptorDistanceTo(
	const CFeature& oFeature, TDescriptorType descriptorToUse,
	bool normalize_distances) const
{
	MRPT_START

	// If we are not ask for a specific descriptor, select the first one found:
	if (descriptorToUse == descAny)
	{
		if (descriptors.hasDescriptorSIFT())
			descriptorToUse = descSIFT;
		else if (descriptors.hasDescriptorSURF())
			descriptorToUse = descSURF;
		else if (descriptors.hasDescriptorSpinImg())
			descriptorToUse = descSpinImages;
		else if (descriptors.hasDescriptorPolarImg())
			descriptorToUse = descPolarImages;
		else if (descriptors.hasDescriptorLogPolarImg())
			descriptorToUse = descLogPolarImages;
		else if (descriptors.hasDescriptorORB())
			descriptorToUse = descORB;
		// # added by Raghavender Sahdev - BLD/LATCH descriptors
		else if (descriptors.hasDescriptorBLD())
			descriptorToUse = descBLD;
		else if (descriptors.hasDescriptorLATCH())
			descriptorToUse = descLATCH;
		else
			THROW_EXCEPTION(
				"Feature has no descriptors and descriptorToUse=descAny");
	}

	switch (descriptorToUse)
	{
		case descSIFT:
			return descriptorSIFTDistanceTo(oFeature, normalize_distances);
		case descSURF:
			return descriptorSURFDistanceTo(oFeature, normalize_distances);
		case descSpinImages:
			return descriptorSpinImgDistanceTo(oFeature, normalize_distances);
		case descPolarImages:
		{
			float minAng;
			return descriptorPolarImgDistanceTo(
				oFeature, minAng, normalize_distances);
		}
		case descLogPolarImages:
		{
			float minAng;
			return descriptorLogPolarImgDistanceTo(
				oFeature, minAng, normalize_distances);
		}
		case descORB:
			return float(descriptorORBDistanceTo(oFeature));
		// # added by Raghavender Sahdev
		case descBLD:
			return (descriptorBLDDistanceTo(oFeature));
		case descLATCH:
			return (descriptorLATCHDistanceTo(oFeature));
		default:
			THROW_EXCEPTION_FMT(
				"Unknown value for 'descriptorToUse'=%u",
				(unsigned)descriptorToUse);
	}

	MRPT_END
}

// --------------------------------------------------
// descriptorSIFTDistanceTo
// --------------------------------------------------
float CFeature::descriptorSIFTDistanceTo(
	const CFeature& oFeature, bool normalize_distances) const
{
	ASSERT_(descriptors.SIFT);
	ASSERT_(oFeature.descriptors.SIFT);
	ASSERT_(descriptors.SIFT->size() == oFeature.descriptors.SIFT->size());
	ASSERT_(
		descriptors.hasDescriptorSIFT() &&
		oFeature.descriptors.hasDescriptorSIFT());

	float dist = 0.0f;
	std::vector<unsigned char>::const_iterator itDesc1, itDesc2;
	for (itDesc1 = descriptors.SIFT->begin(),
		itDesc2 = oFeature.descriptors.SIFT->begin();
		 itDesc1 != descriptors.SIFT->end(); itDesc1++, itDesc2++)
	{
		dist += square(*itDesc1 - *itDesc2);
	}
	if (normalize_distances) dist /= descriptors.SIFT->size();
	dist = sqrt(dist);
	if (normalize_distances) dist /= 64.0f;
	return dist;
}  // end descriptorSIFTDistanceTo

// --------------------------------------------------
// descriptorSURFDistanceTo
// --------------------------------------------------
float CFeature::descriptorSURFDistanceTo(
	const CFeature& oFeature, bool normalize_distances) const
{
	ASSERT_(descriptors.SURF);
	ASSERT_(descriptors.SURF->size() == oFeature.descriptors.SURF->size());
	ASSERT_(
		descriptors.hasDescriptorSURF() &&
		oFeature.descriptors.hasDescriptorSURF());

	float dist = 0.0f;
	std::vector<float>::const_iterator itDesc1, itDesc2;
	for (itDesc1 = descriptors.SURF->begin(),
		itDesc2 = oFeature.descriptors.SURF->begin();
		 itDesc1 != descriptors.SURF->end(); itDesc1++, itDesc2++)
	{
		dist += square(*itDesc1 - *itDesc2);
	}
	if (normalize_distances) dist /= descriptors.SURF->size();
	dist = sqrt(dist);
	if (normalize_distances)
		dist /= 0.20f;  // JL: Ad-hoc value! Investigate where does this come
	// from...
	return dist;
}  // end descriptorSURFDistanceTo

// --------------------------------------------------
// descriptorSpinImgDistanceTo
// --------------------------------------------------
float CFeature::descriptorSpinImgDistanceTo(
	const CFeature& oFeature, bool normalize_by_vector_length) const
{
	ASSERT_(descriptors.SpinImg);

	ASSERT_(
		descriptors.SpinImg->size() == oFeature.descriptors.SpinImg->size());
	ASSERT_(
		descriptors.hasDescriptorSpinImg() &&
		oFeature.descriptors.hasDescriptorSpinImg());
	ASSERT_(!descriptors.SpinImg->empty());
	float dist = 0.0f;
	std::vector<float>::const_iterator itDesc1, itDesc2;
	for (itDesc1 = descriptors.SpinImg->begin(),
		itDesc2 = oFeature.descriptors.SpinImg->begin();
		 itDesc1 != descriptors.SpinImg->end(); itDesc1++, itDesc2++)
	{
		dist += square(*itDesc1 - *itDesc2);
	}

	if (normalize_by_vector_length) dist /= 0.25f * descriptors.SpinImg->size();

	return sqrt(dist);
}  // end descriptorSpinImgDistanceTo

// --------------------------------------------------
//        descriptorPolarImgDistanceTo
// --------------------------------------------------
float CFeature::internal_distanceBetweenPolarImages(
	const CMatrixF& desc1, const CMatrixF& desc2, float& minDistAngle,
	bool normalize_distances, bool dont_shift_angle)
{
	MRPT_START

	// Find the smallest distance:
	unsigned int delta, i, j, ii, height = desc1.rows(), width = desc1.cols();
	float dist, minDist = 0;

	//#define LM_CORR_BIAS_MEAN

#define LM_CORR_METHOD_EUCLID
	//#define LM_CORR_METHOD_MANHATTAN
	//#define LM_CORR_METHOD_CORRELATION

#if defined(LM_CORR_BIAS_MEAN) || defined(LM_CORR_METHOD_CORRELATION)
	const float desc1_mean = desc1.sum() / d2f(width * height);
	const float desc2_mean = desc2.sum() / d2f(width * height);
#endif

	CVectorFloat distances(height, 0);  // Distances for each shift

	for (delta = 0; delta < height; delta++)
	{
#if defined(LM_CORR_METHOD_CORRELATION)
		float s11 = 0;
		float s22 = 0;
		float s12 = 0;
#endif
		// Compute the mean distance between desc1[t] and desc2[t-delta]:
		dist = 0;
		for (i = 0; i < height; i++)
		{
			ii = (i + delta) % height;  // Shifted index
			for (j = 0; j < width; j++)
			{
#ifdef LM_CORR_METHOD_EUCLID
#ifdef LM_CORR_BIAS_MEAN
				dist += square(
					desc1(i, j) - desc1_mean - desc2(ii, j) + desc2_mean);
#else
				dist += square(desc1(i, j) - desc2(ii, j));
#endif
#elif defined(LM_CORR_METHOD_MANHATTAN)
#ifdef LM_CORR_BIAS_MEAN
				dist += std::abs(
					desc1(i, j) - desc1_mean - desc2(ii, j) + desc2_mean);
#else
				dist += std::abs(desc1(i, j) - desc2(ii, j));
#endif
#elif defined(LM_CORR_METHOD_CORRELATION)
				float d1 = desc1(i, j) - desc1_mean;
				float d2 = desc2(ii, j) - desc2_mean;
				s11 += square(d1);
				s22 += square(d2);
				s12 += d1 * d2;
#else
#error A LM_CORR_METHOD_XXX method must be selected!
#endif
			}
		}

		// Average:
		if (normalize_distances) dist /= d2f(width * height);

#ifdef LM_CORR_METHOD_EUCLID
		dist = sqrt(dist);
#endif

#if defined(LM_CORR_METHOD_CORRELATION)
		dist = 1 - (s12 / sqrt(s11 * s22));
#endif

		distances[delta] = dist;
		if (!delta && dont_shift_angle)
		{
			distances.resize(1);
			break;
		}
	}  // end for delta

	size_t minDistIdx;
	minDist = distances.minCoeff(minDistIdx);

	double dist_mean, dist_std;
	mrpt::math::meanAndStd(distances, dist_mean, dist_std);

#if 0
	{
		cout << "min dist: " << minDist << endl;

		static mrpt::gui::CDisplayWindowPlots	win("distances");
		win.plot(distances,"b.4");
		CImage img1(desc1);
		win.image(img1,0,-0.5,0.4*width,0.5,"img1");

		CImage img2(desc2);
		win.image(img2,0.6*width,-0.5,0.4*width,0.5,"img2");

		//win.axis_fit();
		win.waitForKey();
	}
#endif

	// Output:
	minDistAngle = minDistIdx * d2f(M_2PI / width);
	return minDist;

	MRPT_END
}

// --------------------------------------------------
//        descriptorPolarImgDistanceTo
// --------------------------------------------------
float CFeature::descriptorPolarImgDistanceTo(
	const CFeature& oFeature, float& minDistAngle,
	bool normalize_distances) const
{
	MRPT_START

	ASSERT_(descriptors.PolarImg);
	ASSERT_EQUAL_(
		descriptors.PolarImg->rows(), oFeature.descriptors.PolarImg->rows());
	ASSERT_EQUAL_(
		descriptors.PolarImg->cols(), oFeature.descriptors.PolarImg->cols());
	ASSERT_(
		descriptors.hasDescriptorPolarImg() &&
		oFeature.descriptors.hasDescriptorPolarImg());
	ASSERT_GT_(descriptors.PolarImg->rows(), 1);
	ASSERT_GT_(descriptors.PolarImg->cols(), 1);

	// Call the common method for computing these distances:
	return internal_distanceBetweenPolarImages(
		*descriptors.PolarImg, *oFeature.descriptors.PolarImg, minDistAngle,
		normalize_distances, descriptors.polarImgsNoRotation);

	MRPT_END
}  // end descriptorPolarImgDistanceTo

// --------------------------------------------------
//        descriptorLogPolarImgDistanceTo
// --------------------------------------------------
float CFeature::descriptorLogPolarImgDistanceTo(
	const CFeature& oFeature, float& minDistAngle,
	bool normalize_distances) const
{
	MRPT_START

	ASSERT_(descriptors.LogPolarImg);
	ASSERT_(
		descriptors.LogPolarImg->rows() ==
		oFeature.descriptors.LogPolarImg->rows());
	ASSERT_(
		descriptors.LogPolarImg->cols() ==
		oFeature.descriptors.LogPolarImg->cols());
	ASSERT_(
		descriptors.hasDescriptorLogPolarImg() &&
		oFeature.descriptors.hasDescriptorLogPolarImg());
	ASSERT_(
		descriptors.LogPolarImg->rows() > 1 &&
		descriptors.LogPolarImg->cols() > 1);

	// Call the common method for computing these distances:
	return internal_distanceBetweenPolarImages(
		*descriptors.LogPolarImg, *oFeature.descriptors.LogPolarImg,
		minDistAngle, normalize_distances, descriptors.polarImgsNoRotation);

	MRPT_END
}  // end descriptorPolarImgDistanceTo

// --------------------------------------------------
//        descriptorORBDistanceTo
// --------------------------------------------------
uint8_t CFeature::descriptorORBDistanceTo(const CFeature& oFeature) const
{
	ASSERT_(
		descriptors.hasDescriptorORB() &&
		oFeature.descriptors.hasDescriptorORB());
	ASSERT_(descriptors.ORB->size() == oFeature.descriptors.ORB->size());
	const std::vector<uint8_t>& t_desc = *descriptors.ORB;
	const std::vector<uint8_t>& o_desc = *oFeature.descriptors.ORB;

	// Descriptors XOR + Hamming weight
	uint8_t distance = 0;
	for (uint8_t k = 0; k < t_desc.size(); ++k)
	{
		// from : Wegner, Peter (1960), "A technique for counting ones in a
		// binary computer", Communications of the ACM 3 (5): 322,
		// doi:10.1145/367236.367286
		uint8_t x_or = t_desc[k] ^ o_desc[k];
		uint8_t count;
		for (count = 0; x_or; count++)  // ...
			x_or &= x_or - 1;  // ...
		distance += count;
	}

	return distance;
}

// # added by Raghavender Sahdev
// --------------------------------------------------
// descriptorBLDDistanceTo
// --------------------------------------------------
float CFeature::descriptorBLDDistanceTo(
	const CFeature& oFeature, bool normalize_distances) const
{
	ASSERT_(descriptors.BLD->size() == oFeature.descriptors.BLD->size());
	ASSERT_(
		descriptors.hasDescriptorBLD() &&
		oFeature.descriptors.hasDescriptorBLD());

	float dist = 0.0f;
	std::vector<unsigned char>::const_iterator itDesc1, itDesc2;
	for (itDesc1 = descriptors.BLD->begin(),
		itDesc2 = oFeature.descriptors.BLD->begin();
		 itDesc1 != descriptors.BLD->end(); itDesc1++, itDesc2++)
	{
		dist += square(*itDesc1 - *itDesc2);
	}
	if (normalize_distances) dist /= descriptors.BLD->size();
	dist = sqrt(dist);
	if (normalize_distances) dist /= 64.0f;
	return dist;
}  // end descriptorBLDDistanceTo

// --------------------------------------------------
// descriptorLATCHDistanceTo
// --------------------------------------------------
float CFeature::descriptorLATCHDistanceTo(
	const CFeature& oFeature, bool normalize_distances) const
{
	ASSERT_(descriptors.LATCH->size() == oFeature.descriptors.LATCH->size());
	ASSERT_(
		descriptors.hasDescriptorLATCH() &&
		oFeature.descriptors.hasDescriptorLATCH());

	float dist = 0.0f;
	std::vector<unsigned char>::const_iterator itDesc1, itDesc2;
	for (itDesc1 = descriptors.LATCH->begin(),
		itDesc2 = oFeature.descriptors.LATCH->begin();
		 itDesc1 != descriptors.LATCH->end(); itDesc1++, itDesc2++)
	{
		dist += square(*itDesc1 - *itDesc2);
	}
	if (normalize_distances) dist /= descriptors.LATCH->size();
	dist = sqrt(dist);
	if (normalize_distances) dist /= 64.0f;
	return dist;
}  // end descriptorLATCHDistanceTo

// --------------------------------------------------
//              saveToTextFile
// --------------------------------------------------
void CFeature::saveToTextFile(const std::string& filename, bool APPEND)
{
	MRPT_START
	//    "%% Dump of mrpt::vision::CFeatureList. Each line format is:\n"
	//    "%% ID TYPE X Y ORIENTATION SCALE TRACK_STATUS RESPONSE HAS_SIFT
	//    [SIFT] HAS_SURF [SURF] HAS_MULTI [MULTI_i] HAS_ORB [ORB]"
	//    "%% \\---------------------- feature ------------------/ \\---------
	//    descriptors -------/\n"
	//    "%% with:\n"
	//    "%%  TYPE  : The used detector: 0:KLT, 1: Harris, 2: BCD, 3: SIFT, 4:
	//    SURF, 5: Beacon, 6: FAST\n"
	//    "%%  HAS_* : 1 if a descriptor of that type is associated to the
	//    feature. \n"
	//    "%%  SIFT  : Present if HAS_SIFT=1: N DESC_0 ... DESC_N-1 \n"
	//    "%%  SURF  : Present if HAS_SURF=1: N DESC_0 ... DESC_N-1 \n"
	//	  "%%  MULTI : Present if HAS_MULTI=1: SCALE ORI N DESC_0 ... DESC_N-1"
	//	  "%%  ORB   : Present if HAS_ORB=1: VALUE
	//    "%%-------------------------------------------------------------------------------------------\n");
	CFileOutputStream f;

	if (!f.open(filename, APPEND))
		THROW_EXCEPTION(
			"[CFeature::saveToTextFile] ERROR: File could not be open for "
			"writing");

	f.printf(
		"%5u %2d %7.3f %7.3f %6.2f %2d %2d %6.3f ", (unsigned int)keypoint.ID,
		(int)get_type(), keypoint.pt.x, keypoint.pt.y, orientation,
		keypoint.octave, (int)keypoint.track_status, keypoint.response);

	f.printf("%2d ", int(descriptors.hasDescriptorSIFT() ? 1 : 0));
	if (descriptors.hasDescriptorSIFT())
	{
		f.printf("%4d ", int(descriptors.SIFT->size()));
		for (unsigned char k : *descriptors.SIFT) f.printf("%4d ", k);
	}

	f.printf("%2d ", int(descriptors.hasDescriptorSURF() ? 1 : 0));
	if (descriptors.hasDescriptorSURF())
	{
		f.printf("%4d ", int(descriptors.SURF->size()));
		for (float k : *descriptors.SURF) f.printf("%8.5f ", k);
	}

	f.printf("%2d ", int(descriptors.hasDescriptorORB() ? 1 : 0));
	if (descriptors.hasDescriptorORB())
		for (unsigned char k : *descriptors.ORB) f.printf("%d ", k);

	// # ADDED by Raghavender Sahdev
	f.printf("%2d ", int(descriptors.hasDescriptorBLD() ? 1 : 0));
	if (descriptors.hasDescriptorBLD())
	{
		f.printf("%4d ", int(descriptors.BLD->size()));
		for (unsigned char k : *descriptors.BLD) f.printf("%4d ", k);
	}

	f.printf("%2d ", int(descriptors.hasDescriptorLATCH() ? 1 : 0));
	if (descriptors.hasDescriptorLATCH())
	{
		f.printf("%4d ", int(descriptors.LATCH->size()));
		for (unsigned char k : *descriptors.LATCH) f.printf("%4d ", k);
	}

	f.printf("\n");
	f.close();

	MRPT_END
}  // end saveToTextFile

/****************************************************
			   Class CFEATURELIST
*****************************************************/

CFeatureList::~CFeatureList() = default;

// --------------------------------------------------
// saveToTextFile
// --------------------------------------------------
// FORMAT: ID type x y orientation scale [descriptorSIFT] [descriptorSURF]
// track_status response
void CFeatureList::saveToTextFile(const std::string& filename, bool APPEND)
{
	MRPT_START

	CFileOutputStream f;

	if (!f.open(filename, APPEND))
		THROW_EXCEPTION(
			"[CFeatureList::saveToTextFile] ERROR: File could not be open for "
			"writing");

	f.printf(
		"%% Dump of mrpt::vision::CFeatureList. Each line format is:\n"
		"%% ID TYPE X Y ORIENTATION SCALE TRACK_STATUS RESPONSE HAS_SIFT "
		"[SIFT] HAS_SURF [SURF]\n"
		"%% \\---------------------- feature ------------------/ \\--------- "
		"descriptors -------/\n"
		"%% with:\n"
		"%%  TYPE  : The used detector: 0:KLT, 1: Harris, 2: BCD, 3: SIFT, 4: "
		"SURF, 5: Beacon, 6: FAST\n"
		"%%  HAS_* : 1 if a descriptor of that type is associated to the "
		"feature. \n"
		"%%  SIFT  : Present if HAS_SIFT=1: N DESC_0 ... DESC_N-1 \n"
		"%%  SURF  : Present if HAS_SURF=1: N DESC_0 ... DESC_N-1 \n"
		"%%--------------------------------------------------------------------"
		"-----------------------\n");

	for (auto& feat : *this)
	{
		f.printf(
			"%5u %2d %7.3f %7.3f %6.2f %2d %2d %6.3f ",
			(unsigned int)feat.keypoint.ID, (int)feat.get_type(),
			feat.keypoint.pt.x, feat.keypoint.pt.y, feat.orientation,
			feat.keypoint.octave, (int)feat.track_status, feat.response);

		f.printf("%2d ", int(feat.descriptors.hasDescriptorSIFT() ? 1 : 0));
		if (feat.descriptors.hasDescriptorSIFT())
		{
			f.printf("%4d ", int(feat.descriptors.SIFT->size()));
			for (unsigned int k = 0; k < feat.descriptors.SIFT->size(); k++)
				f.printf("%4d ", (*feat.descriptors.SIFT)[k]);
		}

		f.printf("%2d ", int(feat.descriptors.hasDescriptorSURF() ? 1 : 0));
		if (feat.descriptors.hasDescriptorSURF())
		{
			f.printf("%4d ", int(feat.descriptors.SURF->size()));
			for (unsigned int k = 0; k < feat.descriptors.SURF->size(); k++)
				f.printf("%8.5f ", (*feat.descriptors.SURF)[k]);
		}
		// # added by Raghavender Sahdev
		f.printf("%2d ", int(feat.descriptors.hasDescriptorBLD() ? 1 : 0));
		if (feat.descriptors.hasDescriptorBLD())
		{
			f.printf("%4d ", int(feat.descriptors.BLD->size()));
			for (unsigned int k = 0; k < feat.descriptors.BLD->size(); k++)
				f.printf("%4d ", (*feat.descriptors.BLD)[k]);
		}

		f.printf("%2d ", int(feat.descriptors.hasDescriptorLATCH() ? 1 : 0));
		if (feat.descriptors.hasDescriptorLATCH())
		{
			f.printf("%4d ", int(feat.descriptors.LATCH->size()));
			for (unsigned int k = 0; k < feat.descriptors.LATCH->size(); k++)
				f.printf("%4d ", (*feat.descriptors.LATCH)[k]);
		}

		f.printf("\n");
	}  // end for

	f.close();

	MRPT_END
}  // end saveToTextFile

// --------------------------------------------------
// loadFromTextFile
// --------------------------------------------------
void CFeatureList::loadFromTextFile(const std::string& filename)
{
	MRPT_START

	mrpt::io::CTextFileLinesParser parser(filename);
	std::istringstream line;

	while (parser.getNextLine(line))
	{
		try
		{
			CFeature feat;

			int _ID;
			if (!(line >> _ID)) throw std::string("ID");
			feat.keypoint.ID = TFeatureID(_ID);

			int _type;
			if (!(line >> _type)) throw std::string("type");
			feat.type = TKeyPointMethod(_type);

			if (!(line >> feat.keypoint.pt.x >> feat.keypoint.pt.y))
				throw std::string("x,y");
			if (!(line >> feat.orientation)) throw std::string("orientation");
			if (!(line >> feat.keypoint.octave)) throw std::string("scale");

			int _track_st;
			if (!(line >> _track_st)) throw std::string("track_status");
			feat.track_status = TFeatureTrackStatus(_track_st);

			if (!(line >> feat.response)) throw std::string("response");

			int hasSIFT;
			if (!(line >> hasSIFT)) throw std::string("hasSIFT");
			if (hasSIFT)
			{
				size_t N;
				if (!(line >> N)) throw std::string("SIFT-len");
				feat.descriptors.SIFT->resize(N);
				for (size_t i = 0; i < N; i++)
				{
					int val;
					line >> val;
					// DON'T read directly SIFT[i] since it's a
					// uint8_t, interpreted as a cha
					(*feat.descriptors.SIFT)[i] = val;
				}

				if (!line) throw std::string("SIFT-data");
			}

			//# ADDED by Raghavender Sahdev
			int hasBLD;
			if (!(line >> hasBLD)) throw std::string("hasBLD");
			if (hasBLD)
			{
				size_t N;
				if (!(line >> N)) throw std::string("BLD-len");
				feat.descriptors.BLD->resize(N);
				for (size_t i = 0; i < N; i++)
				{
					int val;
					line >> val;
					(*feat.descriptors.BLD)[i] = val;
				}

				if (!line) throw std::string("BLD-data");
			}

			int hasLATCH;
			if (!(line >> hasLATCH)) throw std::string("hasLATCH");
			if (hasBLD)
			{
				size_t N;
				if (!(line >> N)) throw std::string("LATCH-len");
				feat.descriptors.LATCH->resize(N);
				for (size_t i = 0; i < N; i++)
				{
					int val;
					line >> val;
					(*feat.descriptors.LATCH)[i] = val;
				}

				if (!line) throw std::string("LATCH-data");
			}

			int hasSURF;
			if (!(line >> hasSURF)) throw std::string("hasSURF");
			if (hasSURF)
			{
				size_t N;
				if (!(line >> N)) throw std::string("SURF-len");
				feat.descriptors.SURF->resize(N);
				for (size_t i = 0; i < N; i++)
					line >> (*feat.descriptors.SURF)[i];
				if (!line) throw std::string("SURF-data");
			}

			emplace_back(std::move(feat));
		}
		catch (std::string& msg)
		{
			THROW_EXCEPTION(format(
				"%s:%d: Error parsing features text file (%s).",
				filename.c_str(), (int)parser.getCurrentLineNumber(),
				msg.c_str()));
		}
	}

	MRPT_END
}  // end loadFromTextFile

// --------------------------------------------------
// copyListFrom()
// --------------------------------------------------
void CFeatureList::copyListFrom(const CFeatureList& otherList)
{
	resize(otherList.size());
	CFeatureList::const_iterator it1;
	CFeatureList::iterator it2;
	for (it1 = otherList.begin(), it2 = begin(); it1 != otherList.end();
		 ++it1, ++it2)
	{
		*it2 = *it1;
	}
}  // end-copyListFrom

const CFeature* CFeatureList::getByID(const TFeatureID& ID) const
{
	for (const auto& f : *this)
		if (f.keypoint.ID == ID) return &f;

	return nullptr;
}

const CFeature* CFeatureList::getByID(const TFeatureID& ID, int& out_idx) const
{
	int k = 0;
	for (auto it = begin(); it != end(); ++it, ++k)
		if (it->keypoint.ID == ID)
		{
			out_idx = k;
			return &(*it);
		}
	out_idx = -1;
	return nullptr;
}

// --------------------------------------------------
// nearest(x,y)
// --------------------------------------------------
const CFeature* CFeatureList::nearest(
	const float x, const float y, double& dist_prev) const
{
	if (empty()) return nullptr;

	float closest_x, closest_y;
	float closest_sqDist;

	// Look for the closest feature using KD-tree look up:
	const size_t closest_idx =
		kdTreeClosestPoint2D(x, y, closest_x, closest_y, closest_sqDist);
	float closest_dist = std::sqrt(closest_sqDist);

	if (closest_dist <= dist_prev)
	{
		dist_prev = closest_dist;
		return &m_feats[closest_idx];
	}
	else
		return nullptr;
}  // end nearest

// --------------------------------------------------
// getMaxID()
// --------------------------------------------------
TFeatureID CFeatureList::getMaxID() const
{
	MRPT_START
	ASSERT_(!empty());
	vision::TFeatureID maxID = begin()->keypoint.ID;
	for (const auto& f : *this) mrpt::keep_max(maxID, f.keypoint.ID);
	return maxID;
	MRPT_END
}

/****************************************************
		  Class CMATCHEDFEATUREKLT
*****************************************************/

void CMatchedFeatureList::saveToTextFile(const std::string& filename)
{
	// OUTPUT FORMAT: ID_1 x_1 y_1 ID_2 x_2 y_2

	FILE* f = os::fopen(filename.c_str(), "wt");
	if (!f) return;

	for (const auto& p : *this)
	{
		os::fprintf(
			f, "%d %.3f %.3f %d %.3f %.3f\n",
			(unsigned int)(p.first.keypoint.ID), p.first.keypoint.pt.x,
			p.first.keypoint.pt.y, (unsigned int)(p.second.keypoint.ID),
			p.second.keypoint.pt.x, p.second.keypoint.pt.y);

	}  // end for
	os::fclose(f);
}

// --------------------------------------------------
//			getBothFeatureLists
// --------------------------------------------------
const CFeature* CMatchedFeatureList::getByID(
	const TFeatureID& ID, const TListIdx& idx)
{
	CMatchedFeatureList::iterator it;
	for (it = begin(); it != end(); ++it)
	{
		const auto& feat = (idx == firstList) ? it->first : it->second;
		if (feat.keypoint.ID == ID) return &feat;
	}
	return nullptr;
}

// --------------------------------------------------
// updateMaxID()
// --------------------------------------------------
void CMatchedFeatureList::updateMaxID(const TListIdx& idx)
{
	MRPT_START
	TFeatureID maxID1 = begin()->first.keypoint.ID;
	TFeatureID maxID2 = begin()->second.keypoint.ID;
	for (auto itList = begin(); itList != end(); itList++)
	{
		if (idx == firstList || idx == bothLists)
			mrpt::keep_max(maxID1, itList->first.keypoint.ID);
		if (idx == secondList || idx == bothLists)
			mrpt::keep_max(maxID2, itList->second.keypoint.ID);
	}
	if (idx == firstList || idx == bothLists) m_leftMaxID = maxID1;
	if (idx == secondList || idx == bothLists) m_rightMaxID = maxID2;
	MRPT_END
}

// --------------------------------------------------
// getMaxID()
// --------------------------------------------------
void CMatchedFeatureList::getMaxID(
	const TListIdx& idx, TFeatureID& firstListID, TFeatureID& secondListID)
{
	MRPT_START
	ASSERT_(!empty());
	if (idx == firstList || idx == bothLists)
		if (m_leftMaxID == 0) updateMaxID(firstList);
	if (idx == secondList || idx == bothLists)
		if (m_rightMaxID == 0) updateMaxID(secondList);
	firstListID = m_leftMaxID;
	secondListID = m_rightMaxID;
	MRPT_END
}  // end getMaxID()
// --------------------------------------------------
//			getBothFeatureLists
// --------------------------------------------------
void CMatchedFeatureList::getBothFeatureLists(
	CFeatureList& list1, CFeatureList& list2)
{
	MRPT_START
	list1.resize(size());
	list2.resize(size());

	unsigned int k = 0;
	for (auto it = begin(); it != end(); ++it, ++k)
	{
		list1[k] = it->first;
		list2[k] = it->second;
	}  // end for
	MRPT_END
}

// --------------------------------------------------
//			getFirstDescriptorAsMatrix
// --------------------------------------------------
bool CFeature::getFirstDescriptorAsMatrix(mrpt::math::CMatrixFloat& desc) const
{
	if (descriptors.hasDescriptorSIFT())
	{
		desc.setSize(1, descriptors.SIFT->size());
		for (size_t i = 0; i < descriptors.SIFT->size(); i++)
			desc(0, i) = (*descriptors.SIFT)[i];
		return true;
	}
	else if (descriptors.hasDescriptorBLD())
	{
		desc.setSize(1, descriptors.BLD->size());
		for (size_t i = 0; i < descriptors.BLD->size(); i++)
			desc(0, i) = (*descriptors.BLD)[i];
		return true;
	}
	else if (descriptors.hasDescriptorLATCH())
	{
		desc.setSize(1, descriptors.LATCH->size());
		for (size_t i = 0; i < descriptors.LATCH->size(); i++)
			desc(0, i) = (*descriptors.LATCH)[i];
		return true;
	}
	else if (descriptors.hasDescriptorSURF())
	{
		desc.setSize(1, descriptors.SURF->size());
		for (size_t i = 0; i < descriptors.SURF->size(); i++)
			desc(0, i) = (*descriptors.SURF)[i];
		return true;
	}
	else if (descriptors.hasDescriptorSpinImg())
	{
		const size_t nR = descriptors.SpinImg_range_rows;
		const size_t nC =
			descriptors.SpinImg->size() / descriptors.SpinImg_range_rows;
		desc.resize(nR, nC);
		auto itD = descriptors.SpinImg->begin();
		for (size_t r = 0; r < nR; r++)
			for (size_t c = 0; c < nC; c++) desc.coeffRef(r, c) = *itD++;
		return true;
	}
	else if (descriptors.hasDescriptorPolarImg())
	{
		desc = *descriptors.PolarImg;
		return true;
	}
	else if (descriptors.hasDescriptorLogPolarImg())
	{
		desc = *descriptors.LogPolarImg;
		return true;
	}
	else
		return false;
}
