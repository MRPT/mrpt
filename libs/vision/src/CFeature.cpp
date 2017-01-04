/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "vision-precomp.h"   // Precompiled headers


#include <mrpt/utils/CTextFileLinesParser.h>
#include <mrpt/utils/CFileOutputStream.h>
#include <mrpt/utils/CFileInputStream.h>
#include <mrpt/utils/CStdOutStream.h>
#include <mrpt/utils/stl_serialization.h>
#include <mrpt/vision/CFeature.h>
#include <mrpt/vision/types.h>
#include <mrpt/utils/stl_serialization.h>
#include <mrpt/math/data_utils.h>
#include <mrpt/system/os.h>

using namespace mrpt;
using namespace mrpt::vision;
using namespace mrpt::math;
using namespace mrpt::system;
using namespace mrpt::utils;
using namespace std;

IMPLEMENTS_SERIALIZABLE(CFeature, CSerializable, mrpt::vision)

// --------------------------------------------------
//			loadFromConfigFile
// --------------------------------------------------
/**  Load all the params from a config source, in the format described in saveToConfigFile()
  */
void TMultiResDescMatchOptions::loadFromConfigFile( const mrpt::utils::CConfigFileBase &cfg, const std::string &section )
{

    useOriFilter = cfg.read_bool(section,"useOriFilter",true,false);
    oriThreshold = cfg.read_double(section,"oriThreshold",0.2,false);
    lastSeenThreshold = cfg.read_int(section,"lastSeenThreshold",10,false);
    timesSeenThreshold = cfg.read_int(section,"timesSeenThreshold",5,false);
    minFeaturesToFind = cfg.read_int(section,"minFeaturesToFind",5,false);
    minFeaturesToBeLost = cfg.read_int(section,"minFeaturesToBeLost",5,false);

    useDepthFilter = cfg.read_bool(section,"useDepthFilter",true,false);

    matchingThreshold = cfg.read_double(section,"matchingThreshold",1e4,false);
    matchingRatioThreshold = cfg.read_double(section,"matchingRatioThreshold",0.5,false);

    lowScl1 = cfg.read_int(section,"lowScl1",0,false);
    lowScl2 = cfg.read_int(section,"lowScl1",0,false);
    highScl1 = cfg.read_int(section,"highScl1",6,false);
    highScl2 = cfg.read_int(section,"highScl2",6,false);

    searchAreaSize = cfg.read_double(section,"searchAreaSize",20,false);
}

// --------------------------------------------------
//			saveToConfigFile
// --------------------------------------------------
void TMultiResDescMatchOptions::saveToConfigFile( mrpt::utils::CConfigFileBase &cfg, const std::string &section ) const
{
	if( useOriFilter )
	{
	    cfg.write(section,"useOriFilter", "true" );
	    cfg.write(section,"oriThreshold", oriThreshold );
    }
    else
	    cfg.write(section,"useOriFilter", "false" );

    if( useDepthFilter )
	    cfg.write(section,"useDepthFilter", "true" );
    else
	    cfg.write(section,"useDepthFilter", "false" );

	cfg.write(section,"matchingThreshold", matchingThreshold );
	cfg.write(section,"matchingRatioThreshold", matchingRatioThreshold );
	cfg.write(section,"lowScl1", lowScl1 );
	cfg.write(section,"lowScl2", lowScl2 );
	cfg.write(section,"highScl1", highScl1 );
	cfg.write(section,"highScl2", highScl2 );

	cfg.write(section,"searchAreaSize", searchAreaSize );
	cfg.write(section,"lastSeenThreshold", lastSeenThreshold );
	cfg.write(section,"timesSeenThreshold", timesSeenThreshold );
	cfg.write(section,"minFeaturesToFind", minFeaturesToFind );
	cfg.write(section,"minFeaturesToBeLost", minFeaturesToBeLost );
} // end-saveToConfigFile

// --------------------------------------------------
//			dumpToTextStream
// --------------------------------------------------
void TMultiResDescMatchOptions::dumpToTextStream( mrpt::utils::CStream &out) const
{
    out.printf("\n----------- [vision::TMultiResDescMatchOptions] ------------ \n");
	out.printf("Use orientation filter?:        ");
	if( useOriFilter )
	{
	    out.printf("Yes\n");
	    out.printf("· Orientation threshold:        %.1f deg\n", RAD2DEG(oriThreshold) );
    }
    else
	    out.printf("No\n");
    out.printf("Use depth filter?:              ");
    if( useDepthFilter )
	    out.printf("Yes\n");
    else
    {
	    out.printf("No\n" );
    	out.printf("Lowest scale in list1:          %d\n", lowScl1 );
        out.printf("Highest scale in list1:         %d\n", highScl1 );
        out.printf("Lowest scale in list2:          %d\n", lowScl2 );
        out.printf("Highest scale in list2:         %d\n", highScl2 );
    }
	out.printf("#frames last seen threshold:    %d\n", lastSeenThreshold );
	out.printf("#frames to be stable threshold: %d\n", timesSeenThreshold );
	out.printf("min. # features in system:      %d\n", minFeaturesToFind );
	out.printf("min. # features to be lost:     %d\n", minFeaturesToBeLost );
	out.printf("Matching threshold:             %.2f\n", matchingThreshold );
	out.printf("Matching ratio threshold:       %.2f\n", matchingRatioThreshold );
	out.printf("Size of the search window:      %d px\n", searchAreaSize );
	out.printf("-------------------------------------------------------- \n");
} // end-dumpToTextStream

// --------------------------------------------------
//			loadFromConfigFile
// --------------------------------------------------
/**  Load all the params from a config source, in the format described in saveToConfigFile()
  */
void TMultiResDescOptions::loadFromConfigFile( const mrpt::utils::CConfigFileBase &cfg, const std::string &section )
{
    basePSize = cfg.read_double(section,"basePSize", 23, false );
    comLScl = cfg.read_int(section,"comLScl", 0, false );
    comHScl = cfg.read_int(section,"comHScl", 6, false );
    sg1 = cfg.read_double(section,"sg1", 0.5, false );
    sg2 = cfg.read_double(section,"sg2", 7.5, false );
    sg3 = cfg.read_double(section,"sg3", 8.0, false );
    computeDepth = cfg.read_bool(section,"computeDepth", true, false );
    blurImage = cfg.read_bool(section,"blurImage", true, false );
    fx = cfg.read_double(section,"fx",0.0, false);
    cx = cfg.read_double(section,"cx",0.0, false);
    cy = cfg.read_double(section,"cy",0.0, false);
    baseline = cfg.read_double(section,"baseline",0.0, false);
    computeHashCoeffs = cfg.read_bool(section,"computeHashCoeffs", false, false );


    cfg.read_vector(section,"scales",vector<double>(),scales,false);
    if(scales.size() < 1)
    {
        scales.resize(7);
        scales[0] = 0.5;
        scales[1] = 0.8;
        scales[2] = 1.0;
        scales[3] = 1.2;
        scales[4] = 1.5;
        scales[5] = 1.8;
        scales[6] = 2.0;
    } // end-if
}

// --------------------------------------------------
//			saveToConfigFile
// --------------------------------------------------
void TMultiResDescOptions::saveToConfigFile( mrpt::utils::CConfigFileBase &cfg, const std::string &section ) const
{
	cfg.write(section,"basePSize", basePSize);
	cfg.write(section,"comLScl", comLScl );
	cfg.write(section,"comHScl", comHScl );
	cfg.write(section,"sg1", sg1 );
	cfg.write(section,"sg2", sg2 );
	cfg.write(section,"sg3", sg3 );

	cfg.write(section,"computeDepth", computeDepth ? "true" : "false" );
	cfg.write(section,"blurImage", blurImage ? "true" : "false" );
	cfg.write(section,"fx", fx );
	cfg.write(section,"cx", cx );
	cfg.write(section,"cy", cy );
	cfg.write(section,"baseline", baseline );
	cfg.write(section,"computeHashCoeffs", computeHashCoeffs ? "true" : "false" );

	char buf[300];
	for(unsigned int k = 0; k < scales.size(); ++k)
		mrpt::system::os::sprintf( buf, 300, "%.2f ", scales[k] );
	cfg.write(section,"scales", buf );
} // end-saveToConfigFile

// --------------------------------------------------
//			dumpToTextStream
// --------------------------------------------------
void  TMultiResDescOptions::dumpToTextStream( mrpt::utils::CStream &out) const
{
	out.printf("\n----------- [vision::TMultiResDescOptions] ------------ \n");
	out.printf("Base patch size:                %d px\n", basePSize);
	out.printf("Lowest scale to compute:        %d\n", comLScl );
	out.printf("Highest scale to compute:       %d\n", comHScl );
	out.printf("Image smoothing sigma:          %.2f px\n", sg1 );
	out.printf("Orientation histogram sigma:    %.2f\n", sg2 );
	out.printf("Descriptor histogram sigma:     %.2f\n", sg3 );
	out.printf("Compute depth:                  ");
	if( computeDepth )
	{
		out.printf("Yes\n");
		out.printf("Focal length:                   %.2f px\n", fx );
		out.printf("Principal point (cx):           %.2f px\n", cx );
		out.printf("Principal point (cy):           %.2f px\n", cy );
		out.printf("Baseline:                       %.2f m\n", baseline );
	}
	else
		out.printf("No\n");

	out.printf("Compute Hash Coeffs:            ");
	if( computeHashCoeffs )
		out.printf("Yes\n");
	else
		out.printf("No\n");

	out.printf("Blur image previously:          ");
	if( blurImage )
		out.printf("Yes\n");
	else
		out.printf("No\n");

	out.printf("Scales:                         ");
	for(unsigned int k = 0; k < scales.size(); ++k)
		out.printf( "%.2f ", scales[k] );
	out.printf("\n");
	out.printf("-------------------------------------------------------- \n");
} // end-dumpToTextStream

void CFeature::dumpToTextStream( mrpt::utils::CStream &out) const
{
	out.printf("\n----------- [vision::CFeature] ------------ \n");
	out.printf("Feature ID:                     %d\n", (int)ID);
	out.printf("Coordinates:                    (%.2f,%.2f) px\n", x, y );
	out.printf("PatchSize:                      %d\n", patchSize );
	out.printf("Type:                           ");
	switch( type )
	{
	case -1: out.printf("Not defined\n"); break;
	case 0: out.printf("KLT\n"); break;
	case 1: out.printf("Harris\n"); break;
	case 2: out.printf("BCD\n"); break;
	case 3: out.printf("SIFT\n"); break;
	case 4: out.printf("SURF\n"); break;
	case 5: out.printf("Beacon\n"); break;
	case 6: out.printf("FAST\n"); break;
	case 7: out.printf("FASTER-9\n"); break;
	case 8: out.printf("FASTER-10\n"); break;
	case 9: out.printf("FASTER-12\n"); break;
	case 10:out.printf("ORB"); break;
	}
	out.printf("Status:                         ");
	switch( track_status )
	{
	case 0: out.printf("Idle\n"); break;
	case 1: out.printf("[KLT] Out of bounds [KLT]\n"); break;
	case 5: out.printf("[KLT] Tracked\n"); break;
	case 10: out.printf("[KLT] Lost\n"); break;
	}

	out.printf("Response:                       %.2f\n", response );
	out.printf("Main orientation:               %.2f\n", orientation );
	out.printf("Main scale:                     %.2f\n", scale );
	out.printf("# frames seen:                  %d\n", nTimesSeen );
	out.printf("# frames not seen:              %d\n", nTimesNotSeen );
	out.printf("# frames since last seen:       %d\n", nTimesLastSeen );
	out.printf("Initial Depth:                  %.2f m\n", initialDepth );
	out.printf("Depth:                          %.2f m\n", depth );
	out.printf("3D point:                       (%.2f,%.2f,%.2f) m\n", p3D.x, p3D.y, p3D.z );
	out.printf("Is point feature?:              ");
	isPointFeature() ? out.printf("Yes\n") : out.printf("No\n");

	out.printf("Has SIFT descriptor?:           ");
	descriptors.hasDescriptorSIFT() ? out.printf("Yes\n") : out.printf("No\n");
	out.printf("Has SURF descriptor?:           ");
	descriptors.hasDescriptorSURF() ? out.printf("Yes\n") : out.printf("No\n");
	out.printf("Has Spin image descriptor?:     ");
	descriptors.hasDescriptorSpinImg() ? out.printf("Yes\n") : out.printf("No\n");
	out.printf("Has Polar descriptor?:          ");
	descriptors.hasDescriptorPolarImg() ? out.printf("Yes\n") : out.printf("No\n");
	out.printf("Has Log Polar descriptor?:      ");
	descriptors.hasDescriptorLogPolarImg() ? out.printf("Yes\n") : out.printf("No\n");
	out.printf("Has ORB descriptor?:			");
	descriptors.hasDescriptorORB() ? out.printf("Yes\n") : out.printf("No\n");


	out.printf("Has multiscale?:                ");
	if( !descriptors.hasDescriptorMultiSIFT() )
		out.printf("No\n");
	else
	{
		out.printf("Yes [%d]\n", (int)multiScales.size());
		for( int k = 0; k < (int)multiScales.size(); ++k )
		{
			out.printf(" · Scale %d: %.2f\n", k, multiScales[k] );
			for( int m = 0; m < (int)multiOrientations[k].size(); ++m )
			{
				out.printf(" ·· Orientation %d: %.2f\n", m, multiOrientations[k][m] );
				out.printf(" ·· [D] " );
				for( int n = 0; n < (int)descriptors.multiSIFTDescriptors[k][m].size(); ++n )
					out.printf("%d ", descriptors.multiSIFTDescriptors[k][m][n] );
				out.printf("\n");
				if( multiHashCoeffs.size() > 0 )
					out.printf(" ·· HASH coefficients %d,%d,%d\n", multiHashCoeffs[k][m][0], multiHashCoeffs[k][m][1],multiHashCoeffs[k][m][2] );
			}//end-for-m
		}//end-for-k
	} // end else
} // end dumpToTextStream

void CFeature::dumpToConsole() const
{
    CStdOutStream myOut;
    dumpToTextStream( myOut );
}

void  CFeature::writeToStream(mrpt::utils::CStream &out,int *version) const
{
	if (version)
		*version = 2;
	else
	{
		// The coordinates:
		out << x
			<< y
			<< ID
			<< patch
			<< patchSize
			<< (uint32_t)type
			<< (uint32_t)track_status
			<< response
			<< orientation
			<< scale
			<< user_flags
			<< nTimesSeen
			<< nTimesNotSeen
			<< nTimesLastSeen
			<< depth
			<< initialDepth
			<< p3D
			<< multiScales
			<< multiOrientations
			<< multiHashCoeffs
			<< descriptors.SIFT
			<< descriptors.SURF
			<< descriptors.SpinImg
			<< descriptors.SpinImg_range_rows
			<< descriptors.PolarImg
			<< descriptors.LogPolarImg
			<< descriptors.polarImgsNoRotation
			<< descriptors.multiSIFTDescriptors
			<< descriptors.ORB;
	}
}

void  CFeature::readFromStream(mrpt::utils::CStream &in,int version)
{
	switch(version)
	{
	case 0:
	case 1:
	case 2:
		{
			// The coordinates:
			uint32_t aux_type, aux_KLTS;
			in  >> x
				>> y
				>> ID
				>> patch
				>> patchSize
				>> aux_type
				>> aux_KLTS
				>> response
				>> orientation
				>> scale
				>> user_flags;
			if( version > 0 )
			{
				in  >> nTimesSeen
					>> nTimesNotSeen
					>> nTimesLastSeen
					>> depth
					>> initialDepth
					>> p3D
					>> multiScales
					>> multiOrientations
					>> multiHashCoeffs;
			}
			in  >> descriptors.SIFT
				>> descriptors.SURF
				>> descriptors.SpinImg
				>> descriptors.SpinImg_range_rows
				>> descriptors.PolarImg
				>> descriptors.LogPolarImg
				>> descriptors.polarImgsNoRotation;
			if( version > 0 )
				in  >> descriptors.multiSIFTDescriptors;
			if( version > 1 )
				in 	>> descriptors.ORB;

			type		    = (TFeatureType)aux_type;
			track_status	= (TFeatureTrackStatus)aux_KLTS;
		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};
}

/****************************************************
				Class CFEATURE
*****************************************************/
// CONSTRUCTOR
CFeature::CFeature(): x(0.0f), y(0.0f), ID(0), patchSize(21), type(featNotDefined),
	track_status(status_IDLE), response(0.0),
	orientation(0.0), scale(0.0), user_flags(0),
	nTimesSeen(1), nTimesNotSeen(0), nTimesLastSeen(0), depth( 0 ), initialDepth( 0 ), p3D(),
	multiScales(), multiOrientations(), multiHashCoeffs(), descriptors()
{}

// Ctor
CFeature::TDescriptors::TDescriptors() :
	SIFT(),
	SURF(),
	SpinImg(), SpinImg_range_rows(0),
	PolarImg(0,0),
	LogPolarImg(0,0),
	polarImgsNoRotation(false),
	ORB()
{ }

// Return false only for Blob detectors (SIFT, SURF)
bool CFeature::isPointFeature() const
{
	return type == featSIFT || type == featSURF;
}

// --------------------------------------------------
//			patchCorrelationTo
// --------------------------------------------------
float CFeature::patchCorrelationTo( const CFeature &oFeature) const
{
	MRPT_START
	ASSERT_( patch.getWidth()==oFeature.patch.getWidth() )
	ASSERT_( patch.getHeight()==oFeature.patch.getHeight() )
	ASSERT_( patch.getHeight()>0 && patch.getWidth()>0 )

	size_t x_max,y_max;
	double max_val;
	patch.cross_correlation( oFeature.patch, x_max,y_max,max_val );

	return 0.5 - 0.5*max_val;  // Value as "distance" in the range [0,1], best = 0

	MRPT_END
}


// --------------------------------------------------
//			descriptorDistanceTo
// --------------------------------------------------
float CFeature::descriptorDistanceTo(
	const CFeature &oFeature,
	TDescriptorType descriptorToUse,
	bool normalize_distances ) const
{
	MRPT_START

	// If we are not ask for a specific descriptor, select the first one found:
	if (descriptorToUse==descAny)
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
		else THROW_EXCEPTION("Feature has no descriptors and descriptorToUse=descAny")
	}

	switch (descriptorToUse)
	{
	case descSIFT:
		return descriptorSIFTDistanceTo(oFeature,normalize_distances);
	case descSURF:
		return descriptorSURFDistanceTo(oFeature,normalize_distances);
	case descSpinImages:
		return descriptorSpinImgDistanceTo(oFeature,normalize_distances);
	case descPolarImages:
		{
			float minAng;
			return descriptorPolarImgDistanceTo(oFeature,minAng,normalize_distances);
		}
	case descLogPolarImages:
		{
			float minAng;
			return descriptorLogPolarImgDistanceTo(oFeature,minAng,normalize_distances);
		}
	case descORB:
		return float(descriptorORBDistanceTo(oFeature));
	default:
		THROW_EXCEPTION_CUSTOM_MSG1("Unknown value for 'descriptorToUse'=%u",(unsigned)descriptorToUse);
	}


	MRPT_END
}

// --------------------------------------------------
// descriptorSIFTDistanceTo
// --------------------------------------------------
float CFeature::descriptorSIFTDistanceTo( const CFeature &oFeature, bool normalize_distances ) const
{
	ASSERT_( this->descriptors.SIFT.size() == oFeature.descriptors.SIFT.size() );
	ASSERT_( this->descriptors.hasDescriptorSIFT() && oFeature.descriptors.hasDescriptorSIFT() )

	float dist = 0.0f;
	std::vector<unsigned char>::const_iterator itDesc1, itDesc2;
	for( itDesc1 = this->descriptors.SIFT.begin(), itDesc2 = oFeature.descriptors.SIFT.begin();
		 itDesc1 != this->descriptors.SIFT.end();
		 itDesc1++, itDesc2++ )
	{
		dist += square(*itDesc1 - *itDesc2);
	}
	if (normalize_distances) dist/= this->descriptors.SIFT.size();
	dist = sqrt(dist);
	if (normalize_distances) dist/= 64.0f;
	return dist;
} // end descriptorSIFTDistanceTo

// --------------------------------------------------
// descriptorSURFDistanceTo
// --------------------------------------------------
float CFeature::descriptorSURFDistanceTo( const CFeature &oFeature, bool normalize_distances  ) const
{
	ASSERT_( this->descriptors.SURF.size() == oFeature.descriptors.SURF.size() );
	ASSERT_( this->descriptors.hasDescriptorSURF() && oFeature.descriptors.hasDescriptorSURF() )

	float dist = 0.0f;
	std::vector<float>::const_iterator itDesc1, itDesc2;
	for( itDesc1 = this->descriptors.SURF.begin(), itDesc2 = oFeature.descriptors.SURF.begin();
		 itDesc1 != this->descriptors.SURF.end();
		 itDesc1++, itDesc2++ )
	{
		dist += square(*itDesc1 - *itDesc2);
	}
	if (normalize_distances) dist/= this->descriptors.SURF.size();
	dist = sqrt(dist);
	if (normalize_distances) dist/= 0.20f;  // JL: Ad-hoc value! Investigate where does this come from...
	return dist;
} // end descriptorSURFDistanceTo

// --------------------------------------------------
// descriptorSpinImgDistanceTo
// --------------------------------------------------
float CFeature::descriptorSpinImgDistanceTo( const CFeature &oFeature, bool normalize_by_vector_length  ) const
{
	ASSERT_( this->descriptors.SpinImg.size() == oFeature.descriptors.SpinImg.size() );
	ASSERT_( this->descriptors.hasDescriptorSpinImg() && oFeature.descriptors.hasDescriptorSpinImg() )
	ASSERT_( !this->descriptors.SpinImg.empty() )

	float dist = 0.0f;
	std::vector<float>::const_iterator itDesc1, itDesc2;
	for( itDesc1 = this->descriptors.SpinImg.begin(), itDesc2 = oFeature.descriptors.SpinImg.begin();
		 itDesc1 != this->descriptors.SpinImg.end();
		 itDesc1++, itDesc2++ )
	{
		dist += square(*itDesc1 - *itDesc2);
	}

	if (normalize_by_vector_length)
		dist /= 0.25*descriptors.SpinImg.size();

	return sqrt(dist);
} // end descriptorSpinImgDistanceTo


// --------------------------------------------------
//        descriptorPolarImgDistanceTo
// --------------------------------------------------
float CFeature::internal_distanceBetweenPolarImages(
	const CMatrix &desc1,
	const CMatrix &desc2,
	float &minDistAngle,
	bool normalize_distances,
	bool dont_shift_angle )
{
	MRPT_START

	// Find the smallest distance:
	unsigned int	delta,i,j,ii,height = desc1.getRowCount(), width = desc1.getColCount();
	float			dist, minDist=0;

//#define LM_CORR_BIAS_MEAN

#define LM_CORR_METHOD_EUCLID
//#define LM_CORR_METHOD_MANHATTAN
//#define LM_CORR_METHOD_CORRELATION

#if defined(LM_CORR_BIAS_MEAN) || defined(LM_CORR_METHOD_CORRELATION)
	const float desc1_mean = desc1.sum() / static_cast<float>(width*height);
	const float desc2_mean = desc2.sum() / static_cast<float>(width*height);
#endif

	CVectorFloat distances(height,0);  // Distances for each shift

	for (delta=0;delta<height;delta++)
	{

#if defined(LM_CORR_METHOD_CORRELATION)
		float s11=0;
		float s22=0;
		float s12=0;
#endif
		// Compute the mean distance between desc1[t] and desc2[t-delta]:
		dist = 0;
		for (i=0;i<height;i++)
		{
			ii = (i + delta) % height;	// Shifted index
			for (j=0;j<width;j++)
			{
#ifdef LM_CORR_METHOD_EUCLID
	#ifdef LM_CORR_BIAS_MEAN
				dist+= square( desc1.get_unsafe(i,j) - desc1_mean - desc2.get_unsafe(ii,j) + desc2_mean );
	#else
				dist+= square( desc1.get_unsafe(i,j) - desc2.get_unsafe(ii,j) );
	#endif
#elif defined(LM_CORR_METHOD_MANHATTAN)
	#ifdef LM_CORR_BIAS_MEAN
				dist+= abs( desc1.get_unsafe(i,j) - desc1_mean - desc2.get_unsafe(ii,j) + desc2_mean );
	#else
				dist+= abs( desc1.get_unsafe(i,j) - desc2.get_unsafe(ii,j) );
	#endif
#elif defined(LM_CORR_METHOD_CORRELATION)
			float d1 = desc1.get_unsafe(i,j) - desc1_mean;
			float d2 = desc2.get_unsafe(ii,j) - desc2_mean;
			s11 += square(d1);
			s22 += square(d2);
			s12 += d1*d2;
#else
#error A LM_CORR_METHOD_XXX method must be selected!
#endif
			}
		}

		// Average:
		if (normalize_distances)
			dist /= static_cast<float>(width*height);

#ifdef LM_CORR_METHOD_EUCLID
		dist = sqrt(dist);
#endif

#if defined(LM_CORR_METHOD_CORRELATION)
		dist = 1 - (s12 / sqrt(s11 * s22));
#endif

		distances[delta] = dist;
		if (!delta && dont_shift_angle) { distances.resize(1); break; }
	} // end for delta

	size_t minDistIdx;
	minDist = distances.minimum(&minDistIdx);

	double dist_mean,dist_std;
	mrpt::math::meanAndStd(distances,dist_mean,dist_std);

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
	minDistAngle = minDistIdx * M_2PI / static_cast<float>(width) ;
	return minDist;

	MRPT_END
}

// --------------------------------------------------
//        descriptorPolarImgDistanceTo
// --------------------------------------------------
float CFeature::descriptorPolarImgDistanceTo(
	const CFeature &oFeature,
	float &minDistAngle,
	bool normalize_distances) const
{
	MRPT_START

	ASSERT_( size(descriptors.PolarImg,1) == size(oFeature.descriptors.PolarImg,1) )
	ASSERT_( size(descriptors.PolarImg,2) == size(oFeature.descriptors.PolarImg,2) )
	ASSERT_( this->descriptors.hasDescriptorPolarImg() && oFeature.descriptors.hasDescriptorPolarImg() )
	ASSERT_( size(descriptors.PolarImg,1)>1 && size(descriptors.PolarImg,2)>1 )

	// Call the common method for computing these distances:
	return internal_distanceBetweenPolarImages(
		descriptors.PolarImg,
		oFeature.descriptors.PolarImg,
		minDistAngle,
		normalize_distances,
		descriptors.polarImgsNoRotation );

	MRPT_END
} // end descriptorPolarImgDistanceTo

// --------------------------------------------------
//        descriptorLogPolarImgDistanceTo
// --------------------------------------------------
float CFeature::descriptorLogPolarImgDistanceTo(
	const CFeature &oFeature,
	float &minDistAngle,
	bool normalize_distances) const
{
	MRPT_START

	ASSERT_( size(descriptors.LogPolarImg,1) == size(oFeature.descriptors.LogPolarImg,1) )
	ASSERT_( size(descriptors.LogPolarImg,2) == size(oFeature.descriptors.LogPolarImg,2) )
	ASSERT_( this->descriptors.hasDescriptorLogPolarImg() && oFeature.descriptors.hasDescriptorLogPolarImg() )
	ASSERT_( size(descriptors.LogPolarImg,1)>1 && size(descriptors.LogPolarImg,2)>1 )

	// Call the common method for computing these distances:
	return internal_distanceBetweenPolarImages(
		descriptors.LogPolarImg,
		oFeature.descriptors.LogPolarImg,
		minDistAngle,
		normalize_distances,
		descriptors.polarImgsNoRotation );

	MRPT_END
} // end descriptorPolarImgDistanceTo

// --------------------------------------------------
//        descriptorORBDistanceTo
// --------------------------------------------------
uint8_t CFeature::descriptorORBDistanceTo( const CFeature &oFeature ) const
{
	ASSERT_( this->descriptors.hasDescriptorORB() && oFeature.descriptors.hasDescriptorORB() )
	ASSERT_( this->descriptors.ORB.size() == oFeature.descriptors.ORB.size() )

	const std::vector<uint8_t> & t_desc = this->descriptors.ORB;
	const std::vector<uint8_t> & o_desc = oFeature.descriptors.ORB;

	// Descriptors XOR + Hamming weight
	uint8_t distance = 0;
	for( uint8_t k = 0; k < t_desc.size(); ++k )
	{
		uint8_t x_or = t_desc[k] ^ o_desc[k];
		uint8_t count;								// from : Wegner, Peter (1960), "A technique for counting ones in a binary computer", Communications of the ACM 3 (5): 322, doi:10.1145/367236.367286
		for( count = 0; x_or; count++ )				// ...
			x_or &= x_or-1;							// ...
		distance += count;
	}

	return float(distance);
} // end-descriptorORBDistanceTo

// --------------------------------------------------
//              saveToTextFile
// --------------------------------------------------
void CFeature::saveToTextFile( const std::string &filename, bool APPEND )
{
	MRPT_START
//    "%% Dump of mrpt::vision::CFeatureList. Each line format is:\n"
//    "%% ID TYPE X Y ORIENTATION SCALE TRACK_STATUS RESPONSE HAS_SIFT [SIFT] HAS_SURF [SURF] HAS_MULTI [MULTI_i] HAS_ORB [ORB]"
//    "%% \\---------------------- feature ------------------/ \\--------- descriptors -------/\n"
//    "%% with:\n"
//    "%%  TYPE  : The used detector: 0:KLT, 1: Harris, 2: BCD, 3: SIFT, 4: SURF, 5: Beacon, 6: FAST\n"
//    "%%  HAS_* : 1 if a descriptor of that type is associated to the feature. \n"
//    "%%  SIFT  : Present if HAS_SIFT=1: N DESC_0 ... DESC_N-1 \n"
//    "%%  SURF  : Present if HAS_SURF=1: N DESC_0 ... DESC_N-1 \n"
//	  "%%  MULTI : Present if HAS_MULTI=1: SCALE ORI N DESC_0 ... DESC_N-1"
//	  "%%  ORB   : Present if HAS_ORB=1: VALUE
//    "%%-------------------------------------------------------------------------------------------\n");
	CFileOutputStream	f;

	if( !f.open(filename,APPEND) )
		THROW_EXCEPTION( "[CFeature::saveToTextFile] ERROR: File could not be open for writing" );

    f.printf("%5u %2d %7.3f %7.3f %6.2f %6.2f %2d %6.3f ",
            (unsigned int)this->ID, (int)this->get_type(), this->x, this->y,
            this->orientation, this->scale,
            (int)this->track_status, this->response );

    f.printf("%2d ", int(this->descriptors.hasDescriptorSIFT() ? 1:0) );
    if( this->descriptors.hasDescriptorSIFT() )
    {
        f.printf("%4d ", int(this->descriptors.SIFT.size()) );
        for( unsigned int k = 0; k < this->descriptors.SIFT.size(); k++ )
            f.printf( "%4d ", this->descriptors.SIFT[k]);
    }

    f.printf("%2d ", int(this->descriptors.hasDescriptorSURF() ? 1:0) );
    if( this->descriptors.hasDescriptorSURF() )
    {
        f.printf("%4d ", int(this->descriptors.SURF.size()) );
        for( unsigned int k = 0; k < this->descriptors.SURF.size(); k++ )
            f.printf( "%8.5f ", this->descriptors.SURF[k]);
    }

    f.printf("%2d ", int(this->descriptors.hasDescriptorMultiSIFT() ? 1:0) );
    if( this->descriptors.hasDescriptorMultiSIFT() )
    {
        for( int k = 0; k < int(this->multiScales.size()); ++k )
        {
            for( int m = 0; m < int(this->multiOrientations[k].size()); ++m )
            {
                f.printf("%.2f %6.2f ", this->multiScales[k], this->multiOrientations[k][m] );
                f.printf("%4d ", int(this->descriptors.multiSIFTDescriptors[k][m].size()) );
                for( unsigned int n = 0; n < this->descriptors.multiSIFTDescriptors[k][m].size(); ++n )
                    f.printf( "%4d ", this->descriptors.multiSIFTDescriptors[k][m][n]);
            }
        } // end-for
    } // end-if

	f.printf("%2d ", int(this->descriptors.hasDescriptorORB() ? 1:0) );
	if( this->descriptors.hasDescriptorORB() )
		for( size_t k = 0; k < this->descriptors.ORB.size(); ++k )
			f.printf("%d ", this->descriptors.ORB[k] );

    f.printf( "\n");
	f.close();

	MRPT_END
} // end saveToTextFile

/****************************************************
			   Class CFEATURELIST
*****************************************************/
// --------------------------------------------------
// CONSTRUCTOR
// --------------------------------------------------
CFeatureList::CFeatureList()
{} //end constructor

// --------------------------------------------------
// DESTRUCTOR
// --------------------------------------------------
CFeatureList::~CFeatureList()
{
} // end destructor

// --------------------------------------------------
// saveToTextFile
// --------------------------------------------------
// FORMAT: ID type x y orientation scale [descriptorSIFT] [descriptorSURF] track_status response
void CFeatureList::saveToTextFile( const std::string &filename, bool APPEND )
{
	MRPT_START

	CFileOutputStream	f;

	if( !f.open(filename,APPEND) )
		THROW_EXCEPTION( "[CFeatureList::saveToTextFile] ERROR: File could not be open for writing" );

	f.printf(
		"%% Dump of mrpt::vision::CFeatureList. Each line format is:\n"
		"%% ID TYPE X Y ORIENTATION SCALE TRACK_STATUS RESPONSE HAS_SIFT [SIFT] HAS_SURF [SURF]\n"
		"%% \\---------------------- feature ------------------/ \\--------- descriptors -------/\n"
		"%% with:\n"
		"%%  TYPE  : The used detector: 0:KLT, 1: Harris, 2: BCD, 3: SIFT, 4: SURF, 5: Beacon, 6: FAST\n"
		"%%  HAS_* : 1 if a descriptor of that type is associated to the feature. \n"
		"%%  SIFT  : Present if HAS_SIFT=1: N DESC_0 ... DESC_N-1 \n"
		"%%  SURF  : Present if HAS_SURF=1: N DESC_0 ... DESC_N-1 \n"
		"%%-------------------------------------------------------------------------------------------\n");

	for( CFeatureList::iterator it = this->begin(); it != this->end(); ++it )
	{
		f.printf("%5u %2d %7.3f %7.3f %6.2f %6.2f %2d %6.3f ",
				(unsigned int)(*it)->ID, (int)(*it)->get_type(), (*it)->x, (*it)->y,
				(*it)->orientation, (*it)->scale,
				(int)(*it)->track_status, (*it)->response );

		f.printf("%2d ", int((*it)->descriptors.hasDescriptorSIFT() ? 1:0) );
		if( (*it)->descriptors.hasDescriptorSIFT() )
		{
			f.printf("%4d ", int((*it)->descriptors.SIFT.size()) );
			for( unsigned int k = 0; k < (*it)->descriptors.SIFT.size(); k++ )
				f.printf( "%4d ", (*it)->descriptors.SIFT[k]);
		}

		f.printf("%2d ", int((*it)->descriptors.hasDescriptorSURF() ? 1:0) );
		if( (*it)->descriptors.hasDescriptorSURF() )
		{
			f.printf("%4d ", int((*it)->descriptors.SURF.size()) );
			for( unsigned int k = 0; k < (*it)->descriptors.SURF.size(); k++ )
				f.printf( "%8.5f ", (*it)->descriptors.SURF[k]);
		}

		f.printf( "\n");
	} // end for

	f.close();

	MRPT_END
} // end saveToTextFile

// --------------------------------------------------
// loadFromTextFile
// --------------------------------------------------
void CFeatureList::loadFromTextFile( const std::string &filename )
{
	MRPT_START

	mrpt::utils::CTextFileLinesParser  parser(filename);
	std::istringstream line;

	while( parser.getNextLine(line) )
	{
		try
		{
			CFeaturePtr feat_ptr = CFeature::Create();
			CFeature*   feat = feat_ptr.pointer(); // for faster access

 			int _ID;
			if (!(line >> _ID )) throw std::string("ID");
			feat->ID   = TFeatureID(_ID);

 			int _type;
			if (!(line >> _type)) throw std::string("type");
			feat->type = TFeatureType(_type);

			if (!(line >> feat->x >> feat->y )) throw std::string("x,y");
			if (!(line >> feat->orientation )) throw std::string("orientation");
			if (!(line >> feat->scale )) throw std::string("scale");

 			int _track_st;
			if (!(line >> _track_st )) throw std::string("track_status");
			feat->track_status = TFeatureTrackStatus(_track_st);

			if (!(line >> feat->response )) throw std::string("response");

			int hasSIFT;
			if (!(line >> hasSIFT)) throw std::string("hasSIFT");
			if (hasSIFT)
			{
				size_t N;
				if (!(line >> N)) throw std::string("SIFT-len");
				feat->descriptors.SIFT.resize(N);
				for (size_t i=0;i<N;i++)
				{
					int val;
					line >> val;
					feat->descriptors.SIFT[i] = val; // DON'T read directly SIFT[i] since it's a uint8_t, interpreted as a cha
				}

				if (!line)  throw std::string("SIFT-data");
			}

			int hasSURF;
			if (!(line >> hasSURF)) throw std::string("hasSURF");
			if (hasSURF)
			{
				size_t N;
				if (!(line >> N)) throw std::string("SURF-len");
				feat->descriptors.SURF.resize(N);
				for (size_t i=0;i<N;i++)
					line >> feat->descriptors.SURF[i];
				if (!line) throw  std::string("SURF-data");
			}

			push_back( feat_ptr );
		}
		catch(std::string &msg)
		{
			THROW_EXCEPTION(format("%s:%d: Error parsing features text file (%s).",filename.c_str(), (int)parser.getCurrentLineNumber(), msg.c_str() ))
		}
	}

	MRPT_END
} // end loadFromTextFile

// --------------------------------------------------
// copyListFrom()
// --------------------------------------------------
void CFeatureList::copyListFrom( const CFeatureList &otherList )
{
    this->resize( otherList.size() );
    CFeatureList::const_iterator it1;
    CFeatureList::iterator it2;
    for( it1 = otherList.begin(), it2 = this->begin(); it1 != otherList.end(); ++it1, ++it2 )
        (*it2).copy(*it1);
} // end-copyListFrom

// --------------------------------------------------
// getByID()
// --------------------------------------------------
CFeaturePtr CFeatureList::getByID( const TFeatureID &ID ) const
{
	for( CFeatureList::const_iterator it = begin(); it != end(); ++it )
		if( (*it)->ID == ID )
		    return (*it);

	return CFeaturePtr();
} // end getByID

// --------------------------------------------------
// getByID()
// --------------------------------------------------
CFeaturePtr CFeatureList::getByID( const TFeatureID &ID, int &out_idx ) const
{
    int k = 0;
	for( CFeatureList::const_iterator it = begin(); it != end(); ++it, ++k )
		if( (*it)->ID == ID )
		{
		    out_idx = k;
		    return (*it);
		}
    out_idx = -1;
	return CFeaturePtr();
} // end getByID

// --------------------------------------------------
// getByID()
// --------------------------------------------------
void CFeatureList::getByMultiIDs( const vector<TFeatureID> &IDs, vector<CFeaturePtr> &out, vector<int> &outIndex ) const
{
    out.clear();
    outIndex.clear();
    out.reserve( IDs.size() );
    outIndex.reserve( IDs.size() );

    for( int k = 0; k < int(IDs.size()); ++k )
    {
        int idx;
        CFeaturePtr f = getByID( IDs[k], idx );
        out.push_back( f );
        outIndex.push_back( idx );
    }
} // end getByID

// --------------------------------------------------
// nearest(x,y)
// --------------------------------------------------
CFeaturePtr CFeatureList::nearest(  const float x,  const float y, double &dist_prev ) const
{
	if (this->empty())
		return CFeaturePtr();

	float closest_x,closest_y;
	float closest_sqDist;

	// Look for the closest feature using KD-tree look up:
	const size_t closest_idx = this->kdTreeClosestPoint2D(x,y,closest_x,closest_y,closest_sqDist);
	float closest_dist = std::sqrt(closest_sqDist);

	if (closest_dist<=dist_prev)
	{
		dist_prev = closest_dist;
		return m_feats[closest_idx];
	}
	else return CFeaturePtr();
} // end nearest


// --------------------------------------------------
// getMaxID()
// --------------------------------------------------
TFeatureID CFeatureList::getMaxID() const
{
	MRPT_START
	ASSERT_( !empty() )
	vision::TFeatureID maxID	= (*begin())->ID;
	for(CFeatureList::const_iterator itList = begin(); itList != end(); itList++)
		mrpt::utils::keep_max(maxID, (*itList)->ID);
	return maxID;
	MRPT_END

} // end getMaxID()

/****************************************************
		  Class CMATCHEDFEATUREKLT
*****************************************************/
// --------------------------------------------------
// CONSTRUCTOR
// --------------------------------------------------
CMatchedFeatureList::CMatchedFeatureList() : m_leftMaxID(0), m_rightMaxID(0) {}

// --------------------------------------------------
// DESTRUCTOR
// --------------------------------------------------
CMatchedFeatureList::~CMatchedFeatureList()
{
} // end destructor

// --------------------------------------------------
// saveToTextFile
// --------------------------------------------------
void CMatchedFeatureList::saveToTextFile(const std::string &filename)
{
	// OUTPUT FORMAT: ID_1 x_1 y_1 ID_2 x_2 y_2

	FILE *f = os::fopen( filename.c_str(), "wt" );
	if(!f) return;

	CMatchedFeatureList::iterator it;
	for( it = this->begin(); it != this->end(); it++ )
	{
		os::fprintf( f, "%d %.3f %.3f %d %.3f %.3f\n",
			(unsigned int)(*it->first).ID, (*it->first).x, (*it->first).y,
			(unsigned int)(*it->second).ID, (*it->second).x, (*it->second).y);

	} // end for
	os::fclose( f );
}

// --------------------------------------------------
//			getBothFeatureLists
// --------------------------------------------------
CFeaturePtr CMatchedFeatureList::getByID( const TFeatureID & ID, const TListIdx &idx )
{
    CMatchedFeatureList::iterator it;
    for( it = begin(); it != end(); ++it )
    {
        CFeaturePtr feat = (idx == firstList) ? it->first : it->second;
        if( feat->ID == ID )
            return feat;
    }
    return CFeaturePtr();
}

// --------------------------------------------------
// updateMaxID()
// --------------------------------------------------
void CMatchedFeatureList::updateMaxID( const TListIdx &idx )
{
    MRPT_START
    TFeatureID maxID1 = begin()->first->ID;
    TFeatureID maxID2 = begin()->second->ID;
    for(CMatchedFeatureList::const_iterator itList = begin(); itList != end(); itList++)
    {
        if( idx == firstList || idx == bothLists )
            mrpt::utils::keep_max(maxID1, itList->first->ID);
        if( idx == secondList || idx == bothLists )
            mrpt::utils::keep_max(maxID2, itList->second->ID);
    }
    if( idx == firstList || idx == bothLists )
        m_leftMaxID = maxID1;
    if( idx == secondList || idx == bothLists )
        m_rightMaxID = maxID2;
    MRPT_END
}

// --------------------------------------------------
// getMaxID()
// --------------------------------------------------
void CMatchedFeatureList::getMaxID( const TListIdx &idx, TFeatureID & firstListID, TFeatureID & secondListID )
{
	MRPT_START
	ASSERT_( !empty() );
	if( idx == firstList || idx == bothLists )
		if( m_leftMaxID == 0 )
			updateMaxID( firstList );
	if( idx == secondList || idx == bothLists )
		if( m_rightMaxID == 0 )
			updateMaxID( secondList );
	firstListID = m_leftMaxID;
	secondListID = m_rightMaxID;
	MRPT_END
} // end getMaxID()
// --------------------------------------------------
//			getBothFeatureLists
// --------------------------------------------------
void CMatchedFeatureList::getBothFeatureLists( CFeatureList &list1, CFeatureList &list2 )
{
    MRPT_START
    list1.resize( this->size() );
    list2.resize( this->size() );

    unsigned int k = 0;
    for( CMatchedFeatureList::iterator it = this->begin(); it != this->end(); ++it, ++k )
    {
        list1[k] = it->first;
        list2[k] = it->second;
    } // end for
    MRPT_END
}

// --------------------------------------------------
//			getFirstDescriptorAsMatrix
// --------------------------------------------------
bool CFeature::getFirstDescriptorAsMatrix(mrpt::math::CMatrixFloat &desc) const
{
	if (descriptors.hasDescriptorSIFT())
	{
		desc.setSize(1,descriptors.SIFT.size());
		for (size_t i=0;i<descriptors.SIFT.size();i++)
			desc(0,i)=descriptors.SIFT[i];
		return true;
	}
	else if (descriptors.hasDescriptorSURF())
	{
		desc.setSize(1,descriptors.SURF.size());
		for (size_t i=0;i<descriptors.SURF.size();i++)
			desc(0,i)=descriptors.SURF[i];
		return true;
	}
	else if (descriptors.hasDescriptorSpinImg())
	{
		const size_t nR = descriptors.SpinImg_range_rows;
		const size_t nC = descriptors.SpinImg.size() / descriptors.SpinImg_range_rows;
		desc.resize(nR,nC);
		std::vector<float>::const_iterator itD = descriptors.SpinImg.begin();
		for (size_t r=0;r<nR;r++)
			for (size_t c=0;c<nC;c++)
				desc.coeffRef(r,c) = *itD++;
		return true;
	}
	else if (descriptors.hasDescriptorPolarImg())
	{
		desc = descriptors.PolarImg;
		return true;
	}
	else if (descriptors.hasDescriptorLogPolarImg())
	{
		desc = descriptors.LogPolarImg;
		return true;
	}
	else return false;
}
