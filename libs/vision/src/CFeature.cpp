/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                   http://mrpt.sourceforge.net/                            |
   |                                                                           |
   |   Copyright (C) 2005-2010  University of Malaga                           |
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

#include <mrpt/vision.h>  // Precompiled headers


#include <mrpt/utils/CFileOutputStream.h>
#include <mrpt/utils/CFileInputStream.h>
#include <mrpt/vision/CFeature.h>
#include <mrpt/math/CVectorTemplate.h>
#include <mrpt/math/utils.h>

using namespace mrpt;
using namespace mrpt::vision;
using namespace mrpt::math;
using namespace mrpt::system;
using namespace mrpt::utils;

IMPLEMENTS_SERIALIZABLE(CFeature, CSerializable, mrpt::vision)


extern CStartUpClassesRegister  mrpt_vision_class_reg;
const int dumm = mrpt_vision_class_reg.do_nothing(); // Avoid compiler removing this class in static linking


void  CFeature::writeToStream(CStream &out,int *version) const
{
	if (version)
		*version = 0;
	else
	{
		// The coordinates:
		out << x
			<< y
			<< ID
			<< patch
			<< patchSize
			<< (uint32_t)type
			<< (uint32_t)KLT_status
			<< KLT_val
			<< orientation
			<< scale
			<< IDSourceImage
			<< descriptors.SIFT
			<< descriptors.SURF
			<< descriptors.SpinImg
			<< descriptors.SpinImg_range_rows
			<< descriptors.PolarImg
			<< descriptors.LogPolarImg
			<< descriptors.polarImgsNoRotation;
	}
}

void  CFeature::readFromStream(CStream &in,int version)
{
	switch(version)
	{
	case 0:
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
				>> KLT_val
				>> orientation
				>> scale
				>> IDSourceImage
				>> descriptors.SIFT
				>> descriptors.SURF
				>> descriptors.SpinImg
				>> descriptors.SpinImg_range_rows
				>> descriptors.PolarImg
				>> descriptors.LogPolarImg
				>> descriptors.polarImgsNoRotation;

			type		= (TFeatureType)aux_type;
			KLT_status	= (TKLTFeatureStatus)aux_KLTS;
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
	KLT_status(statusKLT_IDLE), KLT_val(0.0),
	orientation(0.0), scale(0.0), IDSourceImage(0),
	descriptors()
{}

// Ctor
CFeature::TDescriptors::TDescriptors() :
	SIFT(),
	SURF(),
	SpinImg(), SpinImg_range_rows(0),
	PolarImg(0,0),
	LogPolarImg(0,0),
	polarImgsNoRotation(false)
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
	const float desc1_mean = desc1.sumAll() / static_cast<float>(width*height);
	const float desc2_mean = desc2.sumAll() / static_cast<float>(width*height);
#endif

	vector_float distances(height,0);  // Distances for each shift

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
	minDist = mrpt::math::minimum(distances,&minDistIdx);

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

/** Must fill out the data points in "data", such as the i'th point will be stored in (data[i][0],...,data[i][nDims-1]). */
void CFeatureList::kdtree_fill_point_data(ANNpointArray &data, const int nDims) const
{
	ASSERTMSG_(nDims==2, "CFeatureList only supports 2D KD-trees.")

	for (size_t i=0;i<m_feats.size();i++)
	{
		data[i][0] = m_feats[i]->x;
		data[i][1] = m_feats[i]->y;
	}
}

// --------------------------------------------------
// saveToTextFile
// --------------------------------------------------
// FORMAT: ID type x y orientation scale [descriptorSIFT] [descriptorSURF] KLT_status KLT_val
void CFeatureList::saveToTextFile( const std::string &filename, bool APPEND )
{
	MRPT_START

	CFileOutputStream	f;

	if( !f.open(filename,APPEND) )
		THROW_EXCEPTION( "[CFeatureList::saveToTextFile] ERROR: File could not be open for writing" );

	for( CFeatureList::iterator it = this->begin(); it != this->end(); ++it )
	{
		f.printf("%d %d %.3f %.3f ", (unsigned int)(*it)->ID, (int)(*it)->get_type(), (*it)->x, (*it)->y );
		f.printf("%.2f %.2f ", (*it)->orientation, (*it)->scale );

		if( (*it)->descriptors.hasDescriptorSIFT() )
			for( unsigned int k = 0; k < (*it)->descriptors.SIFT.size(); k++ )
				f.printf( "%d ", (*it)->descriptors.SIFT[k]);

		if( (*it)->descriptors.hasDescriptorSURF() )
			for( unsigned int k = 0; k < (*it)->descriptors.SURF.size(); k++ )
				f.printf( "%.4f ", (*it)->descriptors.SURF[k]);

		if( (*it)->descriptors.hasDescriptorSpinImg() )
			for( unsigned int k = 0; k < (*it)->descriptors.SpinImg.size(); k++ )
				f.printf( "%.4f ", (*it)->descriptors.SpinImg[k]);

		f.printf( "%d %.3f\n", (int)(*it)->KLT_status, (*it)->KLT_val );
	} // end for

	f.close();

	MRPT_END
} // end saveToTextFile

// --------------------------------------------------
// saveToTextFile
// --------------------------------------------------
// FORMAT: ID type x y orientation scale [descriptorSIFT] [descriptorSURF] KLT_status KLT_val
void CFeatureList::loadFromTextFile( const std::string &filename )
{
	MRPT_START

	CFileInputStream	f;

	int i = 0;		// ID counter

	f.open(filename);

	if( !f.fileOpenCorrectly() )
		THROW_EXCEPTION( format("File %s could not be opened", filename.c_str()).c_str() );

	while( !f.checkEOF() )
	{
		CFeaturePtr feat = CFeature::Create();
		string line;
		f.readLine( line );
		sscanf( line.c_str(), "%f %f\n", &(feat->x), &(feat->y) );
		feat->ID = i;
		push_back( feat );
		i++;
	}

	f.close();

	MRPT_END
} // end saveToTextFile

// --------------------------------------------------
// getByID()
// --------------------------------------------------
CFeaturePtr CFeatureList::getByID( TFeatureID ID ) const
{
	for( CFeatureList::const_iterator it = begin(); it != end(); ++it )
	{
		if( (*it)->ID == ID )
			return (*it);
	}
	return CFeaturePtr();
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

#if 0
	float dist;
	CFeatureList::const_iterator menor = end();
	for( CFeatureList::const_iterator it = begin(); it != end(); ++it ) {
		dist = sqrt( square((*it)->x - x) + square((*it)->y - y) );
		if( dist < dist_prev ){
			dist_prev = dist;
			menor = it;
		}
	}
	return menor==end() ? CFeaturePtr() : *menor;
#endif
} // end nearest


// --------------------------------------------------
// getMaxID()
// --------------------------------------------------
TFeatureID CFeatureList::getMaxID() const
{
	MRPT_START;
	ASSERT_( size() > 0 );

	vision::TFeatureID maxID	= (*begin())->ID;
	CFeatureList::const_iterator itList;

	for(itList = begin(); itList != end(); itList++)
	{
		if( (*itList)->ID > maxID )
			maxID = (*itList)->ID;
	} // end for

	return maxID;

	MRPT_END;

} // end getMaxID()

/****************************************************
		  Class CMATCHEDFEATUREKLT
*****************************************************/
// --------------------------------------------------
// CONSTRUCTOR
// --------------------------------------------------
CMatchedFeatureList::CMatchedFeatureList(){}

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
	else if (descriptors.hasDescriptorSURF())
	{
		desc = CMatrixFloat(
			descriptors.SpinImg_range_rows,
			descriptors.SpinImg.size() / descriptors.SpinImg_range_rows,
			descriptors.SpinImg);
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
