/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "vision-precomp.h"   // Precompiled headers

#include <mrpt/vision/types.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/utils/stl_containers_utils.h>
#include <mrpt/system/string_utils.h>
#include <mrpt/utils/stl_containers_utils.h>
#include <iostream>
#include <iomanip>

using namespace std;
using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::vision;
using namespace mrpt::system;

// ==================== TSequenceFeatureObservations ====================
/** Saves all entries to a text file, with each line having this format: #FRAME_ID  #FEAT_ID  #PIXEL_X  #PIXEL_Y
  * \sa loadFromTextFile  */
void TSequenceFeatureObservations::saveToTextFile(const std::string &filName, bool skipFirstCommentLine ) const
{
	MRPT_START

	ofstream f(filName.c_str());
	if (!f.is_open()) THROW_EXCEPTION_CUSTOM_MSG1("Can't open file: %s",filName.c_str())

	if (!skipFirstCommentLine)
		f << "% FRAME_ID  FEAT_ID   X         Y     \n"
		     "%-------------------------------------\n";

	for (BASE::const_iterator it=BASE::begin();it!=BASE::end();++it)
			f << setw(7) << it->id_frame << setw(7) << it->id_feature << setw(13) << it->px.x << setw(11) << it->px.y << endl;

	MRPT_END
}

/** Load from a text file, in the format described in \a saveToTextFile */
void TSequenceFeatureObservations::loadFromTextFile(const std::string &filName)
{
	MRPT_START

	BASE::clear();	// Erase previous contents

	ifstream f(filName.c_str());
	if (!f.is_open()) THROW_EXCEPTION_CUSTOM_MSG1("Can't open file: %s",filName.c_str())

	unsigned int linNum = 0;
	while (!f.fail())
	{
		linNum++; // Line counter (for error reporting)
		string lin;
		std::getline(f,lin);
		lin=trim(lin);
		if (lin.empty() || lin[0]=='%') continue;

		// Read from each line as a stream:
		std::istringstream s;
		s.str(lin);

		TFeatureID		featID;
		TCameraPoseID  	camID;
		TPixelCoordf	px;
		if (!(s >> camID >> featID >> px.x >> px.y))
			THROW_EXCEPTION(format("%s:%u: Error parsing line: '%s'",filName.c_str(),linNum,lin.c_str()))

		BASE::push_back( TFeatureObservation(featID,camID,px) );
	}

	MRPT_END
}

bool TSequenceFeatureObservations::saveAsSBAFiles(
	const TLandmarkLocationsVec &pts,
	const std::string &pts_file,
	const TFramePosesVec        &cams,
	const std::string &cams_file) const
{
	MRPT_START

	std::map<TLandmarkID, std::map<TCameraPoseID, TPixelCoordf> > obs_by_point;

	for (size_t i=0;i<BASE::size();i++)
	{
		const TFeatureObservation & o = (*this)[i];
		std::map<TCameraPoseID, TPixelCoordf> & m = obs_by_point[ o.id_feature ];
		m[o.id_frame]= o.px;
	}

	// # X Y Z  nframes  frame0 x0 y0  frame1 x1 y1 ...
	ofstream f(pts_file.c_str());
	if (!f.is_open())
		return false;

	f << "# X Y Z  nframes  frame0 x0 y0  frame1 x1 y1 ...\n";
	for (std::map<TLandmarkID, std::map<TCameraPoseID, TPixelCoordf> >::const_iterator it=obs_by_point.begin();it!=obs_by_point.end();++it)
	{
		const std::map<TCameraPoseID, TPixelCoordf> & m = it->second;
		f << pts[it->first].x << " "
		  << pts[it->first].y << " "
		  << pts[it->first].z << " "
		  << m.size() << " ";
		for (std::map<TCameraPoseID, TPixelCoordf>::const_iterator itO=m.begin();itO!=m.end();++itO)
			f << itO->first << " " << itO->second.x << " "<< itO->second.y << " ";
		f << endl;
	}

	ofstream fc(cams_file.c_str());
	if (!fc.is_open())
		return false;

	for (size_t i=0;i<cams.size();i++)
	{
		const mrpt::poses::CPose3D &pos = cams[i];
		const mrpt::poses::CPose3DQuat p(pos);
		fc << p.m_quat[0] << " "
		   << p.m_quat[1] << " "
		   << p.m_quat[2] << " "
		   << p.m_quat[3] << " "
		   << p.x() << " "
		   << p.y() << " "
		   << p.z() << endl;
	}

	return true;
	MRPT_END
}


/** Remove all those features that don't have a minimum number of observations from different camera frame IDs. */
size_t TSequenceFeatureObservations::removeFewObservedFeatures(size_t minNumObservations )
{
	MRPT_START

	size_t remCount = 0;

	// 1st pass: Count total views
	map<TLandmarkID,size_t>  numViews;
	for (BASE::iterator it=BASE::begin();it!=BASE::end(); ++it )
		numViews[it->id_feature]++;

	// 2nd pass: Remove selected ones:
	for (size_t idx=0; idx<BASE::size();  )
	{
		if (numViews[(*this)[idx].id_feature]<minNumObservations)
		{
			BASE::erase( BASE::begin()+idx );
			remCount++;
		}
		else  ++idx;
	}
	return remCount;
	MRPT_END
}

/** Remove one out of \a decimate_ratio camera frame IDs from the list.
  * \sa After calling this you may want to call \a compressIDs */
void TSequenceFeatureObservations::decimateCameraFrames(const size_t decimate_ratio)
{
	ASSERT_ABOVEEQ_(decimate_ratio,1)
	if (decimate_ratio==1)
		return; // =1 -> Delete no one!

	// 1) Make sorted list of frame IDs:
	set<TCameraPoseID>  frameIDs;
	for (BASE::const_iterator it=BASE::begin();it!=BASE::end();++it)
		frameIDs.insert(it->id_frame);

	// 2) Leave in "frameIDs" just the IDs that will survive:
	for (set<TCameraPoseID>::iterator it=frameIDs.begin();it!=frameIDs.end(); )
	{
		// Leave one:
		++it;
		// Remove "decimate_ratio-1"
		for (size_t d=0;d<decimate_ratio-1 && it!=frameIDs.end();d++)
			it=mrpt::utils::erase_return_next(frameIDs,it);
	}

	// 3) Make a new list of observations with only the desired data:
	TSequenceFeatureObservations newLst;
	newLst.reserve(BASE::size() / decimate_ratio);
	for (BASE::const_iterator it=BASE::begin();it!=BASE::end();++it)
		if (frameIDs.find(it->id_frame)!=frameIDs.end())
			newLst.push_back(*it);

	// Finally, save content in "this":
	this->swap(newLst);
}

/** Rearrange frame and feature IDs such as they start at 0 and there are no gaps. */
void TSequenceFeatureObservations::compressIDs(
	std::map<TCameraPoseID,TCameraPoseID>  *old2new_camIDs,
	std::map<TLandmarkID,TLandmarkID>      *old2new_lmIDs )
{
	// 1st pass: Make list of translation IDs.
	std::map<TCameraPoseID,TCameraPoseID>  camIDs;
	std::map<TLandmarkID,TLandmarkID>      lmIDs;

	for (BASE::const_iterator it=BASE::begin();it!=BASE::end();++it)
	{
		const TFeatureID    f_ID = it->id_feature;
		const TCameraPoseID c_ID = it->id_frame;

		if (lmIDs.find(f_ID)==lmIDs.end())
		{
			TLandmarkID nextID = lmIDs.size();  // *IMPORTANT* Separate in 2 lines, otherwise lmIDs[] is called first (!?)
			lmIDs[f_ID]=nextID;
		}
		if (camIDs.find(c_ID)==camIDs.end())
		{
			TCameraPoseID nextID = camIDs.size();  // *IMPORTANT* Separate in 2 lines, otherwise camIDs[] is called first (!?)
			camIDs[c_ID]=nextID;
		}
	}

	// 2nd: Create a new list with the translated IDs:
	const size_t N = BASE::size();
	TSequenceFeatureObservations  newLst(N);
	for (size_t i=0;i<N;++i)
	{
		newLst[i].id_feature = lmIDs [ (*this)[i].id_feature ];
		newLst[i].id_frame   = camIDs[ (*this)[i].id_frame ];
		newLst[i].px         = (*this)[i].px;
	}

	// And save that new list in "this":
	this->swap(newLst);

	// Return translations, if requested:
	if (old2new_camIDs) old2new_camIDs->swap(camIDs);
	if (old2new_lmIDs)  old2new_lmIDs->swap(lmIDs);

}
