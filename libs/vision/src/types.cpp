/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                   http://mrpt.sourceforge.net/                            |
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

#include <mrpt/vision.h>  // Precompiled headers

#include <mrpt/vision/types.h>
#include <mrpt/system/string_utils.h>

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
	for (BASE::iterator it=BASE::begin();it!=BASE::end();  )
	{
		if (numViews[it->id_feature]<minNumObservations)
		{
			it = mrpt::utils::erase_return_next(*this, it);
			remCount++;
		}
		else  ++it;
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
