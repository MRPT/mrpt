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
		for (TFeatureObservations::const_iterator itF=it->second.begin();itF!=it->second.end();++itF)
			f << setw(7) << itF->first << setw(7) << it->first << setw(13) << itF->second.x << setw(11) << itF->second.y << endl;

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

		(*this)[featID][camID] = px;
	}


	MRPT_END
}

/** Remove all those features that don't have a minimum number of observations from different camera frame IDs. */
size_t TSequenceFeatureObservations::removeFewObservedFeatures(size_t minNumObservations )
{
	MRPT_START

	size_t remCount = 0;

	for (BASE::iterator it=BASE::begin();it!=BASE::end();  )
	{
		if (it->second.size()<minNumObservations)
		{
			it = mrpt::utils::erase_return_next(*this, it);
			remCount++;
		}
		else  ++it;
	}
	return remCount;
	MRPT_END
}
