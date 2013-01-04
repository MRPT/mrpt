/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                            |
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

#include <mrpt/base.h>  // Precompiled headers 


#include <mrpt/poses/CPoses3DSequence.h>
using namespace mrpt;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace mrpt::utils;

IMPLEMENTS_SERIALIZABLE( CPoses3DSequence, CSerializable ,mrpt::poses)

/*---------------------------------------------------------------
			Default constructor
  ---------------------------------------------------------------*/
CPoses3DSequence::CPoses3DSequence() :
	m_poses()
{

}

/*---------------------------------------------------------------
			Returns the poses count in the sequence:
  ---------------------------------------------------------------*/
size_t	CPoses3DSequence::posesCount()
{
	return m_poses.size();
}

/*---------------------------------------------------------------
   Implements the writing to a CStream capability of
     CSerializable objects
  ---------------------------------------------------------------*/
void  CPoses3DSequence::writeToStream(CStream &out,int *version) const
{
	if (version)
		*version = 0;
	else
	{
		uint32_t		i,n;

		// The poses:
		n = m_poses.size();
		out << n;
		for (i=0;i<n;i++) out << m_poses[i];
	}
}

/*---------------------------------------------------------------
	Implements the reading from a CStream capability of
		CSerializable objects
  ---------------------------------------------------------------*/
void  CPoses3DSequence::readFromStream(CStream &in, int version)
{
	switch(version)
	{
	case 0:
		{
			uint32_t	i,n;

			// The poses:
			in >> n;
			m_poses.resize(n);
			for (i=0;i<n;i++)	in >> m_poses[i];
		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};
}

/*---------------------------------------------------------------
Reads the stored pose at index "ind", where the first one is 0, the last "posesCount() - 1"
 ---------------------------------------------------------------*/
void  CPoses3DSequence::getPose(unsigned int ind, CPose3D &outPose)
{
	if (ind>=m_poses.size()) THROW_EXCEPTION("getPose: Index out of range!!");

	outPose = m_poses[ind];
}

/*---------------------------------------------------------------
Changes the stored pose at index "ind", where the first one is 0, the last "posesCount() - 1"
 ---------------------------------------------------------------*/
void  CPoses3DSequence::changePose(unsigned int ind, CPose3D &inPose)
{
	if (ind>=m_poses.size()) THROW_EXCEPTION("getPose: Index out of range!!");
	m_poses[ind] = inPose;
}

/*---------------------------------------------------------------
	Appends a new pose at the end of sequence. Remember that poses are relative, incremental to the last one.
 ---------------------------------------------------------------*/
void  CPoses3DSequence::appendPose(CPose3D &newPose)
{
	m_poses.push_back( newPose );
}

/*---------------------------------------------------------------
			Clears the sequence.
 ---------------------------------------------------------------*/
void  CPoses3DSequence::clear()
{
	m_poses.clear();
}

/*---------------------------------------------------------------
 ---------------------------------------------------------------*/
/** Returns the absolute pose of a robot after moving "n" poses, so for "n=0" the origin pose (0,0,0deg) is returned, for "n=1" the first pose is returned, and for "n=posesCount()", the pose
 *  of robot after moving ALL poses is returned, all of them relative to the starting pose.
 */
CPose3D  CPoses3DSequence::absolutePoseOf(unsigned int n)
{
	CPose3D		ret(0,0,0);
	unsigned int			i;

	if (n>m_poses.size()) THROW_EXCEPTION("absolutePoseOf: Index out of range!!");

	for (i=0;i<n;i++)	ret = ret + CPose3D(m_poses[i]);

	return ret;
}


/*---------------------------------------------------------------
	A shortcut for "absolutePoseOf( posesCount() )".
 ---------------------------------------------------------------*/
CPose3D  CPoses3DSequence::absolutePoseAfterAll()
{
	return absolutePoseOf( posesCount() );
}

/*---------------------------------------------------------------
	Returns the traveled distance after moving "n" poses, so for "n=0" it returns 0, for "n=1" the first traveled distance, and for "n=posesCount()", the total
	distance after ALL movements.
 ---------------------------------------------------------------*/
float  CPoses3DSequence::computeTraveledDistanceAfter(unsigned int n)
{
	unsigned int		i;
	float	dist = 0;

	if (n>m_poses.size()) THROW_EXCEPTION("computeTraveledDistanceAfter: Index out of range!!");

	for (i=0;i<n;i++)	dist+= m_poses[i].norm();
	return dist;
}

/*---------------------------------------------------------------
	Returns the traveled distance after ALL movements.
	A shortcut for "computeTraveledDistanceAfter( posesCount() )".
 ---------------------------------------------------------------*/
float  CPoses3DSequence::computeTraveledDistanceAfterAll()
{
	return computeTraveledDistanceAfter( posesCount() );
}
