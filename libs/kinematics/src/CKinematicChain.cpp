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

#include <mrpt/kinematics.h>  // Precompiled headers

#include <mrpt/kinematics/CKinematicChain.h>
#include <mrpt/utils/CStream.h>
#include <mrpt/opengl.h>

//#include <mrpt/math/slerp.h>

using namespace mrpt;
using namespace mrpt::math;
using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace mrpt::kinematics;
using namespace std;

IMPLEMENTS_SERIALIZABLE(CKinematicChain, CSerializable, mrpt::kinematics)


/** Appends a new link to the robotic arm, with the given Denavit-Hartenberg parameters (see TKinematicLink for further details) */
void CKinematicChain::addLink(double theta, double d, double a, double alpha, bool is_prismatic)
{
	m_links.push_back( TKinematicLink(theta,d,a,alpha,is_prismatic) );
}

const TKinematicLink& CKinematicChain::getLink(const size_t idx) const
{
	ASSERT_BELOW_(idx,m_links.size())
	return m_links[idx];
}

TKinematicLink& CKinematicChain::getLinkRef(const size_t idx)
{
	ASSERT_BELOW_(idx,m_links.size())
	return m_links[idx];
}

/*---------------------------------------------------------------
   Implements the writing to a CStream capability of CSerializable objects
  ---------------------------------------------------------------*/
void  CKinematicChain::writeToStream(CStream &out,int *version) const
{
	if (version)
		*version = 0;
	else
	{
		out << m_links;
	}
}

/*---------------------------------------------------------------
	Implements the reading from a CStream capability of CSerializable objects
  ---------------------------------------------------------------*/
void  CKinematicChain::readFromStream(CStream &in,int version)
{
	switch(version)
	{
	case 0:
		{
			in >> m_links;
		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	};
}

/** Go thru all the links of the chain and compute the global pose of each link. The "ground" link pose "pose0" defaults to the origin of coordinates, 
	* but anything else can be passed as the optional argument. */
void CKinematicChain::recomputeAllPoses( mrpt::aligned_containers<mrpt::poses::CPose3D>::vector_t & poses, const mrpt::poses::CPose3D & pose0 )const
{
	const size_t N=m_links.size();

	poses.resize(N);

	CPose3D p = pose0;  // Cummulative pose

	for (size_t i=0;i<N;i++)
	{
		// Build the 3D pose change of the i'th link:
		const double th = m_links[i].theta;
		const double alpha = m_links[i].alpha;
		const double d = m_links[i].d;
		const double a = m_links[i].a;

		const double t_vals[3] = {
			a * cos(th),
			a * sin(th),
			d };
		const double r_vals[3*3] = {
			cos(th), -sin(th)*cos(alpha), sin(th)*sin(alpha),
			sin(th), cos(th)*cos(alpha), -cos(th)*sin(alpha),
			0, sin(alpha), cos(alpha) };

		const CMatrixDouble33 R(r_vals);
		const CArrayDouble<3> t(t_vals);

		CPose3D link(R,t);

		p.composeFrom(p,link);

		poses[i] = p;
	}
}

const float R = 0.01;

void addBar_D(mrpt::opengl::CSetOfObjectsPtr &objs, const double d)
{
	
	mrpt::opengl::CCylinderPtr gl_cyl = mrpt::opengl::CCylinder::Create(R,R, d );
	gl_cyl->setColor_u8( mrpt::utils::TColor(0x00,0x00,0xff) );
	gl_cyl->setName("cyl.d");

	objs->insert(gl_cyl);
}

void addBar_A(mrpt::opengl::CSetOfObjectsPtr &objs, const double a)
{
	mrpt::opengl::CCylinderPtr gl_cyl2 = mrpt::opengl::CCylinder::Create(R,R, -a );
	gl_cyl2->setColor_u8( mrpt::utils::TColor(0xff,0x00,0x00) );
	gl_cyl2->setPose( mrpt::poses::CPose3D(0,0,0, 0, DEG2RAD(90),0) );
	gl_cyl2->setName("cyl.a");

	objs->insert(gl_cyl2);
}

void CKinematicChain::getAs3DObject(mrpt::opengl::CSetOfObjectsPtr &obj) const
{
	ASSERT_(obj.present())
	const size_t N=m_links.size();

	// Recompute current poses:
	mrpt::aligned_containers<mrpt::poses::CPose3D>::vector_t all_poses;
	recomputeAllPoses( all_poses );

	m_last_gl_objects.resize(N+1);

	// Ground [0] and Links [1-N]:
	for (size_t i=0;i<=N;i++)
	{
		mrpt::opengl::CSetOfObjectsPtr gl_corner = mrpt::opengl::stock_objects::CornerXYZSimple( 0.1f, 3.0f );
		if (i>0)
			gl_corner->setPose( all_poses[i-1] );
		
		gl_corner->setName( mrpt::format("%u",static_cast<unsigned int>(i)) );
		gl_corner->enableShowName();

		if (i<N) addBar_D(gl_corner,m_links[i].d);
		if (i>0) addBar_A(gl_corner,m_links[i-1].a);

		obj->insert(gl_corner);
		m_last_gl_objects[i] = gl_corner;
	}
}

void CKinematicChain::update3DObject() const
{
	ASSERTMSG_((m_links.size()+1)==m_last_gl_objects.size(), "The kinematic chain has changed since the last call to getAs3DObject()")

	const size_t N=m_links.size();

	// Recompute current poses:
	mrpt::aligned_containers<mrpt::poses::CPose3D>::vector_t all_poses;
	recomputeAllPoses( all_poses );

	for (size_t i=0;i<=N;i++)
	{
		mrpt::opengl::CSetOfObjectsPtr gl_objs = mrpt::opengl::CSetOfObjectsPtr(m_last_gl_objects[i]);
		if (i>0)
			gl_objs->setPose( all_poses[i-1] );
		
		if (i<N)
		{
			mrpt::opengl::CCylinderPtr glCyl = mrpt::opengl::CCylinderPtr( gl_objs->getByName("cyl.d") );
			const double d = m_links[i].d;
			glCyl->setHeight( d );
		}

		if (i>0)
		{
			mrpt::opengl::CCylinderPtr glCyl2 = mrpt::opengl::CCylinderPtr( gl_objs->getByName("cyl.a") );
			const double a = m_links[i-1].a;
			//glCyl2->setPose( mrpt::poses::CPose3D(0,0,d, 0, DEG2RAD(90),0) );
			glCyl2->setHeight( -a );
		}
	}

}

/** Erases all links and leave the robot arm empty. */
void CKinematicChain::clear()
{
	m_links.clear();
	m_last_gl_objects.clear();
}


mrpt::utils::CStream & mrpt::kinematics::operator>>(mrpt::utils::CStream &in,TKinematicLink &o)
{
	in >> o.theta >> o.d >> o.a >> o.alpha >> o.is_prismatic;
	return in;
}
mrpt::utils::CStream & mrpt::kinematics::operator<<(mrpt::utils::CStream &out,const TKinematicLink &o)
{
	out << o.theta << o.d << o.a << o.alpha << o.is_prismatic;
	return out;
}
