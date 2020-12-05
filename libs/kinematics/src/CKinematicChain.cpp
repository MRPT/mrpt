/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "kinematics-precomp.h"  // Precompiled headers

#include <mrpt/kinematics/CKinematicChain.h>
#include <mrpt/opengl/CCylinder.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/opengl/stock_objects.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/stl_serialization.h>

using namespace mrpt;
using namespace mrpt::math;
using namespace mrpt::poses;
using namespace mrpt::kinematics;
using namespace std;

IMPLEMENTS_SERIALIZABLE(CKinematicChain, CSerializable, mrpt::kinematics)

/** Appends a new link to the robotic arm, with the given Denavit-Hartenberg
 * parameters (see TKinematicLink for further details) */
void CKinematicChain::addLink(
	double theta, double d, double a, double alpha, bool is_prismatic)
{
	m_links.emplace_back(theta, d, a, alpha, is_prismatic);
}

/** Removes one link from the kinematic chain (0<=idx<N) */
void CKinematicChain::removeLink(const size_t idx)
{
	ASSERT_LT_(idx, m_links.size());
	m_links.erase(m_links.begin() + idx);
}

const TKinematicLink& CKinematicChain::getLink(const size_t idx) const
{
	ASSERT_LT_(idx, m_links.size());
	return m_links[idx];
}

TKinematicLink& CKinematicChain::getLinkRef(const size_t idx)
{
	ASSERT_LT_(idx, m_links.size());
	return m_links[idx];
}

void CKinematicChain::setOriginPose(const mrpt::poses::CPose3D& new_pose)
{
	m_origin = new_pose;
}

const mrpt::poses::CPose3D& CKinematicChain::getOriginPose() const
{
	return m_origin;
}

uint8_t CKinematicChain::serializeGetVersion() const { return 1; }
void CKinematicChain::serializeTo(mrpt::serialization::CArchive& out) const
{
	out << m_links << m_origin;
}

void CKinematicChain::serializeFrom(
	mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 0:
		case 1:
		{
			in >> m_links;
			if (version >= 1)
			{
				in >> m_origin;
			}
			else
				m_origin = mrpt::poses::CPose3D();
		}
		break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
	};
}

/** Go thru all the links of the chain and compute the global pose of each link.
 * The "ground" link pose "pose0" defaults to the origin of coordinates,
 * but anything else can be passed as the optional argument. */
void CKinematicChain::recomputeAllPoses(
	std::vector<mrpt::poses::CPose3D>& poses,
	[[maybe_unused]] const mrpt::poses::CPose3D& pose0) const
{
	const size_t N = m_links.size();

	poses.resize(N + 1);

	CPose3D p = m_origin;  // Cummulative pose

	poses[0] = p;

	for (size_t i = 0; i < N; i++)
	{
		// Build the 3D pose change of the i'th link:
		const double th = m_links[i].theta;
		const double alpha = m_links[i].alpha;
		const double d = m_links[i].d;
		const double a = m_links[i].a;

		const double t_vals[3] = {a * cos(th), a * sin(th), d};
		const double r_vals[3 * 3] = {cos(th),
									  -sin(th) * cos(alpha),
									  sin(th) * sin(alpha),
									  sin(th),
									  cos(th) * cos(alpha),
									  -cos(th) * sin(alpha),
									  0,
									  sin(alpha),
									  cos(alpha)};

		const CMatrixDouble33 R(r_vals);
		const CVectorFixedDouble<3> t(t_vals);

		CPose3D link(R, t);

		p.composeFrom(p, link);

		poses[i + 1] = p;
	}
}

const float R = 0.01f;

void addBar_D(mrpt::opengl::CSetOfObjects::Ptr& objs, const double d)
{
	mrpt::opengl::CCylinder::Ptr gl_cyl =
		mrpt::opengl::CCylinder::Create(R, R, d);
	gl_cyl->setColor_u8(mrpt::img::TColor(0x00, 0x00, 0xff));
	gl_cyl->setName("cyl.d");

	objs->insert(gl_cyl);
}

void addBar_A(mrpt::opengl::CSetOfObjects::Ptr& objs, const double a)
{
	mrpt::opengl::CCylinder::Ptr gl_cyl2 =
		mrpt::opengl::CCylinder::Create(R, R, -a);
	gl_cyl2->setColor_u8(mrpt::img::TColor(0xff, 0x00, 0x00));
	gl_cyl2->setPose(mrpt::poses::CPose3D(0, 0, 0, 0, 90.0_deg, 0));
	gl_cyl2->setName("cyl.a");

	objs->insert(gl_cyl2);
}

void CKinematicChain::getAs3DObject(
	mrpt::opengl::CSetOfObjects::Ptr& obj,
	std::vector<mrpt::poses::CPose3D>* out_all_poses) const
{
	ASSERT_(obj);
	const size_t N = m_links.size();

	// Recompute current poses:
	std::vector<mrpt::poses::CPose3D> all_poses;
	recomputeAllPoses(all_poses);

	m_last_gl_objects.resize(N + 1);

	// Ground [0] and Links [1-N]:
	for (size_t i = 0; i <= N; i++)
	{
		mrpt::opengl::CSetOfObjects::Ptr gl_corner =
			mrpt::opengl::stock_objects::CornerXYZSimple(0.1f, 3.0f);
		gl_corner->setPose(all_poses[i]);

		gl_corner->setName(mrpt::format("%u", static_cast<unsigned int>(i)));
		gl_corner->enableShowName();

		if (i < N) addBar_D(gl_corner, m_links[i].d);
		if (i > 0) addBar_A(gl_corner, m_links[i - 1].a);

		obj->insert(gl_corner);
		m_last_gl_objects[i] = gl_corner;
	}

	if (out_all_poses) out_all_poses->swap(all_poses);
}

void CKinematicChain::update3DObject(
	std::vector<mrpt::poses::CPose3D>* out_all_poses) const
{
	ASSERTMSG_(
		(m_links.size() + 1) == m_last_gl_objects.size(),
		"The kinematic chain has changed since the last call to "
		"getAs3DObject()");

	const size_t N = m_links.size();

	// Recompute current poses:
	std::vector<mrpt::poses::CPose3D> all_poses;
	recomputeAllPoses(all_poses);

	for (size_t i = 0; i <= N; i++)
	{
		mrpt::opengl::CSetOfObjects::Ptr gl_objs =
			std::dynamic_pointer_cast<mrpt::opengl::CSetOfObjects>(
				m_last_gl_objects[i]);
		gl_objs->setPose(all_poses[i]);

		if (i < N)
		{
			mrpt::opengl::CCylinder::Ptr glCyl =
				std::dynamic_pointer_cast<mrpt::opengl::CCylinder>(
					gl_objs->getByName("cyl.d"));
			const float d = mrpt::d2f(m_links[i].d);
			glCyl->setHeight(d);
		}

		if (i > 0)
		{
			mrpt::opengl::CCylinder::Ptr glCyl2 =
				std::dynamic_pointer_cast<mrpt::opengl::CCylinder>(
					gl_objs->getByName("cyl.a"));
			const float a = mrpt::d2f(m_links[i - 1].a);
			glCyl2->setHeight(-a);
		}
	}

	if (out_all_poses) out_all_poses->swap(all_poses);
}

/** Erases all links and leave the robot arm empty. */
void CKinematicChain::clear()
{
	m_links.clear();
	m_last_gl_objects.clear();
}

mrpt::serialization::CArchive& mrpt::kinematics::operator>>(
	mrpt::serialization::CArchive& in, TKinematicLink& o)
{
	uint32_t version;
	in >> version;
	switch (version)
	{
		case 0:
			in >> o.theta >> o.d >> o.a >> o.alpha >> o.is_prismatic;
			break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
	}
	return in;
}
mrpt::serialization::CArchive& mrpt::kinematics::operator<<(
	mrpt::serialization::CArchive& out, const TKinematicLink& o)
{
	const uint32_t version = 0;
	out << version;
	out << o.theta << o.d << o.a << o.alpha << o.is_prismatic;
	return out;
}
