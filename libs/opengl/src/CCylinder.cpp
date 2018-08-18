/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "opengl-precomp.h"  // Precompiled header
#include <mrpt/opengl/CCylinder.h>
#include <mrpt/math/geometry.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/CSchemeArchiveBase.h>
#include "opengl_internals.h"

using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::math;

using namespace std;

IMPLEMENTS_SERIALIZABLE(CCylinder, CRenderizableDisplayList, mrpt::opengl)

/*---------------------------------------------------------------
							render
  ---------------------------------------------------------------*/
void CCylinder::render_dl() const
{
#if MRPT_HAS_OPENGL_GLUT
	glEnable(GL_BLEND);
	checkOpenGLError();
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	checkOpenGLError();
	GLUquadricObj* obj = gluNewQuadric();

	// This is required to draw cylinders of negative height.
	const float absHeight = std::abs(mHeight);
	if (mHeight < 0)
	{
		glPushMatrix();
		glTranslatef(0, 0, mHeight);
	}

	gluCylinder(obj, mBaseRadius, mTopRadius, absHeight, mSlices, mStacks);

	if (mHeight < 0) glPopMatrix();

	if (mHasBottomBase) gluDisk(obj, 0, mBaseRadius, mSlices, 1);
	if (mHasTopBase && mTopRadius > 0)
	{
		glPushMatrix();
		glTranslatef(0, 0, mHeight);
		gluDisk(obj, 0, mTopRadius, mSlices, 1);
		glPopMatrix();
	}
	gluDeleteQuadric(obj);
	glDisable(GL_BLEND);

#endif
}
void CCylinder::serializeTo(mrpt::serialization::CSchemeArchiveBase& out) const
{
	SCHEMA_SERIALIZE_DATATYPE_VERSION(1);
	out["baseRadius"] = mBaseRadius;
	out["topRadius"] = mTopRadius;
	out["height"] = mHeight;
	out["slices"] = mSlices;
	out["stacks"] = mStacks;
	out["hasBottomBase"] = mHasBottomBase;
	out["hasTopBase"] = mHasTopBase;
}
void CCylinder::serializeFrom(mrpt::serialization::CSchemeArchiveBase& in)
{
	uint8_t version;
	SCHEMA_DESERIALIZE_DATATYPE_VERSION();
	switch (version)
	{
		case 1:
		{
			mBaseRadius = static_cast<float>(in["baseRadius"]);
			mTopRadius = static_cast<float>(in["topRadius"]);
			mHeight = static_cast<float>(in["height"]);
			mSlices = static_cast<uint32_t>(in["slices"]);
			mStacks = static_cast<uint32_t>(in["stacks"]);
			mHasBottomBase = static_cast<bool>(in["hasBottomBase"]);
			mHasTopBase = static_cast<bool>(in["hasTopBase"]);
		}
		break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	}
}
uint8_t CCylinder::serializeGetVersion() const { return 0; }
void CCylinder::serializeTo(mrpt::serialization::CArchive& out) const
{
	writeToStreamRender(out);
	// version 0
	out << mBaseRadius << mTopRadius << mHeight << mSlices << mStacks
		<< mHasBottomBase << mHasTopBase;
}
void CCylinder::serializeFrom(
	mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 0:
			readFromStreamRender(in);
			in >> mBaseRadius >> mTopRadius >> mHeight >> mSlices >> mStacks >>
				mHasBottomBase >> mHasTopBase;
			break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	};
	CRenderizableDisplayList::notifyChange();
}

bool solveEqn(double a, double b, double c, double& t)
{  // Actually, the b from the quadratic equation is the DOUBLE of this. But
	// this way, operations are simpler.
	if (a < 0)
	{
		a = -a;
		b = -b;
		c = -c;
	}
	if (a >= mrpt::math::getEpsilon())
	{
		double delta = square(b) - a * c;
		if (delta == 0)
			return (t = -b / a) >= 0;
		else if (delta >= 0)
		{
			delta = sqrt(delta);
			if (-b - delta > 0)
			{
				t = (-b - delta) / a;
				return true;
			}
			else if (-b + delta > 0)
			{
				t = (-b + delta) / a;
				return true;
			}  // else return false;	Both solutions are negative
		}  // else return false;	Both solutions are complex
	}
	else if (abs(b) >= mrpt::math::getEpsilon())
	{
		t = -c / (b + b);
		return t >= 0;
	}  // else return false;	This actually isn't an equation
	return false;
}

bool CCylinder::traceRay(const mrpt::poses::CPose3D& o, double& dist) const
{
	TLine3D lin;
	mrpt::math::createFromPoseX((o - this->m_pose).asTPose(), lin);
	lin.unitarize();  // By adding this line, distance from any point of the
	// line to its base is exactly equal to the "t".
	if (abs(lin.director[2]) < getEpsilon())
	{
		if (!reachesHeight(lin.pBase.z)) return false;
		float r;
		return getRadius(static_cast<float>(lin.pBase.z), r)
				   ? solveEqn(
						 square(lin.director[0]) + square(lin.director[1]),
						 lin.director[0] * lin.pBase.x +
							 lin.director[1] * lin.pBase.y,
						 square(lin.pBase.x) + square(lin.pBase.y) - square(r),
						 dist)
				   : false;
	}
	bool fnd = false;
	double nDist, tZ0;
	if (mHasBottomBase && (tZ0 = -lin.pBase.z / lin.director[2]) > 0)
	{
		nDist = sqrt(
			square(lin.pBase.x + tZ0 * lin.director[0]) +
			square(lin.pBase.y + tZ0 * lin.director[1]));
		if (nDist <= mBaseRadius)
		{
			fnd = true;
			dist = tZ0;
		}
	}
	if (mHasTopBase)
	{
		tZ0 = (mHeight - lin.pBase.z) / lin.director[2];
		if (tZ0 > 0 && (!fnd || tZ0 < dist))
		{
			nDist = sqrt(
				square(lin.pBase.x + tZ0 * lin.director[0]) +
				square(lin.pBase.y + tZ0 * lin.director[1]));
			if (nDist <= mTopRadius)
			{
				fnd = true;
				dist = tZ0;
			}
		}
	}
	if (mBaseRadius == mTopRadius)
	{
		if (solveEqn(
				square(lin.director[0]) + square(lin.director[1]),
				lin.director[0] * lin.pBase.x + lin.director[1] * lin.pBase.y,
				square(lin.pBase.x) + square(lin.pBase.y) - square(mBaseRadius),
				nDist))
			if ((!fnd || nDist < dist) &&
				reachesHeight(lin.pBase.z + nDist * lin.director[2]))
			{
				dist = nDist;
				fnd = true;
			}
	}
	else
	{
		double slope = (mTopRadius - mBaseRadius) / mHeight;
		if (solveEqn(
				square(lin.director[0]) + square(lin.director[1]) -
					square(lin.director[2] * slope),
				lin.pBase.x * lin.director[0] + lin.pBase.y * lin.director[1] -
					(mBaseRadius + slope * lin.pBase.z) * slope *
						lin.director[2],
				square(lin.pBase.x) + square(lin.pBase.y) -
					square(mBaseRadius + slope * lin.pBase.z),
				nDist))
			if ((!fnd || nDist < dist) &&
				reachesHeight(lin.pBase.z + nDist * lin.director[2]))
			{
				dist = nDist;
				fnd = true;
			}
	}
	return fnd;
}

void CCylinder::getBoundingBox(
	mrpt::math::TPoint3D& bb_min, mrpt::math::TPoint3D& bb_max) const
{
	bb_min.x = -std::max(mBaseRadius, mTopRadius);
	bb_min.y = bb_min.x;
	bb_min.z = 0;

	bb_max.x = std::max(mBaseRadius, mTopRadius);
	bb_max.y = bb_max.x;
	bb_max.z = mHeight;

	// Convert to coordinates of my parent:
	m_pose.composePoint(bb_min, bb_min);
	m_pose.composePoint(bb_max, bb_max);
}
