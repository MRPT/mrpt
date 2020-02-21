/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "opengl-precomp.h"  // Precompiled header

#include <mrpt/opengl/CEllipsoid2D.h>
#include <mrpt/opengl/CEllipsoid3D.h>
#include <mrpt/opengl/CPointCloud.h>
#include <mrpt/opengl/CSetOfLines.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/opengl/stock_objects.h>

#include <mrpt/poses/CPointPDFParticles.h>
#include <mrpt/poses/CPointPDFSOG.h>
#include <mrpt/poses/CPose3DPDFParticles.h>
#include <mrpt/poses/CPose3DPDFSOG.h>
#include <mrpt/poses/CPose3DQuatPDFGaussian.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/poses/CPosePDFParticles.h>
#include <mrpt/poses/CPosePDFSOG.h>

using namespace mrpt;
using namespace mrpt::math;
using namespace mrpt::opengl;
using namespace mrpt::poses;

const double POSE_TAIL_LENGTH = 0.1;
const double POSE_TAIL_WIDTH = 3.0;
const float POSE_POINT_SIZE = 4.0f;
const float POSE_AXIS_SCALE = 0.1f;

#define POSE_COLOR 0, 0, 1
#define POINT_COLOR 1, 0, 0

/** Returns a representation of a the PDF - this is just an auxiliary function,
 * it's more natural to call
 *    mrpt::poses::CPosePDF::getAs3DObject     */
CSetOfObjects::Ptr CSetOfObjects::posePDF2opengl(const CPosePDF& o)
{
	auto outObj = CSetOfObjects::Create();

	if (IS_CLASS(o, CPosePDFSOG))
	{
		const auto* p = dynamic_cast<const CPosePDFSOG*>(&o);
		ASSERT_(p != nullptr);

		opengl::CSetOfLines::Ptr lins = std::make_shared<opengl::CSetOfLines>();
		lins->setColor(0.0f, 0.0f, 1.0f, 0.6f);
		lins->setLineWidth(POSE_TAIL_WIDTH);

		for (const auto& it : *p)
		{
			auto ellip = mrpt::opengl::CEllipsoid2D::Create();

			ellip->setPose(CPose3D(it.mean.x(), it.mean.y(), 0));
			ellip->setCovMatrix(it.cov.blockCopy<2, 2>());
			ellip->setColor(POSE_COLOR, 0.6f);
			ellip->setQuantiles(3);
			ellip->enableDrawSolid3D(false);

			outObj->insert(ellip);

			lins->appendLine(
				it.mean.x(), it.mean.y(), 0,
				it.mean.x() + POSE_TAIL_LENGTH * cos(it.mean.phi()),
				it.mean.y() + POSE_TAIL_LENGTH * sin(it.mean.phi()), 0);
		}
		outObj->insert(lins);
	}
	else if (IS_CLASS(o, CPosePDFGaussian))
	{
		const auto* p = dynamic_cast<const CPosePDFGaussian*>(&o);
		ASSERT_(p != nullptr);

		opengl::CSetOfLines::Ptr lins = std::make_shared<opengl::CSetOfLines>();
		lins->setColor(POSE_COLOR, 0.6f);
		lins->setLineWidth(POSE_TAIL_WIDTH);

		auto ellip = mrpt::opengl::CEllipsoid2D::Create();

		ellip->setPose(CPose3D(p->mean.x(), p->mean.y(), 0));
		ellip->setCovMatrix(p->cov.blockCopy<2, 2>());
		ellip->setColor(POSE_COLOR, 0.6f);

		ellip->setQuantiles(3);
		ellip->enableDrawSolid3D(false);

		outObj->insert(ellip);

		lins->appendLine(
			p->mean.x(), p->mean.y(), 0,
			p->mean.x() + POSE_TAIL_LENGTH * cos(p->mean.phi()),
			p->mean.y() + POSE_TAIL_LENGTH * sin(p->mean.phi()), 0);

		outObj->insert(lins);
	}
	else if (IS_CLASS(o, CPosePDFParticles))
	{
		const auto* p = dynamic_cast<const CPosePDFParticles*>(&o);
		ASSERT_(p != nullptr);

		auto pnts = opengl::CPointCloud::Create();
		pnts->setColor(POSE_COLOR, 0.6f);
		pnts->setPointSize(POSE_POINT_SIZE);

		auto lins = opengl::CSetOfLines::Create();
		lins->setColor(POSE_COLOR, 0.6f);
		lins->setLineWidth(POSE_TAIL_WIDTH);

		for (size_t i = 0; i < p->size(); ++i)
		{
			const auto po = p->m_particles[i].d;
			pnts->insertPoint(d2f(po.x), d2f(po.y), 0);
			lins->appendLine(
				po.x, po.y, 0, po.x + POSE_TAIL_LENGTH * cos(po.phi),
				po.y + POSE_TAIL_LENGTH * sin(po.phi), 0);
		}
		outObj->insert(pnts);
		outObj->insert(lins);
	}

	return outObj;
}

/** Returns a representation of a the PDF - this is just an auxiliary function,
 * it's more natural to call
 *    mrpt::poses::CPointPDF::getAs3DObject     */
CSetOfObjects::Ptr CSetOfObjects::posePDF2opengl(const CPointPDF& o)
{
	auto outObj = std::make_shared<CSetOfObjects>();

	if (IS_CLASS(o, CPointPDFSOG))
	{
		const auto* p = dynamic_cast<const CPointPDFSOG*>(&o);
		ASSERT_(p != nullptr);

		// For each gaussian node
		for (const auto& it : *p)
		{
			mrpt::opengl::CRenderizable::Ptr obj;
			if (it.val.cov(2, 2) == 0)
			{
				auto ellip = mrpt::opengl::CEllipsoid2D::Create();
				ellip->setCovMatrix(it.val.cov.blockCopy<2, 2>());
				ellip->setQuantiles(3);
				ellip->enableDrawSolid3D(false);
				obj = ellip;
			}
			else
			{
				auto ellip = mrpt::opengl::CEllipsoid3D::Create();
				ellip->setCovMatrix(it.val.cov);
				ellip->setQuantiles(3);
				ellip->enableDrawSolid3D(false);
				obj = ellip;
			}
			obj->setPose(it.val.mean);
			obj->setColor(POINT_COLOR);

			outObj->insert(obj);
		}  // end for each gaussian node
	}
	else if (IS_CLASS(o, CPointPDFGaussian))
	{
		const auto* p = dynamic_cast<const CPointPDFGaussian*>(&o);
		ASSERT_(p != nullptr);

		mrpt::opengl::CRenderizable::Ptr obj;
		if (p->cov(2, 2) == 0)
		{
			auto ellip = mrpt::opengl::CEllipsoid2D::Create();
			ellip->setCovMatrix(p->cov.blockCopy<2, 2>());
			ellip->setQuantiles(3);
			ellip->enableDrawSolid3D(false);
			obj = ellip;
		}
		else
		{
			auto ellip = mrpt::opengl::CEllipsoid3D::Create();
			ellip->setCovMatrix(p->cov);
			ellip->setQuantiles(3);
			ellip->enableDrawSolid3D(false);
			obj = ellip;
		}
		obj->setLocation(p->mean.x(), p->mean.y(), p->mean.z());
		obj->setColor(POINT_COLOR);
		outObj->insert(obj);
	}
	else if (IS_CLASS(o, CPointPDFParticles))
	{
		const auto* p = dynamic_cast<const CPointPDFParticles*>(&o);
		ASSERT_(p != nullptr);

		mrpt::opengl::CPointCloud::Ptr obj =
			mrpt::opengl::CPointCloud::Create();
		const size_t N = p->size();

		obj->resize(N);
		obj->setColor(POINT_COLOR);
		for (size_t i = 0; i < N; i++)
			obj->setPoint(
				i, p->m_particles[i].d->x, p->m_particles[i].d->y,
				p->m_particles[i].d->z);
		outObj->insert(obj);
	}

	return outObj;
}

/** Returns a representation of a the PDF - this is just an auxiliary function,
 * it's more natural to call
 *    mrpt::poses::CPose3DPDF::getAs3DObject     */
CSetOfObjects::Ptr CSetOfObjects::posePDF2opengl(const CPose3DPDF& o)
{
	CSetOfObjects::Ptr outObj = std::make_shared<CSetOfObjects>();

	if (IS_CLASS(o, CPose3DPDFSOG))
	{
		const auto* p = dynamic_cast<const CPose3DPDFSOG*>(&o);
		ASSERT_(p != nullptr);

		// For each gaussian node
		for (const auto& it : *p)
		{
			opengl::CEllipsoid3D::Ptr obj =
				std::make_shared<opengl::CEllipsoid3D>();

			obj->setPose(it.val.mean);
			obj->setCovMatrix(it.val.cov.blockCopy<3, 3>());

			obj->setQuantiles(3);
			obj->enableDrawSolid3D(false);
			obj->setColor(POSE_COLOR);

			outObj->insert(obj);

			opengl::CSetOfObjects::Ptr axes =
				opengl::stock_objects::CornerXYZ();
			axes->setPose(it.val.mean);
			axes->setScale(POSE_AXIS_SCALE);
			outObj->insert(axes);
		}  // end for each gaussian node
	}
	else if (IS_CLASS(o, CPose3DPDFGaussian))
	{
		const auto* p = dynamic_cast<const CPose3DPDFGaussian*>(&o);
		ASSERT_(p != nullptr);

		auto obj = std::make_shared<opengl::CEllipsoid3D>();

		obj->setPose(p->mean);
		obj->setCovMatrix(p->cov.blockCopy<3, 3>());

		obj->setQuantiles(3);
		obj->enableDrawSolid3D(false);
		obj->setColor(POSE_COLOR);

		outObj->insert(obj);

		opengl::CSetOfObjects::Ptr axes = opengl::stock_objects::CornerXYZ();
		axes->setPose(p->mean);
		axes->setScale(POSE_AXIS_SCALE);
		outObj->insert(axes);
	}
	else if (IS_CLASS(o, CPose3DPDFParticles))
	{
		const auto* p = dynamic_cast<const CPose3DPDFParticles*>(&o);
		ASSERT_(p != nullptr);

		for (size_t i = 0; i < p->size(); i++)
		{
			opengl::CSetOfObjects::Ptr axes =
				opengl::stock_objects::CornerXYZSimple(POSE_AXIS_SCALE);
			axes->setPose(p->m_particles[i].d);
			outObj->insert(axes);
		}
	}

	return outObj;
}

/** Returns a representation of a the PDF - this is just an auxiliary function,
 * it's more natural to call
 *    mrpt::poses::CPose3DQuatPDF::getAs3DObject     */
CSetOfObjects::Ptr CSetOfObjects::posePDF2opengl(const CPose3DQuatPDF& o)
{
	CSetOfObjects::Ptr outObj = std::make_shared<CSetOfObjects>();

	if (IS_CLASS(o, CPose3DQuatPDFGaussian))
	{
		const auto* p = dynamic_cast<const CPose3DQuatPDFGaussian*>(&o);
		ASSERT_(p != nullptr);

		auto obj = mrpt::opengl::CEllipsoid3D::Create();

		obj->setPose(CPose3D(p->mean));
		obj->setCovMatrix(p->cov.blockCopy<3, 3>());

		obj->setQuantiles(3);
		obj->enableDrawSolid3D(false);
		obj->setColor(POSE_COLOR);

		outObj->insert(obj);

		opengl::CSetOfObjects::Ptr axes = opengl::stock_objects::CornerXYZ();
		axes->setPose(CPose3D(p->mean));
		axes->setScale(POSE_AXIS_SCALE);
		outObj->insert(axes);
	}

	return outObj;
}
