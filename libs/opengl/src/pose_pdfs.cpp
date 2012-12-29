/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                                 |
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

#include <mrpt/opengl.h>  // Precompiled header

#include <mrpt/opengl/CEllipsoid.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/opengl/CSetOfLines.h>
#include <mrpt/opengl/CPointCloud.h>
#include <mrpt/opengl/stock_objects.h>

#include <mrpt/poses.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::opengl;
using namespace mrpt::poses;


const double POSE_TAIL_LENGTH = 0.1;
const double POSE_TAIL_WIDTH  = 3.0;
const double POSE_POINT_SIZE  = 4.0;
const double POSE_AXIS_SCALE  = 0.1;

#define POSE_COLOR		0,0,1
#define POINT_COLOR		1,0,0


/** Returns a representation of a the PDF - this is just an auxiliary function, it's more natural to call
	*    mrpt::poses::CPosePDF::getAs3DObject     */
CSetOfObjectsPtr CSetOfObjects::posePDF2opengl(const CPosePDF &o)
{
	CSetOfObjectsPtr outObj = CSetOfObjects::Create();

	if (IS_CLASS(&o,CPosePDFSOG))
	{
		const CPosePDFSOG *p = static_cast<const CPosePDFSOG*>(&o);

		opengl::CSetOfLinesPtr lins = opengl::CSetOfLines::Create();
		lins->setColor(0,0,1,0.6);
		lins->setLineWidth(POSE_TAIL_WIDTH);

		for (CPosePDFSOG::const_iterator it=p->begin();it!=p->end();++it)
		{
			opengl::CEllipsoidPtr ellip = opengl::CEllipsoid::Create();

			ellip->setPose( CPose3D((it)->mean.x(), (it)->mean.y(), 0) );
			ellip->setCovMatrix((it)->cov, 2 /* x y */ );
			ellip->setColor(POSE_COLOR,0.6);
			ellip->setQuantiles(3);
			ellip->enableDrawSolid3D(false);

			outObj->insert(ellip);

			lins->appendLine(
				(it)->mean.x(), (it)->mean.y(), 0,
				(it)->mean.x() + POSE_TAIL_LENGTH * cos((it)->mean.phi()) , (it)->mean.y() +  POSE_TAIL_LENGTH * sin((it)->mean.phi()) , 0
				);
		}
		outObj->insert(lins);
	} else
	if (IS_CLASS(&o,CPosePDFGaussian))
	{
		const CPosePDFGaussian *p = static_cast<const CPosePDFGaussian*>(&o);

		opengl::CSetOfLinesPtr lins = opengl::CSetOfLines::Create();
		lins->setColor(POSE_COLOR,0.6);
		lins->setLineWidth(POSE_TAIL_WIDTH);

		opengl::CEllipsoidPtr ellip = opengl::CEllipsoid::Create();

		ellip->setPose( CPose3D(p->mean.x(), p->mean.y(), 0) );
		ellip->setCovMatrix(p->cov, 2 /* x y */ );
		ellip->setColor(POSE_COLOR,0.6);

		ellip->setQuantiles(3);
		ellip->enableDrawSolid3D(false);

		outObj->insert(ellip);

		lins->appendLine(
			p->mean.x(), p->mean.y(), 0,
			p->mean.x() + POSE_TAIL_LENGTH * cos(p->mean.phi()) , p->mean.y() +  POSE_TAIL_LENGTH * sin(p->mean.phi()) , 0
			);

		outObj->insert(lins);
	} else
	if (IS_CLASS(&o,CPosePDFParticles))
	{
		const CPosePDFParticles *p = static_cast<const CPosePDFParticles*>(&o);

		opengl::CPointCloudPtr  pnts = opengl::CPointCloud::Create();
		pnts->setColor(POSE_COLOR,0.6);
		pnts->setPointSize(POSE_POINT_SIZE);

		opengl::CSetOfLinesPtr lins = opengl::CSetOfLines::Create();
		lins->setColor(POSE_COLOR,0.6);
		lins->setLineWidth(POSE_TAIL_WIDTH);

		for (size_t i=0;i<p->size();++i)
		{
			pnts->insertPoint(p->m_particles[i].d->x(), p->m_particles[i].d->y(), 0);
			lins->appendLine(
				p->m_particles[i].d->x(), p->m_particles[i].d->y(), 0,
				p->m_particles[i].d->x() + POSE_TAIL_LENGTH * cos(p->m_particles[i].d->phi()), p->m_particles[i].d->y() + POSE_TAIL_LENGTH * sin(p->m_particles[i].d->phi()), 0
				);
		}
		outObj->insert(pnts);
		outObj->insert(lins);
	}


	return outObj;
}

/** Returns a representation of a the PDF - this is just an auxiliary function, it's more natural to call
	*    mrpt::poses::CPointPDF::getAs3DObject     */
CSetOfObjectsPtr CSetOfObjects::posePDF2opengl(const CPointPDF &o)
{
	CSetOfObjectsPtr outObj = CSetOfObjects::Create();

	if (IS_CLASS(&o,CPointPDFSOG))
	{
		const CPointPDFSOG *p = static_cast<const CPointPDFSOG*>(&o);

		// For each gaussian node
		for (CPointPDFSOG::CListGaussianModes::const_iterator it = p->begin(); it!= p->end();it++)
		{
			opengl::CEllipsoidPtr obj = opengl::CEllipsoid::Create();

			obj->setPose( it->val.mean);
			obj->setCovMatrix(it->val.cov,  it->val.cov(2,2)==0  ?  2:3);

			obj->setQuantiles(3);
			obj->enableDrawSolid3D(false);
			obj->setColor(POINT_COLOR);

			outObj->insert( obj );
		} // end for each gaussian node
	}
	else if (IS_CLASS(&o, CPointPDFGaussian))
	{
		const CPointPDFGaussian *p = static_cast<const CPointPDFGaussian*>(&o);

		CEllipsoidPtr obj = CEllipsoid::Create();
		obj->setLocation(p->mean);
		obj->setCovMatrix(p->cov,  p->cov(2,2)==0  ?  2:3);
		obj->setColor(POINT_COLOR);
		obj->setQuantiles(3);
		obj->enableDrawSolid3D(false);
		outObj->insert( obj );
	}
	else if (IS_CLASS(&o, CPointPDFParticles))
	{
		const CPointPDFParticles *p = static_cast<const CPointPDFParticles*>(&o);

		mrpt::opengl::CPointCloudPtr obj = mrpt::opengl::CPointCloud::Create();
		const size_t N=p->size();

		obj->resize(N);
		obj->setColor(POINT_COLOR);
		for (size_t i=0;i<N;i++)
			obj->setPoint(
				i,
				p->m_particles[i].d->x,
				p->m_particles[i].d->y,
				p->m_particles[i].d->z );
		outObj->insert( obj );
	}

	return outObj;
}

/** Returns a representation of a the PDF - this is just an auxiliary function, it's more natural to call
	*    mrpt::poses::CPose3DPDF::getAs3DObject     */
CSetOfObjectsPtr CSetOfObjects::posePDF2opengl(const CPose3DPDF &o)
{
	CSetOfObjectsPtr outObj = CSetOfObjects::Create();

	if (IS_CLASS(&o,CPose3DPDFSOG))
	{
		const CPose3DPDFSOG *p = static_cast<const CPose3DPDFSOG*>(&o);

		// For each gaussian node
		for (CPose3DPDFSOG::const_iterator it = p->begin(); it!= p->end();it++)
		{
			opengl::CEllipsoidPtr obj = opengl::CEllipsoid::Create();

			obj->setPose( it->val.mean);
			obj->setCovMatrix(CMatrixDouble(it->val.cov),  it->val.cov(2,2)==0  ?  2:3);

			obj->setQuantiles(3);
			obj->enableDrawSolid3D(false);
			obj->setColor(POSE_COLOR);

			outObj->insert( obj );

			opengl::CSetOfObjectsPtr axes = opengl::stock_objects::CornerXYZ();
			axes->setPose(it->val.mean);
			axes->setScale(POSE_AXIS_SCALE);
			outObj->insert(axes);
		} // end for each gaussian node
	}
	else
	if (IS_CLASS(&o,CPose3DPDFGaussian))
	{
		const CPose3DPDFGaussian *p = static_cast<const CPose3DPDFGaussian*>(&o);

		opengl::CEllipsoidPtr obj = opengl::CEllipsoid::Create();

		obj->setPose( p->mean);
		obj->setCovMatrix(CMatrixDouble(p->cov),  p->cov(2,2)==0  ?  2:3);

		obj->setQuantiles(3);
		obj->enableDrawSolid3D(false);
		obj->setColor(POSE_COLOR);

		outObj->insert( obj );

		opengl::CSetOfObjectsPtr axes = opengl::stock_objects::CornerXYZ();
		axes->setPose(p->mean);
		axes->setScale(POSE_AXIS_SCALE);
		outObj->insert(axes);
	}
	else
	if (IS_CLASS(&o,CPose3DPDFParticles))
	{
		const CPose3DPDFParticles *p = static_cast<const CPose3DPDFParticles*>(&o);

		for (size_t i=0;i<p->size();i++)
		{
			opengl::CSetOfObjectsPtr axes = opengl::stock_objects::CornerXYZSimple(POSE_AXIS_SCALE);
			axes->setPose(*p->m_particles[i].d);
			outObj->insert(axes);
		}

	}

	return outObj;
}

/** Returns a representation of a the PDF - this is just an auxiliary function, it's more natural to call
	*    mrpt::poses::CPose3DQuatPDF::getAs3DObject     */
CSetOfObjectsPtr CSetOfObjects::posePDF2opengl(const CPose3DQuatPDF &o)
{
	CSetOfObjectsPtr outObj = CSetOfObjects::Create();

	if (IS_CLASS(&o,CPose3DQuatPDFGaussian))
	{
		const CPose3DQuatPDFGaussian *p = static_cast<const CPose3DQuatPDFGaussian*>(&o);

		opengl::CEllipsoidPtr obj = opengl::CEllipsoid::Create();

		obj->setPose( p->mean);
		obj->setCovMatrix(CMatrixDouble(p->cov),  p->cov(2,2)==0  ?  2:3);

		obj->setQuantiles(3);
		obj->enableDrawSolid3D(false);
		obj->setColor(POSE_COLOR);

		outObj->insert( obj );

		opengl::CSetOfObjectsPtr axes = opengl::stock_objects::CornerXYZ();
		axes->setPose(CPose3D(p->mean));
		axes->setScale(POSE_AXIS_SCALE);
		outObj->insert(axes);
	}

	return outObj;
}

