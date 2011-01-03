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

#include <mrpt/base.h>  // Precompiled headers 



#include <mrpt/poses/CPose3DPDF.h>

#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/poses/CPose3DPDFSOG.h>
#include <mrpt/poses/CPose3DPDFParticles.h>

#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/poses/CPosePDFSOG.h>
#include <mrpt/poses/CPosePDFParticles.h>


using namespace mrpt::poses;
using namespace std;

IMPLEMENTS_VIRTUAL_SERIALIZABLE( CPose3DPDF, CSerializable, mrpt::poses )

/*---------------------------------------------------------------
					copyFrom2D
  ---------------------------------------------------------------*/
CPose3DPDF* CPose3DPDF::createFrom2D(const CPosePDF &o)
{
	MRPT_START

	if (o.GetRuntimeClass()==CLASS_ID(CPosePDFGaussian))
	{
		CPose3DPDFGaussian	*newObj = new CPose3DPDFGaussian();
		const CPosePDFGaussian    *obj = static_cast<const CPosePDFGaussian *>( &o );

		newObj->mean = CPose3D( obj->mean );
		CMatrixDouble  COV = CMatrixDouble(obj->cov);
		COV.setSize(6,6);

		// Move covariances: phi<->z
		COV(3,3)=COV(2,2); COV(2,2)=0;
		COV(0,3)=COV(3,0) = COV(0,2); COV(0,2)=COV(2,0)=0;
		COV(1,3)=COV(3,1) = COV(1,2); COV(1,2)=COV(2,1)=0;

		newObj->cov= CMatrixDouble66(COV);

		return newObj;
	}
	else
		if (o.GetRuntimeClass()==CLASS_ID(CPosePDFParticles))
		{
			const CPosePDFParticles    *obj = static_cast<const CPosePDFParticles*>( &o );
			CPose3DPDFParticles	*newObj = new CPose3DPDFParticles(obj->size());

			CPosePDFParticles::CParticleList::const_iterator  it1;
			CPose3DPDFParticles::CParticleList::iterator      it2;
			for (it1=obj->m_particles.begin(),it2=newObj->m_particles.begin();it1!=obj->m_particles.end();it1++,it2++)
			{
				it2->log_w = it1->log_w;
				(*it2->d) = (*it1->d);
			}

			return newObj;
		}
		else
			if (o.GetRuntimeClass()==CLASS_ID(CPosePDFSOG))
			{
				const CPosePDFSOG    *obj = static_cast<const CPosePDFSOG*>( &o );
				CPose3DPDFSOG	*newObj = new CPose3DPDFSOG(obj->size());
				
				CPosePDFSOG::const_iterator it1;
				CPose3DPDFSOG::iterator     it2;

				for (it1=obj->begin(),it2=newObj->begin();it1!=obj->end();it1++,it2++)
				{
					it2->log_w = it1->log_w;
					it2->val.mean.setFromValues( it1->mean.x(),it1->mean.y(),0, it1->mean.phi(),0,0 );
					
					it2->val.cov.zeros();
					
					it2->val.cov.get_unsafe(0,0) = it1->cov.get_unsafe(0,0);
					it2->val.cov.get_unsafe(1,1) = it1->cov.get_unsafe(1,1);
					it2->val.cov.get_unsafe(3,3) = it1->cov.get_unsafe(2,2); // yaw <- phi

					it2->val.cov.get_unsafe(0,1) = 
					it2->val.cov.get_unsafe(1,0) = it1->cov.get_unsafe(0,1);
					
					it2->val.cov.get_unsafe(0,3) = 
					it2->val.cov.get_unsafe(3,0) = it1->cov.get_unsafe(0,2);

					it2->val.cov.get_unsafe(1,3) = 
					it2->val.cov.get_unsafe(3,1) = it1->cov.get_unsafe(1,2);
				}

				return newObj;
			}
			else
				THROW_EXCEPTION("Class of object not supported by this method!");

	MRPT_END
}

