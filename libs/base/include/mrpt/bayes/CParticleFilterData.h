/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
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
#ifndef CParticleFilterData_H
#define CParticleFilterData_H

#include <mrpt/utils/utils_defs.h>
#include <mrpt/bayes/CProbabilityParticle.h>

#include <algorithm>

namespace mrpt
{
namespace bayes
{
	/** This template class declares the array of particles and its internal data, managing some memory-related issues and providing an easy implementation of virtual methods required for implementing a CParticleFilterCapable.
	 *  By adding IMPLEMENT_PARTICLE_FILTER_CAPABLE(T) to the body of the declaration of classes inheriting from both CParticleFilterData and CParticleFilterCapable, the following
	 *    pure virtual methods are automatically implemented (the param T must be equal to the argument of the template CParticleFilterData).
	  *   - CParticleFilterCapable::getW
	  *   - CParticleFilterCapable::setW
	  *   - CParticleFilterCapable::particlesCount
	  *   - CParticleFilterCapable::normalizeWeights
	  *   - CParticleFilterCapable::ESS
	  *   - CParticleFilterCapable::performSubstitution
	 *
	 *   Since CProbabilityParticle implements all the required operators, the member "m_particles" can be safely copied with "=" or copy constructor operators
	 *    and new objects will be created internally instead of copying the internal pointers, which would lead to memory corruption.
	 *
	 * \sa CParticleFilter, CParticleFilterCapable, IMPLEMENT_PARTICLE_FILTER_CAPABLE
	 * \ingroup mrpt_base_grp
	 */
	template <class T>
	class CParticleFilterData
	{
	public:
		typedef T 							CParticleDataContent; 	//!< This is the type inside the corresponding CParticleData class
		typedef CProbabilityParticle<T> 	CParticleData;			//!< Use this to refer to each element in the m_particles array.
		typedef std::deque<CParticleData> 	CParticleList;			//!< Use this type to refer to the list of particles m_particles.

		CParticleList  m_particles;	//!< The array of particles

		/** Default constructor */
		CParticleFilterData() : m_particles(0)
		{ }

        /** Free the memory of all the particles and reset the array "m_particles" to length zero.
          */
        void clearParticles()
		{
			MRPT_START
			for (typename CParticleList::iterator it=m_particles.begin();it!=m_particles.end();it++)
				if (it->d) delete it->d;
			m_particles.clear();
			MRPT_END
		}

        /** Virtual destructor
          */
        virtual ~CParticleFilterData()
		{
			MRPT_START
			clearParticles();
			MRPT_END
		}

		/** Dumps the sequence of particles and their weights to a stream (requires T implementing CSerializable).
		  * \sa readParticlesFromStream
		  */
		void  writeParticlesToStream( utils::CStream &out ) const
		{
			MRPT_START
			uint32_t	n = static_cast<uint32_t>(m_particles.size());
			out << n;
			typename CParticleList::const_iterator it;
			for (it=m_particles.begin();it!=m_particles.end();it++)
				out << it->log_w << (*it->d);
			MRPT_END
		}

		/** Reads the sequence of particles and their weights from a stream (requires T implementing CSerializable).
		  * \sa writeParticlesToStream
		  */
		void  readParticlesFromStream(utils::CStream &in)
		{
			MRPT_START
			clearParticles();	// Erase previous content:
			uint32_t	n;
			in >> n;
			m_particles.resize(n);
			typename CParticleList::iterator it;
			for (it=m_particles.begin();it!=m_particles.end();it++)
			{
				in >> it->log_w;
				it->d = new T();
				in >> *it->d;
			}
			MRPT_END
		}


		/** Returns a vector with the sequence of the logaritmic weights of all the samples.
		  */
		void getWeights( vector_double &out_logWeights ) const
		{
			MRPT_START
			out_logWeights.resize(m_particles.size());
			vector_double::iterator						it;
			typename CParticleList::const_iterator	it2;
			for (it=out_logWeights.begin(),it2=m_particles.begin();it2!=m_particles.end();it++,it2++)
				*it = it2->log_w;
			MRPT_END
		}

		/** Returns the particle with the highest weight.
		  */
		const CParticleData * getMostLikelyParticle() const
		{
			MRPT_START
			const CParticleData *ret = NULL;
			ASSERT_(m_particles.size()>0)

			typename CParticleList::const_iterator	it;
			for (it=m_particles.begin();it!=m_particles.end();it++)
			{
				if (ret==NULL || it->log_w > ret->log_w)
					ret = &(*it);
			}
			return ret;
			MRPT_END
		}


	}; // End of class def.

	/** This must be placed within the declaration of classes inheriting from both CParticleFilterData and CParticleFilterCapable to implement some pure virtual methods (the param T must be equal to the argument of the template CParticleFilterData).
	  *  The following pure virtual methods are implemented:
	  *   - CParticleFilterCapable::getW
	  *   - CParticleFilterCapable::setW
	  *   - CParticleFilterCapable::particlesCount
	  *   - CParticleFilterCapable::normalizeWeights
	  *   - CParticleFilterCapable::ESS
	  *   - CParticleFilterCapable::performSubstitution
	  */
#define IMPLEMENT_PARTICLE_FILTER_CAPABLE(T) \
		public: \
		virtual double  getW(size_t i) const \
		{ \
			MRPT_START \
			if (i>=m_particles.size()) THROW_EXCEPTION_CUSTOM_MSG1("Index %i is out of range!",(int)i); \
			return m_particles[i].log_w; \
			MRPT_END \
		} \
		virtual void setW(size_t i, double w) \
		{ \
			MRPT_START \
			if (i>=m_particles.size()) THROW_EXCEPTION_CUSTOM_MSG1("Index %i is out of range!",(int)i); \
			m_particles[i].log_w = w; \
			MRPT_END \
		} \
		virtual size_t particlesCount() const { return m_particles.size(); } \
		virtual double normalizeWeights( double *out_max_log_w = NULL ) \
		{ \
			MRPT_START \
			CParticleList::iterator it;\
			\
			if (!m_particles.size()) return 0; \
			double	minW,maxW; \
			minW = maxW = m_particles[0].log_w; \
			/* Compute the max/min of weights: */ \
			for (it=m_particles.begin();it!=m_particles.end();it++) \
			{ \
				maxW = std::max<double>( maxW, it->log_w ); \
				minW = std::min<double>( minW, it->log_w ); \
			} \
			/* Normalize: */ \
			for (it=m_particles.begin();it!=m_particles.end();it++) \
				it->log_w -= maxW; \
			if (out_max_log_w) \
				*out_max_log_w = maxW; \
			/* Return the max/min ratio: */ \
			return exp(maxW-minW); \
			MRPT_END \
		} \
		virtual double ESS() \
		{ \
			MRPT_START \
			CParticleList::iterator it; \
			double	cum = 0; \
			\
			/* Sum of weights: */ \
			double sumLinearWeights = 0; \
			for (it=m_particles.begin();it!=m_particles.end();it++) \
				sumLinearWeights += exp( it->log_w  ); \
			/* Compute ESS: */ \
			for (it=m_particles.begin();it!=m_particles.end();it++) \
				cum+= utils::square( exp( it->log_w ) / sumLinearWeights ); \
			\
			if (cum==0) \
					return 0; \
			else	return 1.0/(m_particles.size()*cum); \
			MRPT_END \
		} \
		/** Replaces the old particles by copies determined by the indexes in "indx", performing an efficient copy of the necesary particles only and allowing the number of particles to change.*/ \
		virtual void  performSubstitution( const std::vector<size_t> &indx) \
		{  \
			MRPT_START \
			CParticleList    	            parts; \
			CParticleList::iterator			itDest,itSrc; \
			size_t									M_old = m_particles.size(); \
			size_t									i,j,lastIndxOld = 0; \
			std::vector<bool>						oldParticlesReused(M_old,false); \
			std::vector<bool>::const_iterator		oldPartIt; \
			std::vector<size_t>						sorted_indx(indx); \
			std::vector<size_t>::iterator			sort_idx_it; \
			/* Assure the input index is sorted: */ \
			std::sort( sorted_indx.begin(), sorted_indx.end() ); \
			/* Set the new size: */ \
			parts.resize( sorted_indx.size() ); \
			for (i=0,itDest=parts.begin();itDest!=parts.end();i++,itDest++) \
			{ \
				const size_t	sorted_idx = sorted_indx[i]; \
				itDest->log_w = m_particles[ sorted_idx ].log_w; \
				/* We can safely delete old m_particles from [lastIndxOld,indx[i]-1] (inclusive): */  \
				for (j=lastIndxOld;j<sorted_idx;j++) \
				{ \
					if (!oldParticlesReused[j])	/* If reused we can not delete that memory! */ \
					{ \
						delete m_particles[j].d; \
						m_particles[j].d = NULL; \
					} \
				} \
				/* For the next iteration:*/ \
				lastIndxOld = sorted_idx; \
				/* If this is the first time that the old particle "indx[i]" appears, */ \
				/*  we can reuse the old "data" instead of creating a new copy: */ \
				if (!oldParticlesReused[sorted_idx]) \
				{ \
					/* Reuse the data from the particle: */ \
					parts[i].d = m_particles[ sorted_idx ].d; \
					oldParticlesReused[sorted_idx]=true; \
				} \
				else \
				{ \
					/* Make a copy of the particle's data: */ \
					ASSERT_( m_particles[ sorted_idx ].d != NULL); \
					parts[i].d = new T( *m_particles[ sorted_idx ].d ); \
				} \
			} \
			/* Free memory of unused particles */ \
			for (itSrc=m_particles.begin(),oldPartIt=oldParticlesReused.begin();itSrc!=m_particles.end();itSrc++,oldPartIt++) \
				if (! *oldPartIt ) \
				{ \
					delete itSrc->d; \
					itSrc->d = NULL; \
				} \
			/* Copy the pointers only to the final destination */ \
			m_particles.resize( parts.size() ); \
			for (itSrc=parts.begin(),itDest=m_particles.begin(); itSrc!=parts.end(); itSrc++, itDest++ ) \
			{ \
				itDest->log_w = itSrc->log_w; \
				itDest->d = itSrc->d; \
				itSrc->d = NULL; \
			} \
			parts.clear(); \
			MRPT_END \
		} \

	} // end namespace
} // end namespace
#endif
