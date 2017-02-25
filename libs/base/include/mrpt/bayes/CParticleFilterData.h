/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CParticleFilterData_H
#define CParticleFilterData_H

#include <mrpt/utils/core_defs.h>
#include <mrpt/bayes/CProbabilityParticle.h>
#include <mrpt/bayes/CParticleFilterCapable.h>

#include <deque>
#include <algorithm>

namespace mrpt
{
namespace bayes
{
	class CParticleFilterCapable;

	/** A curiously recurring template pattern (CRTP) approach to providing the basic functionality of any CParticleFilterData<> class.
	  *  Users should inherit from CParticleFilterData<>, which in turn will automatically inhirit from this base class.
	  * \sa CParticleFilter, CParticleFilterCapable, CParticleFilterData
	  * \ingroup mrpt_base_grp
	  */
	template <class Derived,class particle_list_t>
	struct CParticleFilterDataImpl : public CParticleFilterCapable
	{
		/// CRTP helper method
		inline const Derived& derived() const { return *static_cast<const Derived*>(this); }
		/// CRTP helper method
		inline       Derived& derived()       { return *static_cast<Derived*>(this); }

		double getW(size_t i) const MRPT_OVERRIDE
		{
			if (i>=derived().m_particles.size()) THROW_EXCEPTION_CUSTOM_MSG1("Index %i is out of range!",(int)i);
			return derived().m_particles[i].log_w;
		}

		void setW(size_t i, double w) MRPT_OVERRIDE
		{
			if (i>=derived().m_particles.size()) THROW_EXCEPTION_CUSTOM_MSG1("Index %i is out of range!",(int)i);
			derived().m_particles[i].log_w = w;
		}

		size_t particlesCount() const MRPT_OVERRIDE
		{
			return derived().m_particles.size();
		}

		double normalizeWeights( double *out_max_log_w = NULL ) MRPT_OVERRIDE
		{
			MRPT_START
			if (derived().m_particles.empty()) return 0;
			double minW = derived().m_particles[0].log_w;
			double maxW = minW;

			/* Compute the max/min of weights: */
			for (typename particle_list_t::iterator it=derived().m_particles.begin();it!=derived().m_particles.end();++it)
			{
				maxW = std::max<double>( maxW, it->log_w );
				minW = std::min<double>( minW, it->log_w );
			}
			/* Normalize: */
			for (typename particle_list_t::iterator it=derived().m_particles.begin();it!=derived().m_particles.end();++it)
				it->log_w -= maxW;
			if (out_max_log_w) *out_max_log_w = maxW;

			/* Return the max/min ratio: */
			return exp(maxW-minW);
			MRPT_END
		}

		double ESS() const MRPT_OVERRIDE
		{
			MRPT_START
			double	cum = 0;

			/* Sum of weights: */
			double sumLinearWeights = 0;
			for (typename particle_list_t::const_iterator it=derived().m_particles.begin();it!=derived().m_particles.end();++it)
				sumLinearWeights += exp( it->log_w  );
			/* Compute ESS: */
			for (typename particle_list_t::const_iterator it=derived().m_particles.begin();it!=derived().m_particles.end();++it)
				cum+= utils::square( exp( it->log_w ) / sumLinearWeights );

			if (cum==0)
					return 0;
			else	return 1.0/(derived().m_particles.size()*cum);
			MRPT_END
		}

		/** Replaces the old particles by copies determined by the indexes in "indx", performing an efficient copy of the necesary particles only and allowing the number of particles to change.*/
		void  performSubstitution( const std::vector<size_t> &indx) MRPT_OVERRIDE
		{
			MRPT_START
			particle_list_t                      parts;
			typename particle_list_t::iterator   itDest,itSrc;
			const size_t   M_old = derived().m_particles.size();
			size_t         i,j,lastIndxOld = 0;
			std::vector<bool>  oldParticlesReused(M_old,false);
			std::vector<bool>::const_iterator  oldPartIt;
			std::vector<size_t>           sorted_indx(indx);

			/* Assure the input index is sorted: */
			std::sort( sorted_indx.begin(), sorted_indx.end() );
			/* Set the new size: */
			parts.resize( sorted_indx.size() );
			for (i=0,itDest=parts.begin();itDest!=parts.end();i++,itDest++)
			{
				const size_t sorted_idx = sorted_indx[i];
				itDest->log_w = derived().m_particles[ sorted_idx ].log_w;
				/* We can safely delete old m_particles from [lastIndxOld,indx[i]-1] (inclusive): */
				for (j=lastIndxOld;j<sorted_idx;j++)
				{
					if (!oldParticlesReused[j])	/* If reused we can not delete that memory! */
						derived().m_particles[j].d.reset();
				}

				/* For the next iteration:*/
				lastIndxOld = sorted_idx;

				/* If this is the first time that the old particle "indx[i]" appears, */
				/*  we can reuse the old "data" instead of creating a new copy: */
				if (!oldParticlesReused[sorted_idx])
				{
					/* Reuse the data from the particle: */
					parts[i].d.reset( derived().m_particles[ sorted_idx ].d.get() );
					oldParticlesReused[sorted_idx]=true;
				}
				else
				{
					/* Make a copy of the particle's data: */
					ASSERT_( derived().m_particles[ sorted_idx ].d );
					parts[i].d.reset(new typename Derived::CParticleDataContent(*derived().m_particles[sorted_idx].d));
				}
			}
			/* Free memory of unused particles */
			for (itSrc = derived().m_particles.begin(), oldPartIt = oldParticlesReused.begin(); itSrc != derived().m_particles.end(); itSrc++, oldPartIt++)
				if (!*oldPartIt)
					itSrc->d.reset();
			/* Copy the pointers only to the final destination */
			derived().m_particles.resize( parts.size() );
			for (itSrc=parts.begin(),itDest=derived().m_particles.begin(); itSrc!=parts.end(); itSrc++, itDest++ )
			{
				itDest->log_w = itSrc->log_w;
				itDest->d.move_from(itSrc->d);
			}
			parts.clear();
			MRPT_END
		}

	}; // end CParticleFilterDataImpl<>


	/** This template class declares the array of particles and its internal data, managing some memory-related issues and providing an easy implementation of virtual methods required for implementing a CParticleFilterCapable.
	 *  See also the methods in the base class CParticleFilterDataImpl<>.
	 *
	 *   Since CProbabilityParticle implements all the required operators, the member "m_particles" can be safely copied with "=" or copy constructor operators
	 *    and new objects will be created internally instead of copying the internal pointers, which would lead to memory corruption.
	 *
	 * \sa CParticleFilter, CParticleFilterCapable, CParticleFilterDataImpl
	 * \ingroup mrpt_base_grp
	 */
	template <class T>
	class CParticleFilterData
	{
	public:
		typedef T                         CParticleDataContent; 	//!< This is the type inside the corresponding CParticleData class
		typedef CProbabilityParticle<T>   CParticleData;			//!< Use this to refer to each element in the m_particles array.
		typedef std::deque<CParticleData> CParticleList;			//!< Use this type to refer to the list of particles m_particles.

		CParticleList  m_particles;	//!< The array of particles

		/** Default constructor */
		CParticleFilterData() : m_particles(0) {}

		/** Free the memory of all the particles and reset the array "m_particles" to length zero  */
		void clearParticles()
		{
			m_particles.clear();
		}

		/** Dumps the sequence of particles and their weights to a stream (requires T implementing CSerializable).
		  * \sa readParticlesFromStream
		  */
		template <class STREAM>
		void  writeParticlesToStream( STREAM &out ) const
		{
			MRPT_START
			uint32_t	n = static_cast<uint32_t>(m_particles.size());
			out << n;
			typename CParticleList::const_iterator it;
			for (it=m_particles.begin();it!=m_particles.end();++it)
				out << it->log_w << (*it->d);
			MRPT_END
		}

		/** Reads the sequence of particles and their weights from a stream (requires T implementing CSerializable).
		  * \sa writeParticlesToStream
		  */
		template <class STREAM>
		void  readParticlesFromStream(STREAM &in)
		{
			MRPT_START
			clearParticles(); // Erase previous content:
			uint32_t n;
			in >> n;
			m_particles.resize(n);
			typename CParticleList::iterator it;
			for (it=m_particles.begin();it!=m_particles.end();++it)
			{
				in >> it->log_w;
				it->d.reset(new T());
				in >> *it->d;
			}
			MRPT_END
		}


		/** Returns a vector with the sequence of the logaritmic weights of all the samples.
		  */
		void getWeights( std::vector<double> &out_logWeights ) const
		{
			MRPT_START
			out_logWeights.resize(m_particles.size());
			std::vector<double>::iterator	it;
			typename CParticleList::const_iterator	it2;
			for (it=out_logWeights.begin(),it2=m_particles.begin();it2!=m_particles.end();++it,++it2)
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
			for (it=m_particles.begin();it!=m_particles.end();++it)
			{
				if (ret==NULL || it->log_w > ret->log_w)
					ret = &(*it);
			}
			return ret;
			MRPT_END
		}


	}; // End of class def.


} // end namespace
} // end namespace
#endif
