/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2026, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/
#pragma once

#include <mrpt/bayes/CParticleFilterCapable.h>
#include <mrpt/bayes/CProbabilityParticle.h>
#include <mrpt/core/bits_math.h>
#include <mrpt/core/exceptions.h>

#include <algorithm>
#include <cmath>
#include <deque>
#include <numeric>

namespace mrpt::bayes
{
class CParticleFilterCapable;

/** A curiously recurring template pattern (CRTP) approach to providing the
 * basic functionality of any CParticleFilterData<> class.
 *  Users should inherit from CParticleFilterData<>, which in turn will
 * automatically inhirit from this base class.
 * \sa CParticleFilter, CParticleFilterCapable, CParticleFilterData
 * \ingroup mrpt_bayes_grp
 */
template <class Derived, class particle_list_t>
struct CParticleFilterDataImpl : public CParticleFilterCapable
{
  const Derived& derived() const { return static_cast<const Derived&>(*this); }
  Derived& derived() { return static_cast<Derived&>(*this); }
  double getW(size_t i) const override
  {
    if (i >= derived().m_particles.size()) THROW_EXCEPTION_FMT("Index %i is out of range!", (int)i);
    return derived().m_particles[i].log_w;
  }

  void setW(size_t i, double w) override
  {
    if (i >= derived().m_particles.size()) THROW_EXCEPTION_FMT("Index %i is out of range!", (int)i);
    derived().m_particles[i].log_w = w;
  }

  size_t particlesCount() const override { return derived().m_particles.size(); }

  [[nodiscard]] double normalizeWeights(double* out_max_log_w = nullptr) override
  {
    MRPT_START
    if (derived().m_particles.empty())
    {
      return 0;
    }
    const auto [minIt, maxIt] = std::minmax_element(
        derived().m_particles.begin(), derived().m_particles.end(),
        [](const auto& a, const auto& b) { return a.log_w < b.log_w; });
    const double maxW = maxIt->log_w;
    const double minW = minIt->log_w;
    for (auto& p : derived().m_particles)
    {
      p.log_w -= maxW;
    }
    if (out_max_log_w)
    {
      *out_max_log_w = maxW;
    }
    return std::exp(maxW - minW);
    MRPT_END
  }

  [[nodiscard]] double ESS() const override
  {
    MRPT_START
    if (derived().m_particles.empty())
    {
      return 0;
    }
    // Log-sum-exp trick: shift by max weight to avoid underflow
    const double maxW = std::max_element(
                            derived().m_particles.begin(), derived().m_particles.end(),
                            [](const auto& a, const auto& b) { return a.log_w < b.log_w; })
                            ->log_w;
    const double sumLin = std::transform_reduce(
        derived().m_particles.begin(), derived().m_particles.end(), 0.0, std::plus<>{},
        [maxW](const auto& p) { return std::exp(p.log_w - maxW); });
    if (sumLin == 0)
    {
      return 0;
    }
    const double cum = std::transform_reduce(
        derived().m_particles.begin(), derived().m_particles.end(), 0.0, std::plus<>{},
        [maxW, sumLin](const auto& p) { return mrpt::square(std::exp(p.log_w - maxW) / sumLin); });
    if (cum == 0)
    {
      return 0;
    }
    return 1.0 / (derived().m_particles.size() * cum);
    MRPT_END
  }

  /** Replaces the old particles by copies determined by the indexes in
   * "indx", performing an efficient copy of the necessary particles only and
   * allowing the number of particles to change.
   * \note The input index vector is sorted internally before processing.
   */
  void performSubstitution(const std::vector<size_t>& indx) override
  {
    MRPT_START
    // Ensure input indices are sorted
    std::vector<size_t> sorted_indx(indx);
    std::sort(sorted_indx.begin(), sorted_indx.end());

    /* Temporary buffer: */
    particle_list_t parts;
    parts.resize(sorted_indx.size());

    // Implementation for particles as pointers:
    if constexpr (Derived::PARTICLE_STORAGE == particle_storage_mode::POINTER)
    {
      const size_t M_old = derived().m_particles.size();
      // Index, in the *new* set, of reused particle data, indexed
      // by *old* indices, or "-1" if not reused.
      std::vector<int> reusedIdx(M_old, -1);
      typename particle_list_t::iterator itDest;
      for (size_t i = 0; i < parts.size(); i++)
      {
        const size_t sorted_idx = sorted_indx[i];
        parts[i].log_w = derived().m_particles[sorted_idx].log_w;

        // The first time that the old particle "indx[i]" appears, we
        // can reuse the old "data" instead of creating a new copy:
        const int idx_of_this_in_new_set = reusedIdx[sorted_idx];
        if (idx_of_this_in_new_set == -1)
        {
          // First time: Reuse the data from the particle.
          parts[i].d = std::move(derived().m_particles[sorted_idx].d);
          reusedIdx[sorted_idx] = i;
        }
        else
        {
          // Make a copy of the particle's data
          // (The "= operator" already makes a deep copy)
          parts[i].d = parts[idx_of_this_in_new_set].d;
        }
      }
      // Free memory of unused particles: Done automatically.
    }
    else
    {
      // Implementation for particles as values:
      auto it_idx = sorted_indx.begin();
      auto itDest = parts.begin();
      for (; itDest != parts.end(); ++it_idx, ++itDest) *itDest = derived().m_particles[*it_idx];
    }
    /* Move particles to the final container: */
    derived().m_particles = std::move(parts);
    MRPT_END
  }

};  // end CParticleFilterDataImpl<>

/** This template class declares the array of particles and its internal data,
 * managing some memory-related issues and providing an easy implementation of
 * virtual methods required for implementing a CParticleFilterCapable.
 *  See also the methods in the base class CParticleFilterDataImpl<>.
 *
 *   Since CProbabilityParticle implements all the required operators, the
 * member "m_particles" can be safely copied with "=" or copy constructor
 * operators
 *    and new objects will be created internally instead of copying the internal
 * pointers, which would lead to memory corruption.
 *
 * \sa CParticleFilter, CParticleFilterCapable, CParticleFilterDataImpl
 * \ingroup mrpt_bayes_grp
 */
template <class T, particle_storage_mode STORAGE = particle_storage_mode::POINTER>
class CParticleFilterData
{
 public:
  /** This is the type inside the corresponding CParticleData class */
  using CParticleDataContent = T;
  /** Use this to refer to each element in the m_particles array. */
  using CParticleData = CProbabilityParticle<T, STORAGE>;
  /** Use this type to refer to the list of particles m_particles. */
  using CParticleList = std::deque<CParticleData>;
  static const particle_storage_mode PARTICLE_STORAGE = STORAGE;

  /** The array of particles */
  CParticleList m_particles;

  /** Default constructor */
  CParticleFilterData() : m_particles(0) {}
  /** Free the memory of all the particles and reset the array "m_particles"
   * to length zero  */
  void clearParticles() { m_particles.clear(); }
  /** Dumps the sequence of particles and their weights to a stream (requires
   * T implementing CSerializable).
   * \sa readParticlesFromStream
   */
  template <class STREAM>
  void writeParticlesToStream(STREAM& out) const
  {
    MRPT_START
    out << static_cast<uint32_t>(m_particles.size());
    for (const auto& p : m_particles)
    {
      out << p.log_w;
      if constexpr (STORAGE == particle_storage_mode::POINTER)
      {
        out << (*p.d);
      }
      else
      {
        out << p.d;
      }
    }
    MRPT_END
  }

  /** Reads the sequence of particles and their weights from a stream
   * (requires T implementing CSerializable).
   * \sa writeParticlesToStream
   */
  template <class STREAM>
  void readParticlesFromStream(STREAM& in)
  {
    MRPT_START
    clearParticles();
    uint32_t n;
    in >> n;
    m_particles.resize(n);
    for (auto& p : m_particles)
    {
      in >> p.log_w;
      if constexpr (STORAGE == particle_storage_mode::POINTER)
      {
        p.d.reset(new T());
        in >> *p.d;
      }
      else
      {
        in >> p.d;
      }
    }
    MRPT_END
  }

  /** Fills \a out_logWeights with the logarithmic weights of all particles. */
  void getWeights(std::vector<double>& out_logWeights) const
  {
    out_logWeights.resize(m_particles.size());
    std::transform(
        m_particles.begin(), m_particles.end(), out_logWeights.begin(),
        [](const auto& p) { return p.log_w; });
  }

  /** Returns a vector with the logarithmic weights of all particles. */
  [[nodiscard]] std::vector<double> getWeights() const
  {
    std::vector<double> w;
    getWeights(w);
    return w;
  }

  /** Returns the particle with the highest weight. */
  [[nodiscard]] const CParticleData* getMostLikelyParticle() const
  {
    MRPT_START
    const CParticleData* ret = nullptr;
    ASSERT_(m_particles.size() > 0);

    for (const auto& p : m_particles)
      if (ret == nullptr || p.log_w > ret->log_w) ret = &p;
    return ret;
    MRPT_END
  }

};  // End of class def.

}  // namespace mrpt::bayes
