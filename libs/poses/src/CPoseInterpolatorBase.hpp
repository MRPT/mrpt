/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+
 */

#include <mrpt/poses/CPoseInterpolatorBase.h>

#include <mrpt/serialization/stl_serialization.h>
#include <mrpt/math/slerp.h>
#include <mrpt/math/wrap2pi.h>
#include <mrpt/math/interp_fit.hpp>
#include <mrpt/math/CMatrixD.h>
#include <mrpt/poses/CPose3DPDFParticles.h>
#include <mrpt/system/datetime.h>
#include <fstream>

namespace mrpt::poses
{
template <int DIM>
CPoseInterpolatorBase<DIM>::CPoseInterpolatorBase()
	: maxTimeInterpolation(std::chrono::seconds(-1))

{
}

template <int DIM>
void CPoseInterpolatorBase<DIM>::clear()
{
	m_path.clear();
}

template <int DIM>
void CPoseInterpolatorBase<DIM>::insert(
	const mrpt::Clock::time_point& t, const cpose_t& p)
{
	m_path[t] = p.asTPose();
}
template <int DIM>
void CPoseInterpolatorBase<DIM>::insert(
	const mrpt::Clock::time_point& t, const pose_t& p)
{
	m_path[t] = p;
}

/*---------------------------------------------------------------
						interpolate
  ---------------------------------------------------------------*/
template <int DIM>
typename CPoseInterpolatorBase<DIM>::cpose_t&
	CPoseInterpolatorBase<DIM>::interpolate(
		const mrpt::Clock::time_point& t, cpose_t& out_interp,
		bool& out_valid_interp) const
{
	pose_t p;
	this->interpolate(t, p, out_valid_interp);
	out_interp = cpose_t(p);
	return out_interp;
}

template <int DIM>
typename CPoseInterpolatorBase<DIM>::pose_t&
	CPoseInterpolatorBase<DIM>::interpolate(
		const mrpt::Clock::time_point& t, pose_t& out_interp,
		bool& out_valid_interp) const
{
	// Default value in case of invalid interp
	for (size_t k = 0; k < pose_t::static_size; k++)
	{
		out_interp[k] = 0;
	}
	TTimePosePair p1, p2, p3, p4;
	p1.second = p2.second = p3.second = p4.second = out_interp;

	// We'll look for 4 consecutive time points.
	// Check if the selected method needs all 4 points or just the central 2 of
	// them:
	bool interp_method_requires_4pts;
	switch (m_method)
	{
		case imLinear2Neig:
		case imSplineSlerp:
		case imLinearSlerp:
			interp_method_requires_4pts = false;
			break;
		default:
			interp_method_requires_4pts = true;
			break;
	};

	// Out of range?
	auto it_ge1 = m_path.lower_bound(t);

	// Exact match?
	if (it_ge1 != m_path.end() && it_ge1->first == t)
	{
		out_interp = it_ge1->second;
		out_valid_interp = true;
		return out_interp;
	}

	// Are we in the beginning or the end of the path?
	if (it_ge1 == m_path.end() || it_ge1 == m_path.begin())
	{
		out_valid_interp = false;
		return out_interp;
	}  // end

	p3 = *it_ge1;  // Third pair
	auto it_ge2 = it_ge1;
	++it_ge2;
	if (it_ge2 == m_path.end())
	{
		if (interp_method_requires_4pts)
		{
			out_valid_interp = false;
			return out_interp;
		}
	}
	else
	{
		p4 = *(it_ge2);  // Fourth pair
	}

	p2 = *(--it_ge1);  // Second pair

	if (it_ge1 == m_path.begin())
	{
		if (interp_method_requires_4pts)
		{
			out_valid_interp = false;
			return out_interp;
		}
	}
	else
	{
		p1 = *(--it_ge1);  // First pair
	}

	// Test if the difference between the desired timestamp and the next
	// timestamp is lower than a certain (configurable) value
	const mrpt::Clock::duration dt12 = interp_method_requires_4pts
										   ? (p2.first - p1.first)
										   : mrpt::Clock::duration(0);
	const mrpt::Clock::duration dt23 = (p3.first - p2.first);
	const mrpt::Clock::duration dt34 = interp_method_requires_4pts
										   ? (p4.first - p3.first)
										   : mrpt::Clock::duration(0);

	if (maxTimeInterpolation.count() > 0 &&
		(dt12 > maxTimeInterpolation || dt23 > maxTimeInterpolation ||
		 dt34 > maxTimeInterpolation))
	{
		out_valid_interp = false;
		return out_interp;
	}

	// Do interpolation:
	// ------------------------------------------
	// First Previous point:  p1
	// Second Previous point: p2
	// First Next point:	  p3
	// Second Next point:     p4
	// Time where to interpolate:  t

	impl_interpolation(p1, p2, p3, p4, m_method, t, out_interp);

	out_valid_interp = true;
	return out_interp;

}  // end interpolate

template <int DIM>
bool CPoseInterpolatorBase<DIM>::getPreviousPoseWithMinDistance(
	const mrpt::Clock::time_point& t, double distance, cpose_t& out_pose)
{
	pose_t p;
	bool ret = getPreviousPoseWithMinDistance(t, distance, p);
	out_pose = cpose_t(p);
	return ret;
}

template <int DIM>
bool CPoseInterpolatorBase<DIM>::getPreviousPoseWithMinDistance(
	const mrpt::Clock::time_point& t, double distance, pose_t& out_pose)
{
	if (m_path.size() == 0 || distance <= 0) return false;

	pose_t myPose;

	// Search for the desired timestamp
	auto it = m_path.find(t);
	if (it != m_path.end() && it != m_path.begin())
		myPose = it->second;
	else
		return false;

	double d = 0.0;
	do
	{
		--it;
		d = (point_t(myPose) - point_t(it->second)).norm();
	} while (d < distance && it != m_path.begin());

	if (d >= distance)
	{
		out_pose = it->second;
		return true;
	}
	else
		return false;
}  // end getPreviousPose

template <int DIM>
void CPoseInterpolatorBase<DIM>::setMaxTimeInterpolation(
	const mrpt::Clock::duration& time)
{
	ASSERT_(time.count() > 0);
	maxTimeInterpolation = time;
}

template <int DIM>
mrpt::Clock::duration CPoseInterpolatorBase<DIM>::getMaxTimeInterpolation()
{
	return maxTimeInterpolation;
}

template <int DIM>
bool CPoseInterpolatorBase<DIM>::saveToTextFile(const std::string& s) const
{
	try
	{
		std::ofstream f;
		f.open(s);
		if (!f.is_open()) return false;
		std::string str;
		for (auto i = m_path.begin(); i != m_path.end(); ++i)
		{
			const double t = mrpt::system::timestampTotime_t(i->first);
			const auto& p = i->second;

			str = mrpt::format("%.06f ", t);
			for (unsigned int k = 0; k < p.size(); k++)
				str += mrpt::format("%.06f ", p[k]);
			str += std::string("\n");

			f << str;
		}
		return true;
	}
	catch (...)
	{
		return false;
	}
}

template <int DIM>
bool CPoseInterpolatorBase<DIM>::saveInterpolatedToTextFile(
	const std::string& s, const mrpt::Clock::duration& period) const
{
	using mrpt::Clock;
	try
	{
		std::ofstream f;
		f.open(s);
		if (!f.is_open()) return false;
		if (m_path.empty()) return true;

		std::string str;

		const Clock::time_point t_ini = m_path.begin()->first;
		const Clock::time_point t_end = m_path.rbegin()->first;

		pose_t p;
		bool valid;
		for (Clock::time_point t = t_ini; t <= t_end; t += period)
		{
			this->interpolate(t, p, valid);
			if (!valid) continue;

			str = mrpt::format("%.06f ", mrpt::system::timestampTotime_t(t));
			for (unsigned int k = 0; k < p.size(); k++)
				str += mrpt::format("%.06f ", p[k]);
			str += std::string("\n");
			f << str;
		}
		return true;
	}
	catch (...)
	{
		return false;
	}
}

template <int DIM>
bool CPoseInterpolatorBase<DIM>::loadFromTextFile(const std::string& s)
{
	MRPT_START

	clear();
	mrpt::math::CMatrixD M;

	try
	{
		M.loadFromTextFile(s);
	}
	catch (std::exception&)
	{
		return false;  // error loading file
	}

	// Check valid format:
	if (M.rows() == 0) return false;
	ASSERT_(M.cols() == pose_t::static_size + 1);

	// load into the path:
	const size_t N = M.cols();
	pose_t p;
	for (size_t i = 0; i < N; i++)
	{
		for (unsigned int k = 0; k < pose_t::static_size; k++)
			p[k] = M(i, k + 1);
		insert(mrpt::system::time_tToTimestamp(M(i, 0)), p);
	}
	return true;
	MRPT_END
}

template <int DIM>
void CPoseInterpolatorBase<DIM>::getBoundingBox(
	point_t& Min, point_t& Max) const
{
	MRPT_START
	ASSERT_(!m_path.empty());

	for (unsigned int k = 0; k < point_t::static_size; k++)
	{
		Min[k] = std::numeric_limits<double>::max();
		Max[k] = -std::numeric_limits<double>::max();
	}

	for (auto p = m_path.begin(); p != m_path.end(); ++p)
	{
		for (unsigned int k = 0; k < point_t::static_size; k++)
		{
			mrpt::keep_min(Min[k], p->second[k]);
			mrpt::keep_max(Max[k], p->second[k]);
		}
	}
	MRPT_END
}

template <int DIM>
void CPoseInterpolatorBase<DIM>::setInterpolationMethod(
	mrpt::poses::TInterpolatorMethod method)
{
	m_method = method;
}

template <int DIM>
TInterpolatorMethod CPoseInterpolatorBase<DIM>::getInterpolationMethod() const
{
	return m_method;
}

template <int DIM>
void CPoseInterpolatorBase<DIM>::filter(
	unsigned int component, unsigned int samples)
{
	if (m_path.empty()) return;

	TPath aux;

	int ant, post;
	size_t nitems = size();

	post = (samples % 2) ? (unsigned int)(samples / 2) : samples / 2;
	ant = (unsigned int)(samples / 2);

	int k = 0;
	iterator it1, it2, it3;

	// int asamples;
	for (it1 = m_path.begin(); it1 != m_path.end(); ++it1, ++k)
	{
		it2 = m_path.begin();
		if (k - ant > 0) advance(it2, k - ant);

		if (k + post < (int)nitems)
		{
			it3 = m_path.begin();
			advance(it3, k + post + 1);
		}
		else
		{
			it3 = m_path.end();
		}

		unsigned int nsamples = distance(it2, it3);
		CPose3DPDFParticles particles(nsamples);
		for (unsigned int i = 0; it2 != it3; ++it2, ++i)
		{
			particles.m_particles[i].log_w = 0;
			particles.m_particles[i].d = it1->second;
			switch (component)
			{
				case 0:
					particles.m_particles[i].d.x = it2->second[0];
					break;
				case 1:
					particles.m_particles[i].d.y = it2->second[1];
					break;
				case 2:
					particles.m_particles[i].d.z = it2->second[2];
					break;
				case 3:
					particles.m_particles[i].d.yaw = it2->second[3];
					break;
				case 4:
					particles.m_particles[i].d.pitch = it2->second[4];
					break;
				case 5:
					particles.m_particles[i].d.roll = it2->second[5];
					break;
			}  // end switch
		}  // end for it2

		mrpt::poses::CPose3D auxPose;
		particles.getMean(auxPose);
		aux[it1->first] = pose_t(auxPose.asTPose());
	}  // end for it1
	m_path = aux;
}
}  // namespace mrpt::poses
