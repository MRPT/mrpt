/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#pragma once

#include <type_traits>

namespace mrpt
{
/** Auxiliary helper structure for mrpt::lockHelper()
 * \ingroup mrpt_core_grp
 */
template <class T>
class LockHelper
{
	using Tnc = std::remove_const_t<T>;

   public:
	LockHelper(const Tnc* l) : l_{const_cast<Tnc*>(l)} { l_->lock(); }
	~LockHelper()
	{
		if (l_) l_->unlock();
	}

	LockHelper(const LockHelper& o) = delete;
	LockHelper& operator=(const LockHelper& o) = delete;

	LockHelper(LockHelper&& o) : l_{o.l} { o.l = nullptr; }
	LockHelper& operator=(LockHelper&& o)
	{
		l_ = o.l;
		o.l = nullptr;
		return *this;
	}

   private:
	Tnc* l_{nullptr};
};

/** Syntactic sugar to easily create a locker to any kind of std::mutex
 * \ingroup mrpt_core_grp
 */
template <class T>
LockHelper<T> lockHelper(T& t)
{
	return LockHelper<T>(&t);
}
}  // namespace mrpt
