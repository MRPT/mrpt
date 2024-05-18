/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2024, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <cstdlib>
#include <functional>
#include <map>
#include <mutex>
#include <shared_mutex>
#include <thread>

namespace mrpt::containers
{
/** Creates an instance of the data type T per requesting thread.
 * \ingroup mrpt_containers_grp
 */
template <class T>
class PerThreadDataHolder
{
 public:
  PerThreadDataHolder() = default;

  T& get()
  {
    m_dataMtx.lock();
    T& d = m_data[std::this_thread::get_id()];
    m_dataMtx.unlock();
    return d;
  }
  const T& get() const
  {
    m_dataMtx.lock();
    const T& d = m_data[std::this_thread::get_id()];
    m_dataMtx.unlock();
    return d;
  }

  /// Note: you should define your own mutex for the part of the user data T
  /// that is read & writen from different threads.
  void run_on_all(const std::function<void(T&)>& f)
  {
    m_dataMtx.lock();
    for (auto& kv : m_data) f(kv.second);
    m_dataMtx.unlock();
  }

  void clear()
  {
    m_dataMtx.lock();
    m_data.clear();
    m_dataMtx.unlock();
  }

  /// Copy the data, leave mutexes apart.
  PerThreadDataHolder<T>& operator=(const PerThreadDataHolder<T>& o)
  {
    if (this == &o) return *this;

    m_dataMtx.lock();
    o.m_dataMtx.lock();
    m_data = o.m_data;
    m_dataMtx.unlock();
    o.m_dataMtx.unlock();
    return *this;
  }
  /// Copy the data, leave mutexes apart.
  PerThreadDataHolder(const PerThreadDataHolder<T>& o) { *this = o; }

  /// Move the data, leave mutexes apart.
  PerThreadDataHolder<T>& operator=(PerThreadDataHolder<T>&& o)
  {
    if (this == &o) return *this;

    m_dataMtx.lock();
    o.m_dataMtx.lock();
    m_data = std::move(o.m_data);
    m_dataMtx.unlock();
    o.m_dataMtx.unlock();
    return *this;
  }
  /// Move the data, leave mutexes apart.
  PerThreadDataHolder(PerThreadDataHolder<T>&& o) { *this = std::move(o); }

 private:
  mutable std::map<std::thread::id, T> m_data;
  mutable std::mutex m_dataMtx;  // Mutex for m_data itself only
};

}  // namespace mrpt::containers
