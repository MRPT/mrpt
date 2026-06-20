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

#include <mrpt/core/config.h>  // MRPT_OS_*()
#include <mrpt/core/exceptions.h>
#include <mrpt/system/config.h>
#include <mrpt/system/thread_name.h>

#if defined(MRPT_OS_WINDOWS)
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
//
#include <cwchar>
#include <vector>

#elif defined(MRPT_OS_LINUX) || defined(MRPT_OS_APPLE)
#if HAVE_PTHREAD_H
#include <pthread.h>
#endif
#if defined(MRPT_OS_LINUX) && !MRPT_IN_EMSCRIPTEN
#include <sys/prctl.h>
#endif

namespace
{
void SetThreadName(std::thread& thread, const char* threadName)
{
  auto handle = thread.native_handle();
#if defined(MRPT_OS_APPLE)
  // macOS pthread_setname_np only sets the current thread's name
  (void)handle;
  pthread_setname_np(threadName);
#elif !MRPT_IN_EMSCRIPTEN
  pthread_setname_np(handle, threadName);
#endif
}

std::string GetThreadName(std::thread& thread)
{
  auto handle = thread.native_handle();
  char buf[1000];
  buf[0] = '\0';
#if !MRPT_IN_EMSCRIPTEN
  pthread_getname_np(handle, buf, sizeof(buf));
#else
  (void)handle;
#endif
  return std::string(buf);
}

void SetThreadName(const char* threadName)
{
#if defined(MRPT_OS_APPLE)
  pthread_setname_np(threadName);
#elif !MRPT_IN_EMSCRIPTEN
  prctl(PR_SET_NAME, threadName, 0L, 0L, 0L);
#endif
}
std::string GetThreadName()
{
  char buf[100] = {0};
#if defined(MRPT_OS_APPLE) || !MRPT_IN_EMSCRIPTEN
  pthread_t self = pthread_self();
  pthread_getname_np(self, buf, sizeof(buf));
#endif
  return std::string(buf);
}
}  // namespace
#endif  // Linux or Apple

void mrpt::system::thread_name(const std::string& name)
{
#if defined(MRPT_OS_WINDOWS) && !defined(__MINGW32_MAJOR_VERSION)
  wchar_t wName[50];
  std::mbstowcs(wName, name.c_str(), sizeof(wName) / sizeof(wName[0]));
  SetThreadDescription(GetCurrentThread(), wName);
#elif defined(MRPT_OS_LINUX) || defined(MRPT_OS_APPLE)
  SetThreadName(name.c_str());
#endif
}

void mrpt::system::thread_name(const std::string& name, std::thread& theThread)
{
#if defined(MRPT_OS_WINDOWS) && !defined(__MINGW32_MAJOR_VERSION)
  wchar_t wName[50];
  std::mbstowcs(wName, name.c_str(), sizeof(wName) / sizeof(wName[0]));
  SetThreadDescription(theThread.native_handle(), wName);
#elif defined(MRPT_OS_LINUX) || defined(MRPT_OS_APPLE)
  SetThreadName(theThread, name.c_str());
#endif
}

#if defined(MRPT_OS_WINDOWS)
static std::string w2cstr(wchar_t** wstrnc)
{
  const wchar_t** wstr = const_cast<const wchar_t**>(wstrnc);

  std::mbstate_t state = std::mbstate_t();
  std::size_t len = 1 + std::wcsrtombs(nullptr, wstr, 0, &state);
  std::vector<char> mbstr(len);
  std::wcsrtombs(&mbstr[0], wstr, mbstr.size(), &state);
  return std::string(mbstr.data());
}
#endif

std::string mrpt::system::thread_name()
{
#if defined(MRPT_OS_WINDOWS) && !defined(__MINGW32_MAJOR_VERSION)
  std::string ret = "NoName";
  PWSTR str;
  HRESULT hr = GetThreadDescription(GetCurrentThread(), &str);
  if (SUCCEEDED(hr))
  {
    ret = w2cstr(reinterpret_cast<wchar_t**>(&str));
    LocalFree(str);
  }
  return ret;
#elif defined(MRPT_OS_LINUX) || defined(MRPT_OS_APPLE)
  return GetThreadName();
#else
  return std::string("");
#endif
}

std::string mrpt::system::thread_name(std::thread& theThread)
{
#if defined(MRPT_OS_WINDOWS) && !defined(__MINGW32_MAJOR_VERSION)
  std::string ret = "NoName";
  PWSTR str;
  HRESULT hr = GetThreadDescription(theThread.native_handle(), &str);
  if (SUCCEEDED(hr))
  {
    ret = w2cstr(reinterpret_cast<wchar_t**>(&str));
    LocalFree(str);
  }
  return ret;
#elif defined(MRPT_OS_LINUX) || defined(MRPT_OS_APPLE)
  return GetThreadName(theThread);
#else
  return std::string("");
#endif
}
