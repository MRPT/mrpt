/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "system-precomp.h"  // Precompiled headers
//
#include <mrpt/config.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/system/thread_name.h>

#if defined(MRPT_OS_WINDOWS)
#include <windows.h>

#include <cwchar>
#include <vector>

#elif defined(MRPT_OS_LINUX)
#include <sys/prctl.h>

static void SetThreadName(std::thread& thread, const char* threadName)
{
	auto handle = thread.native_handle();
	pthread_setname_np(handle, threadName);
}

static std::string GetThreadName(std::thread& thread)
{
	auto handle = thread.native_handle();
	char buf[1000];
	buf[0] = '\0';
	pthread_getname_np(handle, buf, sizeof(buf));
	return std::string(buf);
}

static void SetThreadName(const char* threadName)
{
	prctl(PR_SET_NAME, threadName, 0L, 0L, 0L);
}
static std::string GetThreadName()
{
	char buf[100] = {0};
	prctl(PR_GET_NAME, buf, 0L, 0L, 0L);
	return std::string(buf);
}
#endif

void mrpt::system::thread_name(const std::string& name)
{
#if defined(MRPT_OS_WINDOWS)
	wchar_t wName[50];
	std::mbstowcs(wName, name.c_str(), sizeof(wName) / sizeof(wName[0]));
	SetThreadDescription(GetCurrentThread(), wName);
#elif defined(MRPT_OS_LINUX)
	SetThreadName(name.c_str());
#endif
}

void mrpt::system::thread_name(const std::string& name, std::thread& theThread)
{
#if defined(MRPT_OS_WINDOWS)
	wchar_t wName[50];
	std::mbstowcs(wName, name.c_str(), sizeof(wName) / sizeof(wName[0]));
	SetThreadDescription(theThread.native_handle(), wName);
#elif defined(MRPT_OS_LINUX)
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
#if defined(MRPT_OS_WINDOWS)
	std::string ret = "NoName";
	PWSTR str;
	HRESULT hr = GetThreadDescription(GetCurrentThread(), &str);
	if (SUCCEEDED(hr))
	{
		ret = w2cstr(reinterpret_cast<wchar_t**>(&str));
		LocalFree(str);
	}
	return ret;
#elif defined(MRPT_OS_LINUX)
	return GetThreadName();
#else
	return std::string("");
#endif
}

std::string mrpt::system::thread_name(std::thread& theThread)
{
#if defined(MRPT_OS_WINDOWS)
	std::string ret = "NoName";
	PWSTR str;
	HRESULT hr = GetThreadDescription(theThread.native_handle(), &str);
	if (SUCCEEDED(hr))
	{
		ret = w2cstr(reinterpret_cast<wchar_t**>(&str));
		LocalFree(str);
	}
	return ret;
#elif defined(MRPT_OS_LINUX)
	return GetThreadName(theThread);
#else
	return std::string("");
#endif
}
