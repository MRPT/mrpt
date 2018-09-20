/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "gui-precomp.h"  // Precompiled headers

#include "CAboutBoxBase.h"
#include <mrpt/config.h>
#include <mrpt/system/os.h>
#include <mrpt/core/format.h>
#include <mrpt/math/types_math.h>  // for Eigen version

using namespace mrpt;
using namespace mrpt::system;

CAboutBoxBase::CAboutBoxBase(
	const std::string& appName, const std::string& additionalInfo,
	const bool showStandardInfo)
	: m_appName(appName),
	  m_additionalInfo(additionalInfo),
	  m_showStandardInfo(showStandardInfo)
{
}

CAboutBoxBase::~CAboutBoxBase() = default;
std::string CAboutBoxBase::MRPTBuildVersion() const
{
	std::string s("Build: ");
	s += MRPT_getVersion();
	s += " ";
	s += mrpt::system::MRPT_getCompilationDate();
	return s;
}

std::string CAboutBoxBase::tutorial() const
{
	return "Up to date documentation and tutorials are maintained at the MRPT "
		   "website:\n\nhttp://www.mrpt.org/\n\n\n\n";
}

std::string CAboutBoxBase::license() const
{
	return mrpt::system::getMRPTLicense();
}

std::string CAboutBoxBase::information(
	const std::string& guiLibName, const std::string& guiLibVersion) const
{
	std::string str = m_appName + "\n";
	str +=
		"----------------------------------\n"
		"Part of the MRPT project.\n"
		"For bug reports and source code, visit:"
		" https://github.com/MRPT/mrpt \n\n";
	;
	if (!m_additionalInfo.empty())
	{
		str += m_additionalInfo + "\n";
	}
	str += "MRPT version:           " + MRPT_getVersion() + "\n";
	str += "MRPT source timestamp:  " + MRPT_getCompilationDate() + "\n";

	if (m_showStandardInfo)
	{
		str += "Eigen version:          ";
		str += mrpt::format(
			"%u.%u.%u\n", EIGEN_WORLD_VERSION, EIGEN_MAJOR_VERSION,
			EIGEN_MINOR_VERSION);
		str += guiLibName + " version:      " + guiLibVersion;
#if defined(__WXMSW__)
		str += "-Windows";
#elif defined(__UNIX__)
		str += "-Linux";
#endif
#if wxUSE_UNICODE
		str += "-Unicode build";
#else
		str += "-ANSI build";
#endif  // wxUSE_UNICODE

		str += "\nOpenCV version:         ";
#if MRPT_HAS_OPENCV
		str += MRPT_OPENCV_VERSION;
		str += "\n";
#else
		str += "None";
		str += "\n";
#endif
	}

	return str;
}
