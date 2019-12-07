/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "io-precomp.h"  // Precompiled headers

#include <mrpt/core/exceptions.h>
#include <mrpt/io/CTextFileLinesParser.h>
#include <mrpt/system/string_utils.h>
#include <sstream>

using namespace mrpt::io;
using namespace std;

CTextFileLinesParser::CTextFileLinesParser(const std::string& fil)
{
	open(fil);
}

CTextFileLinesParser::CTextFileLinesParser(std::istream& in) { open(in); }
void CTextFileLinesParser::open(std::istream& in)
{
	m_curLineNum = 0;
	m_fileName = "{std::istream}";
	this->close();
	m_in = &in;
}

void CTextFileLinesParser::open(const std::string& fil)
{
	m_curLineNum = 0;
	m_fileName = fil;
	this->close();
	auto ifs = std::make_shared<std::ifstream>();
	ifs->clear();
	ifs->open(fil.c_str());
	if (!ifs->is_open())
		THROW_EXCEPTION_FMT("Error opening file '%s' for reading", fil.c_str());
	m_my_in = std::shared_ptr<std::istream>(ifs);
	m_in = m_my_in.get();
}

void CTextFileLinesParser::close()
{
	if (!m_in) return;
	m_my_in.reset();
	m_in = nullptr;
}
void CTextFileLinesParser::rewind()
{
	m_curLineNum = 0;
	m_in->clear();
	m_in->seekg(0);
}

bool CTextFileLinesParser::getNextLine(std::string& out_str)
{
	std::istringstream buf;
	if (getNextLine(buf))
	{
		out_str = buf.str();
		return true;
	}
	out_str.clear();
	return false;
}

bool CTextFileLinesParser::getNextLine(std::istringstream& buf)
{
	ASSERT_(m_in != nullptr);
	while (!m_in->fail())
	{
		std::string lin;
		std::getline(*m_in, lin);
		m_curLineNum++;
		lin = mrpt::system::trim(lin);
		if (lin.empty()) continue;  // Ignore empty lines.
		// Ignore comments lines, starting with "#" or "//".
		if ((m_filter_SH_comments && mrpt::system::strStarts(lin, "#")) ||
			(m_filter_C_comments && mrpt::system::strStarts(lin, "//")) ||
			(m_filter_MATLAB_comments && mrpt::system::strStarts(lin, "%")))
			continue;
		// Parse the line as a string stream:
		buf.str(lin);
		buf.clear();
		return true;
	};
	return false;
}

size_t CTextFileLinesParser::getCurrentLineNumber() const
{
	return m_curLineNum;
}

void CTextFileLinesParser::enableCommentFilters(
	bool filter_MATLAB_comments, bool filter_C_comments,
	bool filter_SH_comments)
{
	m_filter_MATLAB_comments = filter_MATLAB_comments;
	m_filter_C_comments = filter_C_comments;
	m_filter_SH_comments = filter_SH_comments;
}
