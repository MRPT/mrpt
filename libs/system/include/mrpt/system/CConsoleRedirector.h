/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <streambuf>
#include <iostream>
#include <fstream>
#include <cstdio>  // EOF

namespace mrpt::system
{
/** By creating an object of this class, all the output to std::cout (and
 * std::cerr) will be redirected to a text file, and optionally also shown on
 * the console.
 *  Based on code from http://www.devmaster.net/forums/showthread.php?t=7037
 * \ingroup mrpt_base_grp
 */
class CConsoleRedirector : public std::streambuf
{
   protected:
	/** The text output file stream. */
	std::ofstream m_of;
	/** The "old" std::cout */
	std::streambuf* sbOld;
	/** The "old" std::cout */
	std::streambuf* sbOld_cerr;
	bool m_also_to_console;
	std::mutex m_cs;

   public:
	/** Constructor
	 * \param out_file The file to create / append
	 * \param also_to_console Whether to redirect data to file *and* also dump
	 * data to the console as usual.
	 * \param append_file If set to false the file will be truncated on open
	 * \param bufferSize It's recommended to buffer the data instead of writing
	 * characters one by one.
	 * \param also_cerr Whether to redirect the output to std::cerr in addition
	 * to std::cout.
	 * \exception std::exception If the file cannot be opened.
	 */
	CConsoleRedirector(
		const std::string& out_file, bool also_to_console = true,
		bool also_cerr = true, bool append_file = false, int bufferSize = 1000)
		: m_of(),
		  sbOld(nullptr),
		  sbOld_cerr(nullptr),
		  m_also_to_console(also_to_console),
		  m_cs()
	{
		// Open the file:
		std::ios_base::openmode openMode =
			std::ios_base::binary | std::ios_base::out;
		if (append_file) openMode |= std::ios_base::app;
		m_of.open(out_file.c_str(), openMode);
		if (!m_of.is_open())
			THROW_EXCEPTION_FMT("Error opening file: %s", out_file.c_str())

		if (bufferSize)
		{
			char* ptr = new char[bufferSize];
			setp(ptr, ptr + bufferSize);
		}
		else
			setp(nullptr, nullptr);

		// Redirect:
		sbOld = std::cout.rdbuf();
		std::cout.rdbuf(this);

		if (also_cerr)
		{
			sbOld_cerr = std::cerr.rdbuf();
			std::cerr.rdbuf(this);
		}
	}

	~CConsoleRedirector() override
	{
		sync();
		// Restore normal output:
		std::cout.rdbuf(sbOld);
		if (sbOld_cerr != nullptr) std::cerr.rdbuf(sbOld_cerr);
		if (pbase()) delete[] pbase();
	}

	void flush() { sync(); }
	virtual void writeString(const std::string& str)
	{
		if (m_also_to_console) sbOld->sputn(str.c_str(), str.size());
		m_of << str;
	}

   private:
	int overflow(int c) override
	{
		sync();

		m_cs.lock();
		if (c != EOF)
		{
			if (pbase() == epptr())
			{
				std::string temp;
				temp += char(c);
				writeString(temp);
			}
			else
				sputc(c);
		}

		m_cs.unlock();
		return 0;
	}

	int sync() override
	{
		m_cs.lock();
		if (pbase() != pptr())
		{
			int len = int(pptr() - pbase());
			std::string temp(pbase(), len);
			writeString(temp);
			setp(pbase(), epptr());
		}
		m_cs.unlock();
		return 0;
	}
};
}  // namespace mrpt::system
