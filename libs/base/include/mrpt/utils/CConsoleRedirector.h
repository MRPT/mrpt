/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                                 |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
   +---------------------------------------------------------------------------+ */
#ifndef CConsoleRedirector_H
#define CConsoleRedirector_H

#include <streambuf>
#include <iostream>


namespace mrpt
{
	namespace utils
	{
		/** By creating an object of this class, all the output to std::cout (and std::cerr) will be redirected to a text file, and optionally also shown on the console.
		  *  Based on code from http://www.devmaster.net/forums/showthread.php?t=7037
		 * \ingroup mrpt_base_grp
		  */
		class CConsoleRedirector : public std::streambuf
		{
		protected:
			std::ofstream 	m_of;		//!< The text output file stream.
			std::streambuf 	*sbOld;		//!< The "old" std::cout
			std::streambuf 	*sbOld_cerr;		//!< The "old" std::cout
			bool 			m_also_to_console;
			mrpt::synch::CCriticalSection	m_cs;

		public:
			/** Constructor
			  * \param out_file The file to create / append
			  * \param also_to_console Whether to redirect data to file *and* also dump data to the console as usual.
			  * \param append_file If set to false the file will be truncated on open
			  * \param bufferSize It's recommended to buffer the data instead of writing characters one by one.
			  * \param also_cerr Whether to redirect the output to std::cerr in addition to std::cout.
			  * \exception std::exception If the file cannot be opened.
			  */
			CConsoleRedirector(
				const std::string &out_file,
				bool also_to_console=true,
				bool also_cerr = true,
				bool append_file = false,
				int bufferSize = 1000 ) : m_of(), sbOld(NULL),sbOld_cerr(NULL),m_also_to_console(also_to_console), m_cs()
			{
				// Open the file:
				std::ios_base::openmode  openMode = std::ios_base::binary | std::ios_base::out;
				if ( append_file ) openMode |= std::ios_base::app;
				m_of.open(out_file.c_str(),  openMode );
				if (!m_of.is_open()) THROW_EXCEPTION_CUSTOM_MSG1("Error opening file: %s",out_file.c_str())

				if (bufferSize)
				{
					char *ptr = new char[bufferSize];
					setp(ptr, ptr + bufferSize);
				}
				else
					setp(0, 0);

				// Redirect:
				sbOld = std::cout.rdbuf();
				std::cout.rdbuf( this );

				if (also_cerr)
				{
					sbOld_cerr = std::cerr.rdbuf();
					std::cerr.rdbuf( this );
				}
			}

			virtual ~CConsoleRedirector()
			{
				sync();
				// Restore normal output:
				std::cout.rdbuf(sbOld);
				if (sbOld_cerr!=NULL) std::cerr.rdbuf( sbOld_cerr );
				if (pbase()) delete[] pbase();
			}

			void flush()
			{
				sync();
			}

			virtual void writeString(const std::string &str)
			{
				if (m_also_to_console) sbOld->sputn(str.c_str(),str.size());
				m_of << str;
			}

		private:
			int	overflow(int c)
			{
				sync();

				m_cs.enter();
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

				m_cs.leave();
				return 0;
			}

			int	sync()
			{
				m_cs.enter();
				if (pbase() != pptr())
				{
					int len = int(pptr() - pbase());
					std::string temp(pbase(), len);
					writeString(temp);
					setp(pbase(), epptr());
				}
				m_cs.leave();
				return 0;
			}
		};

	} // end namespace
} // end namespace

#endif
