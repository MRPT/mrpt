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
#ifndef CMyRedirector_H
#define CMyRedirector_H

#include <wx/string.h>
#include <wx/textctrl.h>
#include <wx/app.h>
#include <wx/thread.h>
#include <streambuf>
#include <iostream>

/** This auxiliary class redirects the output sent to a streambuf to a wxTextCtrl object.
  *  Uses code from http://www.devmaster.net/forums/showthread.php?t=7037
  *  Jose Luis Blanco - Dec 2007
  *  NOTE (10-Aug-2009): Added thread-safe support:
  *    We cannot write in a wxTextCtrl from a thread different than the main wx one,
  *    so if this object will be used by several threads, set "m_threadSafe" to true.
  *    In this mode, the object will NEVER write the text to the text control, unless
  *    the method "dumpNow()" is explicitly called FROM THE MAIN THREAD.
  */
class CMyRedirector : public std::streambuf
{
protected:
	wxTextCtrl	*m_txt;
	std::streambuf *sbOld;
	std::streambuf *sbOldErr;
	const bool           m_yieldApplication;
	const bool           m_also_cerr;
	const bool				m_threadSafe;
	const bool     m_also_to_cout_cerr;

	wxCriticalSection	m_cs;
	std::string			m_strbuf;

public:
	CMyRedirector(
		wxTextCtrl	*obj,
		bool yieldApplication = false,
		int bufferSize = 3000,
		bool also_cerr = false,
		bool threadSafe = false,
		bool also_to_cout_cerr = false ) : m_txt(obj), m_yieldApplication(yieldApplication), m_also_cerr(also_cerr),m_threadSafe(threadSafe), m_also_to_cout_cerr(also_to_cout_cerr)
    {
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

		if (m_also_cerr)
		{
			sbOldErr = std::cerr.rdbuf();
			std::cerr.rdbuf( this );
		}
    }
    virtual ~CMyRedirector()
    {
        sync();

		// Restore normal output:
        std::cout.rdbuf(sbOld);

        if (m_also_cerr)
			std::cerr.rdbuf(sbOldErr);

        delete[] pbase();
    }

	void flush()
	{
		sync();
	}

    virtual void writeString(const std::string &str)
	{
		if(!m_threadSafe)
		{
#ifdef wxUSE_UNICODE
			*m_txt  << wxString( str.c_str(), wxConvUTF8 );
#else
			*m_txt  << _U( str.c_str() );
#endif
		}
		else
		{	// Critical section is already adquired.
			m_strbuf+=str;
		}
		if (m_also_to_cout_cerr) ::printf("%s",str.c_str());
		if (m_yieldApplication)  wxTheApp->Yield();  // Let the app. process messages
	}

	/** Writes all the stored strings to the text control (only for threadSafe mode).
	    CALL THIS METHOD FROM THE MAIN THREAD!
	    */
	void dumpNow()
	{
		wxCriticalSectionLocker  lock(m_cs);

		if (!m_strbuf.empty())
		{
			if (m_also_to_cout_cerr) ::printf("%s",m_strbuf.c_str());
#ifdef wxUSE_UNICODE
			*m_txt  << wxString( m_strbuf.c_str(), wxConvUTF8 );
#else
			*m_txt  << _U( m_strbuf.c_str() );
#endif
			m_strbuf.clear();
		}
	}

private:
    int	overflow(int c)
    {
        sync();

        if (c != EOF)
        {
			wxCriticalSectionLocker  lock(m_cs);
            if (pbase() == epptr())
            {
                std::string temp;
                temp += char(c);
                writeString(temp);
            }
            else
                sputc(c);
        }

        return 0;
    }

    int	sync()
    {
		wxCriticalSectionLocker  lock(m_cs);

        if (pbase() != pptr())
        {
            int len = int(pptr() - pbase());
            std::string temp(pbase(), len);
            writeString(temp);
            setp(pbase(), epptr());
        }
        return 0;
    }
};

#endif
