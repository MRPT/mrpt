/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CMyRedirector_H
#define CMyRedirector_H

#include <wx/string.h>
#include <wx/textctrl.h>
#include <wx/app.h>
#include <wx/thread.h>
#include <streambuf>
#include <iostream>
#include <cstdio>
#include <functional>

#include "wx28-fixes.h"


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
			wxString s;
#ifdef wxUSE_UNICODE
			s = wxString(str.c_str(), wxConvUTF8);
#else
			s = _U(str.c_str());
#endif

#if wxCHECK_VERSION(3,0,0)
			m_txt->GetEventHandler()->CallAfter(&wxTextCtrl::WriteText, s);
#else
			m_txt->WriteText(s);  // bad solution, but at least compiles (and may work, unsafely) for old wx2.8 in Ubuntu precise
#endif
		}
		else
		{	// Critical section is already adquired.
			m_strbuf+=str;
		}
		if (m_also_to_cout_cerr) ::printf("%s",str.c_str());
		if (m_yieldApplication && wxThread::IsMain())  wxTheApp->Yield(true);  // Let the app. process messages
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
