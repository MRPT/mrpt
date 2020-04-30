
//  Copyright (c) 2003-2019 Xsens Technologies B.V. or subsidiaries worldwide.
//  All rights reserved.
//  
//  Redistribution and use in source and binary forms, with or without modification,
//  are permitted provided that the following conditions are met:
//  
//  1.	Redistributions of source code must retain the above copyright notice,
//  	this list of conditions, and the following disclaimer.
//  
//  2.	Redistributions in binary form must reproduce the above copyright notice,
//  	this list of conditions, and the following disclaimer in the documentation
//  	and/or other materials provided with the distribution.
//  
//  3.	Neither the names of the copyright holders nor the names of their contributors
//  	may be used to endorse or promote products derived from this software without
//  	specific prior written permission.
//  
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
//  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
//  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
//  THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
//  OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
//  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY OR
//  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.THE LAWS OF THE NETHERLANDS 
//  SHALL BE EXCLUSIVELY APPLICABLE AND ANY DISPUTES SHALL BE FINALLY SETTLED UNDER THE RULES 
//  OF ARBITRATION OF THE INTERNATIONAL CHAMBER OF COMMERCE IN THE HAGUE BY ONE OR MORE 
//  ARBITRATORS APPOINTED IN ACCORDANCE WITH SAID RULES.
//  

#ifndef XSSTRINGSTREAMING_H
#define XSSTRINGSTREAMING_H

#ifdef __cplusplus
#include "xsstring.h"
#include "xsvector.h"
#include "xsmatrix.h"
#include "xsquaternion.h"

#ifndef XSENS_NO_STL
#include <ostream>
namespace std
{
	template<typename _CharT, typename _Traits>
	basic_ostream<_CharT, _Traits>& operator<<(basic_ostream<_CharT, _Traits>& o, XsVector const& xv)
	{
		o << "V<" << xv.size() << ">(";
		for (XsSize i = 0; i < xv.size() - 1; i++)
			o << xv[i] << ", ";
		return (o << xv[xv.size() - 1] << ")");
	}

	template<typename _CharT, typename _Traits>
	basic_ostream<_CharT, _Traits>& operator<<(basic_ostream<_CharT, _Traits>& o, XsMatrix const& xm)
	{
		o << "M<" << xm.rows() << "," << xm.cols() << ">(";
		for (XsSize r = 0; r < xm.rows(); ++r)
		{
			for (XsSize c = 0; c < xm.cols() - 1; ++c)
				o << xm[r][c] << ", ";
			o << xm[r][xm.cols()-1];
			if (r < xm.rows() - 1)
				o << "\n";
		}
		o << ")";
		return o;
	}

	template<typename _CharT, typename _Traits>
	basic_ostream<_CharT, _Traits>& operator<<(basic_ostream<_CharT, _Traits>& o, XsQuaternion const& xq)
	{
		o << "Q(";
		for (int i = 0; i < 3; i++)
			o << xq[i] << ", ";
		return (o << xq[3] << ")");
	}
}
#endif

inline XsString& operator<<(XsString& o, XsSize const& v)
{
	char buffer[32];	// 2e64 = 1.8e19 so this should be enough
	sprintf(buffer, "%" PRINTF_SIZET_MODIFIER "u", v);
	o << XsString(buffer);
	return o;
}

inline XsString& operator<<(XsString& o, XsReal const& v)
{
	char buffer[64];
	sprintf(buffer, "%g", v);
	o << XsString(buffer);
	return o;
}

inline XsString& operator<<(XsString& o, XsVector const& xv)
{
	o << "V<" << xv.size() << ">(";
	for (XsSize i = 0; i < xv.size() - 1; i++)
		o << xv[i] << ", ";
	return (o << xv[xv.size() - 1] << ")");
}

inline XsString& operator<<(XsString& o, XsMatrix const& xm)
{
	o << "M<" << xm.rows() << "," << xm.cols() << ">(";
	for (XsSize r = 0; r < xm.rows(); ++r)
	{
		if (xm.cols())
		{
			if (r > 0)
				o << "\t";
			for (XsSize c = 0; c < xm.cols() - 1; ++c)
				o << xm[r][c] << ", ";
			o << xm[r][xm.cols()-1];
			if (r < xm.rows() - 1)
				o << "\n";
		}
	}
	o << ")";
	return o;
}

inline XsString& operator<<(XsString& o, XsQuaternion const& xq)
{
	o << "Q(";
	for (int i = 0; i < 3; i++)
		o << xq[i] << ", ";
	return (o << xq[3] << ")");
}

#endif

#endif
