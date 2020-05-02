
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

#include "journalfile.h"

/*!	\class JournalFile
	\brief A class containing a journal file and some meta-data
	\details These objects are managed by Journaller through gJournalFileMap.
*/

/*! \var volatile std::atomic_int JournalFile::m_refCount
	\brief A reference counter that tracks how many Journaller objects use this file
*/

/*! \brief Constructor, requires a filename
	\details
	\param name The (path and) filename of the log file to be used
	\param purge When set to true (default) the file will be cleared when opened
*/
JournalFile::JournalFile(const XsString& name, bool purge)
	: m_refCount(1)
	, m_filename(name)
{
	if (purge || (m_file.openText(name, false) != XRV_OK))
		m_file.createText(name, false);
	if (m_file.isOpen())
		m_file.seek_r(0);
}

/*! \brief Destructor, flushes remaining data and closes the file */
JournalFile::~JournalFile()
{
	try
	{
		flush();
		m_file.close();
	}
	catch(...)
	{
	}
}

/*! \brief Flush remaining data to disk */
void JournalFile::flush()
{
	m_file.flush();
}

/*! \brief Increase reference count of JournalFile by 1
	\return The new ref count value
*/
int JournalFile::addRef()
{
	return ++m_refCount;
}

/*! \brief Returns the current ref count value
	\return The ref count value
*/
int JournalFile::refCount() volatile const
{
	return m_refCount.load();
}

/*! \brief Decrease reference count of JournalFile by 1
	\return The new ref count value
*/
int JournalFile::removeRef()
{
	return --m_refCount;
}

/*! \brief Returns the (path +) filename of the open file */
XsString JournalFile::filename() const
{
	return m_filename;
}

/*! \brief Appends \a msg to the end of the current data stream */
JournalFile& JournalFile::operator<<(std::string const& msg)
{
	if (m_file.isOpen())
		m_file.write(msg.c_str(), (XsFilePos) sizeof(char), (XsFilePos) msg.length());
	return *this;
}
