
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

#include "mtbdatalogger.h"
#include "iointerfacefile.h"
#include "protocolhandler.h"


/*! \class MtbDataLogger
	\brief A class for logging the mtb data
*/

/*! \brief Default constructor
*/
MtbDataLogger::MtbDataLogger() :
	m_lastResult(XRV_OK)
{
}

/*! \brief Default destructor
*/
MtbDataLogger::~MtbDataLogger()
{
	close(false);
}

/*! \brief Open a log file for output.
	\details This function opens the supplied log file for writing.
	\param filename The name of the file to open. It is recommended to use a fully qualified path+filename.
	\note This function is only available in configuration mode.
	\returns True if successful
	\see close
*/
bool MtbDataLogger::create(const XsString &filename)
{
	if (m_ioInterfaceFile)
	{
		m_lastResult = XRV_ALREADYOPEN;
		return false;
	}

	m_ioInterfaceFile = std::shared_ptr<IoInterfaceFile>(new IoInterfaceFile);
	m_lastResult = m_ioInterfaceFile->create(filename);
	if (m_lastResult != XRV_OK)
	{
		m_ioInterfaceFile.reset();
		return false;
	}

	//m_readOnly = false;

	// check if we can actually write to the file
	char testData[] = "Xsens";
	XsByteArray test((unsigned char*) testData, 5, XSDF_None);

	m_lastResult = m_ioInterfaceFile->writeData(test);
	if (m_lastResult == XRV_OK)
		m_lastResult = m_ioInterfaceFile->deleteData(0,5);
	if (m_lastResult != XRV_OK)
	{
		m_ioInterfaceFile->close();
		m_ioInterfaceFile.reset();
	}
	return m_lastResult == XRV_OK;
}

/*! \brief Closes the file
*/
void MtbDataLogger::close()
{
	close(false);
}

/*! \brief Closes and if requested deletes the file
	\param deleteFile If set to true then deletes the file
*/
void MtbDataLogger::close(bool deleteFile)
{
	if (m_ioInterfaceFile)
	{
		if (deleteFile)
			m_ioInterfaceFile->closeAndDelete();
		else
			m_ioInterfaceFile->close();
		//removeChainedManager(m_ioInterfaceFile);
		m_ioInterfaceFile.reset();
	}
}

/*! \brief Overloadable function to allow easier testing
*/
bool MtbDataLogger::writeMessage(const XsMessage &message)
{
	if (!m_ioInterfaceFile)
	{
		m_lastResult = XRV_NOFILEOPEN;
		return false;
	}

	XsByteArray raw;
	if (ProtocolHandler::composeMessage(raw, message) != -1)
		m_lastResult = m_ioInterfaceFile->writeData(raw);
	else
		m_lastResult = XRV_DATACORRUPT;

	return m_lastResult == XRV_OK;
}

/*! \returns the filename of the file that we're logging to (if any)
*/
XsString MtbDataLogger::filename() const
{
	if (!m_ioInterfaceFile)
		return XsString();
	return m_ioInterfaceFile->getFileName();
}
