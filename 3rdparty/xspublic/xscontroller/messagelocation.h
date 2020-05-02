
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

#ifndef MESSAGELOCATION_H
#define MESSAGELOCATION_H

/*! \brief Stores the location of a message in a buffer using a start position and a size
*/
class MessageLocation {
public:
	int m_startPos;	//!< The offset of the first byte of the message or -1 if no message
	int m_size;		//!< The size of the message, when less than 0 it indicates the expected message size

	/*! The offset of the first incomplete potentially eventually valid message that has been
		skipped to reach the first valid and complete message at m_startPos.
		If no valid complete message has been found (m_startPos == -1), m_incompletePos still
		contains the offset of the first incomplete message.
		invariant: m_incompletePos == -1 || m_startPos == -1 || m_incompletePos < m_startPos
	*/
	int m_incompletePos;

	/*! The expected size of the first incomplete potentially eventually valid message that has been
		skipped to reach the first valid and complete message at m_startPos.
		If a valid complete message has been found at a non-0 start position and m_incompletePos is not -1,
		m_incompleteSize will contain the expected size of the incomplete message.
	*/
	int m_incompleteSize;

	/*! \brief Constructor, initializes by default to an invalid message
		\param start The offset of the first byte of the message
		\param size The size of the message
		\param incompletePos The offset of the first byte of an incomplete but potentially eventually valid message
		\param incompleteSize The expected size of the message at \a incompletePos, if that is not -1
		\sa isValid() \sa m_startPos \sa m_size \sa IProtocolHandler::findMessage
	*/
	MessageLocation(int start = -1, int size = 0, int incompletePos = -1, int incompleteSize = 0)
		: m_startPos(start)
		, m_size(size)
		, m_incompletePos(incompletePos)
		, m_incompleteSize(incompleteSize)
	{}

	/*! \brief Returns whether the stored message information describes a valid message
	*/
	inline bool isValid() const
	{
		return m_startPos >= 0 && m_size > 0;
	}
};

#endif
