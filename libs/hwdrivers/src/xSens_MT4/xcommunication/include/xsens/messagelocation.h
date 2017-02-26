/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef MESSAGELOCATION_H
#define MESSAGELOCATION_H

/*! \brief Stores the location of a message in a buffer using a start position and a size
*/
class MessageLocation {
public:
	int m_startPos;	//!< The offset of the first byte of the message or -1 if no message
	int m_size;		//!< The size of the message, when less than 0 it indicates the expected message size

	/*! \brief Constructor, initializes by default to an invalid message
		\param start The offset of the first byte of the message
		\param size The size of the message
		\sa isValid() \sa m_startPos \sa m_size \sa IProtocolHandler::findMessage
	*/
	MessageLocation(int start = -1, int size = 0)
		: m_startPos(start)
		, m_size(size)
	{}

	/*! \brief Returns whether the stored message information describes a valid message
	*/
	inline bool isValid() const
	{
		return m_startPos >= 0 && m_size > 0;
	}
};

#endif	// file guard
