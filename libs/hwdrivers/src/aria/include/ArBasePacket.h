/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef ARBASEPACKET_H
#define ARBASEPACKET_H


#include <string>
#include "ariaTypedefs.h"

/// Base packet class
/** This class is a base class for all packets... most software will never
    need to use this class, it is there mostly to help people do more advanced
    client and server communications.

    All of the functions are virtual so it can be completely overridden if
    desired... but the few most likely ones to be overridden are empty and
    makeFinal...

    The theory of the packet works like this, the packet has a buffer,
    headerLength, footer length, readLength, length, and a maxLength.
    When the packet is initialized it is given a buffer and its
    maxLength.  All of the functions that are somethingToBuf put data
    in at the current length of the packet, and advance the length.
    All of the functions that do bufToSomething get the data from
    where readLength points, and advance read length.  If
    bufToSomething would go beyond the data length of the packet it
    returns a 0 (note that this includes if it goes into the footer
    length).  resetRead sets readLength back to the header (since no
    one outside of the person who writes the class should touch the
    header).  empty likewise sets the length back to the header since
    the header will be calculated in the finalizePacket method.



    The base class and most classes of this kind will have an integer before
    the string, denoting the strings length... this is hidden by the function
    calls, but something someone may want to be aware of... it should not
    matter much as this same packet class should be used on both sides.

    Uses of this class that don't get newed and deleted a lot can just go
    ahead and use the constructor with buf = NULL, as this will have the
    packet manage its own memory, making life easier.

*/
class ArBasePacket
{
public:

  /// Constructor
  AREXPORT ArBasePacket(ArTypes::UByte2 bufferSize = 0,
    ArTypes::UByte2 headerLength = 0,
    char * buf = NULL,
    ArTypes::UByte2 footerLength = 0);

  /// Destructor
  AREXPORT virtual ~ArBasePacket();

  /// resets the length for more data to be added
  AREXPORT virtual void empty(void);

  /// MakeFinals the packet in preparation for sending, must be done
  /*AREXPORT*/ virtual void finalizePacket(void) {}

  /// ArLogs the contents of the packet
  AREXPORT virtual void log(void);
  /// ArLogs the contents of the packet in hex
  AREXPORT virtual void printHex(void);

  /// Returns whether the packet is valid, i.e. no error has occurred when reading/writing.
  AREXPORT virtual bool isValid(void);

  /// Resets the valid state of the packet.
  AREXPORT virtual void resetValid();

  // Utility functions to write different data types to a buffer. They will
  // increment the length.

  /// Puts ArTypes::Byte into packets buffer
  AREXPORT virtual void byteToBuf(ArTypes::Byte val);
  /// Puts ArTypes::Byte2 into packets buffer
  AREXPORT virtual void byte2ToBuf(ArTypes::Byte2 val);
  /// Puts ArTypes::Byte4 into packets buffer
  AREXPORT virtual void byte4ToBuf(ArTypes::Byte4 val);

  /// Puts ArTypes::UByte into packets buffer
  AREXPORT virtual void uByteToBuf(ArTypes::UByte val);
  /// Puts ArTypes::UByte2 into packet buffer
  AREXPORT virtual void uByte2ToBuf(ArTypes::UByte2 val);
  /// Puts ArTypes::UByte 4 into packet buffer
  AREXPORT virtual void uByte4ToBuf(ArTypes::UByte4 val);

  /// Puts a NULL-terminated string into packet buffer
  AREXPORT virtual void strToBuf(const char *str);

  /**
   * @brief Copies the given number of bytes from str into packet buffer
   * @deprecated use strToBufPadded instead
  **/
  AREXPORT virtual void strNToBuf(const char *str, int length);
  /// Copies length bytes from str, if str ends before length, pads data with 0s
  AREXPORT virtual void strToBufPadded(const char *str, int length);
  /// Copies length bytes from data into packet buffer
  AREXPORT virtual void dataToBuf(const char *data, int length);
  /// Copies length bytes from data into packet buffer
  AREXPORT virtual void dataToBuf(const unsigned char *data, int length);

  // Utility functions to read differet data types from a bufer. Each read
  // will increment the myReadLength.
  /// Gets a ArTypes::Byte from the buffer
  AREXPORT virtual ArTypes::Byte bufToByte(void);
  /// Gets a ArTypes::Byte2 from the buffer
  AREXPORT virtual ArTypes::Byte2 bufToByte2(void);
  /// Gets a ArTypes::Byte4 from the buffer
  AREXPORT virtual ArTypes::Byte4 bufToByte4(void);

  /// Gets a ArTypes::UByte from the buffer
  AREXPORT virtual ArTypes::UByte bufToUByte(void);
  /// Gets a ArTypes::UByte2 from the buffer
  AREXPORT virtual ArTypes::UByte2 bufToUByte2(void);
  /// Gets a ArTypes::UByte4 from the buffer
  AREXPORT virtual ArTypes::UByte4 bufToUByte4(void);

  /// Gets a string from the buffer
  AREXPORT virtual void bufToStr(char *buf, int len);
  /// Gets length bytes from buffer and puts them into data
  AREXPORT virtual void bufToData(char * data, int length);
  /// Gets length bytes from buffer and puts them into data
  AREXPORT virtual void bufToData(unsigned char * data, int length);

  /// Restart the reading process
  AREXPORT virtual void resetRead(void);

  // Accessors

  /// Gets the total length of the packet
  /*AREXPORT*/ virtual ArTypes::UByte2 getLength(void) { return myLength; }
  /// Gets the length of the data in the packet
  /*AREXPORT*/ virtual ArTypes::UByte2 getDataLength(void) { return myLength - myHeaderLength - myFooterLength; }

  /// Gets how far into the packet that has been read
  /*AREXPORT*/ virtual ArTypes::UByte2 getReadLength(void) { return myReadLength; }
  /// Gets how far into the data of the packet that has been read
  /*AREXPORT*/ virtual ArTypes::UByte2 getDataReadLength(void) { return myReadLength - myHeaderLength; }
  /// Gets the length of the header
  /*AREXPORT*/ virtual ArTypes::UByte2 getHeaderLength(void)
  { return myHeaderLength; }
  /// Gets the length of the header
  /*AREXPORT*/virtual ArTypes::UByte2 getFooterLength(void)
  { return myFooterLength; }

  /// Gets the maximum length packet
  /*AREXPORT*/ virtual ArTypes::UByte2 getMaxLength(void) { return myMaxLength; }

  /// Gets a pointer to the buffer the packet uses
  AREXPORT virtual const char * getBuf(void);

  /// Sets the buffer the packet is using
  AREXPORT virtual void setBuf(char *buf, ArTypes::UByte2 bufferSize);
  /// Sets the maximum buffer size (if new size is <= current does nothing)
  AREXPORT virtual void setMaxLength(ArTypes::UByte2 bufferSize);
  /// Sets the length of the packet
  AREXPORT virtual bool setLength(ArTypes::UByte2 length);
  /// Sets the read length
  AREXPORT virtual void setReadLength(ArTypes::UByte2 readLength);
  /// Sets the length of the header
  AREXPORT virtual bool setHeaderLength(ArTypes::UByte2 length);
  /// Makes this packet a duplicate of another packet
  AREXPORT virtual void duplicatePacket(ArBasePacket *packet);
protected:
  // internal function to make sure we have enough length left to read in the packet
  AREXPORT bool isNextGood(int bytes);

  /// Returns true if there is enough room in the packet to add the specified number of bytes
  AREXPORT bool hasWriteCapacity(int bytes);

  // internal data
  ArTypes::UByte2 myHeaderLength;
  ArTypes::UByte2 myFooterLength;
  ArTypes::UByte2 myMaxLength;

  ArTypes::UByte2 myReadLength;
  bool myOwnMyBuf;

  // Actual packet data
  char *myBuf;
  ArTypes::UByte2 myLength;

  // Whether no error has occurred in reading/writing the packet.
  bool myIsValid;

};


#endif // ARPACKET_H
