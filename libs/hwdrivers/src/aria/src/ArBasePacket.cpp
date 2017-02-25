/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "ArExport.h"
#include "ariaOSDef.h"
#include "ArBasePacket.h"
#include "ArLog.h"

#include <stdlib.h>
#include <string.h>
#include <memory.h>
#include <stdio.h>

/**
@param bufferSize size of the buffer
@param headerLength length of the header
@param buf buffer packet uses, if NULL, instance will allocate memory
@param footerLength length of the footer following the data
*/
AREXPORT ArBasePacket::ArBasePacket(ArTypes::UByte2 bufferSize,
                                    ArTypes::UByte2 headerLength,
                                    char * buf,
                                    ArTypes::UByte2 footerLength)
{
  if (buf == NULL && bufferSize > 0)
  {
    myOwnMyBuf = true;
    myBuf = new char[bufferSize];
  }
  else
  {
    myOwnMyBuf = false;
    myBuf = buf;
  }
  myHeaderLength = headerLength;
  myFooterLength = footerLength;
  myReadLength = myHeaderLength;
  myMaxLength = bufferSize;
  myLength = myHeaderLength;
  myIsValid = true;
}

AREXPORT ArBasePacket::~ArBasePacket()
{
  if (myOwnMyBuf && myBuf != NULL)
    delete[] myBuf;
}


AREXPORT void ArBasePacket::setBuf(char *buf, ArTypes::UByte2 bufferSize)
{
  if (myOwnMyBuf)
  {
    delete[] myBuf;
    myOwnMyBuf = false;
  }
  myBuf = buf;
  myMaxLength = bufferSize;
}

AREXPORT void ArBasePacket::setMaxLength(ArTypes::UByte2 bufferSize)
{
  if (myMaxLength >= bufferSize)
    return;
  if (myOwnMyBuf)
  {
    delete[] myBuf;
    myOwnMyBuf = false;
  }
  myBuf = new char[bufferSize];
  myMaxLength = bufferSize;
  myOwnMyBuf = true;
}

AREXPORT bool ArBasePacket::setLength(ArTypes::UByte2 length)
{
  if (myOwnMyBuf && length > myMaxLength)
    return false;

  myLength = length;
  return true;
}

AREXPORT void ArBasePacket::setReadLength(ArTypes::UByte2 readLength)
{
  myReadLength = readLength;
}

AREXPORT bool ArBasePacket::setHeaderLength(ArTypes::UByte2 length)
{
  if (myOwnMyBuf && length > myMaxLength)
    return false;

  myHeaderLength = length;
  return true;
}

/**
Sets the length read back to the header length so the packet can be
reread using the other methods
*/

AREXPORT void ArBasePacket::resetRead(void)
{
  myReadLength = myHeaderLength;
  resetValid();
}

/**
Sets the packet length back to be the packets header length again
*/

AREXPORT void ArBasePacket::empty(void)
{
  myLength = myHeaderLength;
  resetValid();
}

AREXPORT bool ArBasePacket::isNextGood(int bytes)
{
  if (bytes <= 0)
    return false;

  // make sure it comes in before the header
  if (myReadLength + bytes <= myLength - myFooterLength)
    return true;

  myIsValid = false;

  return false;
}


AREXPORT bool ArBasePacket::hasWriteCapacity(int bytes)
{
  if (bytes < 0) {
    ArLog::log(ArLog::Normal, "ArBasePacket::hasWriteCapacity(%d) cannot write negative amount",
               bytes);
    return false;
  }

  // Make sure there's enough room in the packet
  if ((myLength + bytes) <= myMaxLength) {
     return true;
  }

  myIsValid = false;

  return false;

} // end method hasWriteCapacity


/**
 * A packet is considered "invalid" if an attempt is made to write too much
 * data into the packet, or to read too much data from the packet.  Calls to
 * empty() and resetRead() will restore the valid state.
**/
AREXPORT bool ArBasePacket::isValid(void)
{
  return myIsValid;

} // end method isValid

/**
 * Resets the packet to the "valid" state.  This method should generally
 * only be called externally when the application has taken some recovery
 * action.  For example, if an attempt to write a long string to the packet
 * fails (and isValid() returns false), then a smaller string may be written
 * instead.
**/
AREXPORT void ArBasePacket::resetValid()
{
  myIsValid = true;
}

AREXPORT const char *ArBasePacket::getBuf(void)
{
  return myBuf;
}

AREXPORT void ArBasePacket::byteToBuf(ArTypes::Byte val)
{
  if (!hasWriteCapacity(1)) {
    return;
  }

  memcpy(myBuf+myLength, &val, 1);
  myLength += 1;
}

AREXPORT void ArBasePacket::byte2ToBuf(ArTypes::Byte2 val)
{
  if (!hasWriteCapacity(2)) {
    return;
  }

  unsigned char c;
  c = (val >> 8) & 0xff;
  memcpy(myBuf+myLength+1, &c, 1);
  c = val & 0xff;
  memcpy(myBuf+myLength, &c, 1);
  myLength += 2;
}

AREXPORT void ArBasePacket::byte4ToBuf(ArTypes::Byte4 val)
{
  if (!hasWriteCapacity(4)) {
    return;
  }

  unsigned char c;
  c = (val >> 24) & 0xff;
  memcpy(myBuf+myLength+3, &c, 1);
  c = (val >> 16) & 0xff;
  memcpy(myBuf+myLength+2, &c, 1);
  c = (val >> 8) & 0xff;
  memcpy(myBuf+myLength+1, &c, 1);
  c = val & 0xff;
  memcpy(myBuf+myLength, &c, 1);
  myLength += 4;

}

AREXPORT void ArBasePacket::uByteToBuf(ArTypes::UByte val)
{
  if (!hasWriteCapacity(1)) {
    return;
  }
  memcpy(myBuf+myLength, &val, 1);
  myLength += 1;
}

AREXPORT void ArBasePacket::uByte2ToBuf(ArTypes::UByte2 val)
{
  if (!hasWriteCapacity(2)) {
    return;
  }
  unsigned char c;
  c = (val >> 8) & 0xff;
  memcpy(myBuf+myLength+1, &c, 1);
  c = val & 0xff;
  memcpy(myBuf+myLength, &c, 1);
  myLength += 2;
}

AREXPORT void ArBasePacket::uByte4ToBuf(ArTypes::UByte4 val)
{
  if (!hasWriteCapacity(4)) {
    return;
  }
  memcpy(myBuf+myLength, &val, 4);
  myLength += 4;
}

/**
@param str string to copy into buffer
*/
AREXPORT void ArBasePacket::strToBuf(const char *str)
{
  if (str == NULL) {
    str = "";
  }
  ArTypes::UByte2 tempLen = strlen(str) + 1;

  if (!hasWriteCapacity(tempLen)) {
    return;
  }

  memcpy(myBuf+myLength, str, tempLen);
  myLength += tempLen;
}

/**
 * This method performs no bounds checking on the given length and
 * the contents of the string.  For string operations, strNToBufPadded()
 * is preferred.  For raw data operations, dataToBuf() is preferred.
@param str character array to copy into the packet buffer
@param length how many characters to copy from str into the packet buffer
*/
AREXPORT void ArBasePacket::strNToBuf(const char *str, int length)
{
  // Do not perform bounds checking because it breaks existing code.

  //byte4ToBuf(length);
  memcpy(myBuf+myLength, str, length);
  myLength+=length;

}


/**
If string ends before length it pads the string with NUL ('\\0') characters.
@param str character array to copy into buffer
@param length how many bytes to copy from the str into packet
*/
AREXPORT void ArBasePacket::strToBufPadded(const char *str, int length)
{
  if (str == NULL) {
    str = "";
  }
  ArTypes::UByte2 tempLen = strlen(str);

  if (!hasWriteCapacity(length)) {
    return;
  }

  if (tempLen >= length) {
    memcpy(myBuf + myLength, str, length);
    myLength += length;
  }
  else // string is smaller than given length
  {
    memcpy(myBuf + myLength, str, tempLen);
    myLength += tempLen;
    memset(myBuf + myLength, 0, length - tempLen);
    myLength += length - tempLen;
  }
}


/**
@param data chacter array to copy into buffer
@param length how many bytes to copy from data into packet
*/
AREXPORT void ArBasePacket::dataToBuf(const char *data, int length)
{
  if (data == NULL) {
    ArLog::log(ArLog::Normal, "ArBasePacket::dataToBuf(NULL, %d) cannot add from null address",
               length);
    return;
  }

  if (!hasWriteCapacity(length)) {
    return;
  }

  memcpy(myBuf+myLength, data, length);
  myLength+=length;

}

/**
   This was added to get around having to cast data you put in, since the data shouldn't really matter if its signed or unsigned.
@param data chacter array to copy into buffer
@param length how many bytes to copy from data into packet
*/
AREXPORT void ArBasePacket::dataToBuf(const unsigned char *data, int length)
{
  if (data == NULL) {
    ArLog::log(ArLog::Normal, "ArBasePacket::dataToBuf(NULL, %d) cannot add from null address",
               length);
    return;
  }

  if (!hasWriteCapacity(length)) {
    return;
  }

  memcpy(myBuf+myLength, data, length);
  myLength+=length;

}


AREXPORT ArTypes::Byte ArBasePacket::bufToByte(void)
{
  ArTypes::Byte ret=0;

  if (isNextGood(1))
  {
    memcpy(&ret, myBuf+myReadLength, 1);
    myReadLength+=1;
  }

  return(ret);
}

AREXPORT ArTypes::Byte2 ArBasePacket::bufToByte2(void)
{
  ArTypes::Byte2 ret=0;
  unsigned char c1, c2;

  if (isNextGood(2))
  {
    memcpy(&c1, myBuf+myReadLength, 1);
    memcpy(&c2, myBuf+myReadLength+1, 1);
    ret = (c1 & 0xff) | (c2 << 8);
    myReadLength+=2;
  }

  return ret;
}

AREXPORT ArTypes::Byte4 ArBasePacket::bufToByte4(void)
{
  ArTypes::Byte4 ret=0;
  unsigned char c1, c2, c3, c4;

  if (isNextGood(4))
  {
    memcpy(&c1, myBuf+myReadLength, 1);
    memcpy(&c2, myBuf+myReadLength+1, 1);
    memcpy(&c3, myBuf+myReadLength+2, 1);
    memcpy(&c4, myBuf+myReadLength+3, 1);
    ret = (c1 & 0xff) | (c2 << 8) | (c3 << 16) | (c4 << 24);
    myReadLength+=4;
  }

  return ret;
}

AREXPORT ArTypes::UByte ArBasePacket::bufToUByte(void)
{
  ArTypes::UByte ret=0;

  if (isNextGood(1))
  {
    memcpy(&ret, myBuf+myReadLength, 1);
    myReadLength+=1;
  }

  return(ret);
}

AREXPORT ArTypes::UByte2 ArBasePacket::bufToUByte2(void)
{
  ArTypes::UByte2 ret=0;
  unsigned char c1, c2;

  if (isNextGood(2))
  {
    memcpy(&c1, myBuf+myReadLength, 1);
    memcpy(&c2, myBuf+myReadLength+1, 1);
    ret = (c1 & 0xff) | (c2 << 8);
    myReadLength+=2;
  }

  return ret;
}

AREXPORT ArTypes::UByte4 ArBasePacket::bufToUByte4(void)
{
  ArTypes::Byte4 ret=0;
  unsigned char c1, c2, c3, c4;

  if (isNextGood(4))
  {
    memcpy(&c1, myBuf+myReadLength, 1);
    memcpy(&c2, myBuf+myReadLength+1, 1);
    memcpy(&c3, myBuf+myReadLength+2, 1);
    memcpy(&c4, myBuf+myReadLength+3, 1);
    ret = (c1 & 0xff) | (c2 << 8) | (c3 << 16) | (c4 << 24);
    myReadLength+=4;
  }

  return ret;
}

/**
Copy a string from the packet buffer into the given buffer, stopping when
the end of the packet buffer is reached, the given length is reached,
or a NUL character ('\\0') is reached.  If the given length is not large
enough, then the remainder of the string is flushed from the packet.
@param buf Destination buffer
@param len Maximum number of characters to copy into the destination buffer
*/
AREXPORT void ArBasePacket::bufToStr(char *buf, int len)
{
   if (buf == NULL) {
    ArLog::log(ArLog::Normal, "ArBasePacket::bufToStr(NULL, %d) cannot write to null address",
               len);
    return;
  }

  int i;

  buf[0] = '\0';
  // see if we can read
  if (isNextGood(1))
  {
    // while we can read copy over those bytes
    for (i = 0;
         isNextGood(1) && i < (len - 1) && myBuf[myReadLength] != '\0';
         ++myReadLength, ++i) {
      buf[i] = myBuf[myReadLength];
    }
    // if we stopped because of a null then copy that one too
    if (myBuf[myReadLength] == '\0')
    {
      buf[i] = myBuf[myReadLength];
      myReadLength++;
    }
    else if (i >= (len - 1)) {

      // Otherwise, if we stopped reading because the output buffer was full,
      // then attempt to flush the rest of the string from the packet

      // This is a bit redundant with the code below, but wanted to log the
      // string for debugging
      myBuf[len - 1] = '\0';

      ArLog::log(ArLog::Normal, "ArBasePacket::bufToStr(buf, %d) output buf is not large enough for packet string %s",
                 len, myBuf);

      while (isNextGood(1) && (myBuf[myReadLength] != '\0')) {
        myReadLength++;
      }
      if (myBuf[myReadLength] == '\0') {
        myReadLength++;
      }
    } // end else if output buffer filled before null-terminator
  } // end if something to read

  // Make absolutely sure that the string is null-terminated...
  buf[len - 1] = '\0';

}

/**
copies length bytes from the buffer into data, length is passed in, not read
from packet
@param data character array to copy the data into
@param length number of bytes to copy into data
*/
AREXPORT void ArBasePacket::bufToData(char *data, int length)
{
  if (data == NULL) {
    ArLog::log(ArLog::Normal, "ArBasePacket::bufToData(NULL, %d) cannot write to null address",
               length);
    return;
  }
  if (isNextGood(length))
  {
    memcpy(data, myBuf+myReadLength, length);
    myReadLength += length;
  }
}


/**
   This was added to get around having to cast data you put in, since the data shouldn't really matter if its signed or unsigned.

copies length bytes from the buffer into data, length is passed in, not read
from packet
@param data character array to copy the data into
@param length number of bytes to copy into data
*/
AREXPORT void ArBasePacket::bufToData(unsigned char *data, int length)
{
  if (data == NULL) {
    ArLog::log(ArLog::Normal, "ArBasePacket::bufToData(NULL, %d) cannot write to null address",
               length);
    return;
  }
  if (isNextGood(length))
  {
    memcpy(data, myBuf+myReadLength, length);
    myReadLength += length;
  }
}


/**
Copies the given packets buffer into the buffer of this packet, also
sets this length and readlength to what the given packet has
@param packet the packet to duplicate
*/
AREXPORT void ArBasePacket::duplicatePacket(ArBasePacket *packet)
{
  myLength = packet->getLength();
  myReadLength = packet->getReadLength();
  memcpy(myBuf, packet->getBuf(), myLength);
}

AREXPORT void ArBasePacket::log(void)
{
  int i;
  ArLog::log(ArLog::Terse, "Packet: (length = %i)", myLength);
  for (i = 0; i < myLength; i++)
    ArLog::log(ArLog::Terse, "  [%03i] % 5d\t0x%x\t%s", i,(unsigned char) myBuf[i],
        (unsigned char) myBuf[i],
        i == 0 ? "[header0]" :
          i == 1 ? "[header1]" :
            i == 2 ? "[packet data length]" :
              i == 3 ? "[packet id]" :
                i == (myLength - 2) ? "[first checksum byte]" :
                  i == (myLength - 1) ? "[second checksum byte]" :
                    ""
    );
  ArLog::log(ArLog::Terse, "\n");
}

AREXPORT void ArBasePacket::printHex(void)
{
  int i;
  ArLog::log(ArLog::Terse, "Packet: (length = %i)", myLength);
  for (i = 0; i < myLength; i++)
    ArLog::log(ArLog::Terse, "  [%i] 0x%x ", i,(unsigned char) myBuf[i]);
  ArLog::log(ArLog::Terse, "\n");
}

