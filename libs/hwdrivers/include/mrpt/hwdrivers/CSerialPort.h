/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef CSERIALPORT_H
#define CSERIALPORT_H

#include <mrpt/config.h>
#include <mrpt/utils/CStream.h>
#include <mrpt/utils/CTicTac.h>
#include <mrpt/hwdrivers/link_pragmas.h>

namespace mrpt
{
	namespace hwdrivers
	{
		/** A communications serial port built as an implementation of a utils::CStream.
		 * On communication errors (eg. the given port number does not exist, timeouts,...), most of the methods will
		 * raise an exception of the class "std::exception"
		 *
		 *  The serial port to open is passed in the constructor in the form of a string description,
		 *   which is platform dependent.
		 *
		 *  In windows they are numbered "COM1"-"COM4" and "\\.\COMXXX" for numbers above.
		 *    It is recomended to always use the prefix "\\.\" despite the actual port number.
		 *
		 *  In Linux the name must refer to the device, for example: "ttyUSB0","ttyS0". If the name string does not
		 *   start with "/" (an absolute path), the constructor will assume the prefix "/dev/".
		 *
		 *  History:
		 *    - 1/DEC/2005:  (JLBC) First version
		 *    - 20/DEC/2006: (JLBC) Integration into the MRPT framework
		 *    - 12/DEC/2007: (JLBC) Added support for Linux.
		 *
		 * \todo Add the internal buffer to the Windows implementation also
		 * \ingroup mrpt_hwdrivers_grp
		 */
		class HWDRIVERS_IMPEXP CSerialPort : public mrpt::utils::CStream
		{
			friend class PosixSignalDispatcherImpl;
		public:
			/** Constructor
			 * \param portName The serial port to open. See comments at the begining of this page.
			 * \param openNow Whether to try to open the port now. If not selected, the port should be open later with "open()".
			 *
			 */
			CSerialPort( const std::string &portName, bool openNow = true );

			/** Default constructor: it does not open any port - later you must call "setSerialPortName" and then "open"
			 */
			CSerialPort();

			/** Destructor
			*/
			virtual ~CSerialPort();

			/** Sets the serial port to open (it is an error to try to change this while open yet).
			  * \sa open, close
			  */
			void setSerialPortName( const std::string &COM_name )
			{
				if (isOpen()) THROW_EXCEPTION("Cannot change serial port while open");
				m_serialName = COM_name;
			}

			/** Open the port. If is already open results in no action.
			* \exception std::exception On communication errors
			*/
			void  open();

			/** Open the given serial port. If it is already open and the name does not match, an exception is raised.
			* \exception std::exception On communication errors or a different serial port already open.
			*/
			void  open(const std::string &COM_name)
			{
				if (isOpen() && m_serialName!=COM_name) THROW_EXCEPTION("Cannot change serial port while open");
				if (!isOpen())
				{
					setSerialPortName(COM_name);
					open();
				}
			}


			/** Close the port. If is already closed, results in no action.
			*/
			void  close();

			/** Returns if port has been correctly open.
			*/
			bool  isOpen() const;

			/** Purge tx and rx buffers.
			   * \exception std::exception On communication errors
			   */
			void  purgeBuffers();

			/** Changes the configuration of the port.
			*  \param parity  0:No parity, 1:Odd, 2:Even (WINDOWS ONLY: 3:Mark, 4:Space)
			*  \param baudRate The desired baud rate Accepted values: 50 - 230400
			*  \param bits Bits per word (typ. 8) Accepted values: 5,6,7,8.
			*  \param nStopBits Stop bits (typ. 1) Accepted values: 1,2
			*  \param enableFlowControl Whether to enable the hardware flow control (RTS/CTS) (default=no)
			* \exception std::exception On communication errors
			*/
			void  setConfig(
				int baudRate,
				int parity = 0,
				int bits = 8,
				int nStopBits = 1,
				bool enableFlowControl=false);

			/** Changes the timeouts of the port, in milliseconds.
			* \exception std::exception On communication errors
			*/
			void  setTimeouts(
				int		ReadIntervalTimeout,
				int		ReadTotalTimeoutMultiplier,
				int		ReadTotalTimeoutConstant,
				int		WriteTotalTimeoutMultiplier,
				int		WriteTotalTimeoutConstant );


			/** Implements the virtual method responsible for reading from the stream - Unlike CStream::ReadBuffer, this method will not raise an exception on zero bytes read, as long as there is not any fatal error in the communications.
			  * \exception std::exception On communication errors
			 */
			size_t  Read(void *Buffer, size_t Count);

			/** Reads one text line from the serial port in POSIX "canonical mode".
			  *  This method reads from the serial port until one of the characters in \a eol are found.
			  * \param eol_chars A line reception is finished when one of these characters is found. Default: LF (10), CR (13).
			  * \param total_timeout_ms If >0, the maximum number of milliseconds to wait.
			  * \param out_timeout If provided, will hold true on return if a timeout ocurred, false on a valid read.
			  * \return The read string, without the final
			  * \exception std::exception On communication errors
			  */
			std::string ReadString(const int total_timeout_ms=-1, bool *out_timeout =NULL, const char *eol_chars = "\r\n");

			/** Implements the virtual method responsible for writing to the stream.
			 *  Write attempts to write up to Count bytes to Buffer, and returns the number of bytes actually written.
			* \exception std::exception On communication errors
			 */
			size_t  Write(const void *Buffer, size_t Count);


			/** Introduces a pure virtual method for moving to a specified position in the streamed resource.
			 *   he Origin parameter indicates how to interpret the Offset parameter. Origin should be one of the following values:
			 *	- sFromBeginning	(Default) Offset is from the beginning of the resource. Seek moves to the position Offset. Offset must be >= 0.
			 *	- sFromCurrent		Offset is from the current position in the resource. Seek moves to Position + Offset.
			 *	- sFromEnd			Offset is from the end of the resource. Offset must be <= 0 to indicate a number of bytes before the end of the file.
			 * \return Seek returns the new value of the Position property.
			 */
			uint64_t Seek(uint64_t Offset, CStream::TSeekOrigin Origin = sFromBeginning)
			{
				MRPT_START
				MRPT_UNUSED_PARAM(Origin);
				MRPT_UNUSED_PARAM(Offset);
				THROW_EXCEPTION("Method not applicable to serial communications port CStream!");
				MRPT_END
			}

			/** Returns the total amount of bytes in the stream.
			 */
			uint64_t getTotalBytesCount()
			{
				MRPT_START
				THROW_EXCEPTION("Method not applicable to serial communications port CStream!");
				MRPT_END
			}

			/** Method for getting the current cursor position, where 0 is the first byte and TotalBytesCount-1 the last one.
			 */
			uint64_t getPosition()
			{
				MRPT_START
				THROW_EXCEPTION("Method not applicable to serial communications port CStream!");
				MRPT_END
			}

		protected:

			/** The complete name of the serial port device (i.e. "\\.\COM10","/dev/ttyS2",...)
			  */
			std::string 	m_serialName;
			int				m_baudRate;
			int				m_totalTimeout_ms,m_interBytesTimeout_ms;

			mrpt::utils::CTicTac m_timer; //!< Used only in \a ReadString

		#ifdef MRPT_OS_WINDOWS
			// WINDOWS
			void		*hCOM;
		#else
			// LINUX
			/** The file handle (-1: Not open)
			  */
			int 		hCOM;
			// size_t  ReadUnbuffered(void *Buffer, size_t Count); // JL: Remove??
		#endif

		}; // end of class

	} // end of namespace
} // end of namespace

#endif
