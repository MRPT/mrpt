
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

#include <xstypes/xsthread.h>
#include <xstypes/xsportinfo.h>
#include "usbinterface.h"
#include <errno.h>

#include "xswinusb.h"
#include "xslibusb.h"
#include "rx_tx_log.h"

#ifndef _WIN32
#	include <string.h>		// strcpy
#else
#	include <winbase.h>
#	include <io.h>
#endif
#include <atomic>

#ifndef _CRT_SECURE_NO_DEPRECATE
#	define _CRT_SECURE_NO_DEPRECATE
#	ifdef _WIN32
#		pragma warning(disable:4996)
#	endif
#endif

/*! \brief Private object for UsbInterface */
class UsbInterfacePrivate
{
public:
	#ifdef LOG_RX_TX
		XsFile rx_log;
		XsFile tx_log;
	#endif

		//! The time at which an operation will end in ms, used by several functions.
	uint32_t m_endTime;
		//! The last result of an operation
	XsResultValue m_lastResult;
	/*! The default timeout value to use during blocking operations.
		A value of 0 means that all operations become non-blocking.
	*/
	uint32_t m_timeout;

	int m_interfaceCount;
	int m_interface;
	int m_dataInEndPoint;
	int m_dataOutEndPoint;
	char m_portname[256];

#ifdef USE_WINUSB
	XsWinUsb m_winUsb;
	WINUSB_INTERFACE_HANDLE m_usbHandle[2];
	uint8_t m_bulkInPipe, m_bulkOutPipe, m_interruptPipe, m_deviceSpeed;
	uint16_t m_bulkInPipePacketSize;
	XsIoHandle m_deviceHandle;

	XsThreadId m_threadId;
	XsThread m_threadHandle;
	static const int m_oCount = MAXIMUM_WAIT_OBJECTS-1;
	OVERLAPPED m_overlapped[m_oCount];
	XsByteArray m_varBuffer;
	static const int m_fixedBufferSize = 8192;
	static const int m_fastPolicyThreshold = 3;
	uint8_t m_fixedBuffer[m_oCount][m_fixedBufferSize];
	//int m_offset;
	CRITICAL_SECTION m_mutex;
	HANDLE m_quitEvent;
	volatile std::atomic_bool m_running;
	HANDLE m_waitEvents[m_oCount];
	int m_readIdx;
	XsResultValue m_threadedResult;

	void threadFunc();

#elif defined(HAVE_LIBUSB)
	libusb_device_handle *m_deviceHandle;
	/*! \brief A context manager for libusb

	  For predictable operation with libusb, it is recommended to use at least one context per library.
	*/
	class UsbContext {
	public:
		/*! \brief Create the USB context
		*/
		UsbContext()
		{
			if (!m_claimedContexts)
				m_libUsb.init(&m_usbContext);
			m_claimedContexts++;
			//libusb_set_debug(m_usbContext, 3);
		}

		/*! \brief Destroy the USB context */
		~UsbContext()
		{
			m_claimedContexts--;
			if (!m_claimedContexts)
				m_libUsb.exit(m_usbContext);
		}
		static libusb_context *m_usbContext; // needed for proper use of libusb
		XsLibUsb m_libUsb;
	private:
		static int m_claimedContexts;
	};
	UsbContext m_contextManager;

	/*! \brief Map a libusb_error to XsResultValue

		\a param libusbError [in] the result code to convert
		\a param hint give a hint for the code to return when in doubt
	*/
	XsResultValue libusbErrorToXrv(int libusbError, XsResultValue hint = XRV_ERROR)
	{
		switch (libusbError) {
		case LIBUSB_SUCCESS:
			return XRV_OK;

		case LIBUSB_ERROR_IO:
			return hint;

		case LIBUSB_ERROR_INVALID_PARAM:
			return XRV_INVALIDPARAM;

		case LIBUSB_ERROR_ACCESS:
			return hint;

		case LIBUSB_ERROR_NO_DEVICE:
			return XRV_UNEXPECTED_DISCONNECT;

		case LIBUSB_ERROR_NOT_FOUND:
			return XRV_NOTFOUND;

		case LIBUSB_ERROR_BUSY:
			return XRV_INVALIDOPERATION;

		case LIBUSB_ERROR_TIMEOUT:
			return XRV_TIMEOUT;

		case LIBUSB_ERROR_OVERFLOW:
			return hint;

		case LIBUSB_ERROR_PIPE:
			return hint;

		case LIBUSB_ERROR_INTERRUPTED:
			return XRV_UNEXPECTEDMSG;

		case LIBUSB_ERROR_NO_MEM:
			return XRV_OUTOFMEMORY;

		case LIBUSB_ERROR_NOT_SUPPORTED:
			return XRV_NOTIMPLEMENTED;

		case LIBUSB_ERROR_OTHER:
			return hint;
		}
		return hint;
	}

	/*! \brief Convert a libusb error to a human-readable string */
	const char *libusbErrorToString(int libusbError)
	{
		switch (libusbError) {
		case LIBUSB_SUCCESS:
			return "LIBUSB_SUCCESS";

		case LIBUSB_ERROR_IO:
			return "LIBUSB_ERROR_IO";

		case LIBUSB_ERROR_INVALID_PARAM:
			return "LIBUSB_ERROR_INVALID_PARAM";

		case LIBUSB_ERROR_ACCESS:
			return "LIBUSB_ERROR_ACCESS";

		case LIBUSB_ERROR_NO_DEVICE:
			return "LIBUSB_ERROR_NO_DEVICE";

		case LIBUSB_ERROR_NOT_FOUND:
			return "LIBUSB_ERROR_NOT_FOUND";

		case LIBUSB_ERROR_BUSY:
			return "LIBUSB_ERROR_BUSY";

		case LIBUSB_ERROR_TIMEOUT:
			return "LIBUSB_ERROR_TIMEOUT";

		case LIBUSB_ERROR_OVERFLOW:
			return "LIBUSB_ERROR_OVERFLOW";

		case LIBUSB_ERROR_PIPE:
			return "LIBUSB_ERROR_PIPE";

		case LIBUSB_ERROR_INTERRUPTED:
			return "LIBUSB_ERROR_INTERRUPTED";

		case LIBUSB_ERROR_NO_MEM:
			return "LIBUSB_ERROR_NO_MEM";

		case LIBUSB_ERROR_NOT_SUPPORTED:
			return "LIBUSB_ERROR_NOT_SUPPORTED";

		case LIBUSB_ERROR_OTHER:
			return "LIBUSB_ERROR_OTHER";
		}
		return "Unknown";
	}
#else
	void *m_deviceHandle;
#endif
};

#ifdef USE_WINUSB
DWORD usbReadThreadFunc(void* obj)
{
	UsbInterfacePrivate* d = (UsbInterfacePrivate*) obj;
	d->m_running = true;
	xsSetThreadPriority(::GetCurrentThread(), XS_THREAD_PRIORITY_HIGHEST);	// not using xsGetCurrentThreadHandle here because we can directly use GetCurrentThread(), which is simpler
	xsNameThisThread("USB reader");
	try
	{
		d->threadFunc();
	}
	catch(...)
	{
#ifdef XSENS_DEBUG
		assert(0);
#endif
	}
	d->m_running = false;
	return 0;
}

void UsbInterfacePrivate::threadFunc()
{
	HANDLE handles[1+m_oCount];
	handles[0] = m_quitEvent;
	handles[m_oCount] = m_waitEvents[m_oCount-1];
	//= { m_quitEvent, m_waitEvents[0], m_waitEvents[1] };

	// start first read operation
	for (m_readIdx = 0 ; m_readIdx < (m_oCount-1); ++m_readIdx)
	{
		handles[m_readIdx+1] = m_waitEvents[m_readIdx];
		//m_readIdx = 0;
		m_overlapped[m_readIdx] = OVERLAPPED();
		::ResetEvent(m_waitEvents[m_readIdx]);
		m_overlapped[m_readIdx].hEvent = m_waitEvents[m_readIdx];
		m_winUsb.ReadPipe(m_usbHandle[1],
			m_bulkInPipe,
			m_fixedBuffer[m_readIdx],
			(ULONG)m_fixedBufferSize,
			0,
			&m_overlapped[m_readIdx]);
	}
	int fastCount = 0;
	//m_readIdx = 1;
	bool policyFast = false;
	bool run = true;
	while (run)
	{
		// start follow-up read operation
		m_overlapped[m_readIdx] = OVERLAPPED();
		::ResetEvent(m_waitEvents[m_readIdx]);
		m_overlapped[m_readIdx].hEvent = m_waitEvents[m_readIdx];
		m_winUsb.ReadPipe(m_usbHandle[1],
			m_bulkInPipe,
			m_fixedBuffer[m_readIdx],
			(ULONG)m_fixedBufferSize,
			0,
			&m_overlapped[m_readIdx]);
		m_readIdx = (m_readIdx + 1) % m_oCount;
		int64_t tBegin = XsTime_timeStampNow(0);
		DWORD waitResult = ::WaitForMultipleObjects(1+m_oCount, handles, FALSE, INFINITE);
#if 0	// not sure if this causes problems, but it should help in catching up
		int64_t tEnd = XsTime_timeStampNow(0);
		switch (tEnd - tBegin)
		{
		case 0:
			if (++fastCount > m_fastPolicyThreshold && !policyFast)
			{
				policyFast = true;
				// set fast policy
				UCHAR enable = TRUE;
				m_winUsb.SetPipePolicy(m_usbHandle[1], m_bulkInPipe, IGNORE_SHORT_PACKETS, sizeof(UCHAR), &enable);
			}
			break;

		case 1:
			if (fastCount)
				--fastCount;
			if (policyFast && fastCount <= m_fastPolicyThreshold)
			{
				// reset policy
				policyFast = false;
				UCHAR enable = FALSE;
				m_winUsb.SetPipePolicy(m_usbHandle[1], m_bulkInPipe, IGNORE_SHORT_PACKETS, sizeof(UCHAR), &enable);
			}
			break;

		default:
			fastCount = 0;
			if (policyFast)
			{
				// reset policy
				policyFast = false;
				UCHAR enable = FALSE;
				m_winUsb.SetPipePolicy(m_usbHandle[1], m_bulkInPipe, IGNORE_SHORT_PACKETS, sizeof(UCHAR), &enable);
			}
			break;
		}
#endif

		// handle data
		switch (waitResult)
		{
		case WAIT_TIMEOUT:
		case WAIT_FAILED:
		case WAIT_OBJECT_0:
			run = false;
			break;

		default:
			if (waitResult >= WAIT_ABANDONED_0)
			{
				JLDEBUGG("WFMO abandoned: " << (waitResult - WAIT_OBJECT_0));
				break;
			}

#ifndef XSENS_RELEASE
			JLDEBUGG("WFMO trigger: " << (waitResult - WAIT_OBJECT_0));
#endif
			{
				// put data into buffer
				int idx = m_readIdx;
				DWORD dataRead = 0;
				if (!m_winUsb.GetOverlappedResult(m_usbHandle[0], &m_overlapped[idx], &dataRead, FALSE))
				{
					// error
					DWORD err = ::GetLastError();
					switch (err)
					{
					case ERROR_SEM_TIMEOUT:
					case ERROR_IO_INCOMPLETE:
						//JLDEBUGG("m_winUsb.GetOverlappedResult resulted in acceptable windows error " << err);
						break;

					case ERROR_GEN_FAILURE:
						JLDEBUGG("We seem to have lost contact with the usb device");
						m_threadedResult = XRV_UNEXPECTED_DISCONNECT;
						run = false;
						break;

					default:
						JLALERTG("m_winUsb.GetOverlappedResult resulted in windows error " << err);
						run = false;
						break;
					}
					//assert (err == ERROR_IO_INCOMPLETE);
				}
				else
				{
					// append unread data to var buffer
					JLTRACEG("m_winUsb.GetOverlappedResult resulted in " << dataRead << " bytes being read");
					XsByteArray ref(&m_fixedBuffer[idx][0], dataRead, XSDF_None);
					::EnterCriticalSection(&m_mutex);
					m_varBuffer.append(ref);
					::LeaveCriticalSection(&m_mutex);
				}
			} break;
		}
	}
}

#elif defined(HAVE_LIBUSB)
int UsbInterfacePrivate::UsbContext::m_claimedContexts = 0;
libusb_context *UsbInterfacePrivate::UsbContext::m_usbContext = NULL;
#endif

/*! \class UsbInterface
	\brief An IoInterface for dealing specifically with Xsens USB devices
*/

/*! \brief Default constructor, initializes all members to their default values.
*/
UsbInterface::UsbInterface() :
	d(new UsbInterfacePrivate)
{
	d->m_lastResult = XRV_OK;
	d->m_timeout = 20;
	d->m_endTime = 0;
	d->m_dataInEndPoint = -1;
	d->m_dataOutEndPoint = -1;
	d->m_deviceHandle = 0;
	d->m_portname[0] = 0;

	#ifdef USE_WINUSB
		d->m_threadHandle = INVALID_HANDLE_VALUE;
		::InitializeCriticalSection(&d->m_mutex);
		d->m_usbHandle[0] = 0;
		d->m_usbHandle[1] = 0;
		for (int i = 0; i < d->m_oCount; ++i)
		{
			d->m_waitEvents[i] = ::CreateEvent(NULL, TRUE, FALSE, NULL);
			::ResetEvent(d->m_waitEvents[i]);
		}
		d->m_quitEvent = ::CreateEvent(NULL, TRUE, FALSE, NULL);
		::ResetEvent(d->m_quitEvent);
		d->m_readIdx = 0;
//		d->m_offset = 0;
		d->m_threadedResult = XRV_OK;
	#endif
}

//! Destructor, de-initializes, frees memory allocated for buffers, etc.
UsbInterface::~UsbInterface()
{
	try {
		closeUsb();
#ifdef USE_WINUSB
		::CloseHandle(d->m_quitEvent);
		for (int i = 0; i < d->m_oCount; ++i)
			::CloseHandle(d->m_waitEvents[i]);
		::DeleteCriticalSection(&d->m_mutex);
#endif
		delete d;
	} catch(...)
	{}
}

/*! \brief Close the USB communication port.
*/
XsResultValue UsbInterface::close(void)
{
	return closeUsb();
}

/*! \brief Close the USB communication port.
	\returns XRV_OK if the port was closed successfully
	\note Linux:\n
	If a kernel driver was detached when communication with the device started,
	attach it again. No guarantee is given that udev will pick up on it though.
*/
XsResultValue UsbInterface::closeUsb(void)
{
#ifdef LOG_RX_TX
	d->rx_log.close();
	d->tx_log.close();

#endif
	if (!isOpen())
		return d->m_lastResult = XRV_NOPORTOPEN;

	d->m_lastResult = XRV_OK;
#ifdef USE_WINUSB
	if (d->m_threadHandle != INVALID_HANDLE_VALUE)
	{
		while (d->m_running)
		{
			::SetEvent(d->m_quitEvent);
			XsTime::msleep(10);
		}
		::CloseHandle(d->m_threadHandle);
	}

	flushData();
	if(d->m_usbHandle[0]) {
		d->m_winUsb.Free(d->m_usbHandle[0]);
		d->m_usbHandle[0] = NULL;
	}
	if(d->m_usbHandle[1]) {
		d->m_winUsb.Free(d->m_usbHandle[1]);
		d->m_usbHandle[1] = NULL;
	}
	if (d->m_deviceHandle) {
		CloseHandle(d->m_deviceHandle);
		d->m_deviceHandle = NULL;
	}
#elif defined(HAVE_LIBUSB)
	flushData();
	libusb_device *dev = d->m_contextManager.m_libUsb.get_device(d->m_deviceHandle);
	for (int i = 0; i < d->m_interfaceCount; i++) {
		int result = LIBUSB_ERROR_OTHER;
		while (result != LIBUSB_SUCCESS) {
			result = d->m_contextManager.m_libUsb.release_interface(d->m_deviceHandle, i);
			if (result == LIBUSB_SUCCESS) {
				d->m_contextManager.m_libUsb.attach_kernel_driver(d->m_deviceHandle, i);
			}
		}
	}

	d->m_contextManager.m_libUsb.close(d->m_deviceHandle);
	d->m_deviceHandle = NULL;

	d->m_contextManager.m_libUsb.unref_device(dev);
	d->m_interface = -1;
	d->m_dataInEndPoint = -1;
	d->m_dataOutEndPoint = -1;
#else
	d->m_lastResult = XRV_NOTIMPLEMENTED;
#endif
	d->m_endTime = 0;

	return d->m_lastResult;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Flush all data to be transmitted / received.
XsResultValue UsbInterface::flushData(void)
{
	d->m_lastResult = XRV_OK;
#ifdef USE_WINUSB
	if(d->m_usbHandle[0]) {
		d->m_winUsb.AbortPipe(d->m_usbHandle[0], d->m_bulkInPipe);
		d->m_winUsb.FlushPipe(d->m_usbHandle[0], d->m_bulkInPipe);
		d->m_winUsb.AbortPipe(d->m_usbHandle[0], d->m_bulkOutPipe);
		d->m_winUsb.FlushPipe(d->m_usbHandle[0], d->m_bulkOutPipe);
	}
	if(d->m_usbHandle[1]) {
		d->m_winUsb.AbortPipe(d->m_usbHandle[1], d->m_bulkInPipe);
		d->m_winUsb.FlushPipe(d->m_usbHandle[1], d->m_bulkInPipe);
		d->m_winUsb.AbortPipe(d->m_usbHandle[1], d->m_bulkOutPipe);
		d->m_winUsb.FlushPipe(d->m_usbHandle[1], d->m_bulkOutPipe);
	}
#elif defined(HAVE_LIBUSB)
	unsigned char flushBuffer[256];
	int actual;
	for (int i = 0; i < 64; ++i) {
		if (d->m_contextManager.m_libUsb.bulk_transfer(d->m_deviceHandle, d->m_dataInEndPoint|LIBUSB_ENDPOINT_IN,
								 flushBuffer, sizeof(flushBuffer), &actual, 1) != LIBUSB_SUCCESS)
			break;
		if (actual == 0)
			break;
	}
#else
	d->m_lastResult = XRV_NOTIMPLEMENTED;
#endif
	d->m_endTime = 0;
	return d->m_lastResult;
}

//! Return the error code of the last operation.
XsResultValue UsbInterface::getLastResult(void) const
{
	return d->m_lastResult;
}

//! Return the current timeout value
uint32_t UsbInterface::getTimeout (void) const
{
	return d->m_timeout;
}

//! Return whether the USB communication port is open or not.
bool UsbInterface::isOpen (void) const
{
	return d->m_deviceHandle != NULL;
}

/*! \brief Open a communication channel to the given USB port name. */
XsResultValue UsbInterface::open(const XsPortInfo &portInfo, XsFilePos, XsFilePos, PortOptions)
{
	d->m_endTime = 0;

#ifdef USE_WINUSB
	JLDEBUGG("Open usb port " << portInfo.portName().toStdString());
#else
	JLDEBUGG("Open usb port " << portInfo.usbBus() << ":" << portInfo.usbAddress());
#endif

	if (isOpen())
	{
		JLALERTG("Port " << portInfo.portName().toStdString() << " already open");
		return (d->m_lastResult = XRV_ALREADYOPEN);
	}

#ifdef USE_WINUSB
	d->m_deviceHandle = CreateFileA(portInfo.portName().c_str(),
		GENERIC_WRITE | GENERIC_READ,
		FILE_SHARE_WRITE | FILE_SHARE_READ,
		NULL,
		OPEN_EXISTING,
		FILE_ATTRIBUTE_NORMAL | FILE_FLAG_OVERLAPPED,
		NULL);

	if (d->m_deviceHandle == INVALID_HANDLE_VALUE)
	{
		d->m_deviceHandle = NULL;
		return (d->m_lastResult = XRV_PORTNOTFOUND);
	}

	BOOL result = FALSE;
	UCHAR speed = 0;
	ULONG length = 0;
	USB_INTERFACE_DESCRIPTOR interfaceDescriptor = {0,0,0,0,0,0,0,0,0};
	WINUSB_PIPE_INFORMATION pipeInfo;

	result = d->m_winUsb.Initialize(d->m_deviceHandle, &d->m_usbHandle[0]);
	if (result)
	{
		result = d->m_winUsb.GetAssociatedInterface(d->m_usbHandle[0],0,&d->m_usbHandle[1]);
	}
	else
	{
#ifdef XSENS_DEBUG
		DWORD err = GetLastError();
		assert(result);
#endif
		return (d->m_lastResult = XRV_ERROR);
	}

	for (int k = 0; k<2;k++)
	{
		if(result)
		{
			assert(d->m_usbHandle[k] != 0);
			length = sizeof(UCHAR);
			result = d->m_winUsb.QueryDeviceInformation(d->m_usbHandle[k],
				DEVICE_SPEED,
				&length,
				&speed);
		}

		if(result)
		{
			d->m_deviceSpeed = speed;
			result = d->m_winUsb.QueryInterfaceSettings(d->m_usbHandle[k],
				0,
				&interfaceDescriptor);
		}
		if(result)
		{
			for(int i=0;i<interfaceDescriptor.bNumEndpoints;i++)
			{
				result = d->m_winUsb.QueryPipe(d->m_usbHandle[k],
					0,
					(UCHAR) i,
					&pipeInfo);

				if(pipeInfo.PipeType == UsbdPipeTypeBulk &&
					USB_ENDPOINT_DIRECTION_IN(pipeInfo.PipeId))
				{
					d->m_bulkInPipe = pipeInfo.PipeId;
					d->m_bulkInPipePacketSize =
						pipeInfo.MaximumPacketSize;
				}
				else if(pipeInfo.PipeType == UsbdPipeTypeBulk &&
					USB_ENDPOINT_DIRECTION_OUT(pipeInfo.PipeId))
				{
					d->m_bulkOutPipe = pipeInfo.PipeId;
				}
				else if(pipeInfo.PipeType == UsbdPipeTypeInterrupt)
				{
					d->m_interruptPipe = pipeInfo.PipeId;
				}
				else
				{
					result = FALSE;
					break;
				}
			}
		}
	}

	setTimeout(0);
	flushData();

	sprintf(d->m_portname, "%s", portInfo.portName().c_str());

//	d->m_offset = 0;
	::ResetEvent(&d->m_quitEvent);
	d->m_threadHandle = xsStartThread(usbReadThreadFunc, d, &d->m_threadId);
	if (d->m_threadHandle == XSENS_INVALID_THREAD)
	{
#ifdef XSENS_DEBUG
		assert(0);
#endif
		return (d->m_lastResult = XRV_ERROR);
	}

#elif defined(HAVE_LIBUSB)
	libusb_device **deviceList;
	ssize_t listLength = d->m_contextManager.m_libUsb.get_device_list(d->m_contextManager.m_usbContext, &deviceList);
	if (listLength < 0)
		return d->m_lastResult = d->libusbErrorToXrv((int)listLength);

	// "USBxxx:yyy"
	uint8_t bus = XsPortInfo_usbBus(&portInfo);
	uint8_t address = XsPortInfo_usbAddress(&portInfo);

	XsResultValue xrv = XRV_OK;
	int result;
	libusb_device *device = NULL;
	for (int i = 0; i < listLength && device == NULL; ++i) {
		libusb_device *dev = deviceList[i];
		if (d->m_contextManager.m_libUsb.get_bus_number(dev) != bus || d->m_contextManager.m_libUsb.get_device_address(dev) != address)
			continue;

		libusb_device_descriptor desc;
		result = d->m_contextManager.m_libUsb.get_device_descriptor(dev, &desc);
		if (result != LIBUSB_SUCCESS)
			break;

		libusb_config_descriptor *configDesc;
		result = d->m_contextManager.m_libUsb.get_active_config_descriptor(dev, &configDesc);
		if (result != LIBUSB_SUCCESS)
			break;

		d->m_interface = -1;
		d->m_interfaceCount = configDesc->bNumInterfaces;
		// find the bulk transfer endpoints
		for (uint8_t ifCount = 0; ifCount < configDesc->bNumInterfaces && d->m_interface == -1; ++ifCount) {
			for (uint8_t altsettingCount = 0; altsettingCount < configDesc->interface[ifCount].num_altsetting; altsettingCount++) {
				const libusb_endpoint_descriptor *endpoints = configDesc->interface[ifCount].altsetting[altsettingCount].endpoint;
				int inEndpoint = -1, outEndpoint = -1;
				for (uint8_t i = 0; i < configDesc->interface[ifCount].altsetting[altsettingCount].bNumEndpoints; i++) {
					if ((endpoints[i].bmAttributes&LIBUSB_TRANSFER_TYPE_MASK) != LIBUSB_TRANSFER_TYPE_BULK)
						continue;

					switch (endpoints[i].bEndpointAddress&LIBUSB_ENDPOINT_DIR_MASK) {
					case LIBUSB_ENDPOINT_IN:
						inEndpoint = endpoints[i].bEndpointAddress&LIBUSB_ENDPOINT_ADDRESS_MASK;
						break;

					case LIBUSB_ENDPOINT_OUT:
						outEndpoint = endpoints[i].bEndpointAddress&LIBUSB_ENDPOINT_ADDRESS_MASK;
						break;
					}

				}

				if (outEndpoint == -1 || inEndpoint == -1)
					continue;

				d->m_interface = ifCount;
				d->m_dataOutEndPoint = outEndpoint;
				d->m_dataInEndPoint = inEndpoint;
			}
		}
		if (d->m_interface == -1) {
			xrv = XRV_INPUTCANNOTBEOPENED;
			break;
		}

		d->m_contextManager.m_libUsb.free_config_descriptor(configDesc);
		d->m_contextManager.m_libUsb.ref_device(dev);
		device = dev;
		result = LIBUSB_SUCCESS;
	}

	d->m_contextManager.m_libUsb.free_device_list(deviceList, 1);
	if (result != LIBUSB_SUCCESS) {
		d->m_contextManager.m_libUsb.unref_device(device);
		return d->m_lastResult = d->libusbErrorToXrv(result);
	}

	if (xrv != XRV_OK) {
		d->m_contextManager.m_libUsb.unref_device(device);
		return d->m_lastResult = xrv;
	}

	libusb_device_handle *handle;
	result = d->m_contextManager.m_libUsb.open(device, &handle);
	if (result != LIBUSB_SUCCESS) {
		d->m_contextManager.m_libUsb.unref_device(device);
		return d->m_lastResult = d->libusbErrorToXrv(result);
	}

	// be rude and claim all interfaces
	for (int i = 0; i < d->m_interfaceCount; i++) {
		result = d->m_contextManager.m_libUsb.kernel_driver_active(handle, i);
		if (result > 0)
			result = d->m_contextManager.m_libUsb.detach_kernel_driver(handle, i);
		if (result == LIBUSB_SUCCESS)
			result = d->m_contextManager.m_libUsb.claim_interface(handle, i);
		if (result != LIBUSB_SUCCESS) {
			for (int j = 0; j < i; j++) {
				while (result != LIBUSB_SUCCESS) {
					result = d->m_contextManager.m_libUsb.release_interface(handle, j);
					d->m_contextManager.m_libUsb.attach_kernel_driver(handle, j);
				}
			}

			d->m_contextManager.m_libUsb.close(handle);
			d->m_contextManager.m_libUsb.unref_device(device);
			return d->m_lastResult = d->libusbErrorToXrv(result);
		}
	}

	d->m_deviceHandle = handle;
	sprintf(d->m_portname, "%s", portInfo.portName().c_str());

	flushData();
#else // HAVE_LIBUSB
	d->m_lastResult = XRV_NOTIMPLEMENTED;
#endif
	JLDEBUGG("USB Port opened");
	return (d->m_lastResult = XRV_OK);
}

/*! \brief Read data from the USB port and put it into the data buffer.
	\details This function reads up to \a maxLength bytes from the port (non-blocking) and
	puts it into the \a data buffer.
	\param maxLength The maximum amount of data read.
	\param data The buffer that will store the received data.
	\returns XRV_OK if no error occurred. It can be that no data is available and XRV_OK will be
			returned. Check data.size() for the number of bytes that were read.
*/
XsResultValue UsbInterface::readData(XsFilePos maxLength, XsByteArray& data)
{
	XsFilePos length = 0;
	data.setSize((XsSize) maxLength);
	XsResultValue res = readData(maxLength, data.data(), &length);
	data.pop_back((XsSize) (maxLength - length));
	return res;
}

/*! \brief Read data from the serial port and put it into the data buffer.
	\details This function reads up to \a maxLength bytes from the USB port (non-blocking) and
	puts it into the \a data buffer.
	\param maxLength	The maximum number of bytes to read.
	\param data			Pointer to a buffer that will store the received data.
	\param length		The number of bytes placed into \c data.
	\returns XRV_OK if no error occurred. It can be that no data is available and XRV_OK will be
			returned. Check *length for the number of bytes that were read.
*/
XsResultValue UsbInterface::readData(XsFilePos maxLength, void *data, XsFilePos* length)
{
	JLTRACEG("maxLength=" << maxLength << ", data=" << JLHEXLOG(data) << ", length=" << JLHEXLOG(length));
	XsFilePos ln;
	if (length == NULL)
		length = &ln;

	if (!isOpen())
		return (d->m_lastResult = XRV_NOPORTOPEN);

#ifdef USE_WINUSB
	XsFilePos remaining = 0;
	::EnterCriticalSection(&d->m_mutex);
	remaining = *length = d->m_varBuffer.size();
	if (*length > maxLength)
		*length = maxLength;
	if (*length)
	{
		memcpy(data, d->m_varBuffer.data(), (XsSize) *length);
		d->m_varBuffer.erase(0, (XsSize) *length);
		remaining = d->m_varBuffer.size();
	}

	if (d->m_threadedResult != XRV_OK)
		return (d->m_lastResult = d->m_threadedResult);

	::LeaveCriticalSection(&d->m_mutex);
	JLTRACEG("returned success, read " << *length << " of " << maxLength << " bytes, first: " << JLHEXLOG(((char*)data)[0]) << ", " << remaining << " remaining in buffer");

#elif defined(HAVE_LIBUSB)
	int actual = 0;
	JLTRACEG("starting bulk read, timeout = " << d->m_timeout);
	int res = d->m_contextManager.m_libUsb.bulk_transfer(d->m_deviceHandle, (d->m_dataInEndPoint|LIBUSB_ENDPOINT_IN), (unsigned char *)data, maxLength, &actual, d->m_timeout);
	JLTRACEG("bulk read returned: " << d->libusbErrorToString(res) << ". " << actual << " bytes received");
	if ((res != LIBUSB_SUCCESS && res != LIBUSB_ERROR_TIMEOUT) || (res == LIBUSB_ERROR_TIMEOUT && actual <= 0))
		return d->m_lastResult = d->libusbErrorToXrv(res);

	*length = actual;
#else
	(void)maxLength;
	(void)data;
#endif

#ifdef LOG_RX_TX
	if (*length > 0)
	{
		CHECK_STATE_RX(*length, data, d->rx_log);

		if (!d->rx_log.isOpen())
		{
			char fname[XS_MAX_FILENAME_LENGTH];
			sprintf(fname,"rx_USB%03u_%03u.log", usbBus(), usbAddress());
			d->rx_log.create(XsString(fname), true);
		}
		d->rx_log.write(data,1,*length);
#ifdef LOG_RX_TX_FLUSH
		d->rx_log.flush();
#endif
	}
#endif

	return (d->m_lastResult = XRV_OK);
}

/*! \brief Set the default timeout value to use in blocking operations.
	\details This function sets the value of m_timeout. There is no infinity value. The value 0
	means that the default value is used.
	\param ms The desired timeout in milliseconds
	\returns XRV_OK if the timeout value was successfully updated
*/
XsResultValue UsbInterface::setTimeout(uint32_t ms)
{
#ifdef USE_WINUSB
	JLDEBUGG("Request to set timeout to " << ms << " ms overridden, setting to 0 ms instead");
	ms = 0;		// no timeout ever
	UCHAR enable = FALSE;

	d->m_winUsb.SetPipePolicy(d->m_usbHandle[1], d->m_bulkInPipe, IGNORE_SHORT_PACKETS, sizeof(UCHAR), &enable);
	d->m_winUsb.SetPipePolicy(d->m_usbHandle[1], d->m_bulkInPipe, PIPE_TRANSFER_TIMEOUT, sizeof(ULONG), &ms);

	d->m_timeout = ms;
#else
	JLDEBUGG("Setting timeout to " << ms);
	if (ms == 0)
		d->m_timeout = 1;
	else
		d->m_timeout = ms;
#endif
	return (d->m_lastResult = XRV_OK);
}

/*! \brief Sets the RAWIO mode of the USB interface
	\note Only applies to WinUSB implementations
	\param enable : If true will enable RAW IO mode
*/
void UsbInterface::setRawIo(bool enable)
{
	JLDEBUGG("Setting RAWIO mode to " << enable);

#ifdef USE_WINUSB
	enable = false;	// never use raw IO
	UCHAR rawIo = (UCHAR)enable;
	d->m_winUsb.SetPipePolicy(d->m_usbHandle[1], d->m_bulkInPipe, RAW_IO, sizeof(UCHAR), &rawIo);
#else
	(void)enable;
#endif
	d->m_lastResult = XRV_OK;
}

/*! \brief Retrieves the state of the RAWIO mode of the USB interface
	\returns true if raw IO mode is enabled
	\note Only applies to WinUSB implementations
*/
bool UsbInterface::getRawIo(void)
{
#ifdef USE_WINUSB
	UCHAR rawIo = 0;
	ULONG someSize = sizeof(UCHAR);
	d->m_winUsb.GetPipePolicy(d->m_usbHandle[1], d->m_bulkInPipe, RAW_IO, &someSize, &rawIo);
	return (rawIo != 0);
#else
	return false;
#endif
}

/*! \brief Wait for data to arrive or a timeout to occur.
	\details The function waits until \c maxLength data is available or until a timeout occurs.
	The function returns success if data is available or XsResultValue::TIMEOUT if a
	timeout occurred. A timeout value of 0 indicates that the default timeout stored
	in the class should be used.
	\param maxLength The maximum number of bytes to wait for
	\param data A buffer that will be filled with the read data. It must be able to contain at
			least \a maxLength bytes.
	\param length An optional pointer to storage for the actual number of bytes read.
	\returns XRV_OK if the requested data was read
*/
XsResultValue UsbInterface::waitForData(XsFilePos maxLength, void* data, XsFilePos* length)
{
	JLTRACEG("timeout=" << d->m_timeout << ", data=" << data << ", length=" << length);
	uint32_t timeout = d->m_timeout;

	char *bdata = (char *)data;

	XsFilePos ln;
	if (length == NULL)
		length = &ln;
	uint32_t eTime = XsTime::getTimeOfDay() + timeout;
	XsFilePos newLength = 0;

	*length = 0;
	while ((*length < maxLength) && (XsTime::getTimeOfDay() <= eTime))
	{
		if (readData(maxLength - *length, bdata + *length, &newLength))
			return d->m_lastResult;
		*length += newLength;
	}
	JLTRACEG("read " << length[0] << " of " << maxLength << " bytes");

	if (length[0] < maxLength)
		return (d->m_lastResult = XRV_TIMEOUT);
	else
		return (d->m_lastResult = XRV_OK);
}

/*! \brief Write the data to the USB port.
	\details The function writes the given data to the connected USB port.
	The default timeout is respected in this operation.
	\param data The data to be written
	\param written An optional pointer to storage for the actual number of bytes that were written
	\returns XRV_OK if the data was successfully written
	\sa writeData(XsFilePos, const void *, XsFilePos*)
*/
XsResultValue UsbInterface::writeData(const XsByteArray& data, XsFilePos* written)
{
	return writeData(data.size(), data.data(), written);
}

/*! \brief Write the data to the USB port.
	\details The function writes the given data to the connected USB port.
	The default timeout is respected in this operation.
	\param length The number of bytes to write.
	\param data A pointer to a memory buffer that contains the bytes to send
	\param written An optional pointer to storage for the actual number of bytes that were written
	\returns XRV_OK if the data was successfully written
	\sa writeData(const XsByteArray&, XsFilePos*)
*/
XsResultValue UsbInterface::writeData(XsFilePos length, const void *data, XsFilePos* written)
{
	XsFilePos bytes;
	if (written == NULL)
		written = &bytes;

	if (!isOpen())
		return (d->m_lastResult = XRV_NOPORTOPEN);

	*written = 0;

#ifdef USE_WINUSB
	ULONG dataWritten;
	d->m_winUsb.WritePipe(d->m_usbHandle[1],
		d->m_bulkOutPipe,
		(uint8_t*)data,
		(ULONG)length,
		&dataWritten,
		NULL);

	*written = dataWritten;
#elif defined(HAVE_LIBUSB)
	*written = 0;
	while (*written < length)
	{
		int actual;
		int result = d->m_contextManager.m_libUsb.bulk_transfer(d->m_deviceHandle, (d->m_dataOutEndPoint|LIBUSB_ENDPOINT_OUT), (unsigned char *)data, length, &actual, 0);
		*written += actual;
		if (result != LIBUSB_SUCCESS)
		{
			JLALERTG("bulk write failed: " << d->libusbErrorToString(result) << ". " << actual << " bytes sent");
			return (d->m_lastResult = d->libusbErrorToXrv(result));
		}
	}
#else
	(void)length;
	(void)data;
#endif

#ifdef LOG_RX_TX
	if (*written > 0)
	{
		CHECK_STATE_TX(*written, data, d->tx_log);

		if (!d->tx_log.isOpen())
		{
			char fname[XS_MAX_FILENAME_LENGTH];
			sprintf(fname,"tx_USB%03u_%03u.log", usbBus(), usbAddress());
			d->tx_log.create(XsString(fname), true);
		}
		d->tx_log.write(data,1,*written);
#ifdef LOG_RX_TX_FLUSH
		d->tx_log.flush();
#endif
	}
#endif

	return (d->m_lastResult = XRV_OK);
}

/*! \brief The USB bus number this device is on (libusb/linux only) */
uint8_t UsbInterface::usbBus() const
{
#if defined(HAVE_LIBUSB)
	if (!d->m_deviceHandle)
		return 0;

	libusb_device *dev = d->m_contextManager.m_libUsb.get_device(d->m_deviceHandle);
	return d->m_contextManager.m_libUsb.get_bus_number(dev);
#else
	return 0;
#endif
}

/*! \brief The address of the device (libusb/linux only) */
uint8_t UsbInterface::usbAddress() const
{
#if defined(HAVE_LIBUSB)
	if (!d->m_deviceHandle)
		return 0;

	libusb_device *dev = d->m_contextManager.m_libUsb.get_device(d->m_deviceHandle);
	return d->m_contextManager.m_libUsb.get_device_address(dev);
#else
	return 0;
#endif
}

//! Retrieve the port name that was last successfully opened.
void UsbInterface::getPortName(XsString& portname) const
{
	portname = d->m_portname;
}
