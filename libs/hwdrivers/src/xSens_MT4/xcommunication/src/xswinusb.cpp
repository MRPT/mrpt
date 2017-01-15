/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifdef _WIN32  // patch for MRPT

#include "xswinusb.h"
#include <xslibraryloader.h>

/*! \class XsWinUsb
	\brief Class for dynamic loading of winusb
*/
XsWinUsb::XsWinUsb(void)
{
	m_libraryLoader = new XsLibraryLoader();
	initLibrary();
}

XsWinUsb::~XsWinUsb(void)
{
	delete m_libraryLoader;
}

void XsWinUsb::initLibrary()
{
	if (!m_libraryLoader->isLoaded())
		m_libraryLoader->load("winusb.dll");

	m_winUsb.AbortPipe = NULL;
	m_winUsb.Initialize = NULL;
	m_winUsb.Free = NULL;
	m_winUsb.GetAssociatedInterface = NULL;
	m_winUsb.GetDescriptor = NULL;
	m_winUsb.QueryInterfaceSettings = NULL;
	m_winUsb.QueryDeviceInformation = NULL;
	m_winUsb.SetCurrentAlternateSetting = NULL;
	m_winUsb.GetCurrentAlternateSetting = NULL;
	m_winUsb.QueryPipe = NULL;
	m_winUsb.SetPipePolicy = NULL;
	m_winUsb.GetPipePolicy = NULL;
	m_winUsb.ReadPipe = NULL;
	m_winUsb.WritePipe = NULL;
	m_winUsb.ControlTransfer = NULL;
	m_winUsb.ResetPipe = NULL;
	m_winUsb.AbortPipe = NULL;
	m_winUsb.FlushPipe = NULL;
	m_winUsb.SetPowerPolicy = NULL;
	m_winUsb.GetPowerPolicy = NULL;
	m_winUsb.GetOverlappedResult = NULL;

	if (m_libraryLoader->isLoaded())
	{
		m_winUsb.AbortPipe = (WinUSB_AbortPipe*)m_libraryLoader->resolve("WinUsb_AbortPipe");
		m_winUsb.Initialize = (WinUSB_Initialize*)m_libraryLoader->resolve("WinUsb_Initialize");
		m_winUsb.Free = (WinUSB_Free*)m_libraryLoader->resolve("WinUsb_Free");
		m_winUsb.GetAssociatedInterface = (WinUSB_GetAssociatedInterface*)m_libraryLoader->resolve("WinUsb_GetAssociatedInterface");
		m_winUsb.GetDescriptor = (WinUSB_GetDescriptor*)m_libraryLoader->resolve("WinUsb_GetDescriptor");
		m_winUsb.QueryInterfaceSettings = (WinUSB_QueryInterfaceSettings*)m_libraryLoader->resolve("WinUsb_QueryInterfaceSettings");
		m_winUsb.QueryDeviceInformation = (WinUSB_QueryDeviceInformation*)m_libraryLoader->resolve("WinUsb_QueryDeviceInformation");
		m_winUsb.SetCurrentAlternateSetting = (WinUSB_SetCurrentAlternateSetting*)m_libraryLoader->resolve("WinUsb_SetCurrentAlternateSetting");
		m_winUsb.GetCurrentAlternateSetting = (WinUSB_GetCurrentAlternateSetting*)m_libraryLoader->resolve("WinUsb_GetCurrentAlternateSetting");
		m_winUsb.QueryPipe = (WinUSB_QueryPipe*)m_libraryLoader->resolve("WinUsb_QueryPipe");
		m_winUsb.SetPipePolicy = (WinUSB_SetPipePolicy*)m_libraryLoader->resolve("WinUsb_SetPipePolicy");
		m_winUsb.GetPipePolicy = (WinUSB_GetPipePolicy*)m_libraryLoader->resolve("WinUsb_GetPipePolicy");
		m_winUsb.ReadPipe = (WinUSB_ReadPipe*)m_libraryLoader->resolve("WinUsb_ReadPipe");
		m_winUsb.WritePipe = (WinUSB_WritePipe*)m_libraryLoader->resolve("WinUsb_WritePipe");
		m_winUsb.ControlTransfer = (WinUSB_ControlTransfer*)m_libraryLoader->resolve("WinUsb_ControlTransfer");
		m_winUsb.ResetPipe = (WinUSB_ResetPipe*)m_libraryLoader->resolve("WinUsb_ResetPipe");
		m_winUsb.AbortPipe = (WinUSB_AbortPipe*)m_libraryLoader->resolve("WinUsb_AbortPipe");
		m_winUsb.FlushPipe = (WinUSB_FlushPipe*)m_libraryLoader->resolve("WinUsb_FlushPipe");
		m_winUsb.SetPowerPolicy = (WinUSB_SetPowerPolicy*)m_libraryLoader->resolve("WinUsb_SetPowerPolicy");
		m_winUsb.GetPowerPolicy = (WinUSB_GetPowerPolicy*)m_libraryLoader->resolve("WinUsb_GetPowerPolicy");
		m_winUsb.GetOverlappedResult = (WinUSB_GetOverlappedResult*)m_libraryLoader->resolve("WinUsb_GetOverlappedResult");
	}
}

/*! \brief Creates/opens a WinUsb interface handle from the device list.

	\param[out] InterfaceHandle	Receives a handle configured to the first (default) interface on the device. This handle is required by other WinUsb routines that perform operations on the default interface. The handle is opaque. To release this handle, call the \a Free function.
	\param[in]	DevInfo	The device list element to open.

	\returns On success, TRUE. Otherwise FALSE. Use GetLastError() to get extended error information.
*/
BOOL XsWinUsb::Initialize(HANDLE DeviceHandle, PWINUSB_INTERFACE_HANDLE InterfaceHandle)
{
	if (m_winUsb.Initialize)
		return m_winUsb.Initialize(DeviceHandle, InterfaceHandle);
	else
		return FALSE;
}

/*! \brief Frees a WinUsb interface handle.
	\param[in] InterfaceHandle	Handle to an interface on the device. This handle must be created by a previous call to see \a Initialize or \a GetAssociatedInterface.

	\returns TRUE

	\sa GetAssociatedInterface.
*/
BOOL XsWinUsb::Free(WINUSB_INTERFACE_HANDLE InterfaceHandle)
{
	if (m_winUsb.Free)
		return m_winUsb.Free(InterfaceHandle);
	else
		return FALSE;
}

/*! \brief Retrieves a handle for an associated interface.
	\param[in]	InterfaceHandle	An initialized usb handle, see \a Initialize.
	\param[in]	AssociatedInterfaceIndex	An index that specifies the associated interface to retrieve. A value of 0 indicates the first associated interface, a value of 1 indicates the second associated interface, and so on.
	\param[out]	AssociatedInterfaceHandle	A handle for the associated interface. Callers must pass this interface handle to WinUsb Functions exposed by WinUsb.dll. To close this handle, call Free.

	\returns On success, TRUE. Otherwise FALSE. Use GetLastError() to get extended error information.
*/
BOOL XsWinUsb::GetAssociatedInterface(WINUSB_INTERFACE_HANDLE InterfaceHandle,UCHAR AssociatedInterfaceIndex, PWINUSB_INTERFACE_HANDLE AssociatedInterfaceHandle)
{
	if (m_winUsb.GetAssociatedInterface)
		return m_winUsb.GetAssociatedInterface(InterfaceHandle, AssociatedInterfaceIndex, AssociatedInterfaceHandle);
	else
		return FALSE;
}

/*! \brief Gets the requested descriptor. This is a synchronous operation.
	\param[in]	InterfaceHandle	An initialized usb handle, see \a Initialize.
	\param[in]	DescriptorType	A value that specifies the type of descriptor to return. This parameter corresponds to the bDescriptorType field of a standard device descriptor, whose values are described in the Universal Serial Bus specification.
	\param[in]	Index	The descriptor index. For an explanation of the descriptor index, see the Universal Serial Bus specification (www.usb.org).
	\param[in]	LanguageID	A value that specifies the language identifier, if the requested descriptor is a string descriptor.
	\param[out]	Buffer	A caller-allocated buffer that receives the requested descriptor.
	\param[in]	BufferLength	The length, in bytes, of Buffer.
	\param[out]	LengthTransferred	The number of bytes that were copied into Buffer.

	\returns On success, TRUE. Otherwise FALSE. Use GetLastError() to get extended error information.
*/
BOOL XsWinUsb::GetDescriptor(WINUSB_INTERFACE_HANDLE InterfaceHandle,UCHAR DescriptorType, UCHAR Index,USHORT LanguageID,PUCHAR Buffer,ULONG BufferLength,PULONG LengthTransferred)
{
	if (m_winUsb.GetDescriptor)
		return m_winUsb.GetDescriptor(InterfaceHandle, DescriptorType, Index, LanguageID, Buffer, BufferLength, LengthTransferred);
	else
		return FALSE;
}

/*! \brief Retrieves the interface descriptor for the specified alternate interface settings for a particular interface handle.

	The \a QueryInterfaceSettings call searches the current/default interface array for the alternate interface specified by the caller in the \a AltSettingIndex.
	If the specified alternate interface is found, the function populates the caller-allocated USB_INTERFACE_DESCRIPTOR structure.
	If the specified alternate interface is not found, then the call fails with the ERROR_NO_MORE_ITEMS code.

	\param[in]	InterfaceHandle	An initialized usb handle, see \a Initialize.
	\param[in]	AltSettingIndex	A value that indicates which alternate setting index to return. A value of 0 indicates the first alternate setting, a value of 1 indicates the second alternate setting, and so on.
	\param[out]	UsbAltInterfaceDescriptor	A pointer to a caller-allocated USB_INTERFACE_DESCRIPTOR structure that contains information about the interface that AltSettingNumber specified.

	\returns On success, TRUE. Otherwise FALSE. Use GetLastError() to get extended error information.
*/
BOOL XsWinUsb::QueryInterfaceSettings(WINUSB_INTERFACE_HANDLE InterfaceHandle,UCHAR AlternateInterfaceNumber,PUSB_INTERFACE_DESCRIPTOR UsbAltInterfaceDescriptor)
{
	if (m_winUsb.QueryInterfaceSettings)
		return m_winUsb.QueryInterfaceSettings(InterfaceHandle, AlternateInterfaceNumber, UsbAltInterfaceDescriptor);
	else
		return FALSE;
}

/*! \brief Retrieves information about the physical device that is associated with a WinUSB handle.
	\param[in]	InterfaceHandle	An initialized usb handle, see \a Initialize.
	\param[in]	InformationType	A value that specifies which interface information value to retrieve. On input, InformationType must have the following value: DEVICE_SPEED (0x01).
	\param[in,out]	BufferLength	The maximum number of bytes to read. This number must be less than or equal to the size, in bytes, of Buffer. On output, BufferLength is set to the actual number of bytes that were copied into Buffer.
	\param[in,out]	Buffer	A caller-allocated buffer that receives the requested value. On output, Buffer indicates the device speed:
					(0x01) low/full speed device.
					(0x03) high speed device.

	\returns On success, TRUE. Otherwise FALSE. Use GetLastError() to get extended error information.
*/
BOOL XsWinUsb::QueryDeviceInformation(WINUSB_INTERFACE_HANDLE InterfaceHandle,ULONG InformationType,PULONG BufferLength,PVOID Buffer)
{
	if (m_winUsb.QueryDeviceInformation)
		return m_winUsb.QueryDeviceInformation(InterfaceHandle, InformationType, BufferLength, Buffer);
	else
		return FALSE;
}

/*! \brief Sets the alternate setting of an interface.
	Sets the active \a bAlternateSetting for the current/default interface.

	\param[in]	InterfaceHandle	An initialized usb handle, see \a Initialize.
	\param[in]	AltSettingNumber	The value that is contained in the \a bAlternateSetting member of the USB_INTERFACE_DESCRIPTOR structure. This structure can be populated by the \a QueryInterfaceSettings routine.

	\returns On success, TRUE. Otherwise FALSE. Use GetLastError() to get extended error information.

	\sa QueryInterfaceSettings
*/
BOOL XsWinUsb::SetCurrentAlternateSetting(WINUSB_INTERFACE_HANDLE InterfaceHandle,UCHAR SettingNumber)
{
	if (m_winUsb.SetCurrentAlternateSetting)
		return m_winUsb.SetCurrentAlternateSetting(InterfaceHandle, SettingNumber);
	else
		return FALSE;
}

/*! \brief Gets the current alternate interface setting for an interface.
	Gets the active bAlternateSetting for the current/default interface.

	\param[in]	InterfaceHandle	An initialized usb handle, see \a Initialize.
	\param[out]	AltSettingNumber	A pointer to an unsigned character that receives an integer that indicates the current alternate setting.

	\returns On success, TRUE. Otherwise FALSE. Use GetLastError() to get extended error information.
*/
BOOL XsWinUsb::GetCurrentAlternateSetting(WINUSB_INTERFACE_HANDLE InterfaceHandle,PUCHAR SettingNumber)
{
	if (m_winUsb.GetCurrentAlternateSetting)
		return m_winUsb.GetCurrentAlternateSetting(InterfaceHandle, SettingNumber);
	else
		return FALSE;
}

/*! \brief Retrieves information about a pipe that is associated with an interface.
	The \a QueryPipe function does not retrieve information about the control pipe.

	Each interface on the USB device can have multiple endpoints. To communicate with each of these endpoints, the bus driver creates pipes for each endpoint on the interface.
	The pipe indices are zero-based. Therefore for n number of endpoints, the pipes' indices are set from n-1.
	\a QueryPipe parses the configuration descriptor to get the interface specified by the caller.
	It searches the interface descriptor for the endpoint descriptor associated with the caller-specified pipe.
	If the endpoint is found, the function populates the caller-allocated WINUSB_PIPE_INFORMATION structure with information from the endpoint descriptor.

	\param[in]	InterfaceHandle	An initialized usb handle, see \a Initialize.
	\param[in]	AltSettingNumber	A value that specifies the alternate interface to return the information for.
	\param[in]	PipeIndex	A value that specifies the pipe to return information about. This value is not the same as the bEndpointAddress field in the endpoint descriptor.
				A PipeIndex value of 0 signifies the first endpoint that is associated with the interface, a value of 1 signifies the second endpoint, and so on.
				PipeIndex must be less than the value in the bNumEndpoints field of the interface descriptor.
	\param[out]	PipeInformation	A pointer, on output, to a caller-allocated WINUSB_PIPE_INFORMATION structure that contains pipe information.

	\returns On success, TRUE. Otherwise FALSE. Use GetLastError() to get extended error information.
*/
BOOL XsWinUsb::QueryPipe(WINUSB_INTERFACE_HANDLE InterfaceHandle,UCHAR AlternateInterfaceNumber,UCHAR PipeIndex,PWINUSB_PIPE_INFORMATION PipeInformation)
{
	if (m_winUsb.QueryPipe)
		return m_winUsb.QueryPipe(InterfaceHandle, AlternateInterfaceNumber, PipeIndex, PipeInformation);
	else
		return FALSE;
}

/*! \brief Sets the policy for a specific pipe associated with an endpoint on the device. This is a synchronous operation.
	\param[in]	InterfaceHandle	An initialized usb handle, see \a Initialize.
	\param[in]	PipeID	An 8-bit value that consists of a 7-bit address and a direction bit. This parameter corresponds to the bEndpointAddress field in the endpoint descriptor.
	\param[in]	PolicyType	A UINT variable that specifies the policy parameter to change. The Value parameter contains the new value for the policy parameter.
				See the remarks section for information about each of the pipe policies and the resulting behavior.
	\param[in]	ValueLength	The size, in bytes, of the buffer at Value.
	\param[in]	Value	The new value for the policy parameter that PolicyType specifies. The size of this input parameter depends on the policy to change. For information about the size of this parameter, see the description of the PolicyType parameter.

	\remarks
	The following list describes symbolic constants for \a PolicyType

	- SHORT_PACKET_TERMINATE (0x01)
		- The default value is FALSE.
		- To enable SHORT_PACKET_TERMINATE, in Value pass the address of a caller-allocated UCHAR variable set to TRUE (nonzero).
		- Enabling SHORT_PACKET_TERMINATE causes the driver to send a zero-length packet at the end of every write request to the host controller.

	- AUTO_CLEAR_STALL (0x02)
		- The default value is FALSE. To enable AUTO_CLEAR_STALL, in Value pass the address of a caller-allocated UCHAR variable set to TRUE (nonzero).
		- Enabling AUTO_CLEAR_STALL causes winUSB to reset the pipe in order to automatically clear the stall condition. Data continues to flow on the bulk and interrupt IN endpoints again as soon as a new or a queued transfer arrives on the endpoint. This policy parameter does not affect control pipes.
		- Disabling AUTO_CLEAR_STALL causes all transfers (that arrive to the endpoint after the stalled transfer) to fail until the caller manually resets the endpoint's pipe by calling ResetPipe.

	- PIPE_TRANSFER_TIMEOUT (0x03)
		- The default value is zero. To set a time-out value, in Value pass the address of a caller-allocated UINT variable that contains the time-out interval.
		- The PIPE_TRANSFER_TIMEOUT value specifies the time-out interval, in milliseconds. The host controller cancels transfers that do not complete within the specified time-out interval.
		- A value of zero (default) indicates that transfers do not time out because the host controller never cancels the transfer.

	- IGNORE_SHORT_PACKETS (0x04)
		- The default value is FALSE. To enable IGNORE_SHORT_PACKETS, in Value pass the address of a caller-allocated UCHAR variable set to TRUE (nonzero).
		- Enabling IGNORE_SHORT_PACKETS causes the host controller to not complete a read operation after it receives a short packet. Instead, the host controller completes the operation only after the host has read the specified number of bytes.
		- Disabling IGNORE_SHORT_PACKETS causes the host controller to complete a read operation when either the host has read the specified number of bytes or the host has received a short packet.

	- ALLOW_PARTIAL_READS (0x05)
		- The default value is TRUE (nonzero). To disable ALLOW_PARTIAL_READS, in Value pass the address of a caller-allocated UCHAR variable set to FALSE (zero).
		- Disabling ALLOW_PARTIAL_READS causes the read requests to fail whenever the device returns more data (on bulk and interrupt IN endpoints) than the caller requested.
		- Enabling ALLOW_PARTIAL_READS causes winUSB to save or discard the extra data when the device returns more data (on bulk and interrupt IN endpoints) than the caller requested. This behavior is defined by setting the AUTO_FLUSH value.

	- AUTO_FLUSH (0x06)
		- The default value is FALSE (zero). To enable AUTO_FLUSH, in Value pass the address of a caller-allocated UCHAR variable set to TRUE (nonzero).
		- AUTO_FLUSH must be used with ALLOW_PARTIAL_READS enabled. If ALLOW_PARTIAL_READS is TRUE, the value of AUTO_FLUSH determines the action taken by winUSB when the device returns more data than the caller requested.
		- Disabling ALLOW_PARTIAL_READS causes winUSB to ignore the AUTO_FLUSH value.
		- Disabling AUTO_FLUSH with ALLOW_PARTIAL_READS enabled causes winUSB to save the extra data, add the data to the beginning of the caller's next read request, and send it to the caller in the next read operation.
		- Enabling AUTO_FLUSH with ALLOW_PARTIAL_READS enabled causes winUSB to discard the extra data remaining from the read request.

	- RAW_IO (0x07)
		- The default value is FALSE (zero). To enable RAW_IO, in Value pass the address of a caller-allocated UCHAR variable set to TRUE (nonzero).
		- Enabling RAW_IO causes winUSB to send data directly to the USB driver stack, bypassing winUSB's queuing and error handling mechanism.
		- The buffers that are passed to \a ReadPipe must be configured by the caller as follows:
		- The buffer length must be a multiple of the maximum endpoint packet size.
		- The length must be less than or equal to the value of MAXIMUM_TRANSFER_SIZE retrieved by GetPipePolicy.
		- Disabling RAW_IO (FALSE) does not impose any restriction on the buffers that are passed to \a ReadPipe.

	- RESET_PIPE_ON_RESUME (0x09)
		- The default value is FALSE (zero). To enable RESET_PIPE_ON_RESUME, in Value pass the address of a caller-allocated UCHAR variable set to TRUE (nonzero).
		- TRUE (or a nonzero value) indicates that on resume from suspend, winUSB resets the endpoint before it allows the caller to send new requests to the endpoint.


	\returns On success, TRUE. Otherwise FALSE. Use GetLastError() to get extended error information.
*/
BOOL XsWinUsb::SetPipePolicy(WINUSB_INTERFACE_HANDLE InterfaceHandle,UCHAR PipeID,ULONG PolicyType,ULONG ValueLength,PVOID Value)
{
	if (m_winUsb.SetPipePolicy)
		return m_winUsb.SetPipePolicy(InterfaceHandle, PipeID, PolicyType, ValueLength, Value);
	else
		return FALSE;
}

/*! \brief Gets the policy for a specific pipe (endpoint).
	\param[in]	InterfaceHandle	An initialized usb handle, see \a Initialize.
	\param[in]	PipeID	An 8-bit value that consists of a 7-bit address and a direction bit. This parameter corresponds to the bEndpointAddress field in the endpoint descriptor.
	\param[in]	PolicyType	A UINT variable that specifies the policy parameter to retrieve. The current value for the policy parameter is retrieved the Value parameter.
	\param[in,out]	ValueLength	A pointer to the size, in bytes, of the buffer that Value points to. On output, ValueLength receives the size, in bytes, of the data that was copied into the Value buffer.
	\param[out]	Value	A pointer to a buffer that receives the specified pipe policy value.

	\returns On success, TRUE. Otherwise FALSE. Use GetLastError() to get extended error information.
*/
BOOL XsWinUsb::GetPipePolicy(WINUSB_INTERFACE_HANDLE InterfaceHandle,UCHAR PipeID,ULONG PolicyType,PULONG ValueLength,PVOID Value)
{
	if (m_winUsb.GetPipePolicy)
		return m_winUsb.GetPipePolicy(InterfaceHandle, PipeID, PolicyType, ValueLength, Value);
	else
		return FALSE;
}

/*! \brief Reads data from the specified pipe.
	\param[in]	InterfaceHandle	An initialized usb handle, see \a Initialize.
	\param[in]	PipeID	An 8-bit value that consists of a 7-bit address and a direction bit. This parameter corresponds to the bEndpointAddress field in the endpoint descriptor.
	\param[out]	Buffer	A caller-allocated buffer that receives the data that is read.
	\param[in]	BufferLength	The maximum number of bytes to read. This number must be less than or equal to the size, in bytes, of Buffer.
	\param[out]	LengthTransferred	A pointer to a UINT variable that receives the actual number of bytes that were copied into Buffer. For more information, see Remarks.
	\param[in]	Overlapped	An optional pointer to an overlapped structure for asynchronous operations. This can be a KOVL_HANDLE or a pointer to a standard windows OVERLAPPED structure.
			If this parameter is specified, \a ReadPipe returns immediately rather than waiting synchronously for the operation to complete before returning. An event is signaled when the operation is complete.

	\returns On success, TRUE. Otherwise FALSE. Use GetLastError() to get extended error information.
*/
BOOL XsWinUsb::ReadPipe(WINUSB_INTERFACE_HANDLE InterfaceHandle,UCHAR PipeID,PUCHAR Buffer,ULONG BufferLength,PULONG LengthTransferred,LPOVERLAPPED Overlapped)
{
	if (m_winUsb.ReadPipe)
		return m_winUsb.ReadPipe(InterfaceHandle, PipeID, Buffer, BufferLength, LengthTransferred, Overlapped);
	else
		return FALSE;
}

/*! \brief Writes data to a pipe.
	\param[in]	InterfaceHandle	An initialized usb handle, see \a Initialize.
	\param[in]	PipeID	An 8-bit value that consists of a 7-bit address and a direction bit. This parameter corresponds to the bEndpointAddress field in the endpoint descriptor.
	\param[in]	Buffer	A caller-allocated buffer the data is written from.
	\param[in]	BufferLength	The maximum number of bytes to write. This number must be less than or equal to the size, in bytes, of Buffer.
	\param[out]	LengthTransferred	A pointer to a UINT variable that receives the actual number of bytes that were transferred from Buffer.
	\param[in]	Overlapped	An optional pointer to an overlapped structure for asynchronous operations. This can be a KOVL_HANDLE or a pointer to a standard windows OVERLAPPED structure.
				If this parameter is specified, \a WritePipe returns immediately rather than waiting synchronously for the operation to complete before returning. An event is signaled when the operation is complete.

	\returns On success, TRUE. Otherwise FALSE. Use GetLastError() to get extended error information.
*/
BOOL XsWinUsb::WritePipe(WINUSB_INTERFACE_HANDLE InterfaceHandle,UCHAR PipeID,PUCHAR Buffer,ULONG BufferLength,PULONG LengthTransferred,LPOVERLAPPED Overlapped)
{
	if (m_winUsb.WritePipe)
		return m_winUsb.WritePipe(InterfaceHandle, PipeID, Buffer, BufferLength, LengthTransferred, Overlapped);
	else
		return FALSE;
}

/*! \brief Transmits control data over a default control endpoint.
	A ControlTransfer is never cached. These requests always go directly to the usb device.

	\param[in]	InterfaceHandle	A valid winUSB interface handle returned by:
				- Initialize
				- GetAssociatedInterface
	\param[in]	SetupPacket	The 8-byte setup packet of type WINUSB_SETUP_PACKET.
	\param[in,out]	Buffer	A caller-allocated buffer that contains the data to transfer.
	\param[in]	BufferLength	The number of bytes to transfer, not including the setup packet. This number must be less than or equal to the size, in bytes, of Buffer.
	\param[out]	LengthTransferred	A pointer to a UINT variable that receives the actual number of transferred bytes. If the application does not expect any data to be transferred during the data phase (BufferLength is zero), LengthTransferred can be NULL.
	\param[in]	Overlapped	An optional pointer to an OVERLAPPED structure, which is used for asynchronous operations. If this parameter is specified, ControlTransfer immediately returns, and the event is signaled when the operation is complete. If Overlapped is not supplied, the ControlTransfer function transfers data synchronously.

	\returns On success, TRUE. Otherwise FALSE. Use GetLastError() to get extended error information. If an Overlapped member is supplied and the operation succeeds this function returns FALSE and sets last error to ERROR_IO_PENDING.
*/
BOOL XsWinUsb::ControlTransfer(WINUSB_INTERFACE_HANDLE InterfaceHandle,WINUSB_SETUP_PACKET SetupPacket,PUCHAR Buffer,ULONG BufferLength,PULONG LengthTransferred,LPOVERLAPPED Overlapped)
{
	if (m_winUsb.ControlTransfer)
		return m_winUsb.ControlTransfer(InterfaceHandle, SetupPacket, Buffer, BufferLength, LengthTransferred, Overlapped);
	else
		return FALSE;
}

/*! \brief Resets the data toggle and clears the stall condition on a pipe.
	\param[in]	InterfaceHandle	An initialized usb handle, see \a Initialize.
	\param[in]	PipeID	An 8-bit value that consists of a 7-bit address and a direction bit. This parameter corresponds to the \a bEndpointAddress field in the endpoint descriptor.

	\returns On success, TRUE. Otherwise FALSE. Use GetLastError() to get extended error information.
*/
BOOL XsWinUsb::ResetPipe(WINUSB_INTERFACE_HANDLE InterfaceHandle,UCHAR PipeID)
{
	if (m_winUsb.ResetPipe)
		return m_winUsb.ResetPipe(InterfaceHandle, PipeID);
	else
		return FALSE;
}

/*! \brief Aborts all of the pending transfers for a pipe.
	\param[in]	InterfaceHandle	An initialized usb handle, see \a Initialize.
	\param[in]	PipeID	An 8-bit value that consists of a 7-bit address and a direction bit. This parameter corresponds to the \a bEndpointAddress field in the endpoint descriptor.

	\returns On success, TRUE. Otherwise FALSE. Use GetLastError() to get extended error information.
*/
BOOL XsWinUsb::AbortPipe(WINUSB_INTERFACE_HANDLE InterfaceHandle,UCHAR PipeID)
{
	if (m_winUsb.AbortPipe)
		return m_winUsb.AbortPipe(InterfaceHandle, PipeID);
	else
		return FALSE;
}

/*! \brief Discards any data that is cached in a pipe.
	\param[in]	InterfaceHandle	An initialized usb handle, see \a Initialize.
	\param[in]	PipeID	An 8-bit value that consists of a 7-bit address and a direction bit. This parameter corresponds to the \a bEndpointAddress field in the endpoint descriptor.

	\returns On success, TRUE. Otherwise FALSE. Use GetLastError() to get extended error information.
*/
BOOL XsWinUsb::FlushPipe(WINUSB_INTERFACE_HANDLE InterfaceHandle,UCHAR PipeID)
{
	if (m_winUsb.FlushPipe)
		return m_winUsb.FlushPipe(InterfaceHandle, PipeID);
	else
		return FALSE;
}

/*! \brief Sets the power policy for a device.
	The following list summarizes the effects of changes to power management states:

	- All pipe handles, interface handles, locks, and alternate settings are preserved across power management events.
	- Any transfers that are in progress are suspended when a device transfers to a low power state, and they are resumed when the device is restored to a working state.
	- The device and system must be in a working state before the client can restore a device-specific configuration. Clients can determine whether the device and system are in a working state from the WM_POWERBROADCAST message.
	- The client can indicate that an interface is idle by calling \a SetPowerPolicy.

	\param[in]	InterfaceHandle	An initialized usb handle, see \a Initialize.
	\param[in]	PolicyType	A value that specifies the power policy to set. The following table describes symbolic constants.
				- AUTO_SUSPEND (0x81)
					- Specifies the auto-suspend policy type; the power policy parameter must be specified by the caller in the Value parameter.
					- For auto-suspend, the Value parameter must point to a UCHAR variable.
					- If Value is TRUE (nonzero), the USB stack suspends the device if the device is idle. A device is idle if there are no transfers pending, or if the only pending transfers are IN transfers to interrupt or bulk endpoints.
					- The default value is determined by the value set in the DefaultIdleState registry setting. By default, this value is TRUE.

				- SUSPEND_DELAY (0x83)
					- Specifies the suspend-delay policy type; the power policy parameter must be specified by the caller in the Value parameter.
					- For suspend-delay, Value must point to a UINT variable.
					- Value specifies the minimum amount of time, in milliseconds, that the driver must wait post transfer before it can suspend the device.
					- The default value is determined by the value set in the DefaultIdleTimeout registry setting. By default, this value is five seconds.

	\param[in]	ValueLength	The size, in bytes, of the buffer at Value.
	\param[in]	Value	The new value for the power policy parameter. Data type and value for Value depends on the type of power policy passed in PolicyType. For more information, see PolicyType.

	\returns On success, TRUE. Otherwise FALSE. Use GetLastError() to get extended error information.
*/
BOOL XsWinUsb::SetPowerPolicy(WINUSB_INTERFACE_HANDLE InterfaceHandle,ULONG PolicyType,ULONG ValueLength,PVOID Value)
{
	if (m_winUsb.SetPowerPolicy)
		return m_winUsb.SetPowerPolicy(InterfaceHandle, PolicyType, ValueLength, Value);
	else
		return FALSE;
}

/*! \brief Gets the power policy for a device.
	\param[in]	InterfaceHandle	An initialized usb handle, see \a Initialize
	\param[in]	PolicyType	A value that specifies the power policy parameter to retrieve in Value. The following table describes symbolic constants that are valid.
				- AUTO_SUSPEND (0x81)
					- If the caller specifies a power policy of AUTO_SUSPEND, \a GetPowerPolicy returns the value of the auto suspend policy parameter in the Value parameter.
					- If Value is TRUE (that is, nonzero), the USB stack suspends the device when no transfers are pending or the only transfers pending are IN transfers on an interrupt or bulk endpoint.
					- The value of the DefaultIdleState registry value determines the default value of the auto suspend policy parameter.
					- The Value parameter must point to a UCHAR variable.

				- SUSPEND_DELAY (0x83)
					- If the caller specifies a power policy of SUSPEND_DELAY, \a GetPowerPolicy returns the value of the suspend delay policy parameter in Value.
					- The suspend delay policy parameter specifies the minimum amount of time, in milliseconds, that the driver must wait after any transfer before it can suspend the device.
					- Value must point to a UINT variable.

	\param[in,out]	ValueLength	A pointer to the size of the buffer that Value. On output, ValueLength receives the size of the data that was copied into the Value buffer.
	\param[out]	Value	A buffer that receives the specified power policy parameter. For more information, see PolicyType.

	\returns On success, TRUE. Otherwise FALSE. Use GetLastError() to get extended error information.
*/
BOOL XsWinUsb::GetPowerPolicy(WINUSB_INTERFACE_HANDLE InterfaceHandle,ULONG PolicyType,PULONG ValueLength,PVOID Value)
{
	if (m_winUsb.GetPowerPolicy)
		return m_winUsb.GetPowerPolicy(InterfaceHandle, PolicyType, ValueLength, Value);
	else
		return FALSE;
}

/*! \brief Retrieves the results of an overlapped operation on the specified winUSB handle.

	This function is like the Win32 API routine, GetOverlappedResult, with one difference; instead of passing a file handle that is returned from CreateFile, the caller passes an interface handle that is returned from \a Initialize, or \a GetAssociatedInterface. The caller can use either API routine, if the appropriate handle is passed. The \a GetOverlappedResult function extracts the file handle from the interface handle and then calls GetOverlappedResult.
	The results that are reported by the GetOverlappedResult function are those from the specified handle's last overlapped operation to which the specified standard windows OVERLAPPED structure was provided, and for which the operation's results were pending. A pending operation is indicated when the function that started the operation returns FALSE, and the GetLastError routine returns ERROR_IO_PENDING. When an I/O operation is pending, the function that started the operation resets the hEvent member of the standard windows OVERLAPPED structure to the nonsignaled state. Then when the pending operation has been completed, the system sets the event object to the signaled state.
	The caller can specify that an event object is manually reset in the standard windows OVERLAPPED structure. If an automatic reset event object is used, the event handle must not be specified in any other wait operation in the interval between starting the overlapped operation and the call to \a GetOverlappedResult. For example, the event object is sometimes specified in one of the wait routines to wait for the operation to be completed. When the wait routine returns, the system sets an auto-reset event's state to nonsignaled, and a successive call to \a GetOverlappedResult with the bWait parameter set to TRUE causes the function to be blocked indefinitely.
	If the bWait parameter is TRUE, GetOverlappedResult determines whether the pending operation has been completed by waiting for the event object to be in the signaled state.

	If the hEvent member of the standard windows OVERLAPPED structure is NULL, the system uses the state of the file handle to signal when the operation has been completed. Do not use file handles for this purpose. It is better to use an event object because of the confusion that can occur when multiple concurrent overlapped operations are performed on the same file. In this situation, you cannot know which operation caused the state of the object to be signaled.

	\param[in]	InterfaceHandle	An initialized usb handle, see \a Initialize
	\param[in]	Overlapped	A pointer to a standard windows OVERLAPPED structure that was specified when the overlapped operation was started.
	\param[out]	lpNumberOfBytesTransferred	A pointer to a variable that receives the number of bytes that were actually transferred by a read or write operation.
	\param[in]	bWait	If this parameter is TRUE, the function does not return until the operation has been completed. If this parameter is FALSE and the operation is still pending, the function returns FALSE and the GetLastError function returns ERROR_IO_INCOMPLETE.

	\returns On success, TRUE. Otherwise FALSE. Use GetLastError() to get extended error information.
*/
BOOL XsWinUsb::GetOverlappedResult(WINUSB_INTERFACE_HANDLE InterfaceHandle,LPOVERLAPPED lpOverlapped,LPDWORD lpNumberOfBytesTransferred,BOOL bWait)
{
	if (m_winUsb.GetOverlappedResult)
		return m_winUsb.GetOverlappedResult(InterfaceHandle, lpOverlapped, lpNumberOfBytesTransferred, bWait);
	else
		return FALSE;
}

#endif // patch for MRPT
