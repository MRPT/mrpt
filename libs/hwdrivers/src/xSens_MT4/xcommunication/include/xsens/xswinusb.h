/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef XSWINUSB
#define XSWINUSB

#include <windows.h>
#include <winusb.h>

struct XsLibraryLoader;

typedef BOOL __stdcall WinUSB_Initialize (HANDLE DeviceHandle, WINUSB_INTERFACE_HANDLE* InterfaceHandle);
typedef BOOL __stdcall WinUSB_Free (WINUSB_INTERFACE_HANDLE InterfaceHandle);
typedef BOOL __stdcall WinUSB_GetAssociatedInterface (WINUSB_INTERFACE_HANDLE InterfaceHandle, UCHAR AssociatedInterfaceIndex, WINUSB_INTERFACE_HANDLE* AssociatedInterfaceHandle);
typedef BOOL __stdcall WinUSB_GetDescriptor (WINUSB_INTERFACE_HANDLE InterfaceHandle, UCHAR DescriptorType, UCHAR Index, USHORT LanguageID, PUCHAR Buffer, ULONG BufferLength, PULONG LengthTransferred);
typedef BOOL __stdcall WinUSB_QueryInterfaceSettings (WINUSB_INTERFACE_HANDLE InterfaceHandle, UCHAR AltSettingIndex, PUSB_INTERFACE_DESCRIPTOR UsbAltInterfaceDescriptor);
typedef BOOL __stdcall WinUSB_QueryDeviceInformation (WINUSB_INTERFACE_HANDLE InterfaceHandle, ULONG InformationType,	PULONG BufferLength, PVOID Buffer);
typedef BOOL __stdcall WinUSB_SetCurrentAlternateSetting (WINUSB_INTERFACE_HANDLE InterfaceHandle, UCHAR AltSettingNumber);
typedef BOOL __stdcall WinUSB_GetCurrentAlternateSetting (WINUSB_INTERFACE_HANDLE InterfaceHandle, PUCHAR AltSettingNumber);
typedef BOOL __stdcall WinUSB_QueryPipe (WINUSB_INTERFACE_HANDLE InterfaceHandle, UCHAR AltSettingNumber,	UCHAR PipeIndex, PWINUSB_PIPE_INFORMATION PipeInformation);
typedef BOOL __stdcall WinUSB_SetPipePolicy (WINUSB_INTERFACE_HANDLE InterfaceHandle,	UCHAR PipeID, ULONG PolicyType,	ULONG ValueLength, PVOID Value);
typedef BOOL __stdcall WinUSB_GetPipePolicy (WINUSB_INTERFACE_HANDLE InterfaceHandle,	UCHAR PipeID, ULONG PolicyType, PULONG ValueLength,	PVOID Value);
typedef BOOL __stdcall WinUSB_ReadPipe (WINUSB_INTERFACE_HANDLE InterfaceHandle, UCHAR PipeID, PUCHAR Buffer, ULONG BufferLength, PULONG LengthTransferred, LPOVERLAPPED Overlapped);
typedef BOOL __stdcall WinUSB_WritePipe (WINUSB_INTERFACE_HANDLE InterfaceHandle, UCHAR PipeID, PUCHAR Buffer, ULONG BufferLength, PULONG LengthTransferred, LPOVERLAPPED Overlapped);
typedef BOOL __stdcall WinUSB_ControlTransfer (WINUSB_INTERFACE_HANDLE InterfaceHandle, WINUSB_SETUP_PACKET SetupPacket, PUCHAR Buffer, ULONG BufferLength, PULONG LengthTransferred, LPOVERLAPPED Overlapped);
typedef BOOL __stdcall WinUSB_ResetPipe (WINUSB_INTERFACE_HANDLE InterfaceHandle, UCHAR PipeID);
typedef BOOL __stdcall WinUSB_AbortPipe (WINUSB_INTERFACE_HANDLE InterfaceHandle, UCHAR PipeID);
typedef BOOL __stdcall WinUSB_FlushPipe (WINUSB_INTERFACE_HANDLE InterfaceHandle, UCHAR PipeID);
typedef BOOL __stdcall WinUSB_SetPowerPolicy (WINUSB_INTERFACE_HANDLE InterfaceHandle, ULONG PolicyType, ULONG ValueLength, PVOID Value);
typedef BOOL __stdcall WinUSB_GetPowerPolicy (WINUSB_INTERFACE_HANDLE InterfaceHandle, ULONG PolicyType, PULONG ValueLength, PVOID Value);
typedef BOOL __stdcall WinUSB_GetOverlappedResult (WINUSB_INTERFACE_HANDLE InterfaceHandle, LPOVERLAPPED Overlapped, LPDWORD lpNumberOfBytesTransferred, BOOL bWait);

class XsWinUsb
{
public:
	XsWinUsb(void);
	~XsWinUsb(void);

	WinUSB_Initialize Initialize;
	WinUSB_Free Free;
	WinUSB_GetAssociatedInterface GetAssociatedInterface;
	WinUSB_GetDescriptor GetDescriptor;
	WinUSB_QueryInterfaceSettings QueryInterfaceSettings;
	WinUSB_QueryDeviceInformation QueryDeviceInformation;
	WinUSB_SetCurrentAlternateSetting SetCurrentAlternateSetting;
	WinUSB_GetCurrentAlternateSetting GetCurrentAlternateSetting;
	WinUSB_QueryPipe QueryPipe;
	WinUSB_SetPipePolicy SetPipePolicy;
	WinUSB_GetPipePolicy GetPipePolicy;
	WinUSB_ReadPipe ReadPipe;
	WinUSB_WritePipe WritePipe;
	WinUSB_ControlTransfer ControlTransfer;
	WinUSB_ResetPipe ResetPipe;
	WinUSB_AbortPipe AbortPipe;
	WinUSB_FlushPipe FlushPipe;
	WinUSB_SetPowerPolicy SetPowerPolicy;
	WinUSB_GetPowerPolicy GetPowerPolicy;
	WinUSB_GetOverlappedResult GetOverlappedResult;

private:

	typedef struct _WINUSB_API
	{
		WinUSB_Initialize (* Initialize);
		WinUSB_Free (* Free);
		WinUSB_GetAssociatedInterface (*GetAssociatedInterface);
		WinUSB_GetDescriptor (* GetDescriptor);
		WinUSB_QueryInterfaceSettings (* QueryInterfaceSettings);
		WinUSB_QueryDeviceInformation (* QueryDeviceInformation);
		WinUSB_SetCurrentAlternateSetting (* SetCurrentAlternateSetting);
		WinUSB_GetCurrentAlternateSetting (* GetCurrentAlternateSetting);
		WinUSB_QueryPipe (* QueryPipe);
		WinUSB_SetPipePolicy (* SetPipePolicy);
		WinUSB_GetPipePolicy (* GetPipePolicy);
		WinUSB_ReadPipe (* ReadPipe);
		WinUSB_WritePipe (* WritePipe);
		WinUSB_ControlTransfer (* ControlTransfer);
		WinUSB_ResetPipe (* ResetPipe);
		WinUSB_AbortPipe (* AbortPipe);
		WinUSB_FlushPipe (* FlushPipe);
		WinUSB_SetPowerPolicy (* SetPowerPolicy);
		WinUSB_GetPowerPolicy (* GetPowerPolicy);
		WinUSB_GetOverlappedResult (*GetOverlappedResult);
	} WINUSB_API;
	
	WINUSB_API m_winUsb;
	XsLibraryLoader* m_libraryLoader;

	void initLibrary();
};

#endif // file guard

