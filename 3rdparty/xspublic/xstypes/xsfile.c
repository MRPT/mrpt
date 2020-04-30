
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

#include "xsfile.h"
#include "xsstring.h"
#include <errno.h>
#ifndef _WIN32
#	include <unistd.h>		// close
#	include <sys/ioctl.h>	// ioctl
#	include <fcntl.h>		// open, O_RDWR
#	include <string.h>		// strcpy
#	include <sys/param.h>
#	include <sys/stat.h>
#	include <stdarg.h>
#	include <stdlib.h>
#else
#	include <winbase.h>
#	include <sys/stat.h>
#	include <io.h>
#endif

// helper
static FILE* openFile(const struct XsString* filename, const struct XsString* mode);

/*! \class XsFile
	\brief Encapsulates a file, providing a platform independent interface
*/
/*! \addtogroup cinterface C Interface
	@{
*/


/*!	\relates XsFile
	\brief Frees the resources of this object by closing the file if it is open.
*/
void XsFile_destruct(struct XsFile* thisPtr)
{
	if (thisPtr->m_handle != NULL)
	{
		(void)XsFile_close(thisPtr);
	}
}

/*!	\relates XsFile
	\brief Creates a new binary file with name \a filename, contents of existing files will be discarded
	\param filename name of the file to create
	\param writeOnly passing 0 will create the file using "w+b", any other value will create it using "wb"
	\returns XRV_OK if the file was opened, an error otherwise
*/
XsResultValue XsFile_create(struct XsFile *thisPtr, const struct XsString* filename, int writeOnly)
{
	XsString mode;
	XsString_construct(&mode);
	XsString_resize(&mode, 16);

	if (thisPtr->m_handle != NULL)
		return XRV_ALREADYOPEN;

#ifdef _WIN32
	wchar_t filenameW[XS_MAX_FILENAME_LENGTH];
	// actively delete the file first to ensure that the file creation time is properly set
	XsString_copyToWCharArray(filename, filenameW, XS_MAX_FILENAME_LENGTH);
	(void) _wunlink(filenameW);	// don't care about return value
#else
	unlink(filename->m_data);
#endif

	if (writeOnly)
	{
		XsString_assign(&mode, 3, "wb");
		thisPtr->m_handle = openFile(filename, &mode);
	}
	else
	{
		XsString_assign(&mode, 4, "w+b");
		thisPtr->m_handle = openFile(filename, &mode);
	}

	XsString_destruct(&mode);
	if (thisPtr->m_handle == NULL)
		return XRV_OUTPUTCANNOTBEOPENED;
	else
		return XRV_OK;
}

/*!	\relates XsFile
	\brief Creates a new text file with name \a filename, contents of existing files will be discarded
	\param filename name of the file to create
	\param writeOnly passing 0 will create the file using "w+t", any other value will create it using "wt"
	\returns XRV_OK if the file was opened, an error otherwise
*/
XsResultValue XsFile_createText(struct XsFile *thisPtr, const struct XsString* filename, int writeOnly)
{
	XsString mode;
	XsString_construct(&mode);
	XsString_resize(&mode, 16);

	if (thisPtr->m_handle != NULL)
		return XRV_ALREADYOPEN;

	if (writeOnly)
	{
		XsString_assign(&mode, 3, "wt");
		thisPtr->m_handle = openFile(filename, &mode);
	}
	else
	{
		XsString_assign(&mode, 4, "w+t");
		thisPtr->m_handle = openFile(filename, &mode);
	}

	XsString_destruct(&mode);
	if (thisPtr->m_handle == NULL)
		return XRV_OUTPUTCANNOTBEOPENED;
	else
		return XRV_OK;
}

/*!	\relates XsFile
	\brief Opens an existing binary file with name \a filename
	\param filename name of the file to open
	\param readOnly passing 0 will open the file "r+b" (read/update), any other value will create it using "rb" (read)
	\returns XRV_OK if the file was opened, an error otherwise
*/
XsResultValue XsFile_open(struct XsFile *thisPtr, const struct XsString* filename, int readOnly)
{
	XsString mode;
	XsString_construct(&mode);
	XsString_resize(&mode, 16);

	if (thisPtr->m_handle != NULL)
		return XRV_ALREADYOPEN;

	if (readOnly)
	{
		XsString_assign(&mode, 3, "rb");
		thisPtr->m_handle = openFile(filename, &mode);
	}
	else
	{
		XsString_assign(&mode, 4, "r+b");
		thisPtr->m_handle = openFile(filename, &mode);
	}

	XsString_destruct(&mode);
	if (thisPtr->m_handle == NULL)
		return XRV_OUTPUTCANNOTBEOPENED;
	else
		return XRV_OK;
}

/*!	\relates XsFile
	\brief Opens an existing binary file with name \a filename
	\param filename name of the file to open
	\param readOnly passing 0 will open the file "r+t" (read/update), any other value will create it using "rt" (read)
	\returns XRV_OK if the file was opened, an error otherwise
*/
XsResultValue XsFile_openText(struct XsFile *thisPtr, const struct XsString* filename, int readOnly)
{
	XsString mode;
	XsString_construct(&mode);
	XsString_resize(&mode, 16);

	if (thisPtr->m_handle != NULL)
		return XRV_ALREADYOPEN;

	if (readOnly)
	{
		XsString_assign(&mode, 3, "rt");
		thisPtr->m_handle = openFile(filename, &mode);
	}
	else
	{
		XsString_assign(&mode, 4, "r+t");
		thisPtr->m_handle = openFile(filename, &mode);
	}

	XsString_destruct(&mode);
	if (thisPtr->m_handle == NULL)
		return XRV_OUTPUTCANNOTBEOPENED;
	else
		return XRV_OK;
}

/*! \brief Helper for file opening */
static FILE* openFile(const struct XsString* filename, const struct XsString* mode)
{
#ifdef _WIN32
	wchar_t filenameW[XS_MAX_FILENAME_LENGTH];
	wchar_t modeW[16];
	(void)XsString_copyToWCharArray(filename, filenameW, XS_MAX_FILENAME_LENGTH);
	(void)XsString_copyToWCharArray(mode, modeW, 16);

	__try
	{
		return _wfopen(filenameW, modeW);
	}
	__except(EXCEPTION_EXECUTE_HANDLER)
	{
		return NULL;
	}
#else
	return fopen(filename->m_data, mode->m_data);
#endif
}

/*!	\relates XsFile
	\brief Reopens a file
	\param filename Name of the file to open after the current one has been closed
	\param mode Mode to reopen the file with
	\returns 0 if a file is open, another value otherwise
*/
XsResultValue XsFile_reopen(struct XsFile *thisPtr, const struct XsString* filename, const struct XsString* mode)
{
#ifdef _WIN32
	wchar_t filenameW[XS_MAX_FILENAME_LENGTH];
	wchar_t modeW[16];

	(void)XsString_copyToWCharArray(filename, filenameW, XS_MAX_FILENAME_LENGTH);
	(void)XsString_copyToWCharArray(mode, modeW, 16);

	thisPtr->m_handle = _wfreopen(filenameW, modeW, thisPtr->m_handle);
#else
	thisPtr->m_handle = freopen(filename->m_data, mode->m_data, thisPtr->m_handle);
#endif

	if (thisPtr->m_handle == NULL)
		return XRV_OUTPUTCANNOTBEOPENED;
	else
		return XRV_OK;
}

/*!	\relates XsFile
	\brief Checks if a file is open
	\returns 0 if a file is open, another value otherwise
*/
int XsFile_isOpen(const struct XsFile *thisPtr)
{
	return (thisPtr->m_handle != NULL) ? 0 : 1;
}

/*! \relates XsFile
	\brief Checks if the file exists (can be accessed)
	\param filename Name of the file to check for existence
	\returns 0 if the file exists, another value otherwise
*/
int XsFile_exists(const struct XsString* filename)
{
#ifdef _WIN32
	wchar_t filenameW[XS_MAX_FILENAME_LENGTH];
	struct _stat buffer;
	(void)XsString_copyToWCharArray(filename, filenameW, XS_MAX_FILENAME_LENGTH);
	return _wstat(filenameW, &buffer);
#else
	struct stat buffer;
	return stat(filename->m_data, &buffer);
#endif
}

/*!	\relates XsFile
	\brief Closes the file
	\returns XRV_OK if the file was closed properly, XRV_ENDOFFILE otherwise
*/
XsResultValue XsFile_close(struct XsFile *thisPtr)
{
	int rv;

	if (thisPtr->m_handle == NULL)
		return XRV_NOFILEOPEN;

	fflush(thisPtr->m_handle);	// always try to flush first, result is irrelevant
	rv = fclose(thisPtr->m_handle);

	// Or keep the handle in case of failure?
	thisPtr->m_handle = NULL;

	if (rv == EOF)
		return XRV_ENDOFFILE;
	else
		return XRV_OK;
}


/*!	\relates XsFile
	\brief Writes unwritten data to the file
	\returns XRV_OK if the flushing was succesful, an XRV_ERROR otherwise

*/
XsResultValue XsFile_flush(struct XsFile *thisPtr)
{
	return fflush(thisPtr->m_handle) ? XRV_ERROR : XRV_OK;
}

/*!	\relates XsFile
	\brief Reduces the file to a maximum size of \a fileSize bytes
	\param fileSize The new size for the file
	\returns XRV_OK if the file was truncated, an error otherwise
*/
XsResultValue XsFile_truncate(struct XsFile *thisPtr, XsFilePos fileSize)
{
	return XsFile_resize(thisPtr, fileSize);
}

/*!	\relates XsFile
	\brief Resizes the file to \a fileSize bytes
	\param fileSize The new size for the file
	\returns XRV_OK if the file was resized, an error otherwise
*/
XsResultValue XsFile_resize(struct XsFile *thisPtr, XsFilePos fileSize)
{
#ifdef _WIN32
	int rv = _chsize_s(_fileno(thisPtr->m_handle), fileSize);
#else
	int rv = ftruncate(fileno(thisPtr->m_handle), fileSize);
#endif
	if (rv != 0)
	{
		switch(errno)
		{
		case EACCES:
			return XRV_BUSY;
		case EBADF:
			return XRV_ACCESSDENIED;
		case ENOSPC:
			return XRV_OUTOFMEMORY;
		case EINVAL:
			return XRV_INVALIDPARAM;
		default:
			return XRV_ERROR;
		}
	}
	else
		return XRV_OK;
}

/*!	\relates XsFile
	\brief Deletes a file with name \a filename
	\param filename Name of the file to delete
	\returns XRV_OK if the file was erased, an error otherwise
*/
XsResultValue XsFile_erase(const struct XsString* filename)
{
#ifdef _WIN32
	wchar_t filenameW[XS_MAX_FILENAME_LENGTH];
	(void)XsString_copyToWCharArray(filename, filenameW, XS_MAX_FILENAME_LENGTH);
	if (_wunlink(filenameW) != 0)
#else
	if (unlink(filename->m_data) != 0)
#endif
	{
		switch (errno)
		{
		case EACCES:
			return XRV_READONLY;
		case ENOENT:
			return XRV_NOTFOUND;
		default:
			return XRV_ERROR;
		}
	}
	else
		return XRV_OK;
}

/*!	\relates XsFile
	\brief Reads a number of elements from a file
	\param destination Buffer to store the read data in
	\param size Size of each individual element to read
	\param count Number of elements to read
	\returns Total number of elements successfully read
*/
XsFilePos XsFile_read(struct XsFile *thisPtr, void *destination, XsFilePos size, XsFilePos count)
{
	return (XsFilePos) fread(destination, (size_t) size, (size_t) count, thisPtr->m_handle);
}

/*!	\relates XsFile
	\brief Writes a number of elements to a file
	\param source Buffer that contains the elements to be written
	\param size Size of each individual element to write
	\param count Number of elements to write
	\returns Total number of elements successfully written
*/
XsFilePos XsFile_write(struct XsFile *thisPtr, const void *source, XsFilePos size, XsFilePos count)
{
	return (XsFilePos) fwrite(source, (size_t) size, (size_t) count, thisPtr->m_handle);
}

/*!	\relates XsFile
	\brief Gets and returns the next byte from a file
	\returns The byte read from the stream or -1 in case of failure
*/
int XsFile_getc(struct XsFile *thisPtr)
{
	return fgetc(thisPtr->m_handle);
}

/*!	\relates XsFile
	\brief Writes a character to the file
	\param character The caharacter to write
	\returns XRV_OK on success, XRV_ERROR otehrwise
*/
XsResultValue XsFile_putc(struct XsFile *thisPtr, int character)
{
	return fputc(character, thisPtr->m_handle) == EOF ? XRV_ERROR : XRV_OK;
}

/*!	\relates XsFile
	\brief Reads characters from this file and stores them into str until (num-1) characters have been read or either a newline or the end-of-file is reached, whichever happens first.
	\param str The destination for the read string
	\param num The size of the destination buffer
	\returns A pointer to the read string on success, NULL on failure
*/
char* XsFile_gets(struct XsFile *thisPtr, char *str, int num)
{
	return fgets(str, num, thisPtr->m_handle);
}

/*!	\relates XsFile
	\brief Writes a null terminated c-string to the file
	\param str The null terminated c-string to write
	\returns XRV_OK on success, XRV_ERROR otehrwise
*/
XsResultValue XsFile_puts(struct XsFile *thisPtr, const char *str)
{
	return fputs(str, thisPtr->m_handle) != EOF ? XRV_OK : XRV_ERROR;
}

/*!	\relates XsFile
	\brief Moves the current file position relative to the start of the file
	\param offset Position in the file to move to, relative to the start of the file
	\returns XRV_OK if the seek was succesful
*/
XsResultValue XsFile_seek(struct XsFile *thisPtr, XsFilePos offset)
{
#ifdef _WIN32
	return _fseeki64(thisPtr->m_handle, offset, SEEK_SET) ? XRV_ERROR : XRV_OK;
#else
	return fseeko(thisPtr->m_handle, offset, SEEK_SET) ? XRV_ERROR : XRV_OK;
#endif
}

/*!	\relates XsFile
	\brief Moves the current file position relative to the end of the file
	\param offset Position in the file to move to, relative to the end of the file
	\returns XRV_OK if the seek was succesful
*/
XsResultValue XsFile_seek_r(struct XsFile *thisPtr, XsFilePos offset)
{
#ifdef _WIN32
	return _fseeki64(thisPtr->m_handle, offset, SEEK_END) ? XRV_ERROR : XRV_OK;
#else
	return fseeko(thisPtr->m_handle, offset, SEEK_END) ? XRV_ERROR : XRV_OK;
#endif
}

/*!	\relates XsFile
	\brief Returns the current position in the file
	\returns Current position in the current file
*/
XsFilePos XsFile_tell(struct XsFile const* thisPtr)
{
#ifdef _WIN32
	return _ftelli64(thisPtr->m_handle);
#else
	return ftello(thisPtr->m_handle);
#endif
}

/*!	\relates XsFile
	\returns Returns 0 if the current file position is not 'end of file', non 0 if the position is 'end of file'
*/
int XsFile_eof(struct XsFile const* thisPtr)
{
	return feof(thisPtr->m_handle);
}

/*!	\relates XsFile
	\returns Returns XRV_ERROR if the error flag for the file has been set, XRV_OK otherwise
*/
XsResultValue XsFile_error(struct XsFile const* thisPtr)
{
	return ferror(thisPtr->m_handle) ? XRV_ERROR : XRV_OK;
}

/*!	\relates XsFile
	\brief Retrieves the full path for a filename
	\param[in] filename The filename to expand
	\param[out] fullPath The filename with a fully expanded path is returned in this parameter. This parameter must be allocated by the caller.
	\returns XRV_OK if the fullpath could be retrieved, XRV_NULLPTR if fullPath is NULL, XRV_ERROR otherwise
*/
XsResultValue XsFile_fullPath(const struct XsString* filename, struct XsString* fullPath)
{
	XsResultValue result = XRV_OK;
	if (fullPath == NULL)
	{
		result = XRV_NULLPTR;
	}
	else
	{
#ifdef _WIN32
		wchar_t filenamew[XS_MAX_FILENAME_LENGTH];
		wchar_t fullpath[XS_MAX_FILENAME_LENGTH];
		(void)XsString_copyToWCharArray(filename, filenamew, XS_MAX_FILENAME_LENGTH);

		if (_wfullpath(fullpath, filenamew, XS_MAX_FILENAME_LENGTH) == NULL)
			result = XRV_ERROR;
		else
			XsString_assignWCharArray(fullPath, fullpath);
#else
		// based on the assumption that this doesn't concern the serial port, handle
		// it the same way using realpath(). Apparently realpath() doesn't require a
		// maximum length. One would possibly want to write a wrapper for it.
		char fullpath[XS_MAX_FILENAME_LENGTH*2];
		if (realpath(filename->m_data, fullpath) == NULL)
			result = XRV_ERROR;
		else
			XsString_assignCharArray(fullPath, fullpath);
#endif
	}
	return result;
}

/*!	\relates XsFile
	\brief Reads a full line from the file
	\param line The result of the read
	\returns XRV_OK if a line could be read
*/
XsResultValue XsFile_getline(struct XsFile *thisPtr, struct XsString *line)
{
	int b;
	XsResultValue ok = XRV_ENDOFFILE;

	XsString_erase(line, 0, line->m_size);
	XsArray_reserve(line, 256);

	b = fgetc(thisPtr->m_handle);
	if (b != -1)
		ok = XRV_OK;

	while (b != -1)
	{
		XsString_push_back(line, (char)b);
		if (b == '\n')
			break;
		b = fgetc(thisPtr->m_handle);
	}
	return ok;
}

/*! \relates XsFile
	\returns The file handle
*/
FILE* XsFile_handle(struct XsFile *thisPtr)
{
	return thisPtr->m_handle;
}

/*! @} */
