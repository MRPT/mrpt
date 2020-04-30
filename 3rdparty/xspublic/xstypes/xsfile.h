
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

#ifndef XSFILE_H
#define XSFILE_H

#include "xstypesconfig.h"
#include "pstdint.h"
#include <stdio.h>
#include "xsresultvalue.h"
#include "xsfilepos.h"
#include "xsstring.h"

#ifdef _MSC_VER
#	define XS_MAX_FILENAME_LENGTH	512
#else
#	define XS_MAX_FILENAME_LENGTH	PATH_MAX
#endif

#ifdef __cplusplus
extern "C" {
#endif
#ifndef __cplusplus
#define XSFILE_INITIALIZER { 0 }
#endif

struct XsFile;

XSTYPES_DLL_API void XsFile_destruct(struct XsFile* thisPtr);

XSTYPES_DLL_API XsResultValue XsFile_create(struct XsFile *thisPtr, const struct XsString* filename, int writeOnly);
XSTYPES_DLL_API XsResultValue XsFile_createText(struct XsFile *thisPtr, const struct XsString* filename, int writeOnly);
XSTYPES_DLL_API XsResultValue XsFile_open(struct XsFile *thisPtr, const struct XsString* filename, int readOnly);
XSTYPES_DLL_API XsResultValue XsFile_openText(struct XsFile *thisPtr, const struct XsString* filename, int readOnly);
XSTYPES_DLL_API XsResultValue XsFile_reopen(struct XsFile *thisPtr, const struct XsString* filename, const struct XsString* mode);
XSTYPES_DLL_API int XsFile_isOpen(const struct XsFile *thisPtr);
XSTYPES_DLL_API XsResultValue XsFile_close(struct XsFile *thisPtr);

XSTYPES_DLL_API int XsFile_exists(const struct XsString* filename);

XSTYPES_DLL_API XsResultValue XsFile_flush(struct XsFile *thisPtr);
XSTYPES_DLL_API XsResultValue XsFile_truncate(struct XsFile *thisPtr, XsFilePos fileSize);
XSTYPES_DLL_API XsResultValue XsFile_resize(struct XsFile *thisPtr, XsFilePos fileSize);
XSTYPES_DLL_API XsResultValue XsFile_erase(const struct XsString* filename);

XSTYPES_DLL_API XsFilePos XsFile_read(struct XsFile *thisPtr, void *destination, XsFilePos size, XsFilePos count);
XSTYPES_DLL_API XsFilePos XsFile_write(struct XsFile *thisPtr, const void *source, XsFilePos size, XsFilePos count);
XSTYPES_DLL_API int XsFile_getc(struct XsFile *thisPtr);
XSTYPES_DLL_API XsResultValue XsFile_putc(struct XsFile *thisPtr, int character);
XSTYPES_DLL_API char* XsFile_gets(struct XsFile *thisPtr, char *str, int num);
XSTYPES_DLL_API XsResultValue XsFile_puts(struct XsFile *thisPtr, const char *str);

XSTYPES_DLL_API XsResultValue XsFile_seek(struct XsFile* thisPtr, XsFilePos offset);
XSTYPES_DLL_API XsResultValue XsFile_seek_r(struct XsFile* thisPtr, XsFilePos offset);
XSTYPES_DLL_API XsFilePos XsFile_tell(struct XsFile const* thisPtr);

XSTYPES_DLL_API int XsFile_eof(struct XsFile const* thisPtr);
XSTYPES_DLL_API XsResultValue XsFile_error(struct XsFile const* thisPtr);
XSTYPES_DLL_API XsResultValue XsFile_fullPath(const struct XsString* filename, struct XsString* fullPath);

XSTYPES_DLL_API XsResultValue XsFile_getline(struct XsFile *thisPtr, struct XsString *line);

XSTYPES_DLL_API FILE* XsFile_handle(struct XsFile *thisPtr);

#ifdef __cplusplus
} // extern "C"
#endif

struct XsFile {
#ifdef __cplusplus
	/*! \brief Default constructor, creates an empty file object */
	explicit inline XsFile()
		: m_handle(NULL)
	{
		// Silence -Wunused-private-field
		// This field is used in the C back-end, but
		// some compilers may not notice this
		(void)m_handle;
	}

	/*! \brief \copybrief XsFile_destruct */
	inline ~XsFile()
	{
		XsFile_destruct(this);
	}

	/*! \brief \copybrief XsFile_create */
	inline XsResultValue create(const XsString &filename, bool writeOnly)
	{
		return XsFile_create(this, &filename, writeOnly ? 1 : 0);
	}

	/*! \brief \copybrief XsFile_createText */
	inline XsResultValue createText(const XsString &filename, bool writeOnly)
	{
		return XsFile_createText(this, &filename, writeOnly ? 1 : 0);
	}

	/*! \brief \copybrief XsFile_open */
	inline XsResultValue open(const XsString &fileName, bool readOnly)
	{
		return XsFile_open(this, &fileName, readOnly ? 1 : 0);
	}

	/*! \brief \copybrief XsFile_openText */
	inline XsResultValue openText(const XsString &fileName, bool readOnly)
	{
		return XsFile_openText(this, &fileName, readOnly ? 1 : 0);
	}

	/*! \brief \copybrief XsFile_openText */
	inline XsResultValue reopen(const XsString &fileName, const XsString &mode)
	{
		return XsFile_reopen(this, &fileName, &mode);
	}

	/*! \brief \copybrief XsFile_isOpen */
	inline bool isOpen() const
	{
		return (XsFile_isOpen(this) == 0);
	}

	/*! \brief \copybrief XsFile_exists */
	static inline bool exists(const XsString &fileName)
	{
		return (XsFile_exists(&fileName) == 0);
	}

	/*! \brief \copybrief XsFile_close */
	inline XsResultValue close()
	{
		return XsFile_close(this);
	}

	/*! \brief \copybrief XsFile_flush */
	inline XsResultValue flush()
	{
		return XsFile_flush(this);
	}

	/*! \brief \copybrief XsFile_truncate */
	inline XsResultValue truncate(XsFilePos fileSize)
	{
		return XsFile_truncate(this, fileSize);
	}

	/*! \brief \copybrief XsFile_resize */
	inline XsResultValue resize(XsFilePos fileSize)
	{
		return XsFile_resize(this, fileSize);
	}

	/*! \brief \copybrief XsFile_erase */
	static XsResultValue erase(const XsString& filename)
	{
		return XsFile_erase(&filename);
	}

	/*! \brief \copybrief XsFile_read */
	inline XsFilePos read(void *destination, XsFilePos size, XsFilePos count)
	{
		return XsFile_read(this, destination, size, count);
	}

	/*! \brief \copybrief XsFile_write */
	inline XsFilePos write(const void *source, XsFilePos size, XsFilePos count)
	{
		return XsFile_write(this, source, size, count);
	}

	/*! \brief \copybrief XsFile_getc */
	inline int getc()
	{
		return XsFile_getc(this);
	}

	/*! \brief \copybrief XsFile_putc */
	inline XsResultValue putc(int character)
	{
		return XsFile_putc(this, character);
	}

	/*! \brief \copybrief XsFile_gets */
	inline char* gets(char *destination, int maxCount)
	{
		return XsFile_gets(this, destination, maxCount);
	}

	/*! \brief \copybrief XsFile_puts */
	inline XsResultValue puts(const char *source)
	{
		return XsFile_puts(this, source);
	}

	/*! \brief \copybrief XsFile_seek */
	inline XsResultValue seek(XsFilePos offset)
	{
		return XsFile_seek(this, offset);
	}

	/*! \brief \copybrief XsFile_seek_r */
	inline XsResultValue seek_r(XsFilePos offset)
	{
		return XsFile_seek_r(this, offset);
	}

	/*! \brief \copybrief XsFile_tell */
	inline XsFilePos tell() const
	{
		return XsFile_tell(this);
	}

	/*! \brief \copybrief XsFile_eof */
	inline bool eof() const
	{
		return (0!=XsFile_eof(this));
	}

	/*! \brief \copybrief XsFile_error */
	inline XsResultValue error() const
	{
		return XsFile_error(this);
	}

	/*! \brief \copybrief XsFile_fullPath */
	static XsResultValue fullPath(const XsString &filename, XsString &fullPath)
	{
		return XsFile_fullPath(&filename, &fullPath);
	}

	/*! \brief \copybrief XsFile_getline */
	inline XsResultValue getline(XsString& line)
	{
		return XsFile_getline(this, &line);
	}

	/*! \brief \copybrief XsFile_getline */
	inline XsResultValue getline(std::string& line)
	{
		XsString tmp;
		XsResultValue rv = XsFile_getline(this, &tmp);
		if (rv == XRV_OK)
			line = tmp.toStdString();
		return rv;
	}

	/*! \brief \copybrief XsFile_handle */
	inline FILE* handle()
	{
		return XsFile_handle(this);
	}
private:
#endif
	FILE* m_handle;
};

typedef struct XsFile XsFile;

#endif
