/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#pragma once

// Disable these warnings in MSVC:
//  4127 "conditional expression is constant" as the conversion classes trigger
//  it with the statement if (sizeof(SI_CHAR) == sizeof(char)). This test will
//  be optimized away in a release build.
//  4503 'insert' : decorated name length exceeded, name was truncated
//  4702 "unreachable code" as the MS STL header causes it in release mode.
//  Again, the code causing the warning will be cleaned up by the compiler.
//  4786 "identifier truncated to 256 characters" as this is thrown hundreds
//  of times VC6 as soon as STL is used.
#ifdef _MSC_VER
#pragma warning(disable : 4127 4503 4702 4786)
#endif

#include <string>
#include <map>
#include <list>
#include <algorithm>
#include <functional>
#include <cstdio>
#include <cstring>
#include <mrpt/core/common.h>
#include <mrpt/system/string_utils.h>
#include <mrpt/expr/CRuntimeCompiledExpression.h>

#define SI_SUPPORT_IOSTREAMS

#ifdef SI_SUPPORT_IOSTREAMS
#include <iostream>
#endif  // SI_SUPPORT_IOSTREAMS

#ifdef _DEBUG
#include <assert.h>
#define SI_ASSERT(x) assert(x)
#else
#define SI_ASSERT(x)
#endif

namespace mrpt::config::simpleini
{
enum SI_Error
{
	/** No error */
	SI_OK = 0,
	/** An existing value was updated */
	SI_UPDATED = 1,
	/** A new value was inserted */
	SI_INSERTED = 2,

	// note: test for any error with (retval < 0)
	/** Generic failure */
	SI_FAIL = -1,
	/** Out of memory error */
	SI_NOMEM = -2,
	/** File error (see errno for detail error) */
	SI_FILE = -3
};

#ifdef _WIN32
#define SI_NEWLINE_A "\r\n"
#define SI_NEWLINE_W L"\r\n"
#else  // !_WIN32
#define SI_NEWLINE_A "\n"
#define SI_NEWLINE_W L"\n"
#endif  // _WIN32

#if defined(_WIN32)
#define SI_HAS_WIDE_FILE
#define SI_WCHAR_T wchar_t
#elif defined(SI_CONVERT_ICU)
#define SI_HAS_WIDE_FILE
#define SI_WCHAR_T UChar
#endif

// ---------------------------------------------------------------------------
//                              MAIN TEMPLATE CLASS
// ---------------------------------------------------------------------------

/** Simple INI file reader.

	This can be instantiated with the choice of unicode or native characterset,
	and case sensitive or insensitive comparisons of section and key names.
	The supported combinations are pre-defined with the following typedefs:

	<table>
		<tr><th>Interface   <th>Case-sensitive  <th>Typedef
		<tr><td>char        <td>No              <td>CSimpleIniA
		<tr><td>char        <td>Yes             <td>CSimpleIniCaseA
		<tr><td>wchar_t     <td>No              <td>CSimpleIniW
		<tr><td>wchar_t     <td>Yes             <td>CSimpleIniCaseW
	</table>

	Note that using other types for the SI_CHAR is supported. For instance,
	unsigned char, unsigned short, etc. Note that where the alternative type
	is a different size to char/wchar_t you may need to supply new helper
	classes for SI_STRLESS and SI_CONVERTER.
 */
template <class SI_CHAR, class SI_STRLESS, class SI_CONVERTER>
class CSimpleIniTempl
{
   public:
	/** key entry */
	struct Entry
	{
		const SI_CHAR* pItem;
		const SI_CHAR* pComment;
		int nOrder;

		Entry(const SI_CHAR* a_pszItem = nullptr, int a_nOrder = 0)
			: pItem(a_pszItem), pComment(nullptr), nOrder(a_nOrder)
		{
		}
		Entry(const Entry& rhs) { operator=(rhs); }
		Entry& operator=(const Entry& rhs)
		{
			pItem = rhs.pItem;
			pComment = rhs.pComment;
			nOrder = rhs.nOrder;
			return *this;
		}

#if (defined(_MSC_VER) && _MSC_VER <= 1200)
		/** STL of VC6 doesn't allow me to specify my own comparator for
		 * list::sort() */
		bool operator<(const Entry& rhs) const
		{
			return LoadOrder()(*this, rhs);
		}
		bool operator>(const Entry& rhs) const
		{
			return LoadOrder()(rhs, *this);
		}
#endif

		/** Strict less ordering by name of key only */
		struct KeyOrder : std::function<bool(Entry, Entry)>
		{
			bool operator()(const Entry& lhs, const Entry& rhs) const
			{
				const static SI_STRLESS isLess = SI_STRLESS();
				return isLess(lhs.pItem, rhs.pItem);
			}
		};

		/** Strict less ordering by order, and then name of key */
		struct LoadOrder : std::function<bool(Entry, Entry)>
		{
			bool operator()(const Entry& lhs, const Entry& rhs) const
			{
				if (lhs.nOrder != rhs.nOrder)
				{
					return lhs.nOrder < rhs.nOrder;
				}
				return KeyOrder()(lhs.pItem, rhs.pItem);
			}
		};
	};

	/** map keys to values */
	using TKeyVal =
		std::multimap<Entry, const SI_CHAR*, typename Entry::KeyOrder>;

	/** map sections to key/value map */
	using TSection = std::map<Entry, TKeyVal, typename Entry::KeyOrder>;

	/** set of dependent string pointers. Note that these pointers are
		dependent on memory owned by CSimpleIni.
	*/
	using TNamesDepend = std::list<Entry>;

	/** interface definition for the OutputWriter object to pass to Save()
		in order to output the INI file data.
	*/
	class OutputWriter
	{
	   public:
		OutputWriter() {}
		virtual ~OutputWriter() {}
		virtual void Write(const char* a_pBuf) = 0;

	   private:
		OutputWriter(const OutputWriter&);  // disable
		OutputWriter& operator=(const OutputWriter&);  // disable
	};

	/** OutputWriter class to write the INI data to a file */
	class FileWriter : public OutputWriter
	{
		FILE* m_file;

	   public:
		FileWriter(FILE* a_file) : m_file(a_file) {}
		void Write(const char* a_pBuf) override { fputs(a_pBuf, m_file); }

	   private:
		FileWriter(const FileWriter&);  // disable
		FileWriter& operator=(const FileWriter&);  // disable
	};

	/** OutputWriter class to write the INI data to a string */
	class StringWriter : public OutputWriter
	{
		std::string& m_string;

	   public:
		StringWriter(std::string& a_string) : m_string(a_string) {}
		void Write(const char* a_pBuf) override { m_string.append(a_pBuf); }

	   private:
		StringWriter(const StringWriter&);  // disable
		StringWriter& operator=(const StringWriter&);  // disable
	};

#ifdef SI_SUPPORT_IOSTREAMS
	/** OutputWriter class to write the INI data to an ostream */
	class StreamWriter : public OutputWriter
	{
		std::ostream& m_ostream;

	   public:
		StreamWriter(std::ostream& a_ostream) : m_ostream(a_ostream) {}
		void Write(const char* a_pBuf) { m_ostream << a_pBuf; }

	   private:
		StreamWriter(const StreamWriter&);  // disable
		StreamWriter& operator=(const StreamWriter&);  // disable
	};
#endif  // SI_SUPPORT_IOSTREAMS

	/** Characterset conversion utility class to convert strings to the
		same format as is used for the storage.
	*/
	class Converter : private SI_CONVERTER
	{
	   public:
		Converter() : SI_CONVERTER() { m_scratch.resize(1024); }
		Converter(const Converter& rhs) { operator=(rhs); }
		Converter& operator=(const Converter& rhs)
		{
			m_scratch = rhs.m_scratch;
			return *this;
		}
		bool ConvertToStore(const SI_CHAR* a_pszString)
		{
			size_t uLen = this->SizeToStore(a_pszString);
			if (uLen == (size_t)(-1))
			{
				return false;
			}
			while (uLen > m_scratch.size())
			{
				m_scratch.resize(m_scratch.size() * 2);
			}
			return SI_CONVERTER::ConvertToStore(
				a_pszString, const_cast<char*>(m_scratch.data()),
				m_scratch.size());
		}
		const char* Data() { return m_scratch.data(); }

	   private:
		std::string m_scratch;
	};

   public:
	/*-----------------------------------------------------------------------*/

	/** Default constructor.

		@param a_bMultiKey   See the method SetMultiKey() for details.
		@param a_bMultiLine  See the method SetMultiLine() for details.
	 */
	CSimpleIniTempl(bool a_bMultiKey = false, bool a_bMultiLine = false);

	/** Copy **/
	CSimpleIniTempl(const CSimpleIniTempl<SI_CHAR, SI_STRLESS, SI_CONVERTER>& o)
	{
		std::string str;
		o.Save(str);
		Load(str);
	};

	CSimpleIniTempl<SI_CHAR, SI_STRLESS, SI_CONVERTER>& operator=(
		const CSimpleIniTempl<SI_CHAR, SI_STRLESS, SI_CONVERTER>& o)
	{
		std::string str;
		o.Save(str);
		Load(str);
		return *this;
	}

	/** Destructor */
	~CSimpleIniTempl();

	/** Deallocate all memory stored by this object */
	void Reset();

	/*-----------------------------------------------------------------------*/
	/** @{ @name Settings */

	/** Should multiple identical keys be permitted in the file. If set to false
		then the last value encountered will be used as the value of the key.
		If set to true, then all values will be available to be queried. For
		example, with the following input:

		<pre>
		[section]
		test=value1
		test=value2
		</pre>

		Then with SetMultiKey(true), both of the values "value1" and "value2"
		will be returned for the key test. If SetMultiKey(false) is used, then
		the value for "test" will only be "value2". This value may be changed
		at any time.

		\param a_bAllowMultiKey  Allow multi-keys in the source?
	 */
	void SetMultiKey(bool a_bAllowMultiKey = true)
	{
		m_bAllowMultiKey = a_bAllowMultiKey;
	}

	/** Get the storage format of the INI data. */
	bool IsMultiKey() const { return m_bAllowMultiKey; }
	/** Should data values be permitted to span multiple lines in the file. If
		set to false then the multi-line construct <<<TAG as a value will be
		returned as is instead of loading the data. This value may be changed
		at any time.

		\param a_bAllowMultiLine     Allow multi-line values in the source?
	 */
	void SetMultiLine(bool a_bAllowMultiLine = true)
	{
		m_bAllowMultiLine = a_bAllowMultiLine;
	}

	/** Query the status of multi-line data */
	bool IsMultiLine() const { return m_bAllowMultiLine; }
	/*-----------------------------------------------------------------------*/
	/** @}
		@{ @name Loading INI Data */

	/** Load an INI file from disk into memory

		@param a_pszFile    Path of the file to be loaded. This will be passed
							to fopen() and so must be a valid path for the
							current platform.

		@return SI_Error    See error definitions
	 */
	SI_Error LoadFile(const char* a_pszFile);

#ifdef SI_HAS_WIDE_FILE
	/** Load an INI file from disk into memory

		@param a_pwszFile   Path of the file to be loaded in UTF-16.

		@return SI_Error    See error definitions
	 */
	SI_Error LoadFile(const SI_WCHAR_T* a_pwszFile);
#endif  // SI_HAS_WIDE_FILE

	/** Load the file from a file pointer.

		@param a_fpFile     Valid file pointer to read the file data from. The
							file will be read until end of file.

		@return SI_Error    See error definitions
	*/
	SI_Error LoadFile(FILE* a_fpFile);

#ifdef SI_SUPPORT_IOSTREAMS
	/** Load INI file data from an istream.

		@param a_istream    Stream to read from

		@return SI_Error    See error definitions
	 */
	SI_Error Load(std::istream& a_istream);
#endif  // SI_SUPPORT_IOSTREAMS

	/** Load INI file data direct from a std::string

		@param a_strData    Data to be loaded

		@return SI_Error    See error definitions
	 */
	SI_Error Load(const std::string& a_strData)
	{
		return Load(a_strData.c_str(), a_strData.size());
	}

	/** Load INI file data direct from memory

		@param a_pData      Data to be loaded
		@param a_uDataLen   Length of the data in bytes

		@return SI_Error    See error definitions
	 */
	SI_Error Load(const char* a_pData, size_t a_uDataLen);

	/*-----------------------------------------------------------------------*/
	/** @}
		@{ @name Saving INI Data */

	/** Save an INI file from memory to disk

		@param a_pszFile    Path of the file to be saved. This will be passed
							to fopen() and so must be a valid path for the
							current platform.

		@return SI_Error    See error definitions
	 */
	SI_Error SaveFile(const char* a_pszFile) const;

#ifdef SI_HAS_WIDE_FILE
	/** Save an INI file from memory to disk

		@param a_pwszFile   Path of the file to be saved in UTF-16.

		@return SI_Error    See error definitions
	 */
	SI_Error SaveFile(const SI_WCHAR_T* a_pwszFile) const;
#endif  // _WIN32

	/** Save the INI data to a file. See Save() for details.

		@param a_pFile      Handle to a file. File should be opened for
							binary output.

		@return SI_Error    See error definitions
	 */
	SI_Error SaveFile(FILE* a_pFile) const;

	/** Save the INI data. The data will be written to the output device
		in a format appropriate to the current data, selected by:

		<table>
			<tr><th>SI_CHAR     <th>FORMAT
			<tr><td>char        <td>same format as when loaded (MBCS or UTF-8)
			<tr><td>wchar_t     <td>UTF-8
			<tr><td>other       <td>UTF-8
		</table>

		Note that comments, etc from the original data are not preserved. Only
		valid data contents stored in the file are written out. The order of
		the sections and values from the original file will be preserved.

		Any data prepended or appended to the output device must use the the
		same format (MBCS or UTF-8). You may use the GetConverter() method to
		convert text to the correct format regardless of the output format
		being used by SimpleIni.

		To add a BOM to UTF-8 data, write it out manually at the very beginning
		like is done in SaveFile when a_bUseBOM is true.

		@param a_oOutput    Output writer to write the data to.

		@return SI_Error    See error definitions
	 */
	SI_Error Save(OutputWriter& a_oOutput) const;

#ifdef SI_SUPPORT_IOSTREAMS
	/** Save the INI data to an ostream. See Save() for details.

		@param a_ostream    String to have the INI data appended to.

		@return SI_Error    See error definitions
	 */
	SI_Error Save(std::ostream& a_ostream) const
	{
		StreamWriter writer(a_ostream);
		return Save(writer);
	}
#endif  // SI_SUPPORT_IOSTREAMS

	/** Append the INI data to a string. See Save() for details.

		@param a_sBuffer    String to have the INI data appended to.

		@return SI_Error    See error definitions
	 */
	SI_Error Save(std::string& a_sBuffer) const
	{
		StringWriter writer(a_sBuffer);
		return Save(writer);
	}

	/*-----------------------------------------------------------------------*/
	/** @}
		@{ @name Accessing INI Data */

	/** Retrieve all section names. The list is returned as an STL vector of
		names and can be iterated or searched as necessary. Note that the
		collation order of the returned strings is NOT DEFINED.

		NOTE! This structure contains only pointers to strings. The actual
		string data is stored in memory owned by CSimpleIni. Ensure that the
		CSimpleIni object is not destroyed or Reset() while these pointers
		are in use!

		@param a_names          Vector that will receive all of the section
								names. See note above!
	 */
	void GetAllSections(TNamesDepend& a_names) const;

	/** Retrieve all unique key names in a section. The collation order of the
		returned strings is NOT DEFINED. Only unique key names are returned.

		NOTE! This structure contains only pointers to strings. The actual
		string data is stored in memory owned by CSimpleIni. Ensure that the
		CSimpleIni object is not destroyed or Reset() while these strings
		are in use!

		@param a_pSection       Section to request data for
		@param a_names          List that will receive all of the key
								names. See note above!

		@return true            Section was found.
		@return false           Matching section was not found.
	 */
	bool GetAllKeys(const SI_CHAR* a_pSection, TNamesDepend& a_names) const;

	/** Retrieve all values for a specific key. This method can be used when
		multiple keys are both enabled and disabled.

		NOTE! The returned values are pointers to string data stored in memory
		owned by CSimpleIni. Ensure that the CSimpleIni object is not destroyed
		or Reset while you are using this pointer!

		@param a_pSection       Section to search
		@param a_pKey           Key to search for
		@param a_values         List to return if the key is not found

		@return true            Key was found.
		@return false           Matching section/key was not found.
	 */
	bool GetAllValues(
		const SI_CHAR* a_pSection, const SI_CHAR* a_pKey,
		TNamesDepend& a_values) const;

	/** Query the number of keys in a specific section. Note that if multiple
		keys are enabled, then this value may be different to the number of
		keys returned by GetAllKeys.

		@param a_pSection       Section to request data for

		@return -1              Section does not exist in the file
		@return >=0             Number of keys in the section
	 */
	int GetSectionSize(const SI_CHAR* a_pSection) const;

	/** Retrieve all key and value pairs for a section. The data is returned
		as a pointer to an STL map and can be iterated or searched as
		desired. Note that multiple entries for the same key may exist when
		multiple keys have been enabled.

		NOTE! This structure contains only pointers to strings. The actual
		string data is stored in memory owned by CSimpleIni. Ensure that the
		CSimpleIni object is not destroyed or Reset() while these strings
		are in use!

		@param a_pSection       Name of the section to return
		@return boolean         Was a section matching the supplied
								name found.
	 */
	const TKeyVal* GetSection(const SI_CHAR* a_pSection) const;

	/** Retrieve the value for a specific key. If multiple keys are enabled
		(see SetMultiKey) then only the first value associated with that key
		will be returned, see GetAllValues for getting all values with multikey.

		NOTE! The returned value is a pointer to string data stored in memory
		owned by CSimpleIni. Ensure that the CSimpleIni object is not destroyed
		or Reset while you are using this pointer!

		@param a_pSection       Section to search
		@param a_pKey           Key to search for
		@param a_pDefault       Value to return if the key is not found
		@param a_pHasMultiple   Optionally receive notification of if there are
								multiple entries for this key.

		@return a_pDefault      Key was not found in the section
		@return other           Value of the key
	 */
	const SI_CHAR* GetValue(
		const SI_CHAR* a_pSection, const SI_CHAR* a_pKey,
		const SI_CHAR* a_pDefault = nullptr,
		bool* a_pHasMultiple = nullptr) const;

	/** Add or update a section or value. This will always insert
		when multiple keys are enabled.

		@param a_pSection   Section to add or update
		@param a_pKey       Key to add or update. Set to nullptr to
							create an empty section.
		@param a_pValue     Value to set. Set to nullptr to create an
							empty section.
		@param a_pComment   Comment to be associated with the section or the
							key. If a_pKey is nullptr then it will be associated
							with the section, otherwise the key. Note that a
							comment may be set ONLY when the section or key is
							first created (i.e. when this function returns the
							value SI_INSERTED). If you wish to create a section
							with a comment then you need to create the section
							separately to the key. The comment string must be
							in full comment form already (have a comment
							character starting every line).

		@return SI_Error    See error definitions
		@return SI_UPDATED  Value was updated
		@return SI_INSERTED Value was inserted
	 */
	SI_Error SetValue(
		const SI_CHAR* a_pSection, const SI_CHAR* a_pKey,
		const SI_CHAR* a_pValue, const SI_CHAR* a_pComment = nullptr)
	{
		return AddEntry(a_pSection, a_pKey, a_pValue, a_pComment, true);
	}

	/** Delete an entire section, or a key from a section. Note that the
		data returned by GetSection is invalid and must not be used after
		anything has been deleted from that section using this method.
		Note when multiple keys is enabled, this will delete all keys with
		that name; there is no way to selectively delete individual key/values
		in this situation.

		@param a_pSection       Section to delete key from, or if
								a_pKey is nullptr, the section to remove.
		@param a_pKey           Key to remove from the section. Set to
								nullptr to remove the entire section.
		@param a_bRemoveEmpty   If the section is empty after this key has
								been deleted, should the empty section be
								removed?

		@return true            Key or section was deleted.
		@return false           Key or section was not found.
	 */
	bool Delete(
		const SI_CHAR* a_pSection, const SI_CHAR* a_pKey,
		bool a_bRemoveEmpty = false);

	/*-----------------------------------------------------------------------*/
	/** @}
		@{ @name Converter */

	/** Return a conversion object to convert text to the same encoding
		as is used by the Save(), SaveFile() and SaveString() functions.
		Use this to prepare the strings that you wish to append or prepend
		to the output INI data.
	 */
	Converter GetConverter() const { return Converter(); }
	/*-----------------------------------------------------------------------*/
	/** @} */

   private:
	/** Parse the data looking for a file comment and store it if found.
	 */
	SI_Error FindFileComment(SI_CHAR*& a_pData, bool a_bCopyStrings);

	/** Parse the data looking for the next valid entry. The memory pointed to
		by a_pData is modified by inserting nullptr characters. The pointer is
		updated to the current location in the block of text.
	*/
	bool FindEntry(
		SI_CHAR*& a_pData, const SI_CHAR*& a_pSection, const SI_CHAR*& a_pKey,
		const SI_CHAR*& a_pVal, const SI_CHAR*& a_pComment) const;

	/** Add the section/key/value to our data.

		@param a_pSection   Section name. Sections will be created if they
							don't already exist.
		@param a_pKey       Key name. May be nullptr to create an empty section.
							Existing entries will be updated. New entries will
							be created.
		@param a_pValue     Value for the key.
		@param a_pComment   Comment to be associated with the section or the
							key. If a_pKey is nullptr then it will be associated
							with the section, otherwise the key. This must be
							a string in full comment form already (have a
							comment character starting every line).
		@param a_bCopyStrings   Should copies of the strings be made or not.
							If false then the pointers will be used as is.
	*/
	SI_Error AddEntry(
		const SI_CHAR* a_pSection, const SI_CHAR* a_pKey,
		const SI_CHAR* a_pValue, const SI_CHAR* a_pComment,
		bool a_bCopyStrings);

	/** Is the supplied character a whitespace character? */
	inline bool IsSpace(SI_CHAR ch) const
	{
		return (ch == ' ' || ch == '\t' || ch == '\r' || ch == '\n');
	}

	/** Does the supplied character start a comment line? */
	inline bool IsComment(SI_CHAR ch) const { return (ch == ';' || ch == '#'); }
	/** Skip over a newline character (or characters) for either DOS or UNIX */
	inline void SkipNewLine(SI_CHAR*& a_pData) const
	{
		a_pData += (*a_pData == '\r' && *(a_pData + 1) == '\n') ? 2 : 1;
	}

	/** Make a copy of the supplied string, replacing the original pointer */
	SI_Error CopyString(const SI_CHAR*& a_pString);

	/** Delete a string from the copied strings buffer if necessary */
	void DeleteString(const SI_CHAR* a_pString);

	/** Internal use of our string comparison function */
	bool IsLess(const SI_CHAR* a_pLeft, const SI_CHAR* a_pRight) const
	{
		const static SI_STRLESS isLess = SI_STRLESS();
		return isLess(a_pLeft, a_pRight);
	}

	bool IsMultiLineTag(const SI_CHAR* a_pData) const;
	bool IsMultiLineData(const SI_CHAR* a_pData) const;
	bool LoadMultiLineText(
		SI_CHAR*& a_pData, const SI_CHAR*& a_pVal, const SI_CHAR* a_pTagName,
		bool a_bAllowBlankLinesInComment = false) const;
	bool IsNewLineChar(SI_CHAR a_c) const;

	bool OutputMultiLineText(
		OutputWriter& a_oOutput, Converter& a_oConverter,
		const SI_CHAR* a_pText) const;

   private:
	/** Copy of the INI file data in our character format. This will be
		modified when parsed to have nullptr characters added after all
		interesting string entries. All of the string pointers to sections,
		keys and values point into this block of memory.
	 */
	SI_CHAR* m_pData;

	/** Length of the data that we have stored. Used when deleting strings
		to determine if the string is stored here or in the allocated string
		buffer.
	 */
	size_t m_uDataLen;

	/** File comment for this data, if one exists. */
	const SI_CHAR* m_pFileComment;

	/** Parsed INI data. Section -> (Key -> Value). */
	TSection m_data;

	/** This vector stores allocated memory for copies of strings that have
		been supplied after the file load. It will be empty unless SetValue()
		has been called.
	 */
	TNamesDepend m_strings;

	/** Are multiple values permitted for the same key? */
	bool m_bAllowMultiKey;

	/** Are data values permitted to span multiple lines? */
	bool m_bAllowMultiLine;

	/** Next order value, used to ensure sections and keys are output in the
		same order that they are loaded/added.
	 */
	int m_nOrder;
};

// ---------------------------------------------------------------------------
//                                  IMPLEMENTATION
// ---------------------------------------------------------------------------

template <class SI_CHAR, class SI_STRLESS, class SI_CONVERTER>
CSimpleIniTempl<SI_CHAR, SI_STRLESS, SI_CONVERTER>::CSimpleIniTempl(
	bool a_bAllowMultiKey, bool a_bAllowMultiLine)
	: m_pData(0),
	  m_uDataLen(0),
	  m_pFileComment(NULL),
	  m_bAllowMultiKey(a_bAllowMultiKey),
	  m_bAllowMultiLine(a_bAllowMultiLine),
	  m_nOrder(0)
{
}

template <class SI_CHAR, class SI_STRLESS, class SI_CONVERTER>
CSimpleIniTempl<SI_CHAR, SI_STRLESS, SI_CONVERTER>::~CSimpleIniTempl()
{
	Reset();
}

template <class SI_CHAR, class SI_STRLESS, class SI_CONVERTER>
void CSimpleIniTempl<SI_CHAR, SI_STRLESS, SI_CONVERTER>::Reset()
{
	// remove all data
	delete[] m_pData;
	m_pData = nullptr;
	m_uDataLen = 0;
	m_pFileComment = nullptr;
	if (!m_data.empty())
	{
		m_data.erase(m_data.begin(), m_data.end());
	}

	// remove all strings
	if (!m_strings.empty())
	{
		auto i = m_strings.begin();
		for (; i != m_strings.end(); ++i)
		{
			delete[] const_cast<SI_CHAR*>(i->pItem);
		}
		m_strings.erase(m_strings.begin(), m_strings.end());
	}
}

template <class SI_CHAR, class SI_STRLESS, class SI_CONVERTER>
SI_Error CSimpleIniTempl<SI_CHAR, SI_STRLESS, SI_CONVERTER>::LoadFile(
	const char* a_pszFile)
{
	FILE* fp = nullptr;
#if __STDC_WANT_SECURE_LIB__
	fopen_s(&fp, a_pszFile, "rb");
#else
	fp = fopen(a_pszFile, "rb");
#endif
	if (!fp)
	{
		return SI_FILE;
	}
	SI_Error rc = LoadFile(fp);
	fclose(fp);
	return rc;
}

#ifdef SI_HAS_WIDE_FILE
template <class SI_CHAR, class SI_STRLESS, class SI_CONVERTER>
SI_Error CSimpleIniTempl<SI_CHAR, SI_STRLESS, SI_CONVERTER>::LoadFile(
	const SI_WCHAR_T* a_pwszFile)
{
#ifdef _WIN32
	FILE* fp = nullptr;
#if __STDC_WANT_SECURE_LIB__
	_wfopen_s(&fp, a_pwszFile, L"rb");
#else
	fp = _wfopen(a_pwszFile, L"rb");
#endif
	if (!fp) return SI_FILE;
	SI_Error rc = LoadFile(fp);
	fclose(fp);
	return rc;
#else  // SI_CONVERT_ICU
	char szFile[256];
	u_austrncpy(szFile, a_pwszFile, sizeof(szFile));
	return LoadFile(szFile);
#endif
}
#endif  // SI_HAS_WIDE_FILE

template <class SI_CHAR, class SI_STRLESS, class SI_CONVERTER>
SI_Error CSimpleIniTempl<SI_CHAR, SI_STRLESS, SI_CONVERTER>::LoadFile(
	FILE* a_fpFile)
{
	// load the raw file data
	int retval = fseek(a_fpFile, 0, SEEK_END);
	if (retval != 0)
	{
		return SI_FILE;
	}
	long lSize = ftell(a_fpFile);
	if (lSize < 0)
	{
		return SI_FILE;
	}
	char* pData = new char[lSize];
	if (!pData)
	{
		return SI_NOMEM;
	}
	fseek(a_fpFile, 0, SEEK_SET);
	size_t uRead = fread(pData, sizeof(char), lSize, a_fpFile);
	if (uRead != (size_t)lSize)
	{
		delete[] pData;
		return SI_FILE;
	}

	// convert the raw data to unicode
	SI_Error rc = Load(pData, uRead);
	delete[] pData;
	return rc;
}

template <class SI_CHAR, class SI_STRLESS, class SI_CONVERTER>
SI_Error CSimpleIniTempl<SI_CHAR, SI_STRLESS, SI_CONVERTER>::Load(
	const char* a_pData, size_t a_uDataLen)
{
	SI_CONVERTER converter;

	// determine the length of the converted data
	size_t uLen = converter.SizeFromStore(a_pData, a_uDataLen);
	if (uLen == (size_t)(-1))
	{
		return SI_FAIL;
	}

	// allocate memory for the data, ensure that there is a NULL
	// terminator wherever the converted data ends
	auto* pData = new SI_CHAR[uLen + 1];
	if (!pData)
	{
		return SI_NOMEM;
	}
	memset(pData, 0, sizeof(SI_CHAR) * (uLen + 1));

	// convert the data
	if (!converter.ConvertFromStore(a_pData, a_uDataLen, pData, uLen))
	{
		delete[] pData;
		return SI_FAIL;
	}

	// parse it
	const static SI_CHAR empty = 0;
	SI_CHAR* pWork = pData;
	const SI_CHAR* pSection = &empty;
	const SI_CHAR* pItem = nullptr;
	const SI_CHAR* pVal = nullptr;
	const SI_CHAR* pComment = nullptr;

	// We copy the strings if we are loading data into this class when we
	// already have stored some.
	bool bCopyStrings = (m_pData != nullptr);

	// find a file comment if it exists, this is a comment that starts at the
	// beginning of the file and continues until the first blank line.
	SI_Error rc = FindFileComment(pWork, bCopyStrings);
	if (rc < 0) return rc;

	// add every entry in the file to the data table
	while (FindEntry(pWork, pSection, pItem, pVal, pComment))
	{
		rc = AddEntry(pSection, pItem, pVal, pComment, bCopyStrings);
		if (rc < 0) return rc;
	}

	// store these strings if we didn't copy them
	if (bCopyStrings)
	{
		delete[] pData;
	}
	else
	{
		m_pData = pData;
		m_uDataLen = uLen + 1;
	}

	return SI_OK;
}

#ifdef SI_SUPPORT_IOSTREAMS
template <class SI_CHAR, class SI_STRLESS, class SI_CONVERTER>
SI_Error CSimpleIniTempl<SI_CHAR, SI_STRLESS, SI_CONVERTER>::Load(
	std::istream& a_istream)
{
	std::string strData;
	char szBuf[512];
	do
	{
		a_istream.get(szBuf, sizeof(szBuf), '\0');
		strData.append(szBuf);
	} while (a_istream.good());
	return Load(strData);
}
#endif  // SI_SUPPORT_IOSTREAMS

template <class SI_CHAR, class SI_STRLESS, class SI_CONVERTER>
SI_Error CSimpleIniTempl<SI_CHAR, SI_STRLESS, SI_CONVERTER>::FindFileComment(
	SI_CHAR*& a_pData, bool a_bCopyStrings)
{
	// there can only be a single file comment
	if (m_pFileComment)
	{
		return SI_OK;
	}

	// Load the file comment as multi-line text, this will modify all of
	// the newline characters to be single \n chars
	if (!LoadMultiLineText(a_pData, m_pFileComment, nullptr, false))
	{
		return SI_OK;
	}

	// copy the string if necessary
	if (a_bCopyStrings)
	{
		SI_Error rc = CopyString(m_pFileComment);
		if (rc < 0) return rc;
	}

	return SI_OK;
}

template <class SI_CHAR, class SI_STRLESS, class SI_CONVERTER>
bool CSimpleIniTempl<SI_CHAR, SI_STRLESS, SI_CONVERTER>::FindEntry(
	SI_CHAR*& a_pData, const SI_CHAR*& a_pSection, const SI_CHAR*& a_pKey,
	const SI_CHAR*& a_pVal, const SI_CHAR*& a_pComment) const
{
	a_pComment = nullptr;

	SI_CHAR* pTrail = nullptr;
	while (*a_pData)
	{
		// skip spaces and empty lines
		while (*a_pData && IsSpace(*a_pData))
		{
			++a_pData;
		}
		if (!*a_pData)
		{
			break;
		}

		// skip processing of comment lines but keep a pointer to
		// the start of the comment.
		if (IsComment(*a_pData))
		{
			LoadMultiLineText(a_pData, a_pComment, nullptr, true);
			continue;
		}

		// process section names
		if (*a_pData == '[')
		{
			// skip leading spaces
			++a_pData;
			while (*a_pData && IsSpace(*a_pData))
			{
				++a_pData;
			}

			// find the end of the section name (it may contain spaces)
			// and convert it to lowercase as necessary
			a_pSection = a_pData;
			while (*a_pData && *a_pData != ']' && !IsNewLineChar(*a_pData))
			{
				++a_pData;
			}

			// if it's an invalid line, just skip it
			if (*a_pData != ']')
			{
				continue;
			}

			// remove trailing spaces from the section
			pTrail = a_pData - 1;
			while (pTrail >= a_pSection && IsSpace(*pTrail))
			{
				--pTrail;
			}
			++pTrail;
			*pTrail = 0;

			// skip to the end of the line
			++a_pData;  // safe as checked that it == ']' above
			while (*a_pData && !IsNewLineChar(*a_pData))
			{
				++a_pData;
			}

			a_pKey = nullptr;
			a_pVal = nullptr;
			return true;
		}

		// find the end of the key name (it may contain spaces)
		// and convert it to lowercase as necessary
		a_pKey = a_pData;
		while (*a_pData && *a_pData != '=' && !IsNewLineChar(*a_pData))
		{
			++a_pData;
		}

		// if it's an invalid line, just skip it
		if (*a_pData != '=')
		{
			continue;
		}

		// empty keys are invalid
		if (a_pKey == a_pData)
		{
			while (*a_pData && !IsNewLineChar(*a_pData))
			{
				++a_pData;
			}
			continue;
		}

		// remove trailing spaces from the key
		pTrail = a_pData - 1;
		while (pTrail >= a_pKey && IsSpace(*pTrail))
		{
			--pTrail;
		}
		++pTrail;
		*pTrail = 0;

		// skip leading whitespace on the value
		++a_pData;  // safe as checked that it == '=' above
		while (*a_pData && !IsNewLineChar(*a_pData) && IsSpace(*a_pData))
		{
			++a_pData;
		}

		// find the end of the value which is the end of this line
		a_pVal = a_pData;
		while (*a_pData && !IsNewLineChar(*a_pData))
		{
			++a_pData;
		}

		// remove trailing spaces from the value
		pTrail = a_pData - 1;
		if (*a_pData)
		{  // prepare for the next round
			SkipNewLine(a_pData);
		}
		while (pTrail >= a_pVal && IsSpace(*pTrail))
		{
			--pTrail;
		}
		++pTrail;
		*pTrail = 0;

		// check for multi-line entries
		if (m_bAllowMultiLine && IsMultiLineTag(a_pVal))
		{
			// skip the "<<<" to get the tag that will end the multiline
			const SI_CHAR* pTagName = a_pVal + 3;
			return LoadMultiLineText(a_pData, a_pVal, pTagName);
		}

		// return the standard entry
		return true;
	}

	return false;
}

template <class SI_CHAR, class SI_STRLESS, class SI_CONVERTER>
bool CSimpleIniTempl<SI_CHAR, SI_STRLESS, SI_CONVERTER>::IsMultiLineTag(
	const SI_CHAR* a_pVal) const
{
	// check for the "<<<" prefix for a multi-line entry
	if (*a_pVal++ != '<') return false;
	if (*a_pVal++ != '<') return false;
	if (*a_pVal++ != '<') return false;
	return true;
}

template <class SI_CHAR, class SI_STRLESS, class SI_CONVERTER>
bool CSimpleIniTempl<SI_CHAR, SI_STRLESS, SI_CONVERTER>::IsMultiLineData(
	const SI_CHAR* a_pData) const
{
	// data is multi-line if it has any of the following features:
	//  * whitespace prefix
	//  * embedded newlines
	//  * whitespace suffix

	// empty string
	if (!*a_pData)
	{
		return false;
	}

	// check for prefix
	if (IsSpace(*a_pData))
	{
		return true;
	}

	// embedded newlines
	while (*a_pData)
	{
		if (IsNewLineChar(*a_pData))
		{
			return true;
		}
		++a_pData;
	}

	// check for suffix
	if (IsSpace(*--a_pData))
	{
		return true;
	}

	return false;
}

template <class SI_CHAR, class SI_STRLESS, class SI_CONVERTER>
bool CSimpleIniTempl<SI_CHAR, SI_STRLESS, SI_CONVERTER>::IsNewLineChar(
	SI_CHAR a_c) const
{
	return (a_c == '\n' || a_c == '\r');
}

template <class SI_CHAR, class SI_STRLESS, class SI_CONVERTER>
bool CSimpleIniTempl<SI_CHAR, SI_STRLESS, SI_CONVERTER>::LoadMultiLineText(
	SI_CHAR*& a_pData, const SI_CHAR*& a_pVal, const SI_CHAR* a_pTagName,
	bool a_bAllowBlankLinesInComment) const
{
	// we modify this data to strip all newlines down to a single '\n'
	// character. This means that on Windows we need to strip out some
	// characters which will make the data shorter.
	// i.e.  LINE1-LINE1\r\nLINE2-LINE2\0 will become
	//       LINE1-LINE1\nLINE2-LINE2\0
	// The pDataLine entry is the pointer to the location in memory that
	// the current line needs to start to run following the existing one.
	// This may be the same as pCurrLine in which case no move is needed.
	SI_CHAR* pDataLine = a_pData;
	SI_CHAR* pCurrLine;

	// value starts at the current line
	a_pVal = a_pData;

	// find the end tag. This tag must start in column 1 and be
	// followed by a newline. No whitespace removal is done while
	// searching for this tag.
	SI_CHAR cEndOfLineChar = *a_pData;
	for (;;)
	{
		// if we are loading comments then we need a comment character as
		// the first character on every line
		if (!a_pTagName && !IsComment(*a_pData))
		{
			// if we aren't allowing blank lines then we're done
			if (!a_bAllowBlankLinesInComment)
			{
				break;
			}

			// if we are allowing blank lines then we only include them
			// in this comment if another comment follows, so read ahead
			// to find out.
			SI_CHAR* pCurr = a_pData;
			int nNewLines = 0;
			while (IsSpace(*pCurr))
			{
				if (IsNewLineChar(*pCurr))
				{
					++nNewLines;
					SkipNewLine(pCurr);
				}
				else
				{
					++pCurr;
				}
			}

			// we have a comment, add the blank lines to the output
			// and continue processing from here
			if (IsComment(*pCurr))
			{
				for (; nNewLines > 0; --nNewLines) *pDataLine++ = '\n';
				a_pData = pCurr;
				continue;
			}

			// the comment ends here
			break;
		}

		// find the end of this line
		pCurrLine = a_pData;
		while (*a_pData && !IsNewLineChar(*a_pData)) ++a_pData;

		// move this line down to the location that it should be if necessary
		if (pDataLine < pCurrLine)
		{
			memmove(pDataLine, pCurrLine, a_pData - pCurrLine);
			pDataLine[a_pData - pCurrLine] = '\0';
		}

		// end the line with a NULL
		cEndOfLineChar = *a_pData;
		*a_pData = 0;

		// if are looking for a tag then do the check now. This is done before
		// checking for end of the data, so that if we have the tag at the end
		// of the data then the tag is removed correctly.
		if (a_pTagName &&
			(!IsLess(pDataLine, a_pTagName) && !IsLess(a_pTagName, pDataLine)))
		{
			break;
		}

		// if we are at the end of the data then we just automatically end
		// this entry and return the current data.
		if (!cEndOfLineChar)
		{
			return true;
		}

		// otherwise we need to process this newline to ensure that it consists
		// of just a single \n character.
		pDataLine += (a_pData - pCurrLine);
		*a_pData = cEndOfLineChar;
		SkipNewLine(a_pData);
		*pDataLine++ = '\n';
	}

	// if we didn't find a comment at all then return false
	if (a_pVal == a_pData)
	{
		a_pVal = nullptr;
		return false;
	}

	// the data (which ends at the end of the last line) needs to be
	// null-terminated BEFORE before the newline character(s). If the
	// user wants a new line in the multi-line data then they need to
	// add an empty line before the tag.
	*--pDataLine = '\0';

	// if looking for a tag and if we aren't at the end of the data,
	// then move a_pData to the start of the next line.
	if (a_pTagName && cEndOfLineChar)
	{
		SI_ASSERT(IsNewLineChar(cEndOfLineChar));
		*a_pData = cEndOfLineChar;
		SkipNewLine(a_pData);
	}

	return true;
}

template <class SI_CHAR, class SI_STRLESS, class SI_CONVERTER>
SI_Error CSimpleIniTempl<SI_CHAR, SI_STRLESS, SI_CONVERTER>::CopyString(
	const SI_CHAR*& a_pString)
{
	size_t uLen = 0;
	if (sizeof(SI_CHAR) == sizeof(char))
	{
		uLen = strlen((const char*)a_pString);
	}
	else if (sizeof(SI_CHAR) == sizeof(wchar_t))
	{
		uLen = wcslen((const wchar_t*)a_pString);
	}
	else
	{
		for (; a_pString[uLen]; ++uLen) /*loop*/
			;
	}
	++uLen;  // nullptr character
	auto* pCopy = new SI_CHAR[uLen];
	if (!pCopy)
	{
		return SI_NOMEM;
	}
	memcpy(pCopy, a_pString, sizeof(SI_CHAR) * uLen);
	m_strings.push_back(pCopy);
	a_pString = pCopy;
	return SI_OK;
}

template <class SI_CHAR, class SI_STRLESS, class SI_CONVERTER>
SI_Error CSimpleIniTempl<SI_CHAR, SI_STRLESS, SI_CONVERTER>::AddEntry(
	const SI_CHAR* a_pSection, const SI_CHAR* a_pKey, const SI_CHAR* a_pValue,
	const SI_CHAR* a_pComment, bool a_bCopyStrings)
{
	SI_Error rc;
	bool bInserted = false;

	SI_ASSERT(!a_pComment || IsComment(*a_pComment));

	// if we are copying strings then make a copy of the comment now
	// because we will need it when we add the entry.
	if (a_bCopyStrings && a_pComment)
	{
		rc = CopyString(a_pComment);
		if (rc < 0) return rc;
	}

	// check for existence of the section first if we need string copies
	auto iSection = m_data.end();
	if (a_bCopyStrings)
	{
		iSection = m_data.find(a_pSection);
		if (iSection == m_data.end())
		{
			// if the section doesn't exist then we need a copy as the
			// string needs to last beyond the end of this function
			// because we will be inserting the section next
			rc = CopyString(a_pSection);
			if (rc < 0) return rc;
		}
	}

	// create the section entry
	if (iSection == m_data.end())
	{
		Entry oKey(a_pSection, ++m_nOrder);
		if (a_pComment && (!a_pKey || !a_pValue))
		{
			oKey.pComment = a_pComment;
		}
		typename TSection::value_type oEntry(oKey, TKeyVal());
		using SectionIterator = typename TSection::iterator;
		std::pair<SectionIterator, bool> i = m_data.insert(oEntry);
		iSection = i.first;
		bInserted = true;
	}
	if (!a_pKey || !a_pValue)
	{
		// section only entries are specified with pItem and pVal as NULL
		return bInserted ? SI_INSERTED : SI_UPDATED;
	}

	// check for existence of the key
	TKeyVal& keyval = iSection->second;
	auto iKey = keyval.find(a_pKey);

	// make string copies if necessary
	if (a_bCopyStrings)
	{
		if (m_bAllowMultiKey || iKey == keyval.end())
		{
			// if the key doesn't exist then we need a copy as the
			// string needs to last beyond the end of this function
			// because we will be inserting the key next
			rc = CopyString(a_pKey);
			if (rc < 0) return rc;
		}

		// we always need a copy of the value
		rc = CopyString(a_pValue);
		if (rc < 0) return rc;
	}

	// create the key entry
	if (iKey == keyval.end() || m_bAllowMultiKey)
	{
		Entry oKey(a_pKey, ++m_nOrder);
		if (a_pComment)
		{
			oKey.pComment = a_pComment;
		}
		typename TKeyVal::value_type oEntry(
			oKey, static_cast<const char*>(nullptr));
		iKey = keyval.insert(oEntry);
		bInserted = true;
	}
	iKey->second = a_pValue;
	return bInserted ? SI_INSERTED : SI_UPDATED;
}

template <class SI_CHAR, class SI_STRLESS, class SI_CONVERTER>
const SI_CHAR* CSimpleIniTempl<SI_CHAR, SI_STRLESS, SI_CONVERTER>::GetValue(
	const SI_CHAR* a_pSection, const SI_CHAR* a_pKey, const SI_CHAR* a_pDefault,
	bool* a_pHasMultiple) const
{
	if (a_pHasMultiple)
	{
		*a_pHasMultiple = false;
	}
	if (!a_pSection || !a_pKey)
	{
		return a_pDefault;
	}
	auto iSection = m_data.find(a_pSection);
	if (iSection == m_data.end())
	{
		return a_pDefault;
	}
	auto iKeyVal = iSection->second.find(a_pKey);
	if (iKeyVal == iSection->second.end())
	{
		return a_pDefault;
	}

	// check for multiple entries with the same key
	if (m_bAllowMultiKey && a_pHasMultiple)
	{
		auto iTemp = iKeyVal;
		if (++iTemp != iSection->second.end())
		{
			if (!IsLess(a_pKey, iTemp->first.pItem))
			{
				*a_pHasMultiple = true;
			}
		}
	}

	return iKeyVal->second;
}

template <class SI_CHAR, class SI_STRLESS, class SI_CONVERTER>
bool CSimpleIniTempl<SI_CHAR, SI_STRLESS, SI_CONVERTER>::GetAllValues(
	const SI_CHAR* a_pSection, const SI_CHAR* a_pKey,
	TNamesDepend& a_values) const
{
	if (!a_pSection || !a_pKey)
	{
		return false;
	}
	auto iSection = m_data.find(a_pSection);
	if (iSection == m_data.end())
	{
		return false;
	}
	auto iKeyVal = iSection->second.find(a_pKey);
	if (iKeyVal == iSection->second.end())
	{
		return false;
	}

	// insert all values for this key
	a_values.push_back(iKeyVal->second);
	if (m_bAllowMultiKey)
	{
		++iKeyVal;
		while (iKeyVal != iSection->second.end() &&
			   !IsLess(a_pKey, iKeyVal->first.pItem))
		{
			a_values.push_back(Entry(iKeyVal->second, iKeyVal->first.nOrder));
			++iKeyVal;
		}
	}

	return true;
}

template <class SI_CHAR, class SI_STRLESS, class SI_CONVERTER>
int CSimpleIniTempl<SI_CHAR, SI_STRLESS, SI_CONVERTER>::GetSectionSize(
	const SI_CHAR* a_pSection) const
{
	if (!a_pSection)
	{
		return -1;
	}

	typename TSection::const_iterator iSection = m_data.find(a_pSection);
	if (iSection == m_data.end())
	{
		return -1;
	}
	const TKeyVal& section = iSection->second;

	// if multi-key isn't permitted then the section size is
	// the number of keys that we have.
	if (!m_bAllowMultiKey || section.empty())
	{
		return (int)section.size();
	}

	// otherwise we need to count them
	int nCount = 0;
	const SI_CHAR* pLastKey = nullptr;
	typename TKeyVal::const_iterator iKeyVal = section.begin();
	for (int n = 0; iKeyVal != section.end(); ++iKeyVal, ++n)
	{
		if (!pLastKey || IsLess(pLastKey, iKeyVal->first.pItem))
		{
			++nCount;
			pLastKey = iKeyVal->first.pItem;
		}
	}
	return nCount;
}

template <class SI_CHAR, class SI_STRLESS, class SI_CONVERTER>
const typename CSimpleIniTempl<SI_CHAR, SI_STRLESS, SI_CONVERTER>::TKeyVal*
	CSimpleIniTempl<SI_CHAR, SI_STRLESS, SI_CONVERTER>::GetSection(
		const SI_CHAR* a_pSection) const
{
	if (a_pSection)
	{
		typename TSection::const_iterator i = m_data.find(a_pSection);
		if (i != m_data.end())
		{
			return &(i->second);
		}
	}
	return 0;
}

template <class SI_CHAR, class SI_STRLESS, class SI_CONVERTER>
void CSimpleIniTempl<SI_CHAR, SI_STRLESS, SI_CONVERTER>::GetAllSections(
	TNamesDepend& a_names) const
{
	auto i = m_data.begin();
	for (int n = 0; i != m_data.end(); ++i, ++n)
	{
		a_names.push_back(i->first);
	}
}

template <class SI_CHAR, class SI_STRLESS, class SI_CONVERTER>
bool CSimpleIniTempl<SI_CHAR, SI_STRLESS, SI_CONVERTER>::GetAllKeys(
	const SI_CHAR* a_pSection, TNamesDepend& a_names) const
{
	if (!a_pSection)
	{
		return false;
	}

	auto iSection = m_data.find(a_pSection);
	if (iSection == m_data.end())
	{
		return false;
	}

	const TKeyVal& section = iSection->second;
	const SI_CHAR* pLastKey = nullptr;
	auto iKeyVal = section.begin();
	for (int n = 0; iKeyVal != section.end(); ++iKeyVal, ++n)
	{
		if (!pLastKey || IsLess(pLastKey, iKeyVal->first.pItem))
		{
			a_names.push_back(iKeyVal->first);
			pLastKey = iKeyVal->first.pItem;
		}
	}

	return true;
}

template <class SI_CHAR, class SI_STRLESS, class SI_CONVERTER>
SI_Error CSimpleIniTempl<SI_CHAR, SI_STRLESS, SI_CONVERTER>::SaveFile(
	const char* a_pszFile) const
{
	FILE* fp = nullptr;
#if __STDC_WANT_SECURE_LIB__
	fopen_s(&fp, a_pszFile, "wb");
#else
	fp = fopen(a_pszFile, "wb");
#endif
	if (!fp) return SI_FILE;
	SI_Error rc = SaveFile(fp);
	fclose(fp);
	return rc;
}

#ifdef SI_HAS_WIDE_FILE
template <class SI_CHAR, class SI_STRLESS, class SI_CONVERTER>
SI_Error CSimpleIniTempl<SI_CHAR, SI_STRLESS, SI_CONVERTER>::SaveFile(
	const SI_WCHAR_T* a_pwszFile) const
{
#ifdef _WIN32
	FILE* fp = _wfopen(a_pwszFile, L"wb");
	if (!fp) return SI_FILE;
	SI_Error rc = SaveFile(fp);
	fclose(fp);
	return rc;
#else  // SI_CONVERT_ICU
	char szFile[256];
	u_austrncpy(szFile, a_pwszFile, sizeof(szFile));
	return SaveFile(szFile);
#endif
}
#endif  // SI_HAS_WIDE_FILE

template <class SI_CHAR, class SI_STRLESS, class SI_CONVERTER>
SI_Error CSimpleIniTempl<SI_CHAR, SI_STRLESS, SI_CONVERTER>::SaveFile(
	FILE* a_pFile) const
{
	FileWriter writer(a_pFile);
	return Save(writer);
}

template <class SI_CHAR, class SI_STRLESS, class SI_CONVERTER>
SI_Error CSimpleIniTempl<SI_CHAR, SI_STRLESS, SI_CONVERTER>::Save(
	OutputWriter& a_oOutput) const
{
	Converter convert;

	// get all of the sections sorted in load order
	TNamesDepend oSections;
	GetAllSections(oSections);
#if (defined(_MSC_VER) && _MSC_VER <= 1200)
	oSections.sort();
#else
	oSections.sort(typename Entry::LoadOrder());
#endif

	// write the file comment if we have one
	bool bNeedNewLine = false;
	if (m_pFileComment)
	{
		if (!OutputMultiLineText(a_oOutput, convert, m_pFileComment))
		{
			return SI_FAIL;
		}
		bNeedNewLine = true;
	}

	// iterate through our sections and output the data
	auto iSection = oSections.begin();
	for (; iSection != oSections.end(); ++iSection)
	{
		// write out the comment if there is one
		if (iSection->pComment)
		{
			if (!convert.ConvertToStore(iSection->pComment))
			{
				return SI_FAIL;
			}
			if (bNeedNewLine)
			{
				a_oOutput.Write(SI_NEWLINE_A);
				a_oOutput.Write(SI_NEWLINE_A);
			}
			a_oOutput.Write(convert.Data());
			a_oOutput.Write(SI_NEWLINE_A);
			bNeedNewLine = false;
		}

		if (bNeedNewLine)
		{
			a_oOutput.Write(SI_NEWLINE_A);
			a_oOutput.Write(SI_NEWLINE_A);
			bNeedNewLine = false;
		}

		// write the section (unless there is no section name)
		if (*iSection->pItem)
		{
			if (!convert.ConvertToStore(iSection->pItem))
			{
				return SI_FAIL;
			}
			a_oOutput.Write("[");
			a_oOutput.Write(convert.Data());
			a_oOutput.Write("]");
			a_oOutput.Write(SI_NEWLINE_A);
		}

		// get all of the keys sorted in load order
		TNamesDepend oKeys;
		GetAllKeys(iSection->pItem, oKeys);
#if (defined(_MSC_VER) && _MSC_VER <= 1200)
		oKeys.sort();
#else
		oKeys.sort(typename Entry::LoadOrder());
#endif

		// write all keys and values
		auto iKey = oKeys.begin();
		for (; iKey != oKeys.end(); ++iKey)
		{
			// get all values for this key
			TNamesDepend oValues;
			GetAllValues(iSection->pItem, iKey->pItem, oValues);

			// write out the comment if there is one
			if (iKey->pComment)
			{
				a_oOutput.Write(SI_NEWLINE_A);
				if (!OutputMultiLineText(a_oOutput, convert, iKey->pComment))
				{
					return SI_FAIL;
				}
			}

			auto iValue = oValues.begin();
			for (; iValue != oValues.end(); ++iValue)
			{
				// write the key
				if (!convert.ConvertToStore(iKey->pItem))
				{
					return SI_FAIL;
				}
				a_oOutput.Write(convert.Data());

				// write the value
				if (!convert.ConvertToStore(iValue->pItem))
				{
					return SI_FAIL;
				}
				a_oOutput.Write("=");
				if (m_bAllowMultiLine && IsMultiLineData(iValue->pItem))
				{
					// multi-line data needs to be processed specially to ensure
					// that we use the correct newline format for the current
					// system
					a_oOutput.Write("<<<SI-END-OF-MULTILINE-TEXT" SI_NEWLINE_A);
					if (!OutputMultiLineText(a_oOutput, convert, iValue->pItem))
					{
						return SI_FAIL;
					}
					a_oOutput.Write("SI-END-OF-MULTILINE-TEXT");
				}
				else
				{
					a_oOutput.Write(convert.Data());
				}
				a_oOutput.Write(SI_NEWLINE_A);
			}
		}

		bNeedNewLine = true;
	}

	return SI_OK;
}

template <class SI_CHAR, class SI_STRLESS, class SI_CONVERTER>
bool CSimpleIniTempl<SI_CHAR, SI_STRLESS, SI_CONVERTER>::OutputMultiLineText(
	OutputWriter& a_oOutput, Converter& a_oConverter,
	const SI_CHAR* a_pText) const
{
	const SI_CHAR* pEndOfLine;
	SI_CHAR cEndOfLineChar = *a_pText;
	while (cEndOfLineChar)
	{
		// find the end of this line
		pEndOfLine = a_pText;
		for (; *pEndOfLine && *pEndOfLine != '\n'; ++pEndOfLine) /*loop*/
			;
		cEndOfLineChar = *pEndOfLine;

		// temporarily null terminate, convert and output the line
		*const_cast<SI_CHAR*>(pEndOfLine) = 0;
		if (!a_oConverter.ConvertToStore(a_pText))
		{
			return false;
		}
		*const_cast<SI_CHAR*>(pEndOfLine) = cEndOfLineChar;
		a_pText += (pEndOfLine - a_pText) + 1;
		a_oOutput.Write(a_oConverter.Data());
		a_oOutput.Write(SI_NEWLINE_A);
	}
	return true;
}

template <class SI_CHAR, class SI_STRLESS, class SI_CONVERTER>
bool CSimpleIniTempl<SI_CHAR, SI_STRLESS, SI_CONVERTER>::Delete(
	const SI_CHAR* a_pSection, const SI_CHAR* a_pKey, bool a_bRemoveEmpty)
{
	if (!a_pSection)
	{
		return false;
	}

	typename TSection::iterator iSection = m_data.find(a_pSection);
	if (iSection == m_data.end())
	{
		return false;
	}

	// remove a single key if we have a keyname
	if (a_pKey)
	{
		typename TKeyVal::iterator iKeyVal = iSection->second.find(a_pKey);
		if (iKeyVal == iSection->second.end())
		{
			return false;
		}

		// remove any copied strings and then the key
		typename TKeyVal::iterator iDelete;
		do
		{
			iDelete = iKeyVal++;

			DeleteString(iDelete->first.pItem);
			DeleteString(iDelete->second);
			iSection->second.erase(iDelete);
		} while (iKeyVal != iSection->second.end() &&
				 !IsLess(a_pKey, iKeyVal->first.pItem));

		// done now if the section is not empty or we are not pruning away
		// the empty sections. Otherwise let it fall through into the section
		// deletion code
		if (!a_bRemoveEmpty || !iSection->second.empty())
		{
			return true;
		}
	}
	else
	{
		// delete all copied strings from this section. The actual
		// entries will be removed when the section is removed.
		typename TKeyVal::iterator iKeyVal = iSection->second.begin();
		for (; iKeyVal != iSection->second.end(); ++iKeyVal)
		{
			DeleteString(iKeyVal->first.pItem);
			DeleteString(iKeyVal->second);
		}
	}

	// delete the section itself
	DeleteString(iSection->first.pItem);
	m_data.erase(iSection);

	return true;
}

template <class SI_CHAR, class SI_STRLESS, class SI_CONVERTER>
void CSimpleIniTempl<SI_CHAR, SI_STRLESS, SI_CONVERTER>::DeleteString(
	const SI_CHAR* a_pString)
{
	// strings may exist either inside the data block, or they will be
	// individually allocated and stored in m_strings. We only physically
	// delete those stored in m_strings.
	if (a_pString < m_pData || a_pString >= m_pData + m_uDataLen)
	{
		typename TNamesDepend::iterator i = m_strings.begin();
		for (; i != m_strings.end(); ++i)
		{
			if (a_pString == i->pItem)
			{
				delete[] const_cast<SI_CHAR*>(i->pItem);
				m_strings.erase(i);
				break;
			}
		}
	}
}

// ---------------------------------------------------------------------------
//                              CONVERSION FUNCTIONS
// ---------------------------------------------------------------------------

/**
 * Generic case-sensitive less than comparison. This class returns numerically
 * ordered ASCII case-sensitive text for all possible sizes and types of
 * SI_CHAR.
 */
template <class SI_CHAR>
struct SI_GenericCase
{
	bool operator()(const SI_CHAR* pLeft, const SI_CHAR* pRight) const
	{
		long cmp;
		for (; *pLeft && *pRight; ++pLeft, ++pRight)
		{
			cmp = (long)*pLeft - (long)*pRight;
			if (cmp != 0)
			{
				return cmp < 0;
			}
		}
		return *pRight != 0;
	}
};

/**
 * Generic ASCII case-insensitive less than comparison. This class returns
 * numerically ordered ASCII case-insensitive text for all possible sizes
 * and types of SI_CHAR. It is not safe for MBCS text comparison where
 * ASCII A-Z characters are used in the encoding of multi-byte characters.
 */
template <class SI_CHAR>
struct SI_GenericNoCase
{
	inline SI_CHAR locase(SI_CHAR ch) const
	{
		return (ch < 'A' || ch > 'Z') ? ch : (ch - 'A' + 'a');
	}
	bool operator()(const SI_CHAR* pLeft, const SI_CHAR* pRight) const
	{
		long cmp;
		for (; *pLeft && *pRight; ++pLeft, ++pRight)
		{
			cmp = (long)locase(*pLeft) - (long)locase(*pRight);
			if (cmp != 0)
			{
				return cmp < 0;
			}
		}
		return *pRight != 0;
	}
};

/**
 * Null conversion class for MBCS/UTF-8 to char (or equivalent).
 */
template <class SI_CHAR>
class SI_ConvertA
{
   public:
	SI_ConvertA() {}
	/* copy and assignment */
	SI_ConvertA(const SI_ConvertA& rhs) { operator=(rhs); }
	SI_ConvertA& operator=(const SI_ConvertA& rhs) { return *this; }
	/** Calculate the number of SI_CHAR required for converting the input
	 * from the storage format. The storage format is always UTF-8 or MBCS.
	 *
	 * @param a_pInputData  Data in storage format to be converted to SI_CHAR.
	 * @param a_uInputDataLen Length of storage format data in bytes. This
	 *                      must be the actual length of the data, including
	 *                      nullptr byte if nullptr terminated string is
	 * required.
	 * @return              Number of SI_CHAR required by the string when
	 *                      converted. If there are embedded nullptr bytes in
	 * the
	 *                      input data, only the string up and not including
	 *                      the nullptr byte will be converted.
	 * @return              -1 cast to size_t on a conversion error.
	 */
	virtual size_t SizeFromStore(
		const char* a_pInputData, size_t a_uInputDataLen)
	{
		(void)a_pInputData;
		SI_ASSERT(a_uInputDataLen != (size_t)-1);

		// ASCII/MBCS/UTF-8 needs no conversion
		return a_uInputDataLen;
	}

	/** Convert the input string from the storage format to SI_CHAR.
	 * The storage format is always UTF-8 or MBCS.
	 *
	 * @param a_pInputData  Data in storage format to be converted to SI_CHAR.
	 * @param a_uInputDataLen Length of storage format data in bytes. This
	 *                      must be the actual length of the data, including
	 *                      nullptr byte if nullptr terminated string is
	 * required.
	 * @param a_pOutputData Pointer to the output buffer to received the
	 *                      converted data.
	 * @param a_uOutputDataSize Size of the output buffer in SI_CHAR.
	 * @return              true if all of the input data was successfully
	 *                      converted.
	 */
	virtual bool ConvertFromStore(
		const char* a_pInputData, size_t a_uInputDataLen,
		SI_CHAR* a_pOutputData, size_t a_uOutputDataSize)
	{
		// ASCII/MBCS/UTF-8 needs no conversion
		if (a_uInputDataLen > a_uOutputDataSize)
		{
			return false;
		}
		memcpy(a_pOutputData, a_pInputData, a_uInputDataLen);
		return true;
	}

	/** Calculate the number of char required by the storage format of this
	 * data. The storage format is always UTF-8 or MBCS.
	 *
	 * @param a_pInputData  nullptr terminated string to calculate the number of
	 *                      bytes required to be converted to storage format.
	 * @return              Number of bytes required by the string when
	 *                      converted to storage format. This size always
	 *                      includes space for the terminating nullptr
	 * character.
	 * @return              -1 cast to size_t on a conversion error.
	 */
	size_t SizeToStore(const SI_CHAR* a_pInputData)
	{
		// ASCII/MBCS/UTF-8 needs no conversion
		return strlen((const char*)a_pInputData) + 1;
	}

	/** Convert the input string to the storage format of this data.
	 * The storage format is always UTF-8 or MBCS.
	 *
	 * @param a_pInputData  nullptr terminated source string to convert. All of
	 *                      the data will be converted including the
	 *                      terminating nullptr character.
	 * @param a_pOutputData Pointer to the buffer to receive the converted
	 *                      string.
	 * @param a_uOutputDataSize Size of the output buffer in char.
	 * @return              true if all of the input data, including the
	 *                      terminating nullptr character was successfully
	 *                      converted.
	 */
	bool ConvertToStore(
		const SI_CHAR* a_pInputData, char* a_pOutputData,
		size_t a_uOutputDataSize)
	{
		// calc input string length (SI_CHAR type and size independent)
		size_t uInputLen = strlen((const char*)a_pInputData) + 1;
		if (uInputLen > a_uOutputDataSize)
		{
			return false;
		}

		// ascii/UTF-8 needs no conversion
		memcpy(a_pOutputData, a_pInputData, uInputLen);
		return true;
	}
};

/** MRPT custom INI file parser to allow minimal file preprocessing:
 * - multiline entries via an end-of-line backslash ('\')
 */
struct MRPT_IniFileParser : public SI_ConvertA<char>
{
	MRPT_IniFileParser() : SI_ConvertA<char>() {}
	/* copy and assignment */
	MRPT_IniFileParser(const MRPT_IniFileParser& rhs) : SI_ConvertA<char>(rhs)
	{
		SI_ConvertA<char>::operator=(rhs);
	}
	MRPT_IniFileParser& operator=(const MRPT_IniFileParser& rhs)
	{
		SI_ConvertA<char>::operator=(rhs);
		return *this;
	}

	size_t SizeFromStore(
		const char* a_pInputData, size_t a_uInputDataLen) override
	{
		SI_ASSERT(a_uInputDataLen != (size_t)-1);
		return do_parse(a_pInputData, a_uInputDataLen, nullptr);
	}

	bool ConvertFromStore(
		const char* a_pInputData, size_t a_uInputDataLen, char* a_pOutputData,
		size_t a_uOutputDataSize) override
	{
		this->do_parse(a_pInputData, a_uInputDataLen, a_pOutputData);
		return true;
	}

   private:
	struct ParseContext
	{
		std::map<std::string, std::string> defined_vars;
		std::map<std::string, double> defined_vars_values;
		unsigned int line_count = 1;
	};

	// Return a string or a number (as string) if expr = "$eval{...}"
	std::string parse_process_var_eval(const ParseContext& pc, std::string expr)
	{
		expr = mrpt::system::trim(expr);
		while (expr.size() > 5)
		{
			auto p = expr.find("$env{");
			if (p != std::string::npos)
			{
				auto pend = expr.find("}", p);
				if (pend == std::string::npos)
					throw std::runtime_error(mrpt::format(
						"Line %u: Expected closing `}` near: `%s`",
						pc.line_count, expr.c_str()));
				const auto substr = expr.substr(p + 5, pend - p - 5);
				std::string new_expr = expr.substr(0, p);
				auto env_val = ::getenv(substr.c_str());
				if (env_val) new_expr += std::string(env_val);
				new_expr += expr.substr(pend + 1);
				new_expr.swap(expr);
			}
			else if ((p = expr.find("$eval{")) != std::string::npos)
			{
				auto pend = expr.find("}", p);
				if (pend == std::string::npos)
					throw std::runtime_error(mrpt::format(
						"Line %u: Expected closing `}` near: `%s`",
						pc.line_count, expr.c_str()));

				const auto substr = expr.substr(p + 6, pend - p - 6);
				mrpt::expr::CRuntimeCompiledExpression cexpr;
				cexpr.compile(
					substr, pc.defined_vars_values,
					mrpt::format("Line %u: ", pc.line_count));

				std::string new_expr = expr.substr(0, p);
				new_expr += mrpt::format("%e", cexpr.eval());
				new_expr += expr.substr(pend + 1);
				new_expr.swap(expr);
			}
			else
				break;  // nothing else to evaluate
		}
		return expr;
	}

	void parse_process_var_define(
		ParseContext& pc, const std::string& var_name,
		const std::string& var_value)
	{
		if (!var_name.empty())
		{
			pc.defined_vars[var_name] = var_value;
			if (!var_value.empty())
			{
				pc.defined_vars_values[var_name] =
					::atof(parse_process_var_eval(pc, var_value).c_str());
			}
		}
	}

	/** Shared code for the two virtual methods. If out_str==NULL, just count
	 * output bytes */
	size_t do_parse(const char* in_str, const size_t in_len, char* out_str)
	{
		ParseContext pc;
		size_t out_len = 0, i = 0;
		while (i < in_len)
		{
			const char c = in_str[i];
			if (c == '\n')
			{
				pc.line_count++;
			}

			if (c == '\\' && i < in_len - 1 &&
				(in_str[i + 1] == '\r' || in_str[i + 1] == '\n'))
			{
				// Skip the backslash + one newline: CR "\r", LF "\n", CR+LF
				// "\r\n"
				if (i < in_len - 2 && in_str[i + 1] == '\r' &&
					in_str[i + 2] == '\n')
				{
					// out_len += 0;
					i += 3;
				}
				else if (in_str[i + 1] == '\r' || in_str[i + 1] == '\n')
				{
					// out_len += 0;
					i += 2;
				}
				else
				{
					throw std::runtime_error(
						"MRPT_IniFileParser: parse error, shouldn't reach "
						"here!");
				}
			}
			else
			{
				// Handle "@define varname value"
				if (in_len > i + 7 && !::strncmp(in_str + i, "@define", 7))
				{
					// Extract rest of this line:
					i += 7;
					std::string var_name, var_value;
					bool in_var_name = false, done_var_name = false;
					while (i < in_len && in_str[i] != '\r' && in_str[i] != '\n')
					{
						const char ch = in_str[i];
						i++;
						if (ch != ' ' && ch != '\t')
						{
							// not whitespace
							if (!in_var_name && !done_var_name)
							{
								in_var_name = true;
							}
						}
						else
						{
							// whitespace
							if (in_var_name)
							{
								in_var_name = false;
								done_var_name = true;
							}
						}
						if (in_var_name)
						{
							var_name += ch;
						}
						if (done_var_name)
						{
							var_value += ch;
						}
					}

					parse_process_var_define(pc, var_name, var_value);
					continue;
				}

				// Handle "${varname}"
				if (in_len > i + 4 && in_str[i] == '$' && in_str[i + 1] == '{')
				{
					// extract varname:
					i += 2;
					std::string varname;
					bool end_ok = false;
					while (i < in_len && in_str[i] != '\n' && in_str[i] != '\r')
					{
						const char ch = in_str[i];
						i++;
						if (ch == '}')
						{
							end_ok = true;
							break;
						}
						varname += ch;
					}
					if (!end_ok)
					{
						throw std::runtime_error(mrpt::format(
							"Line %u: Expected closing `}` near: `%s`",
							pc.line_count, varname.c_str()));
					}

					const auto it = pc.defined_vars.find(varname);
					if (it == pc.defined_vars.end())
						throw std::runtime_error(mrpt::format(
							"Line %u: Unknown variable `${%s}`", pc.line_count,
							varname.c_str()));

					const auto str_out = parse_process_var_eval(pc, it->second);

					for (const char ch : str_out)
					{
						if (out_str) out_str[out_len] = ch;
						out_len++;
					}
					continue;
				}

				// Handle "$eval{expression}"
				if (in_len > i + 7 && !strncmp(in_str + i, "$eval{", 6))
				{
					// extract expression:
					std::string expr;
					bool end_ok = false;
					while (i < in_len && in_str[i] != '\n' && in_str[i] != '\r')
					{
						const char ch = in_str[i];
						i++;
						expr += ch;
						if (ch == '}')
						{
							end_ok = true;
							break;
						}
					}
					if (!end_ok)
					{
						throw std::runtime_error(mrpt::format(
							"Line %u: Expected closing `}` near: `%s`",
							pc.line_count, expr.c_str()));
					}

					const std::string res = parse_process_var_eval(pc, expr);

					for (const char ch : res)
					{
						if (out_str) out_str[out_len] = ch;
						out_len++;
					}
					continue;
				}

				// Handle "$env{var}"
				if (in_len > i + 6 && !strncmp(in_str + i, "$env{", 5))
				{
					// extract expression:
					std::string expr;
					bool end_ok = false;
					while (i < in_len && in_str[i] != '\n' && in_str[i] != '\r')
					{
						const char ch = in_str[i];
						i++;
						expr += ch;
						if (ch == '}')
						{
							end_ok = true;
							break;
						}
					}
					if (!end_ok)
					{
						throw std::runtime_error(mrpt::format(
							"Line %u: Expected closing `}` near: `%s`",
							pc.line_count, expr.c_str()));
					}

					const std::string res = parse_process_var_eval(pc, expr);

					for (const char ch : res)
					{
						if (out_str) out_str[out_len] = ch;
						out_len++;
					}
					continue;
				}

				// Normal case:
				if (out_str)
				{
					out_str[out_len] = c;
				}
				out_len++;
				i++;
			}
		}
		return out_len;
	}
};

// ---------------------------------------------------------------------------
//                                  TYPE DEFINITIONS
// ---------------------------------------------------------------------------

using CSimpleIniA =
	CSimpleIniTempl<char, SI_GenericNoCase<char>, SI_ConvertA<char>>;
using CSimpleIniCaseA =
	CSimpleIniTempl<char, SI_GenericCase<char>, SI_ConvertA<char>>;

using MRPT_CSimpleIni =
	CSimpleIniTempl<char, SI_GenericNoCase<char>, MRPT_IniFileParser>;

#ifdef _UNICODE
#define CSimpleIni CSimpleIniW
#define CSimpleIniCase CSimpleIniCaseW
#define SI_NEWLINE SI_NEWLINE_W
#else  // !_UNICODE
#define CSimpleIni CSimpleIniA
#define CSimpleIniCase CSimpleIniCaseA
#define SI_NEWLINE SI_NEWLINE_A
#endif  // _UNICODE

}  // namespace mrpt::config::simpleini
