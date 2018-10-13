/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <fstream>
#include <string>
#include <iosfwd>

namespace mrpt::io
{
/** A class for parsing text files, returning each non-empty and non-comment
 * line, along its line number. Lines are strip out of leading and trailing
 * whitespaces. By default, lines starting with either "#", "//" or "%" are
 * skipped as comment lines, unless this behavior is explicitly disabled with
 * \a enableCommentFilters.
 * \ingroup mrpt_io_grp
 */
class CTextFileLinesParser
{
   public:
	/** Default constructor; should call \a open() at some moment later. */
	CTextFileLinesParser() = default;
	/** Constructor for opening a file  \exception std::exception On error
	 * opening file */
	explicit CTextFileLinesParser(const std::string& filename);

	/** Constructor for reading from a generic std::istream. Note that a
	 * reference to the stream is stored in the object, so it's the user
	 * responsibility to make sure the stream is not destroyed before than
	 * this object.
	 */
	explicit CTextFileLinesParser(std::istream& in);

	/** Open a file (an alternative to the constructor with a file name) */
	void open(const std::string& fil);

	/** Opens for reading a generic std::istream. Note that a
	 * reference to the stream is stored in the object, so it's the user
	 * responsibility to make sure the stream is not destroyed before than
	 * this object.
	 */
	void open(std::istream& in);

	/** Close the file (no need to call it normally, the file is closed upon
	 * destruction) */
	void close();
	/** Reset the read pointer to the beginning of the file */
	void rewind();

	/** Reads from the file and return the next (non-comment) line, as a
	 * std::string
	 * \return false on EOF.
	 */
	bool getNextLine(std::string& out_str);

	/** Reads from the file and stores the next (non-comment) line into the
	 * given stream buffer.
	 * \return false on EOF.
	 */
	bool getNextLine(std::istringstream& buf);

	/** Return the line number of the last line returned with \a getNextLine */
	size_t getCurrentLineNumber() const;

	/** Enable/disable filtering of lines starting with "%", "//" or "#",
	 * respectively. */
	void enableCommentFilters(
		bool filter_MATLAB_comments, bool filter_C_comments,
		bool filter_SH_comments);

   private:
	std::string m_fileName;
	std::istream* m_in{nullptr};
	bool m_in_ownership{true};
	size_t m_curLineNum{0};
	bool m_filter_MATLAB_comments{true};
	bool m_filter_C_comments{true};
	bool m_filter_SH_comments{true};

};  // end of CTextFileLinesParser
}  // namespace mrpt::io
