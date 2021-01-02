/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

/** Following functions investigate the directory manipulation capabilities in
 * MRPT. Functions are called from the main function in the end of the script.
 * See each function for the corresponding usage.
 * See https://reference.mrpt.org/stable/group__filesystem.html on the
 * documentation of the functions
 */

#include <mrpt/io/CFileOutputStream.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPosePDF.h>
#include <mrpt/system/datetime.h>
#include <mrpt/system/filesystem.h>

#include <iostream>
#include <sstream>
#include <string>

using namespace mrpt;
using namespace mrpt::poses;
using namespace mrpt::io;
using namespace mrpt::system;
using namespace std;

/** Create a directory
 * Open and write some content in a file inside the directory
 * If directory exists delete it altogether.
 */
void setupDirContents()
{
	CFileOutputStream f;

	string dir_name = "dir_a";
	string file_name = "file_b";
	CPose2D a_pose(1, 2, 40.0_deg);

	if (!directoryExists(dir_name))
	{
		cout << "Creating directory... " << endl;
		createDirectory(dir_name);
		f.open(dir_name + "/" + file_name);
		if (f.fileOpenCorrectly())
		{  // checking for errors...
			cout << "file was opened correctly" << endl;
			// CSerializable form (binary)
			f.printf("some random text ...\n");
			f.printf("some more random text.\n");
			f.printf("CPose2D: %s", a_pose.asString().c_str());
			f.close();
		}
		else
		{
			cout << "file was NOT opened successfully" << endl;
			return;
		}
	}
	else
	{
		cout << "directory " << dir_name << " exists. " << endl;
		cout << "removing directory altogether... " << endl;
		deleteFilesInDirectory(
			dir_name,
			/* deleteDirectoryAsWell = */ true);
	}
}

/** Initialize a directory along with some dummy content.
 * Rename the directory and filenames inside it to
 * ${PREVIOUS_FNAME}_renamed_datetime
 */
void renameDirContents()
{
	string dir_name = "dir_b";
	CFileOutputStream f;
	string fname = "file";
	stringstream ss_tmp;

	// get the current datetime as a string - add it to the renamed fnames
	TTimeStamp cur_time(getCurrentTime());
	string cur_time_str = dateTimeToString(cur_time);
	string cur_time_validstr(fileNameStripInvalidChars(cur_time_str));
	string string_to_add = "_renamed_" + cur_time_validstr;

	bool success;  // flag for specifying whether an operation was successfully
	// completed

	if (!directoryExists(dir_name))
	{
		cout << "directory " << dir_name << " doesn't exist. " << endl;
		cout << "Creating it.. " << endl;
		success = createDirectory(dir_name);
		if (!success)
		{
			THROW_EXCEPTION(
				"There was an error creating the directory: " + dir_name);
		}
	}

	// build the initial directory contents
	for (int i = 0; i < 10; i++)
	{
		ss_tmp.str("");
		ss_tmp << dir_name << "/" << fname << i;
		f.open(ss_tmp.str());
		f.printf("dummy text in file...");
		f.close();
	}

	// rename all the contents (of depth 1)
	for (int i = 0; i < 10; i++)
	{
		ss_tmp.str("");
		ss_tmp << dir_name << "/" << fname << i;
		success = renameFile(
			ss_tmp.str(),
			/*new_name = */ ss_tmp.str() + string_to_add);
	}

	// finally rename the directory itself
	cout << "Renaming directory " << dir_name << " to: " << dir_name
		 << string_to_add << endl;
	string* err_msg = nullptr;  // flag for catching the error msg if any..
	success = renameFile(dir_name, dir_name + string_to_add, err_msg);
	if (success)
	{
		cout << "Directory renaming was successful!" << endl;
	}
	else
	{
		THROW_EXCEPTION("Error while trying to rename directory: " + dir_name);
	}
}

//
// MAIN
//
int main()
{
	try
	{
		cout << "Running setupDirContents fun..." << endl;
		cout << "------------------------------" << endl;
		setupDirContents();
		cout << "Press a key to continue..." << endl;
		// char c=
		getchar();

		cout << "Running RenameDirContents fun..." << endl;
		cout << "------------------------------" << endl;
		renameDirContents();

		return 0;
	}
	catch (const std::exception& e)
	{
		std::cerr << "MRPT error: " << mrpt::exception_to_str(e) << std::endl;
		return -1;
	}
	catch (...)
	{
		printf("Untyped exception!!");
		return -1;
	}
}
