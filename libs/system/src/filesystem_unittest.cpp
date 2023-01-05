/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <gtest/gtest.h>
#include <mrpt/system/filesystem.h>

using namespace mrpt;
using namespace std;

TEST(FileSystem, fileNameChangeExtension)
{
	EXPECT_EQ(
		mrpt::system::fileNameChangeExtension("cool.bar", "txt"), "cool.txt");
	EXPECT_EQ(
		mrpt::system::fileNameChangeExtension("d:/dataset.rawlog", "log"),
		"d:/dataset.log");
	EXPECT_EQ(
		mrpt::system::fileNameChangeExtension("d:/dataset.rawlog", ""),
		"d:/dataset");
	EXPECT_EQ(
		mrpt::system::fileNameChangeExtension("d:/dataset.", ""), "d:/dataset");
	EXPECT_EQ(
		mrpt::system::fileNameChangeExtension("d:/dataset", "rawlog"),
		"d:/dataset.rawlog");
	EXPECT_EQ(
		mrpt::system::fileNameChangeExtension("c:\\foo.txt", "bar"),
		"c:\\foo.bar");
	EXPECT_EQ(
		mrpt::system::fileNameChangeExtension("/home/pepe/foo.txt", "bar"),
		"/home/pepe/foo.bar");
}

TEST(FileSystem, extractFileExtension)
{
	EXPECT_EQ(mrpt::system::extractFileExtension("c:\\foo.txt"), "txt");
	EXPECT_EQ(mrpt::system::extractFileExtension("/home/pepe/foo.txt"), "txt");
	EXPECT_EQ(mrpt::system::extractFileExtension("/home/pepe/foo"), "");
	EXPECT_EQ(mrpt::system::extractFileExtension("/home/pepe/foo."), "");
}

TEST(FileSystem, extractFileDirectory)
{
#ifdef _WIN32
	EXPECT_EQ(
		mrpt::system::extractFileDirectory("D:\\imgs\\foo.txt"), "D:\\imgs");
#else
	EXPECT_EQ(
		mrpt::system::extractFileDirectory("/home/pepe/foo.txt"), "/home/pepe");
#endif
}

TEST(FileSystem, extractFileName)
{
#ifdef _WIN32
	EXPECT_EQ(
		mrpt::system::extractFileName("d:\\imgs\\dataset.log"), "dataset");
#else
	EXPECT_EQ(mrpt::system::extractFileName("/home/pepe/foo.txt"), "foo");
#endif
}

TEST(FileSystem, filePathSeparatorsToNative)
{
#ifdef _WIN32
	EXPECT_EQ(mrpt::system::filePathSeparatorsToNative("C:"), "C:");
	EXPECT_EQ(mrpt::system::filePathSeparatorsToNative("C:\\"), "C:\\");
	EXPECT_EQ(mrpt::system::filePathSeparatorsToNative("C:/"), "C:\\");
	EXPECT_EQ(mrpt::system::filePathSeparatorsToNative("/loco"), "\\loco");
	EXPECT_EQ(
		mrpt::system::filePathSeparatorsToNative("/loco/perico"),
		"\\loco\\perico");
	EXPECT_EQ(
		mrpt::system::filePathSeparatorsToNative("/loco\\perico"),
		"\\loco\\perico");
	EXPECT_EQ(
		mrpt::system::filePathSeparatorsToNative("/loco\\perico/"),
		"\\loco\\perico\\");
#else
	EXPECT_EQ(mrpt::system::filePathSeparatorsToNative("C:"), "C:");
	EXPECT_EQ(mrpt::system::filePathSeparatorsToNative("C:\\"), "C:/");
	EXPECT_EQ(mrpt::system::filePathSeparatorsToNative("C:/"), "C:/");
	EXPECT_EQ(mrpt::system::filePathSeparatorsToNative("/loco"), "/loco");
	EXPECT_EQ(
		mrpt::system::filePathSeparatorsToNative("/loco/perico"),
		"/loco/perico");
	EXPECT_EQ(
		mrpt::system::filePathSeparatorsToNative("/loco\\perico"),
		"/loco/perico");
	EXPECT_EQ(
		mrpt::system::filePathSeparatorsToNative("/loco\\perico/"),
		"/loco/perico/");
#endif
}

TEST(FileSystem, toAbsolutePath)
{
	auto tmpFile = mrpt::system::getTempFileName();

	EXPECT_EQ(
		mrpt::system::toAbsolutePath(tmpFile),	//
		tmpFile);

	EXPECT_EQ(
		mrpt::system::toAbsolutePath(".", true /*canonical*/),	//
		mrpt::system::getcwd());
}

TEST(FileSystem, pathJoin)
{
#ifdef _WIN32
	EXPECT_EQ(
		mrpt::system::pathJoin({"C:\\", "joe", "p.ini"}),  //
		"C:\\joe\\p.ini");
#else
	EXPECT_EQ(
		mrpt::system::pathJoin({"/home", "joe", "p.ini"}),	//
		"/home/joe/p.ini");
#endif
}
