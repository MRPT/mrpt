/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/system/filesystem.h>
#include <gtest/gtest.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace std;


TEST(FileSystem, fileNameChangeExtension)
{
	EXPECT_EQ( mrpt::system::fileNameChangeExtension("cool.bar","txt"), "cool.txt" );
	EXPECT_EQ( mrpt::system::fileNameChangeExtension("d:/dataset.rawlog","log"), "d:/dataset.log" );
	EXPECT_EQ( mrpt::system::fileNameChangeExtension("d:/dataset.rawlog",""), "d:/dataset." );
	EXPECT_EQ( mrpt::system::fileNameChangeExtension("d:/dataset.",""), "d:/dataset." );
	EXPECT_EQ( mrpt::system::fileNameChangeExtension("d:/dataset","rawlog"), "d:/dataset.rawlog" );
	EXPECT_EQ( mrpt::system::fileNameChangeExtension("c:\\foo.txt","bar"), "c:\\foo.bar" );
	EXPECT_EQ( mrpt::system::fileNameChangeExtension("/home/pepe/foo.txt","bar"), "/home/pepe/foo.bar" );
}

TEST(FileSystem, extractFileExtension)
{
	EXPECT_EQ( mrpt::system::extractFileExtension("c:\\foo.txt"),"txt");
	EXPECT_EQ( mrpt::system::extractFileExtension("/home/pepe/foo.txt"),"txt");
	EXPECT_EQ( mrpt::system::extractFileExtension("/home/pepe/foo"),"");
	EXPECT_EQ( mrpt::system::extractFileExtension("/home/pepe/foo."),"");
}

TEST(FileSystem, extractFileDirectory)
{
	EXPECT_EQ( mrpt::system::extractFileDirectory("/home/pepe/foo.txt"),"/home/pepe/");
	EXPECT_EQ( mrpt::system::extractFileDirectory("D:\\imgs\\foo.txt"),"D:\\imgs\\");
}


TEST(FileSystem, extractFileName)
{
	EXPECT_EQ( mrpt::system::extractFileName("/home/pepe/foo.txt"),"foo");
	EXPECT_EQ( mrpt::system::extractFileName("d:\\imgs\\dataset.log"),"dataset");
}

TEST(FileSystem, filePathSeparatorsToNative)
{
#ifdef MRPT_OS_WINDOWS
	EXPECT_EQ( mrpt::system::filePathSeparatorsToNative("C:"),"C:");
	EXPECT_EQ( mrpt::system::filePathSeparatorsToNative("C:\\"),"C:\\");
	EXPECT_EQ( mrpt::system::filePathSeparatorsToNative("C:/"),"C:\\");
	EXPECT_EQ( mrpt::system::filePathSeparatorsToNative("/loco"),"\\loco");
	EXPECT_EQ( mrpt::system::filePathSeparatorsToNative("/loco/perico"),"\\loco\\perico");
	EXPECT_EQ( mrpt::system::filePathSeparatorsToNative("/loco\\perico"),"\\loco\\perico");
	EXPECT_EQ( mrpt::system::filePathSeparatorsToNative("/loco\\perico/"),"\\loco\\perico\\");
#else
	EXPECT_EQ( mrpt::system::filePathSeparatorsToNative("C:"),"C:");
	EXPECT_EQ( mrpt::system::filePathSeparatorsToNative("C:\\"),"C:/");
	EXPECT_EQ( mrpt::system::filePathSeparatorsToNative("C:/"),"C:/");
	EXPECT_EQ( mrpt::system::filePathSeparatorsToNative("/loco"),"/loco");
	EXPECT_EQ( mrpt::system::filePathSeparatorsToNative("/loco/perico"),"/loco/perico");
	EXPECT_EQ( mrpt::system::filePathSeparatorsToNative("/loco\\perico"),"/loco/perico");
	EXPECT_EQ( mrpt::system::filePathSeparatorsToNative("/loco\\perico/"),"/loco/perico/");
#endif
}

