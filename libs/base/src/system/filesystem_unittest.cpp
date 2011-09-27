/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */

#include <mrpt/base.h>
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

