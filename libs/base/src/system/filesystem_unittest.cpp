/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                                 |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
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

