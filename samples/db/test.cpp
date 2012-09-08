/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
   | Copyright (c) 2012, University of Almeria                                 |
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

#include <mrpt/utils.h>

using namespace mrpt::utils;

// ------------------------------------------------------
//				TestDB
// ------------------------------------------------------
void TestDB()
{
	CSimpleDatabase		db,db2,db3;
	CSimpleDatabaseTablePtr table;
	size_t			i;
	CTicTac			tictac;


	table = db.createTable("table1");

	table->addField("name");
	table->addField("value");

	i=table->appendRecord();
	table->set(i,"name","cell_11");
	table->set(i,"value","cell_12");

	i=table->appendRecord();
	table->set(i,"name","cell_21");
	table->set(i,"value","cell_22");

	i=table->appendRecord();
	table->set(i,"name","another cell");
	table->set(i,"value","And this is an\n example of a multi-line \n \t\t and formated string.");

	// Save/load as binary:
	// ------------------------------
	tictac.Tic();
	CFileStream("test_db.bin",fomWrite) << db;
	CFileStream("test_db.bin",fomRead) >> db2;
	double t_bin = tictac.Tac();

	printf("Read test: %u tables in DB. table[0]=%s, records=%u\n",
			(unsigned int)db2.tablesCount(),
			db2.tablesName(0).c_str(),
			(unsigned int)db2.getTable("table1")->getRecordCount()
		);

	int query = db2.getTable("table1")->query("name","cell_11");
	printf("query=%i,", query);
	printf(" '%s'\n", db2.getTable("table1")->get(query,"value").c_str() );

	// Save/load as XML:
	// ------------------------------
	tictac.Tic();
	db.saveAsXML("test_db.xml");
	db3.loadFromXML("test_db.xml");
	double t_xml = tictac.Tac();
	db3.saveAsXML("test_db_out.xml");

	printf("Binary write+read: %f ms\n",t_bin*1e3);
	printf("XML write+read: %f ms\n",t_xml*1e3);

}

// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------
int main()
{
	try
	{
		 TestDB();

		return 0;
	} catch (std::exception &e)
	{
		std::cout << "MRPT exception caught: " << e.what() << std::endl;
		return -1;
	}
	catch (...)
	{
		printf("Untyped exception!!");
		return -1;
	}
}
