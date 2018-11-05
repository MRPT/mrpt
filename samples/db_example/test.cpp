/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include <mrpt/db/CSimpleDatabase.h>
#include <mrpt/io/CFileStream.h>
#include <mrpt/system/CTicTac.h>
#include <mrpt/serialization/CArchive.h>
#include <cstdio>
#include <iostream>

// ------------------------------------------------------
//				TestDB
// ------------------------------------------------------
void TestDB()
{
	using namespace mrpt::db;
	using namespace mrpt::io;
	using namespace mrpt::serialization;

	CSimpleDatabase db, db2, db3;
	CSimpleDatabaseTable::Ptr table;
	size_t i;
	mrpt::system::CTicTac tictac;

	table = db.createTable("table1");

	table->addField("name");
	table->addField("value");

	i = table->appendRecord();
	table->set(i, "name", "cell_11");
	table->set(i, "value", "cell_12");

	i = table->appendRecord();
	table->set(i, "name", "cell_21");
	table->set(i, "value", "cell_22");

	i = table->appendRecord();
	table->set(i, "name", "another cell");
	table->set(
		i, "value",
		"And this is an\n example of a multi-line \n \t\t and formated "
		"string.");

	// Save/load as binary:
	// ------------------------------
	tictac.Tic();
	{
		CFileStream f("test_db.bin", fomWrite);
		archiveFrom(f) << db;
	}
	{
		CFileStream f("test_db.bin", fomRead);
		archiveFrom(f) >> db2;
	}
	double t_bin = tictac.Tac();

	printf(
		"Read test: %u tables in DB. table[0]=%s, records=%u\n",
		(unsigned int)db2.tablesCount(), db2.tablesName(0).c_str(),
		(unsigned int)db2.getTable("table1")->getRecordCount());

	int query = db2.getTable("table1")->query("name", "cell_11");
	printf("query=%i,", query);
	printf(" '%s'\n", db2.getTable("table1")->get(query, "value").c_str());

	// Save/load as XML:
	// ------------------------------
	tictac.Tic();
	db.saveAsXML("test_db.xml");
	db3.loadFromXML("test_db.xml");
	double t_xml = tictac.Tac();
	db3.saveAsXML("test_db_out.xml");

	printf("Binary write+read: %f ms\n", t_bin * 1e3);
	printf("XML write+read: %f ms\n", t_xml * 1e3);
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
	}
	catch (const std::exception& e)
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
