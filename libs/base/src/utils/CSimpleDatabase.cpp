/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "base-precomp.h"  // Precompiled headers

#include <mrpt/utils/CSimpleDatabase.h>
#include <mrpt/utils/CStream.h>
#include <mrpt/system/os.h>

using namespace mrpt::utils;
using namespace mrpt::system;
using namespace std;

#undef _UNICODE			// JLBC

#include "xmlparser/xmlParser.h"

#include <iostream>

// This must be added to any CSerializable class implementation file.
IMPLEMENTS_SERIALIZABLE(CSimpleDatabase, CSerializable, mrpt::utils)
IMPLEMENTS_SERIALIZABLE(CSimpleDatabaseTable, CSerializable, mrpt::utils)

/*---------------------------------------------------------------
						writeToStream
 ---------------------------------------------------------------*/
void  CSimpleDatabase::writeToStream(mrpt::utils::CStream &out, int *out_Version) const
{
	if (out_Version)
		*out_Version = 0;
	else
	{
		// Save all tables in DB:
		uint32_t n = (uint32_t)m_tables.size();
		out << n;

		for (const_iterator i=m_tables.begin();i!=m_tables.end();++i)
		{
			out << i->first; //.c_str();
			out << *i->second;
		}
	}
}
/*---------------------------------------------------------------
						readFromStream
 ---------------------------------------------------------------*/
void  CSimpleDatabase::readFromStream(mrpt::utils::CStream &in, int version)
{
	switch (version)
	{
	case 0:
	{
		std::string	aux;

		// Clear existing tables:
		clear();

		// Load all tables in DB:
		uint32_t n;
		in >> n;

		for (uint32_t i=0;i<n;i++)
		{
			in >> aux;

			CSimpleDatabaseTablePtr newTb = CSimpleDatabaseTable::Create();
			in >> (*newTb);

			m_tables[aux] = newTb;
		}
	}
	break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};
}

/*---------------------------------------------------------------
						writeToStream
 ---------------------------------------------------------------*/
void  CSimpleDatabaseTable::writeToStream(mrpt::utils::CStream &out, int *out_Version) const
{
	if (out_Version)
		*out_Version = 0;
	else
	{
		uint32_t	row,col,nRec = (uint32_t) getRecordCount(), nFie=(uint32_t) fieldsCount();

		out << nRec << nFie;

		for (col=0;col<nFie;col++)
			out << field_names[col]; //.c_str();

		for (row=0;row<nRec;row++)
			for (col=0;col<nFie;col++)
				out << data[row][col]; //.c_str();
	}
}
/*---------------------------------------------------------------
						readFromStream
 ---------------------------------------------------------------*/
void  CSimpleDatabaseTable::readFromStream(mrpt::utils::CStream &in, int version)
{
	switch (version)
	{
	case 0:
	{
		uint32_t	row,col,nRec,nFie;
		//char		str[10000];

		in >> nRec >> nFie;

		data.resize(nRec);
		field_names.resize(nFie);

		for (col=0;col<nFie;col++)
			in >> field_names[col];

		for (row=0;row<nRec;row++)
		{
			data[row].resize(nFie);

			for (col=0;col<nFie;col++)
				in >> data[row][col];
		}
	}
	break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};
}

/*---------------------------------------------------------------
						Constructor
 ---------------------------------------------------------------*/
CSimpleDatabase::CSimpleDatabase( )
{

}

/*---------------------------------------------------------------
						Destructor
 ---------------------------------------------------------------*/
CSimpleDatabase::~CSimpleDatabase( )
{
	clear();
}

/*---------------------------------------------------------------
						Clear the DB
 ---------------------------------------------------------------*/
void  CSimpleDatabase::clear()
{
	m_tables.clear();
}

/*---------------------------------------------------------------
						getTable
 ---------------------------------------------------------------*/
CSimpleDatabaseTablePtr  CSimpleDatabase::getTable(const std::string &tableName)
{
	MRPT_START

	iterator it = m_tables.find(tableName);
	if (it!=m_tables.end())
		return it->second;

	THROW_EXCEPTION_CUSTOM_MSG1("Table '%s' was not found",tableName.c_str())

	MRPT_END
}

/*---------------------------------------------------------------
						getTable
 ---------------------------------------------------------------*/
CSimpleDatabaseTablePtr  CSimpleDatabase::getTable(size_t tableIndex)
{
	MRPT_START

	ASSERT_(tableIndex<tablesCount() )

	iterator it = m_tables.begin();
	std::advance(it,tableIndex);
	return it->second;

	MRPT_END
}

/*---------------------------------------------------------------
					tablesCount
 ---------------------------------------------------------------*/
size_t CSimpleDatabase::tablesCount() const
{
	return m_tables.size();
}

/*---------------------------------------------------------------
					tablesName
 ---------------------------------------------------------------*/
string	 CSimpleDatabase::tablesName(size_t tableIndex) const
{
	MRPT_START

	ASSERT_( tableIndex<tablesCount() )
	const_iterator it = m_tables.begin();
	std::advance(it,tableIndex);
	return it->first;

	MRPT_END
}

/*---------------------------------------------------------------
						createTable
 ---------------------------------------------------------------*/
CSimpleDatabaseTablePtr  CSimpleDatabase::createTable(const string &name)
{
	CSimpleDatabaseTablePtr table = CSimpleDatabaseTable::Create();
	m_tables[name] = table;
	return table;
}

/*---------------------------------------------------------------
						Constructor
 ---------------------------------------------------------------*/
CSimpleDatabaseTable::CSimpleDatabaseTable( )
{
}

/*---------------------------------------------------------------
						Destructor
 ---------------------------------------------------------------*/
CSimpleDatabaseTable::~CSimpleDatabaseTable( )
{
}

/*---------------------------------------------------------------
					fieldsCount
 ---------------------------------------------------------------*/
size_t  CSimpleDatabaseTable::fieldsCount() const
{
	return field_names.size();
}

/*---------------------------------------------------------------
						addField
 ---------------------------------------------------------------*/
void  CSimpleDatabaseTable::addField(const char *fieldName)
{
	field_names.push_back(string(fieldName));
	data.clear();
}

/*---------------------------------------------------------------
						getFieldName
 ---------------------------------------------------------------*/
string  CSimpleDatabaseTable::getFieldName(size_t fieldIndex) const
{
	MRPT_START

	ASSERT_( fieldIndex<fieldsCount() );
	return field_names[fieldIndex];

	MRPT_END
}

/*---------------------------------------------------------------
					fieldIndex
 ---------------------------------------------------------------*/
size_t CSimpleDatabaseTable::fieldIndex(const char *fieldName) const
{
	MRPT_START

	size_t		i,n = field_names.size();

	for (i=0;i<n;i++)
		if (!os::_strcmpi(fieldName,field_names[i].c_str()))
			return (int)i;

	THROW_EXCEPTION_CUSTOM_MSG1("fieldIndex: Field '%s' not found",fieldName);

	MRPT_END
}

/*---------------------------------------------------------------
						getRecordCount
 ---------------------------------------------------------------*/
size_t CSimpleDatabaseTable::getRecordCount() const
{
	return data.size();
}

/*---------------------------------------------------------------
						get
 ---------------------------------------------------------------*/
string  CSimpleDatabaseTable::get(
    size_t      recordIndex,
    string		field ) const
{
	MRPT_START
	ASSERT_(recordIndex<getRecordCount());
	return data[recordIndex][fieldIndex(field.c_str())];
	MRPT_END
}

/*---------------------------------------------------------------
						get
 ---------------------------------------------------------------*/
string  CSimpleDatabaseTable::get(
    size_t			recordIndex,
    size_t          fieldIndex ) const
{
	MRPT_START
	ASSERT_(recordIndex<getRecordCount());
	ASSERT_(fieldIndex<fieldsCount() );
	return data[recordIndex][fieldIndex];
	MRPT_END
}

/*---------------------------------------------------------------
						set
 ---------------------------------------------------------------*/
void  CSimpleDatabaseTable::set(
    size_t      recordIndex,
    string		field,
    string		value)
{
	MRPT_START

	ASSERT_(recordIndex<getRecordCount());
	data[recordIndex][fieldIndex(field.c_str())]=value;

	MRPT_END
}

/*---------------------------------------------------------------
						set
 ---------------------------------------------------------------*/
void  CSimpleDatabaseTable::set(
    size_t			recordIndex,
    size_t			fieldIndex,
    string		value)
{
	MRPT_START

	ASSERT_(recordIndex<getRecordCount());
	ASSERT_(fieldIndex<fieldsCount() );
	data[recordIndex][fieldIndex]=value;

	MRPT_END
}

/*---------------------------------------------------------------
						query
 ---------------------------------------------------------------*/
int  CSimpleDatabaseTable::query(
    string		field,
    string		value ) const
{
	int		fieldInd,i,n = (uint32_t) getRecordCount();

	try
	{
		fieldInd = (uint32_t) fieldIndex(field.c_str());
	}
	catch (...)
	{
		return -1;
	}

	for (i=0;i<n;i++)
	{
		if (!os::_strcmpi(value.c_str(),data[i][fieldInd].c_str()))
			return i;
	}

	// Do not found:
	return -1;
}

/*---------------------------------------------------------------
						appendRecord
 ---------------------------------------------------------------*/
size_t CSimpleDatabaseTable::appendRecord()
{
	vector_string	new_rec;

	new_rec.resize( fieldsCount() );
	data.push_back( new_rec );

	return data.size()-1;
}

/*---------------------------------------------------------------
						deleteRecord
 ---------------------------------------------------------------*/
void CSimpleDatabaseTable::deleteRecord(size_t recordIndex)
{
	MRPT_START
	ASSERT_(recordIndex<getRecordCount())

	std::vector<vector_string>::iterator it = data.begin();
	std::advance(it,recordIndex);
	data.erase(it);

	MRPT_END
}


/*---------------------------------------------------------------
						saveAsXML
 ---------------------------------------------------------------*/
bool CSimpleDatabase::saveAsXML( const string &fileName ) const
{
	try
	{
		// Root node:
		XMLNode  rootXml = XMLNode::createXMLTopNode("CSimpleDatabase-MRPT-Object");

		// For each table:
		for (const_iterator it=m_tables.begin();it!=m_tables.end();++it)
		{
			CSimpleDatabaseTablePtr t = it->second;
			XMLNode tabNod = rootXml.addChild("table");
			tabNod.addAttribute( "name", it->first.c_str() );

			// Add field descriptions:
			// ------------------------
			size_t  nFields = t->fieldsCount();
			size_t  nRecs   = t->getRecordCount();

			XMLNode fNod = tabNod.addChild("fields");
			for (unsigned int i=0;i<nFields;i++)
				fNod.addChild( t->getFieldName(i).c_str() );

			// Add record contents:
			// ------------------------
			for (unsigned int i=0;i<nRecs;i++)
			{
				XMLNode recNod = tabNod.addChild("record");
				for (size_t j=0;j<nFields;j++)
				{
					XMLNode recContent = recNod.addChild( t->getFieldName(j).c_str() );
					recContent.addText( t->get(i,j).c_str() );
				}
			}

		} // end for each table.

		rootXml.writeToFile( fileName.c_str() );

		return true; // Ok
	}
	catch (exception &e)
	{
		cerr << "[CSimpleDatabase::saveAsXML] Exception ignored:" << endl << e.what() << endl;
		return false;   // Errors found
	}
	catch (...)
	{
		return false;   // Errors found
	}
}


/*---------------------------------------------------------------
						loadFromXML
 ---------------------------------------------------------------*/
bool CSimpleDatabase::loadFromXML( const string &fileName )
{
	try
	{
		XMLResults 	results;
		XMLNode 	root = XMLNode::parseFile( fileName.c_str(), NULL, &results );

		if (results.error != eXMLErrorNone)
		{
			cerr << "[CSimpleDatabase::loadFromXML] Error loading XML file: " <<
					XMLNode::getError( results.error ) << " at line " << results.nLine << ":" << results.nColumn << endl;
			return false;
		}

		root = root.getChildNode("CSimpleDatabase-MRPT-Object");
		if (root.isEmpty())
		{
			cerr << "[CSimpleDatabase::loadFromXML] Loaded XML file does not have a 'CSimpleDatabase-MRPT-Object' tag";
			return false;
		}

		// Clear previous contents:
		clear();

		// Get tables:
		size_t i,j, nTables = root.nChildNode("table");
		for (i=0;i<nTables;i++)
		{
			XMLNode tabNod = root.getChildNode("table",(int)i);
			ASSERT_(!tabNod.isEmpty())

			// Create table:
			CSimpleDatabaseTablePtr t = createTable( tabNod.getAttribute("name") );

			// Create fields:
			XMLNode fNod = tabNod.getChildNode("fields");
			ASSERT_(!fNod.isEmpty())

			size_t  nFields = fNod.nChildNode();
			for (j=0;j<nFields;j++)
			{
				t->addField( fNod.getChildNode((int)j).getName() );
			} // end for each field

			// Add record data:
			size_t nRecs = tabNod.nChildNode("record");
			for (size_t k=0;k<nRecs;k++)
			{
				size_t recIdx = t->appendRecord();

				XMLNode recNod = tabNod.getChildNode("record",(int)k);
				ASSERT_(!recNod.isEmpty())

				for (j=0;j<nFields;j++)
				{
					XMLCSTR  str=recNod.getChildNode(t->getFieldName(j).c_str() ).getText();
					t->set(recIdx,j, str!=NULL ?  string(str) : string() );
				}

			} // end for each record

		} // for each table

		return true; // Ok
	}
	catch (exception &e)
	{
		cerr << "[CSimpleDatabase::loadFromXML] Exception ignored:" << endl << e.what() << endl;
		return false;   // Errors found
	}
	catch (...)
	{
		return false;   // Errors found
	}
}

/*---------------------------------------------------------------
						dropTable
 ---------------------------------------------------------------*/
void CSimpleDatabase::dropTable(const std::string &tableName)
{
	MRPT_START

	iterator it = m_tables.find(tableName);
	if (it==m_tables.end())
		THROW_EXCEPTION_CUSTOM_MSG1("Table '%s' was not found",tableName.c_str())

	m_tables.erase(it);


	MRPT_END
}

/*---------------------------------------------------------------
						renameTable
 ---------------------------------------------------------------*/
void CSimpleDatabase::renameTable(
	const std::string &tableName,
	const std::string &newTableName )
{
	MRPT_START

	if (tableName==newTableName) return; // done

	iterator it = m_tables.find(tableName);
	if (it==m_tables.end())
		THROW_EXCEPTION_CUSTOM_MSG1("Table '%s' was not found",tableName.c_str())

	{
		iterator itNew = m_tables.find(newTableName);
		if (itNew !=m_tables.end())
			THROW_EXCEPTION_CUSTOM_MSG1("A table with the name '%s' already exists",newTableName.c_str())
	}

	CSimpleDatabaseTablePtr tb = it->second;

	m_tables.erase(it);
	m_tables[newTableName] = tb;


	MRPT_END
}


