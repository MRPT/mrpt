/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/serialization/CSerializable.h>
#include <map>

namespace mrpt::db
{
/**  This class implements the tables of databases.
 * \sa CSimpleDatabase \ingroup mrpt_base_grp
 */
class CSimpleDatabaseTable : public mrpt::serialization::CSerializable
{
	DEFINE_SERIALIZABLE(CSimpleDatabaseTable)
   public:
	/** Default constructor
	 */
	CSimpleDatabaseTable();

	/** Destructor
	 */
	~CSimpleDatabaseTable() override;

	/** Get the count of fields.
	 */
	size_t fieldsCount() const;

	/** Append a new and empty record at the end of the table, and return the
	 * index of the newly added record.
	 * \sa deleteRecord
	 */
	size_t appendRecord();

	/** Add a new field to the table. The table is cleared in this operation. */
	void addField(const char* fieldName);

	/** Add a new field to the table. The table is cleared in this operation. */
	void addField(const std::string& fieldName) { addField(fieldName.c_str()); }
	/** Get the name of a field by its index
	 * \exception std::exception On index out of bounds
	 */
	std::string getFieldName(size_t fieldIndex) const;

	/** Get the index for a given field name
	 * \exception std::exception On field not found
	 */
	size_t fieldIndex(const char* fieldName) const;

	/** Get the index for a given field name
	 * \exception std::exception On field not found
	 */
	size_t fieldIndex(const std::string& fieldName) const
	{
		return fieldIndex(fieldName.c_str());
	}

	/** Get the records count in the table
	 */
	size_t getRecordCount() const;

	/**  Returns the cell content of the record indicates by its index, and the
	 * field indicated in "field".
	 * \exception std::exception On field or record not found
	 */
	std::string get(size_t recordIndex, std::string field) const;

	/**  Returns the cell content of the record indicates by its index, and the
	 * field indicated by its index.
	 * \exception std::exception On field or record not found
	 */
	std::string get(size_t recordIndex, size_t fieldIndex) const;

	/**  Sets the cell content of the record indicates by its index, and the
	 * field indicated in "field".
	 * \exception std::exception On field or record not found
	 */
	void set(size_t recordIndex, std::string field, std::string value);

	/**  Sets the cell content of the record indicates by its index, and the
	 * field indicated by its index.
	 * \exception std::exception On field or record not found
	 */
	void set(size_t recordIndex, size_t fieldIndex, std::string value);

	/**  Executes a query in the table, returning the record index which a given
	 * field has a given value, case insensitive, or -1 if not found.
	 */
	int query(std::string field, std::string value) const;

	/** Delete the record at the given index \sa appendRecord */
	void deleteRecord(size_t recordIndex);

   private:
	/** Field names */
	std::vector<std::string> field_names;
	/** Data for each cell */
	std::vector<std::vector<std::string>> data;

};  // end of class definition

/**  This class impements a very simple database system. A database is
 *   a collection of tables, each one being a CSimpleDatabaseTable object.
 * Tables are
 *   a rectangular arrrangement of cells, organized as records of fields.
 *  There are XML export/import methods in saveAsXML, loadFromXML.
 *
 * \note This class is NOT safe for read/write access from different threads.
 * If needed, use critical sections.
 *
 * \sa CSimpleDatabaseTable
 */
class CSimpleDatabase : public mrpt::serialization::CSerializable
{
	DEFINE_SERIALIZABLE(CSimpleDatabase)

   public:
	/** Default constructor
	 */
	CSimpleDatabase();

	/** Destructor
	 */
	~CSimpleDatabase() override;

	/** Clears the DB.
	 */
	void clear();

	/** Creates a new table in the DB, initially empty.
	 */
	CSimpleDatabaseTable::Ptr createTable(const std::string& name);

	/** Returns the table with the indicated name
	 * \exception std::exception On table not found.
	 */
	CSimpleDatabaseTable::Ptr getTable(const std::string& tableName);

	/** Deletes the given table.
	 * \exception std::exception On table not found.
	 */
	void dropTable(const std::string& tableName);

	/** Changes the name of a given table
	 * \exception std::exception On table not found or new name already
	 * existed.
	 */
	void renameTable(
		const std::string& tableName, const std::string& newTableName);

	/** Returns the table by index.
	 * \exception std::exception On index out of bounds
	 */
	CSimpleDatabaseTable::Ptr getTable(size_t tableIndex);

	/** Returns the tables count in the DB.
	 */
	size_t tablesCount() const;

	/** Returns the tables names in the DB.
	 * \exception std::exception On index out of bounds
	 */
	std::string tablesName(size_t tableIndex) const;

	/** Saves this database as a XML file.
	 * \return false on any error, true if successful.
	 * \sa loadFromXML
	 */
	bool saveAsXML(const std::string& fileName) const;

	/** Loads the content of this database from a a XML file.
	 * \return false on any error, true if successful.
	 * \sa saveAsXML
	 */
	bool loadFromXML(const std::string& fileName);

   private:
	/** The tables of the DB indexed by their names: */
	using TTableList = std::map<std::string, CSimpleDatabaseTable::Ptr>;
	using iterator = std::map<std::string, CSimpleDatabaseTable::Ptr>::iterator;
	using const_iterator =
		std::map<std::string, CSimpleDatabaseTable::Ptr>::const_iterator;

	TTableList m_tables;

};  // end of class definition
}  // namespace mrpt::db
