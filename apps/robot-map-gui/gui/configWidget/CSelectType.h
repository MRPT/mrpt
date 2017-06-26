/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#pragma once
#include <QDialog>

#include <memory>


class QListWidget;
class QListWidgetItem;
namespace Ui
{
class CSelectType;
}

class CSelectType: public QDialog
{
public:
	enum TypeOfMaps
	{
		None = -1,
		PointsMap = 0,
		Occupancy = 1,
		Landmarks = 2,
		Beacon = 3,
		GasGrid = 4
	};
	CSelectType(QWidget *parent = nullptr);
	virtual ~CSelectType();
	TypeOfMaps selectedItem() const;

private:
	void addItem(const QString &name, TypeOfMaps type);

	std::unique_ptr<Ui::CSelectType> m_ui;
};
