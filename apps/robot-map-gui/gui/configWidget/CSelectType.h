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

#include "TypeOfConfig.h"


class QListWidget;
class QListWidgetItem;
namespace Ui
{
class CSelectType;
}

class CSelectType: public QDialog
{
public:
	CSelectType(QWidget *parent = nullptr);
	virtual ~CSelectType();
	int selectedItem() const;

private:
	void addItem(const QString &name, TypeOfConfig type);

	std::unique_ptr<Ui::CSelectType> m_ui;
};
