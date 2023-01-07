/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once
#include <QDialog>

#include "CAboutBoxBase.h"

class QTextEdit;
class QTabWidget;

class CAboutBoxQt : public QDialog, public CAboutBoxBase
{
   public:
	CAboutBoxQt(
		const std::string& appName, const std::string& additionalInfo,
		const bool showStandardInfo);
	~CAboutBoxQt() override = default;

   private:
	QTextEdit* widgetForTabs(const std::string& str, QTabWidget* parent) const;
};
