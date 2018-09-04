/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once
#include "CAboutBoxBase.h"

#include <QDialog>

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
