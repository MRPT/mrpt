#pragma once
#include <QMainWindow>


class GlWidget;

class CMainWindow : public QMainWindow {
	Q_OBJECT

public:
	CMainWindow(QWidget *parent = 0);
	virtual ~CMainWindow();

private slots:
	void openMap();

private:
	void initMenu();

	GlWidget *glWidget_;
};
