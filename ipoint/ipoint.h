#pragma once

#include <QMainWindow>
#include <QMenu>
#include <QToolBar>
#include <QAction>
#include <QLabel>
#include "iview.h"


class IntrusionPointWindow : public QMainWindow
{

  Q_OBJECT

public:
  IntrusionPointWindow(QWidget * parent = 0);
  ~IntrusionPointWindow();

private slots:

  void onNew();
  void updateMousePt(const QPoint & );
  void updateTrianglesCount(size_t);

protected:

  void closeEvent(QCloseEvent *);
  void createMenu();
  void createToolBar();
  void createStatusBar();


  ViewWindow * view_;
  QMenu * fileMenu_;
  QAction * onNewAction_;
  QToolBar * fileToolbar_;
  QLabel * statusLabelMouse_, * statusLabelTris_;
  QLabel * mousePosLabel_;
  QLabel * trianglesLabel_;
};
