#pragma once

#include <QWidget>
#include <QPoint>
#include <QMouseEvent>
#include <QResizeEvent>
#include <QPaintEvent>
#include "ipoint_alg.h"

class ViewWindow : public QWidget
{
  Q_OBJECT

public:

  ViewWindow(QWidget * parent);
  ~ViewWindow();

  void reset();

signals:

  void mouseMoved(const QPoint & pos);
  void trianglesChanged(size_t);

protected:

  // events
  void mouseMoveEvent(QMouseEvent * );
  void mouseReleaseEvent(QMouseEvent * );
  void resizeEvent(QResizeEvent *);
  void paintEvent(QPaintEvent *);

private slots:

  void onPosChanged(const QPoint &);
  void onTrianglesChanged(size_t);

private:
  void draw();
  void recalcScreen();
  bool isOverFirstPt(const Vec3f & p);

  IntrusionPointAlgorithm alg_;
  Screen screen_;
  QPoint curPt_;

  const double distToPt_;
  const int radiusPt_;
};