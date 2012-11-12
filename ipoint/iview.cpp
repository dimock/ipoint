#include "iview.h"
#include <QMessageBox>
#include <QPainter>
#include <vector>


using namespace std;


ViewWindow::ViewWindow(QWidget * parent) :
  QWidget(parent), distToPt_(6), radiusPt_(4)
{
  screen_.size.z = 1;
  setMouseTracking(true);
  connect(this, SIGNAL(mouseMoved(const QPoint & )), this, SLOT(onPosChanged(const QPoint & )));
  setAttribute(Qt::WA_DeleteOnClose, true);
}

ViewWindow::~ViewWindow()
{
}

void ViewWindow::reset()
{
  alg_.reset();
  repaint();
}

void ViewWindow::mouseMoveEvent(QMouseEvent * event)
{
  emit mouseMoved(event->pos());
}

void ViewWindow::resizeEvent(QResizeEvent * event)
{
  QSize sz = event->size();

  screen_.size.x = sz.width();
  screen_.size.y = sz.height();

  recalcScreen();
}

void ViewWindow::recalcScreen()
{
  if ( screen_.size.x > 0 && screen_.size.y > 0 )
  {
    Vec3f s(1, 1, 1);
    float r = screen_.size.x/screen_.size.y;
    if ( r > 1 )
      s.x = r;
    else
      s.y = 1/r;
    //if ( alg_.getRect().isValid() )
    //  screen_.rect = alg_.getRect();
    //else
      screen_.rect.set(Vec3f(0,0,0), Vec3f(1,1,1));
    screen_.rect.scale(s);
  }
}

void ViewWindow::mouseReleaseEvent(QMouseEvent * event)
{
  const QPoint & pt = event->pos();
  Vec3f pts(pt.x(), pt.y(), 0);
  Vec3f ptw(screen_.toWorld(pts));
  alg_.addPoint(ptw, isOverFirstPt(pts));
  repaint();
}

bool ViewWindow::isOverFirstPt(const Vec3f & p)
{
  if ( alg_.pointsCount() < 1 )
    return false;

  Vec3f pt = screen_.toScreen(alg_[0]);
  if ( (pt-p).length() < distToPt_ )
    return true;

  return false;
}

void ViewWindow::paintEvent(QPaintEvent * event)
{
  draw();
}

void ViewWindow::onPosChanged(const QPoint & pt)
{
  if ( curPt_ == pt )
    return;
  Vec3f pts(pt.x(), pt.y(), 0);
  Vec3f ptw(screen_.toWorld(pts));
  alg_.setCursorPt(ptw);
  repaint();
}

void ViewWindow::draw()
{
  QPainter painter(this);
  
  QRect rc(0, 0, size().width(), size().height());
  painter.fillRect(rc, Qt::white);

  if ( alg_.pointsCount() == 0 )
	  return;

  painter.setPen( QPen(Qt::blue, 2, Qt::SolidLine, Qt::RoundCap) );

  // draw polyline first
  vector<QPoint> qpoints(alg_.pointsCount());
  for (size_t i = 0; i < alg_.pointsCount(); ++i)
  {
    Vec3f pt = screen_.toScreen(alg_[i]);
    qpoints[i] = QPoint(pt.x, pt.y);
  }
  
  // add 1st point to close contour
  if ( alg_.isClosed() )
    qpoints.push_back(qpoints[0]);

  painter.drawPolyline(&qpoints[0], qpoints.size());

  // remove duplicated point if added
  if ( alg_.isClosed() )
  {
    vector<QPoint>::iterator iter = qpoints.begin();
    advance(iter, qpoints.size()-1);
    qpoints.erase(iter);
  }

  Vec3f cursorPt = screen_.toScreen(alg_.getCursorPt());
  bool overFirst = isOverFirstPt(cursorPt);
  Vec3f isectPt;
  bool selfIsect = alg_.haveSelfIsect(alg_.getCursorPt(), isectPt);

  // draw triangulation
  vector<Triangle> * tris = 0;
  if ( alg_.triangulation(tris) && tris )
  {
    painter.setPen( QPen(Qt::black, 1, Qt::SolidLine, Qt::RoundCap) );
    painter.setBrush( QBrush(Qt::green, Qt::SolidPattern) );

    for (size_t i = 0; i < tris->size(); ++i)
    {
      Triangle & tri = tris->at(i);
      vector<QPoint> qpts;
      for (int j = 0; j < 3; ++j)
      {
        Vec3f p = screen_.toScreen( alg_[tri.v[j]] );
        qpts.push_back( QPoint(p.x, p.y) );
      }
      qpts.push_back(qpts[0]);
      painter.drawPolygon(&qpts[0], qpts.size());
    }
  }

  // draw dotted line from last point to cursor if contour isn't closed
  if ( !alg_.isClosed() )
  {
    Vec3f lastPt = screen_.toScreen(alg_[alg_.pointsCount()-1]);

    QPoint p0(lastPt.x, lastPt.y);
    QPoint p1(cursorPt.x, cursorPt.y);

    Qt::GlobalColor color = Qt::black;
    int lineWidth = 1;
    if ( selfIsect )
    {
      color = Qt::red;
    }
    else if ( overFirst )
    {
      color = Qt::blue;
      lineWidth = 2;
    }
    painter.setPen( QPen(color, lineWidth, Qt::DashDotLine, Qt::RoundCap) );
    painter.drawLine(p0, p1);
  }

  // draw points
  for (size_t i = 0; i < qpoints.size(); ++i)
  {
    QPoint & qpt = qpoints[i];
    Qt::BrushStyle style = Qt::SolidPattern;
    Qt::GlobalColor color = Qt::black;
    if ( i == 0 && overFirst )
      style = Qt::NoBrush;
    painter.setPen( QPen(color, 1, Qt::SolidLine, Qt::RoundCap) );
    painter.setBrush( QBrush(color, style) );
    painter.drawEllipse(qpt, (int)radiusPt_, (int)radiusPt_);
  }

  // draw self-intersection point if found
  if ( selfIsect && !alg_.isClosed() )
  {
    Vec3f pt = screen_.toScreen(isectPt);
    QPoint qpt(pt.x, pt.y);
    painter.setPen( QPen(Qt::red, 1, Qt::SolidLine, Qt::RoundCap) );
    painter.setBrush( QBrush(Qt::red, Qt::SolidPattern) );
    painter.drawEllipse(qpt, (int)radiusPt_, (int)radiusPt_);
  }

}