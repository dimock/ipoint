#include "ipoint_alg.h"
#include "imath.h"
#include "delaunay.h"
#include <QTextStream>
#include <QFile>
#include <QStringList>

using namespace std;
using namespace iMath;

IntrusionPointAlgorithm::IntrusionPointAlgorithm() :
  closed_(false), pointCount_(0)
{
}

void IntrusionPointAlgorithm::load(const QString & fname)
{
  QFile qf(fname);
  if ( !qf.open(QIODevice::ReadOnly) )
    return;

  reset();

  QTextStream qts(&qf);
  for ( ; !qts.atEnd(); )
  {
    QString sline = qts.readLine();
    if ( sline.isEmpty() )
      continue;
    if ( sline[0] == '{' )
      continue;
    if ( sline[0] == '}' )
      break;

    QStringList slist = sline.split( QRegExp("[{},;\\s]+"), QString::SkipEmptyParts);
    if ( slist.size() < 2 )
      break;

    Vec3f p;
    p.x = slist[0].toDouble();
    p.y = slist[1].toDouble();

    points_.push_back(p);
  }

  pointCount_ = points_.size();
  closed_ = true;

  triangulate();
}

void IntrusionPointAlgorithm::save(const QString & fname) const
{
  QFile qf(fname);
  if ( !qf.open(QIODevice::WriteOnly) )
    return;

  QTextStream qts(&qf);

  size_t n = std::min(pointCount_, points_.size());

  qts << tr("{\n");
  for (size_t i = 0; i < n; ++i)
  {
    const Vec3f & p = points_.at(i);
    QString str;
    str.sprintf("  {%g, %g}\n", p.x, p.y);
    qts << str;
  }
  qts << tr("}\n");
}

void IntrusionPointAlgorithm::reset()
{
  closed_ = false;
  pointCount_ = 0;
  points_.clear();
  rect_.makeInvalid();
  tris_.clear();
}

void IntrusionPointAlgorithm::addPoint(const Vec3f & pt, bool close)
{
  if  ( closed_ )
    return;

  Vec3f r;
  if ( haveSelfIsect(pt, r) )
    return;

  if ( close )
  {
    closed_ = true;
    triangulate();
  }
  else
  {
      points_.push_back(pt);
      pointCount_ = points_.size();
  }
  recalc();
}

bool IntrusionPointAlgorithm::isClosed() const
{
  return closed_;
}

bool IntrusionPointAlgorithm::haveSelfIsect(const Vec3f & q1, Vec3f & r) const
{
  if ( points_.size() < 2 )
    return false;
  double p0dist = (q1-points_.front()).length();
  bool to1stpt = false;
  if ( p0dist < 1e-2 )
    to1stpt = true;
  const Vec3f & q0 = points_.back();
  for (size_t i = 0; i < points_.size()-2; ++i)
  {
    const Vec3f & p0 = points_[i];
    const Vec3f & p1 = points_[i+1];
    double dist;
    if ( !edges_isect(p0, p1, q0, q1, r, dist) )
      continue;
    if ( i == 0 && to1stpt )
      continue;
    return true;
  }
  const Vec3f & p0 = points_[points_.size()-2];
  const Vec3f & p1 = points_[points_.size()-1];
  Vec3f rp = p1-p0; rp.norm();
  Vec3f rq = q1-q0; rq.norm();
  double v = rp*rq;
  if ( v < -0.9999 )
  {
    r = q1;
    return true;
  }
  return false;
}

void IntrusionPointAlgorithm::removePoint(size_t idx)
{
  if ( idx < pointsCount() )
  {
    Points3f::iterator i = points_.begin();
    advance(i, idx);
    points_.erase(i);
  }
  recalc();
  if ( points_.size() == 0 )
    reset();
}

void IntrusionPointAlgorithm::insertPoint(size_t idx, const Vec3f & pt)
{
  if ( idx >= points_.size() || !closed_ )
    return;

  Points3f::iterator iter = points_.begin();
  advance(iter, idx);
  points_.insert(iter, pt);
  recalc();
}

size_t IntrusionPointAlgorithm::pointsCount() const
{
  return pointCount_;
}

Vec3f & IntrusionPointAlgorithm::operator [] (size_t idx)
{
  return points_.at(idx);
}

void IntrusionPointAlgorithm::setCursorPt(const Vec3f & pt)
{
  cursorPt_ = pt;
}

const Vec3f & IntrusionPointAlgorithm::getCursorPt() const
{
  return cursorPt_;
}

const Rect3f & IntrusionPointAlgorithm::getRect() const
{
  return rect_;
}

void IntrusionPointAlgorithm::recalc()
{
  rect_.makeInvalid();
  for (size_t i = 0; i < points_.size(); ++i)
    rect_.add(points_[i]);
  if ( !closed_ || points_.size() == 0 )
    return;
}

bool IntrusionPointAlgorithm::triangulate()
{
  if ( tris_.size() > 0 )
    return true;

  DelanayTriangulator dtr(points_);

  try
  {
    dtr.triangulate(tris_);
    recalc();
    emit trianglesChanged(tris_.size());
    return true;
  }
  catch ( std::runtime_error &  )
  {
    return false;
  }
}

bool IntrusionPointAlgorithm::triangulation(std::vector<Triangle> *& tris)
{
  tris = &tris_;
  return true;
}