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

    if ( sline.isEmpty() || sline[0] == '{' )
      continue;

    if ( sline[0] == '}' )
      break;

    QStringList slist = sline.split( QRegExp("[{},;\\s]+"), QString::SkipEmptyParts );
    if ( slist.size() < 2 )
      break;

    Vec3f p, n(0,0,1);
    p.x = slist[0].toDouble();
    p.y = slist[1].toDouble();

    if ( slist.size() > 2 )
      p.z = slist[2].toDouble();

    if ( slist.size() > 5 )
    {
      n.x = slist[3].toDouble();
      n.y = slist[4].toDouble();
      n.z = slist[5].toDouble();
    }

    verts_.push_back( Vertex(p, n) );
  }

  pointCount_ = verts_.size();
  closed_ = true;

  triangulate();
}

void IntrusionPointAlgorithm::save(const QString & fname) const
{
  QFile qf(fname);
  if ( !qf.open(QIODevice::WriteOnly) )
    return;

  QTextStream qts(&qf);

  size_t n = std::min(pointCount_, verts_.size());

  qts << tr("{\n");
  for (size_t i = 0; i < n; ++i)
  {
    const Vec3f & p = verts_.at(i).p();
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
  verts_.clear();
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
      verts_.push_back( Vertex(pt, Vec3f(0,0,1)) );
      pointCount_ = verts_.size();
  }
  recalc();
}

bool IntrusionPointAlgorithm::isClosed() const
{
  return closed_;
}

bool IntrusionPointAlgorithm::haveSelfIsect(const Vec3f & q1, Vec3f & r) const
{
  if ( verts_.size() < 2 )
    return false;
  double p0dist = (q1-verts_.front().p()).length();
  bool to1stpt = false;
  if ( p0dist < 1e-2 )
    to1stpt = true;
  const Vec3f & q0 = verts_.back().p();
  for (size_t i = 0; i < verts_.size()-2; ++i)
  {
    const Vec3f & p0 = verts_.at(i).p();
    const Vec3f & p1 = verts_.at(i+1).p();
    double dist;
    if ( !edges_isect(p0, p1, q0, q1, r, dist) )
      continue;
    if ( i == 0 && to1stpt )
      continue;
    return true;
  }
  const Vec3f & p0 = verts_.at(verts_.size()-2).p();
  const Vec3f & p1 = verts_.at(verts_.size()-1).p();
  Vec3f rp = p1-p0; rp.normalize();
  Vec3f rq = q1-q0; rq.normalize();
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
    Vertices::iterator i = verts_.begin();
    advance(i, idx);
    verts_.erase(i);
  }
  recalc();
  if ( verts_.size() == 0 )
    reset();
}

void IntrusionPointAlgorithm::insertPoint(size_t idx, const Vec3f & pt)
{
  if ( idx >= verts_.size() || !closed_ )
    return;

  Vertices::iterator iter = verts_.begin();
  advance(iter, idx);
  verts_.insert(iter, Vertex(pt, Vec3f(0,0,1)));
  recalc();
}

size_t IntrusionPointAlgorithm::pointsCount() const
{
  return pointCount_;
}

const Vec3f & IntrusionPointAlgorithm::operator [] (size_t idx) const
{
  return verts_.at(idx).p();
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
  for (size_t i = 0; i < verts_.size(); ++i)
    rect_.add(verts_.at(i).p());
  if ( !closed_ || verts_.size() == 0 )
    return;
}

bool IntrusionPointAlgorithm::triangulate()
{
  if ( tris_.size() > 0 )
    return true;

  DelaunayTriangulator dtr(verts_);

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