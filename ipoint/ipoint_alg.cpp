#include "ipoint_alg.h"
#include "imath.h"
#include "delaunay.h"

using namespace std;


class Triangulator
{
  Points2f & points_;

public:

  Triangulator(Points2f & points) :
      points_(points)
  {
    cw();
  }

  bool triangulate(vector<Triangle> & tris);

private:

  bool cw_;

  void cw();
  bool triangulate(vector<int> & indices, vector<Triangle> & tris);
  int  find_convex_vertex(vector<int> & indices);
  int  find_intrude_vertex(int j, vector<int> & indices);
};

void Triangulator::cw()
{
  cw_ = ::cw(points_);
  //if ( points_.size() < 3 )
  //  return;
  //double s = 0;
  //Vec3f p0(points_[0].x, points_[0].y, 0);
  //for (size_t i = 1; i < points_.size(); ++i)
  //{
  //  size_t j = i+1;
  //  if ( j >= points_.size() )
  //    break;
  //  Vec3f p1(points_[i].x, points_[i].y, 0);
  //  Vec3f p2(points_[j].x, points_[j].y, 0);
  //  Vec3f v = (p1 - p0) ^ (p2 - p0);
  //  s += v.z;
  //}
  //cw_ = s < 0;
}

bool Triangulator::triangulate(vector<Triangle> & tris)
{
  if ( points_.size() < 3 )
    return false;

  vector<int> indices(points_.size());
  for (size_t i = 0; i < points_.size(); ++i)
    indices[i] = i;

  return triangulate(indices, tris);
}

int Triangulator::find_convex_vertex(vector<int> & indices)
{
  if ( indices.size() < 3 )
    return -1;

  for (int j = 0; j < (int)indices.size(); ++j)
  {
    int i = j-1;
    if ( i < 0 )
      i = indices.size()-1;
    int k = j+1;
    if ( k >= indices.size() )
      break;

    const Vec2f & p0 = points_[indices[i]];
    const Vec2f & p1 = points_[indices[j]];
    const Vec2f & p2 = points_[indices[k]];

    Vec3f pp0(p0.x, p0.y, 0);
    Vec3f pp1(p1.x, p1.y, 0);
    Vec3f pp2(p2.x, p2.y, 0);

    Vec3f v = (pp1 - pp0) ^ (pp2 - pp0);
    if ( v.z < 0 == cw_ )
    {
      return j;
    }
  }
  return -1;
}

int Triangulator::find_intrude_vertex(int j, vector<int> & indices)
{
  if ( indices.size() < 3 || (size_t)j >= indices.size() )
    return -1;

  int i = j-1;
  if ( i < 0 )
    i = indices.size()-1;
  int k = j+1;
  if ( k >= indices.size() )
    k = 0;

  const Vec2f & p0 = points_[indices[i]];
  const Vec2f & p1 = points_[indices[j]];
  const Vec2f & p2 = points_[indices[k]];


  bool outside = false;
  double dist_p1 = dist_to_line(p0, p2, p1, outside);
  double dist = 0;
  int m = -1;
  int n = k+1;
  if ( n >= indices.size() )
    n = 0;
  for (; n != i; )
  {
    const Vec2f & q = points_[indices[n]];
    double d = dist_to_line(p0, p2, q, outside);
    bool inside = inside_tri(p0, p1, p2, q, cw_);
    if ( inside && d*dist_p1 > 0 )
    {
      if ( d < 0 )
        d = -d;
      if ( d > dist )
      {
        m = (int)n;
        dist = d;
      }
    }
    if ( ++n >= indices.size() )
      n = 0;
  }
  return m;
}

bool Triangulator::triangulate(vector<int> & indices, vector<Triangle> & tris)
{
  if ( indices.size() < 3 )
    return true;

  int j = find_convex_vertex(indices);
  if ( j < 0 )
    return false;

  int i = j-1;
  if ( i < 0 )
    i = indices.size()-1;
  int k = j+1;
  if ( k >= indices.size() )
    k = 0;

  int m = find_intrude_vertex(j, indices);
  if ( m < 0 )
  {

    tris.push_back( Triangle(indices[i], indices[j], indices[k]) );

    vector<int>::iterator iter = indices.begin();
    advance(iter, j);
    indices.erase(iter);

    return triangulate(indices, tris);
  }

  vector<int> indices1;
  indices1.push_back(indices[i]);
  indices1.push_back(indices[j]);
  for (size_t n = m; n != i; )
  {
    indices1.push_back(indices[n]);
    if ( ++n >= indices.size() )
      n = 0;
  }

  vector<int> indices2;
  indices2.push_back(indices[m]);
  indices2.push_back(indices[j]);
  for (size_t n = k; n != m; )
  {
    indices2.push_back(indices[n]);
    if ( ++n >= indices.size() )
      n = 0;
  }

  if ( !triangulate(indices1, tris) )
    return false;

  if ( !triangulate(indices2, tris) )
    return false;

  return true;
}

IntrusionPointAlgorithm::IntrusionPointAlgorithm() :
  closed_(false), pointCount_(0)
{
}

void IntrusionPointAlgorithm::reset()
{
  closed_ = false;
  pointCount_ = 0;
  points_.clear();
  rect_.makeInvalid();
  tris_.clear();
}

void IntrusionPointAlgorithm::addPoint(const Vec2f & pt, bool close)
{
  if  ( closed_ )
    return;
  Vec2f r;
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

bool IntrusionPointAlgorithm::haveSelfIsect(const Vec2f & q1, Vec2f & r) const
{
  if ( points_.size() < 2 )
    return false;
  double p0dist = (q1-points_.front()).vecmod();
  bool to1stpt = false;
  if ( p0dist < 1e-2 )
    to1stpt = true;
  const Vec2f & q0 = points_.back();
  for (size_t i = 0; i < points_.size()-2; ++i)
  {
    const Vec2f & p0 = points_[i];
    const Vec2f & p1 = points_[i+1];
    if ( !edges_isect(p0, p1, q0, q1, r) )
      continue;
    if ( i == 0 && to1stpt )
      continue;
    return true;
  }
  const Vec2f & p0 = points_[points_.size()-2];
  const Vec2f & p1 = points_[points_.size()-1];
  Vec2f rp = p1-p0; rp.norm();
  Vec2f rq = q1-q0; rq.norm();
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
    Points2f::iterator i = points_.begin();
    advance(i, idx);
    points_.erase(i);
  }
  recalc();
  if ( points_.size() == 0 )
    reset();
}

void IntrusionPointAlgorithm::insertPoint(size_t idx, const Vec2f & pt)
{
  if ( idx >= points_.size() || !closed_ )
    return;

  Points2f::iterator iter = points_.begin();
  advance(iter, idx);
  points_.insert(iter, pt);
  recalc();
}

size_t IntrusionPointAlgorithm::pointsCount() const
{
  return pointCount_;
}

Vec2f & IntrusionPointAlgorithm::operator [] (size_t idx)
{
  return points_.at(idx);
}

void IntrusionPointAlgorithm::setCursorPt(const Vec2f & pt)
{
  cursorPt_ = pt;
}

const Vec2f & IntrusionPointAlgorithm::getCursorPt() const
{
  return cursorPt_;
}

const Rect2f & IntrusionPointAlgorithm::getRect() const
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

  //Triangulator triangulator(points_);
  //return triangulator.triangulate(tris_);

  DelanayTriangulator dtr(points_);
  return dtr.triangulate(tris_);
}

bool IntrusionPointAlgorithm::triangulation(std::vector<Triangle> *& tris)
{
  //if ( !closed_ )
  //{
  //  tris_.clear();
  //  if ( !triangulate() )
  //    return false;
  //}

  tris = &tris_;
  return true;
}