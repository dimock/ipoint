#include "ipoint_alg.h"
#include "imath.h"
#include "delaunay.h"

using namespace std;
using namespace iMath;

class Triangulator
{
  Points3f & points_;

public:

  Triangulator(Points3f & points) :
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

    const Vec3f & p0 = points_[indices[i]];
    const Vec3f & p1 = points_[indices[j]];
    const Vec3f & p2 = points_[indices[k]];

    Vec3f v = (p1 - p0) ^ (p2 - p0);
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

  const Vec3f & p0 = points_[indices[i]];
  const Vec3f & p1 = points_[indices[j]];
  const Vec3f & p2 = points_[indices[k]];


  bool outside = false;
  double dist_p1 = dist_to_line(p0, p2, p1, outside);
  double dist = 0;
  int m = -1;
  int n = k+1;
  if ( n >= indices.size() )
    n = 0;
  for (; n != i; )
  {
    const Vec3f & q = points_[indices[n]];
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