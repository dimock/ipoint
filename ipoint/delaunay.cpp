#include "delaunay.h"
#include "imath.h"
#include <time.h>
#include <algorithm>

DelanayTriangulator::DelanayTriangulator(Points2f & points) :
  points_(points), boundaryN_(points.size())
{
  if ( !cw(points_) )
    std::reverse(points_.begin(), points_.end());
}

bool DelanayTriangulator::triangulate(Triangles & tris)
{
  if ( points_.size() < 3 )
    return false;

  addPoints();

  OrEdges edges;
  for (size_t i = 0; i < boundaryN_; ++i)
    edges.insert( OrEdge(i, (i+1) % boundaryN_) );

  for ( ; !edges.empty(); )
  {
    OrEdges::iterator iter = edges.begin();
    OrEdge edge = *iter;
    edges.erase(iter);

    int i = findTri(edge);
    if ( i < 0 )
      return false;

    update(edges, i, edge.org());
    update(edges, edge.dst(), i);
    tris.push_back( Triangle(edge.org(), edge.dst(), i));
  }

  return true;
}

void DelanayTriangulator::addPoints()
{
  srand(time(0));
  size_t n = boundaryN_*sqrtf(boundaryN_);

  Rect2f rect;
  for (size_t i = 0; i < boundaryN_; ++i)
    rect.add(points_[i]);

  for (size_t i = 0; i < n; ++i)
  {
    double x = ((double)rand())/RAND_MAX;
    double y = ((double)rand())/RAND_MAX;

    Vec2f p(x*rect.width(), y*rect.height());

    p += rect.origin();

    if ( pointInside(p) )
      points_.push_back(p);
  }
}

bool DelanayTriangulator::pointInside(const Vec2f & q) const
{
  int num = 0;
  Vec2f rq(0.423, 0.5347);
  for (size_t i = 0; i < boundaryN_; ++i)
  {
    const Vec2f & p0 = points_[i];
    const Vec2f & p1 = points_[(i+1) % boundaryN_];

    Vec2f r;
    if ( edge_halfline_isect(p0, p1, q, rq, r) )
      num++;
  }

  return (num & 1) != 0;
}


int DelanayTriangulator::findTri(const OrEdge & edge)
{
  int index = -1;  
  double bestt = DBL_MAX;
  
  const Vec2f & p0 = points_[edge.org()];
  const Vec2f & p1 = points_[edge.dst()];

  Vec2f p = (p0 + p1) * 0.5;
  Vec2f rp = p1 - p0;

  std::swap(rp.x, rp.y);
  rp.y = -rp.y;

  for (size_t i = 0; i < points_.size(); ++i)
  {
    if ( i == edge.org() || i == edge.dst() )
      continue;

    bool outside = false;
    double dist = dist_to_line(p0, p1, points_[i], outside);
    if ( dist > 0 )
    {
      OrEdge g(edge.dst(), i);
      
      const Vec2f & q0 = points_[g.org()];
      const Vec2f & q1 = points_[g.dst()];
      Vec2f q = (q0 + q1) * 0.5;
      Vec2f rq = q1 - q0;
      std::swap(rq.x, rq.y);
      rq.y = -rq.y;

      Vec2f r;
      if ( !line_line_isect(p, rp, q, rq, r) )
        continue;

      double t = dist_to_line(p0, p1, r, outside);
      if ( t < bestt )
      {
        if ( !isectEdge(p0, points_[i], edge.org(), i) && !isectEdge(p1, points_[i], edge.dst(), i) )
        {
          index = i;
          bestt = t;
        }
      }
    }
  }

  return index;
}

bool DelanayTriangulator::isectEdge(const Vec2f & p0, const Vec2f & p1, size_t i0, size_t i1) const
{
  for (size_t i = 0; i < boundaryN_; ++i)
  {
    size_t j = (i+1) % boundaryN_;
    if ( i == i0 || j == i0 || i == i1 || j == i1 )
      continue;

    const Vec2f & q0 = points_[i];
    const Vec2f & q1 = points_[j];

    Vec2f r;
    if ( edges_isect(p0, p1, q0, q1, r) )
      return true;
  }
  return false;
}

void DelanayTriangulator::update(OrEdges & edges, int from, int to)
{
  OrEdge e(from, to);

  OrEdges::iterator iter = edges.find(e);
  if ( iter != edges.end() )
    edges.erase(iter);
  else
  {
    e.flip();
    edges.insert(e);
  }
}
