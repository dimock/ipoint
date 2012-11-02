#include "delaunay.h"
#include "imath.h"
#include <time.h>
#include <algorithm>

using namespace iMath;

DelanayTriangulator::DelanayTriangulator(Points3f & points) :
  points_(points), boundaryN_(points.size())
{
  if ( !iMath::cw(points_) )
    std::reverse(points_.begin(), points_.end());
}

DelanayTriangulator::~DelanayTriangulator()
{
  for (std::list<OrEdge*>::iterator i = edgesList_.begin(); i != edgesList_.end(); ++i)
    delete *i;
}

bool DelanayTriangulator::triangulate(Triangles & tris)
{
  if ( points_.size() < 3 )
    return false;

  addPoints();

  edgesTree_.reset( new OcTree<OrEdge>(&points_, 5) );

  OrEdges edges;
  for (size_t i = 0; i < boundaryN_; ++i)
  {
    OrEdge * e = new OrEdge(i, (i+1) % boundaryN_, &points_);
    edges.insert(e);
    edgesList_.push_back(e);
    edgesTree_->add(e);
  }

  for ( ; !edges.empty(); )
  {
    OrEdges::iterator iter = edges.begin();
    OrEdge * edge = iter->e_;
    edges.erase(iter);

    int i = findTri(*edge);
    if ( i < 0 )
      return false;

    update(edges, i, edge->org());
    update(edges, edge->dst(), i);
    tris.push_back( Triangle(edge->org(), edge->dst(), i));
  }

  return true;
}

void DelanayTriangulator::addPoints()
{
  srand(time(0));
  size_t n = boundaryN_*boundaryN_;

  Rect3f rect;
  for (size_t i = 0; i < boundaryN_; ++i)
    rect.add(points_[i]);

  for (size_t i = 0; i < n; ++i)
  {
    double x = ((double)rand())/RAND_MAX;
    double y = ((double)rand())/RAND_MAX;

    Vec3f p(x*rect.width(), y*rect.height(), 0);

    p += rect.origin();

    if ( pointInside(p) )
      points_.push_back(p);
  }
}

bool DelanayTriangulator::pointInside(const Vec3f & q) const
{
  int num = 0;
  Vec3f rq(0.423, 0.5347, 0); // direction

  for (size_t i = 0; i < boundaryN_; ++i)
  {
    const Vec3f & p0 = points_[i];
    const Vec3f & p1 = points_[(i+1) % boundaryN_];

    Vec3f r;
    double dist;
    if ( edge_halfline_isect(p0, p1, q, rq, r, dist) )
      num++;
  }

  return (num & 1) != 0;
}


int DelanayTriangulator::findTri(const OrEdge & edge)
{
  int index = -1;  
  double bestt = DBL_MAX;
  
  const Vec3f & p0 = points_[edge.org()];
  const Vec3f & p1 = points_[edge.dst()];

  Vec3f p = (p0 + p1) * 0.5;
  Vec3f rp = p1 - p0;

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
      OrEdge g(edge.dst(), i, &points_);
      
      const Vec3f & q0 = points_[g.org()];
      const Vec3f & q1 = points_[g.dst()];
      Vec3f q = (q0 + q1) * 0.5;
      Vec3f rq = q1 - q0;
      std::swap(rq.x, rq.y);
      rq.y = -rq.y;

      Vec3f r;
      double dist;
      if ( !line_line_isect(p, rp, q, rq, r, dist) )
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

bool DelanayTriangulator::isectEdge(const Vec3f & p0, const Vec3f & p1, size_t i0, size_t i1) const
{
  double lp = (p0 - p1).length();
  for (size_t i = 0; i < boundaryN_; ++i)
  {
    size_t j = (i+1) % boundaryN_;
    if ( i == i0 || j == i0 || i == i1 || j == i1 )
      continue;

    const Vec3f & q0 = points_[i];
    const Vec3f & q1 = points_[j];

    Vec3f r;
    double dist;
    if ( edges_isect(p0, p1, q0, q1, r, dist) && dist < lp )
      return true;
  }
  return false;
}

void DelanayTriangulator::update(OrEdges & edges, int from, int to)
{
  OrEdge * e = new OrEdge(from, to, &points_);

  OrEdges::iterator iter = edges.find(OrEdgeWrp(e));
  if ( iter != edges.end() )
    edges.erase(iter);
  else
  {
    e->flip();
    edges.insert(e);
  }
}
