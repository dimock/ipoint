#include "delaunay.h"
#include "imath.h"
#include <time.h>
#include <algorithm>

using namespace iMath;

DelanayTriangulator::DelanayTriangulator(Points3f & points) :
  points_(points), boundaryN_(points.size())
{
  //points_.clear();
  //load("d:\\scenes\\3dpad\\points.txt");

  if ( !iMath::cw(points_) )
    std::reverse(points_.begin(), points_.end());

  if ( points_.size() < 3 )
    throw std::logic_error("not enough points for triangulation");

  addPoints();

  save("d:\\scenes\\3dpad\\points.txt");
}

void DelanayTriangulator::load(const char * fname)
{
  FILE * f = fopen(fname, "rt");
  char buff[256];
  boundaryN_ = 0; 
  bool stop = false;
  const char * sepr = "\t ,;{}\n\r";
  for ( ; fgets(buff, sizeof(buff), f); )
  {
    if ( buff[0] == '{' )
      continue;

    if ( buff[0] == '}' )
    {
      stop = true;
      continue;
    }

    char * s = strtok(buff, sepr);
    double v[3] = {0};
    for (int i = 0; s && i < 3; ++i)
    {
      float x;
      sscanf(s, "%f", &x);
      v[i] = x;
      s = strtok(0, sepr);
    }
    points_.push_back( Vec3f(v[0], v[1], v[2]) );
    if ( !stop )
      boundaryN_++;
  }
  fclose(f);
}

void DelanayTriangulator::save(const char * fname)
{
  FILE * f = fopen(fname, "wt");
  fprintf(f, "{\n");
  for (size_t i = 0; i < boundaryN_; ++i)
  {
    const Vec3f & p = points_[i];
    fprintf(f, "  {%g, %g}\n", p.x, p.y);
  }
  fprintf(f, "}\n");
  for (size_t i = boundaryN_; i < points_.size(); ++i)
  {
    const Vec3f & p = points_[i];
    fprintf(f, "{\n");
    fprintf(f, "  {%g, %g}\n", p.x, p.y);
    fprintf(f, "}\n");
  }
  fclose(f);
}

DelanayTriangulator::~DelanayTriangulator()
{
}

void DelanayTriangulator::pushEdge(OrEdges & edges, OrEdge_shared e)
{
  edges.insert(e.get());
  edgesList_.push_back(e);
  edgesTree_->add(e.get());
}

bool DelanayTriangulator::triangulate(Triangles & tris)
{
  edgesTree_.reset( new OcTree<OrEdge>(&points_, 3) );

  OrEdges edges;
  for (size_t i = 0; i < boundaryN_; ++i)
  {
    OrEdge_shared e( new OrEdge(i, (i+1) % boundaryN_, &points_) );
    pushEdge(edges, e);
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

  edgesList_.clear();
  vertsList_.clear();

  return true;
}

void DelanayTriangulator::addPoints()
{
  srand(time(0));
  size_t n = boundaryN_*sqrtf(boundaryN_);

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
        OrEdge oe0(edge.org(), i, &points_);
        OrEdge oe1(edge.dst(), i, &points_);

        if ( !hasIsect(oe0) && !hasIsect(oe1) )
        {
          index = i;
          bestt = t;
        }
      }
    }
  }

  return index;
}

bool DelanayTriangulator::hasIsect(const OrEdge & oe) const
{
  double dim = oe.rect().dimension().length();
 
  std::set<const OrEdge*> target;
  edgesTree_->collect(oe.rect(), target);

  for (std::set<const OrEdge*>::iterator iter = target.begin(); iter != target.end(); ++iter)
  {
    const OrEdge * e = *iter;
    if ( e->touches(oe) )
      continue;

    Vec3f r;
    double dist;
    if ( e->isectEdge(oe, r, dist) && dist < dim )
      return true;
  }
  return false;
}

void DelanayTriangulator::update(OrEdges & edges, int from, int to)
{
  std::auto_ptr<OrEdge> e( new OrEdge(from, to, &points_) );

  OrEdges::iterator iter = edges.find(OrEdgeWrp(e.get()));
  if ( iter != edges.end() )
    edges.erase(iter);
  else
  {
    e->flip();
    pushEdge(edges, OrEdge_shared(e.release()));
  }
}
