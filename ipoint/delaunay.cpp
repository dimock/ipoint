#include "delaunay.h"
#include "imath.h"
#include <time.h>
#include <algorithm>

using namespace iMath;

DelanayTriangulator::DelanayTriangulator(Points3f & points) :
  points_(points),edgeLength_(0)
{
  //points_.clear();
  //load("d:\\scenes\\3dpad\\points.txt");
  //save("d:\\scenes\\3dpad\\points.txt");

  cw_ = iMath::cw_dir(points_);

  if ( points_.size() < 3 )
    throw std::logic_error("not enough points for triangulation");

  int D = (int)(log((double)points_.size())/(2*log(2.0))+1.0);
  if ( D > 6 )
    D = 6;

  edgesTree_.reset( new OcTree<OrEdge>(&points_, D) );
  vertexTree_.reset( new OcTree<Vertex>(&points_, D) );

  for (size_t i = 0; i < points_.size(); ++i)
  {
    vertsList_.push_back( Vertex_shared(new Vertex(i, &points_)) );
    vertexTree_->add(vertsList_.back().get());
  }

  prebuild();
}

void DelanayTriangulator::load(const char * fname)
{
  FILE * f = fopen(fname, "rt");
  char buff[256];
  const char * sepr = "\t ,;{}\n\r";
  for ( ; fgets(buff, sizeof(buff), f); )
  {
    if ( buff[0] == '{' )
      continue;

    if ( buff[0] == '}' )
      break;

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
  }
  fclose(f);
}

void DelanayTriangulator::save(const char * fname)
{
  FILE * f = fopen(fname, "wt");
  fprintf(f, "{\n");
  for (size_t i = 0; i < points_.size(); ++i)
  {
    const Vec3f & p = points_[i];
    fprintf(f, "  {%g, %g}\n", p.x, p.y);
  }
  fprintf(f, "}\n");
  fclose(f);
}

DelanayTriangulator::~DelanayTriangulator()
{
}

OrEdge * DelanayTriangulator::newOrEdge(int o, int d)
{
  OrEdge_shared e(new OrEdge(o, d, this));
  edgesList_.push_back(e);
  edgesTree_->add(e.get());
  return e.get();
}

bool DelanayTriangulator::triangulate(Triangles & tris)
{
  std::set<OrEdge*> used;
  for (OrEdgesList::iterator i = edgesList_.begin(); i != edgesList_.end(); ++i)
  {
    OrEdge * e = i->get();
    if ( used.find(e) != used.end() )
      continue;

    Triangle t = e->tri();
    tris.push_back(t);
    used.insert(e);
    used.insert(e->prev());
    used.insert(e->next());
  }

  edgesList_.clear();
  vertsList_.clear();

  return true;
}

void DelanayTriangulator::prebuild()
{
  OrEdge * curr = 0, * first = 0;
  for (size_t i = 0; i < points_.size(); ++i)
  {
    OrEdge * e = newOrEdge(i, (i+1) % points_.size());
    edgeLength_ += e->length();
    if ( !first )
      first = e;
    else
      curr->set_next(e);
    curr = e;
  }
  curr->set_next(first);

  if ( edgesList_.size() > 0 )
    edgeLength_ /= edgesList_.size();

  intrusionPoint(curr);
}

//////////////////////////////////////////////////////////////////////////
void DelanayTriangulator::intrusionPoint(OrEdge * from)
{
  OrEdge * cv_edge = findConvexEdge(from);
  if ( !cv_edge )
    return;

  // wrong topology!
  if ( !cv_edge->prev() )
    return;

  // 1 triangle
  if ( cv_edge->prev()->prev() == cv_edge->next() )
    return;

  OrEdge * ir_edge = findIntrudeEdge(cv_edge);
  if ( !ir_edge )
  {
    OrEdge * prev = cv_edge->prev();
    OrEdge * next = cv_edge->next();
    if ( !next || !prev )
      return;

    OrEdge * pprev = prev->prev();

    OrEdge * e = newOrEdge(prev->org(), cv_edge->dst());
    OrEdge * a = e->adjacent();

    pprev->set_next(e);
    e->set_next(next);
    
    cv_edge->set_next(a);
    a->set_next(prev);

    intrusionPoint(e);
  }
  else
  {
    OrEdge * prev = cv_edge->prev();
    OrEdge * next = ir_edge->next();

    if ( !prev || !next )
      return;

    OrEdge * e = newOrEdge(ir_edge->dst(), cv_edge->org());
    OrEdge * a = e->adjacent();

    e->set_next(cv_edge);
    ir_edge->set_next(e);

    prev->set_next(a);
    a->set_next(next);

    intrusionPoint(e);
    intrusionPoint(a);
  }
}

OrEdge * DelanayTriangulator::findConvexEdge(OrEdge * from)
{
  if ( !from )
    return 0;

  for (OrEdge * curr = from->next(); curr != from; curr = curr->next())
  {
    int i = curr->prev()->org();
    int j = curr->org();
    int k = curr->dst();

    const Vec3f & p0 = points_[i];
    const Vec3f & p1 = points_[j];
    const Vec3f & p2 = points_[k];

    Vec3f v = (p1 - p0) ^ (p2 - p0);
    if ( cw_ * v > 0 ) // CW
      return curr;
  }

  return 0;
}

OrEdge * DelanayTriangulator::findIntrudeEdge(OrEdge * cv_edge)
{
  if ( !cv_edge )
    return 0;

  int i = cv_edge->prev()->org();
  int j = cv_edge->org();
  int k = cv_edge->dst();

  const Vec3f & p0 = points_[i];
  const Vec3f & p1 = points_[j];
  const Vec3f & p2 = points_[k];


  bool outside = false;
  Vec3f vdist_p1 = dist_to_line(p0, p2, p1, outside);
  double dist = 0;

  OrEdge * ir_edge = 0;
  OrEdge * last = cv_edge->prev()->prev();

  for (OrEdge * curr = cv_edge->next(); curr != last; curr = curr->next())
  {
    int n = curr->dst();
    const Vec3f & q = points_[n];
    Vec3f vd = dist_to_line(p0, p2, q, outside);
    bool inside = inside_tri(p0, p1, p2, q);
    if ( inside && vd*vdist_p1 > 0 )
    {
      double d = vd.length();
      if ( d > dist )
      {
        ir_edge = curr;
        dist = d;
      }
    }
  }

  return ir_edge;
}
