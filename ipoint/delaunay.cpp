#include "delaunay.h"
#include "imath.h"
#include <time.h>
#include <algorithm>

using namespace iMath;

DelanayTriangulator::DelanayTriangulator(Points3f & points) :
  points_(points), container_(points_), edgeLength_(0), rotateThreshold_(0), splitThreshold_(0)
{
  points_.clear();
  load("d:\\scenes\\3dpad\\points4.txt");
//  save("d:\\scenes\\3dpad\\points.txt");

  cw_ = iMath::cw_dir(points_);

  if ( points_.size() < 3 )
    throw std::logic_error("not enough points for triangulation");

  prebuild();
}

void DelanayTriangulator::load(const char * fname)
{
  FILE * f = fopen(fname, "rt");
  if ( !f )
    return;

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
  if ( !f )
    return;

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

void DelanayTriangulator::triangulate(Triangles & tris)
{
  for ( ;; )
  {
    int n = makeDelaunay();
    if ( !n )
      break;
  }

  split();

  for ( ;;)
  {
    int n = makeDelaunay();
    if ( !n )
      break;
  }

  postbuild(tris);
}

int DelanayTriangulator::makeDelaunay()
{
  std::set<OrEdge*> used;
  std::set<OrEdge*> adj_used;
  for (OrEdgesList::iterator i = container_.edges().begin(); i != container_.edges().end(); ++i)
  {
    OrEdge * e = i->get();
    if ( adj_used.find(e) != adj_used.end() || !e->get_adjacent() )
      continue;

    used.insert(e);
    adj_used.insert(e->get_adjacent());
  }

  int num = 0;
  for (std::set<OrEdge*>::iterator i = used.begin(); i != used.end(); ++i)
  {
    OrEdge * e = *i;
    if ( !e->needRotate(cw_, rotateThreshold_) )
      continue;

    e->rotate();
    num++;
  }

  return num;
}

void DelanayTriangulator::makeDelaunay(std::set<OrEdge*> & edges, std::set<OrEdge*> & eg_list, double threshold)
{
  for ( ; !edges.empty(); )
  {
    std::set<OrEdge*>::iterator i = edges.begin();
    OrEdge * e = *i;
    edges.erase(i);

    if ( !e->needRotate(cw_, rotateThreshold_) )
      continue;

    e->rotate();

    OrEdge * rnext = e->next();
    OrEdge * rprev = e->prev();

    OrEdge * lnext = e->get_adjacent()->next();
    OrEdge * lprev = e->get_adjacent()->prev();

    edges.insert(rnext);
    edges.insert(rprev);
    edges.insert(lnext);
    edges.insert(lprev);

    if ( rnext->get_adjacent() && rnext->length() > threshold )
      eg_list.insert(rnext);
    if ( rprev->get_adjacent() && rprev->length() > threshold )
      eg_list.insert(rprev);
    if ( lprev->get_adjacent() && lprev->length() > threshold )
      eg_list.insert(lprev);
    if ( lnext->get_adjacent() && lnext->length() > threshold )
      eg_list.insert(lnext);
  }
}

void DelanayTriangulator::split()
{
  std::set<OrEdge*> adj_used;
 // EdgesQueueSorted eg_queue;

  std::set<OrEdge *> eg_list;

  for (OrEdgesList::iterator i = container_.edges().begin(); i != container_.edges().end(); ++i)
  {
    OrEdge * e = i->get();
    if ( adj_used.find(e) != adj_used.end() )
      continue;

    if ( e->get_adjacent() && e->length() > splitThreshold_ )
    {
      eg_list.insert(e);
      if ( e->get_adjacent() )
        adj_used.insert(e->get_adjacent());
    }
  }

  int itersN = 0;
  for ( ; !eg_list.empty(); ++itersN)
  {
    std::set<OrEdge *>::iterator iter = eg_list.begin();
    OrEdge * e = *iter;
    eg_list.erase(iter);

    OrEdge * adj = e->get_adjacent();
    if ( !adj )
      continue;

    double l = e->length();
    if ( l < splitThreshold_ )
      continue;

    int o = e->org();
    int d = e->dst();
    if ( o == 101 && d == 995 )
    {
      int ttt = 0;
    }

    const Vec3f & p0 = points_.at(e->org());
    const Vec3f & p1 = points_.at(e->dst());
    Vec3f p = (p0 + p1) * 0.5;

    // thin triangle?
    const Vec3f & q0 = points_.at(e->next()->dst());
    const Vec3f & q1 = points_.at(e->get_adjacent()->next()->dst());

    double stopThreshold = splitThreshold_*0.1;

    double dist0 = (q0 - p).length();
    double dist1 = (q1 - p).length();
    if ( dist0 < stopThreshold || dist1 < stopThreshold )
      continue;

    bool outside;
    double h = iMath::dist_to_line(p0, q0, p, outside).length();
    if ( h < stopThreshold )
      continue;
    
    h = iMath::dist_to_line(p1, q0, p, outside).length();
    if ( h < stopThreshold )
      continue;

    h = iMath::dist_to_line(p0, q1, p, outside).length();
    if ( h < stopThreshold )
      continue;

    h = iMath::dist_to_line(p1, q1, p, outside).length();
    if ( h < stopThreshold )
      continue;



    int index = (int)points_.size();
    points_.push_back(p);

    if ( !e->splitEdge(index) )
      throw std::runtime_error("couldn't split edge");

    OrEdge * a1 = e->prev();
    OrEdge * b1 = a1->get_adjacent();
    OrEdge * c1 = b1->prev();

    OrEdge * a2 = adj->next();
    OrEdge * b2 = a2->get_adjacent();
    OrEdge * c2 = b2->next();

    if ( c2 != c1->get_adjacent() )
      throw std::runtime_error("wrong topology");

    std::list<OrEdge*> egs;
    egs.push_back(a1);
    egs.push_back(c1);
    egs.push_back(a2);
    egs.push_back(e);

    OrEdge * rnext = e->next();
    OrEdge * lprev = adj->prev();
    OrEdge * c2next = c2->next();

    std::set<OrEdge*> edges;
    edges.insert(e);
    edges.insert(c1);
    edges.insert(b1);
    edges.insert(rnext);
    edges.insert(lprev);
    edges.insert(c2next);
    makeDelaunay(edges, eg_list, splitThreshold_);

    for (std::list<OrEdge*>::iterator i = egs.begin(); i != egs.end(); ++i)
    {
      OrEdge * e1 = *i;
      double l1 = e1->length();
      if ( l1 == 0 )
      {
        int rrr = 0;
        return;
      }
      if ( !e1->get_adjacent() || l1 < splitThreshold_ )
        continue;

      eg_list.insert(e1);
    }
  }
}

void DelanayTriangulator::postbuild(Triangles & tris)
{
  std::set<const OrEdge *> used;
  for (OrEdgesList::const_iterator i = container_.edges().begin(); i != container_.edges().end(); ++i)
  {
    const OrEdge * e = i->get();
    if ( used.find(e) != used.end() )
      continue;

    Triangle t = e->tri();
    tris.push_back(t);
    used.insert(e);
    used.insert(e->prev());
    used.insert(e->next());
  }
}

void DelanayTriangulator::prebuild()
{
  OrEdge * curr = 0, * first = 0;
  for (size_t i = 0; i < points_.size(); ++i)
  {
    OrEdge * e = container_.new_edge(i, (i+1) % points_.size());
    edgeLength_ += e->length();
    if ( !first )
      first = e;
    else
      curr->set_next(e);
    curr = e;
  }
  curr->set_next(first);

  if ( container_.edges().size() > 0 )
    edgeLength_ /= container_.edges().size();

  //edgeLength_  = 0.047255099481883819;
  rotateThreshold_ = edgeLength_*0.0001;
  splitThreshold_ = edgeLength_*1.5;

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

    OrEdge * e = container_.new_edge(prev->org(), cv_edge->dst());
    OrEdge * a = e->create_adjacent();

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

    OrEdge * e = container_.new_edge(ir_edge->dst(), cv_edge->org());
    OrEdge * a = e->create_adjacent();

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
