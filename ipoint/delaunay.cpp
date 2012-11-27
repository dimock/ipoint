#include "delaunay.h"
#include "imath.h"
#include <time.h>
#include <algorithm>

using namespace iMath;

DelaunayTriangulator::DelaunayTriangulator(Points3f & points) :
  container_(points),
  edgeLength_(0), rotateThreshold_(0), splitThreshold_(0), thinThreshold_(0)
{
  if ( container_.points().size() < 3 )
    throw std::logic_error("not enough points for triangulation");

  cw_ = iMath::cw_dir(container_.points());

  prebuild();
}

DelaunayTriangulator::~DelaunayTriangulator()
{
}

void DelaunayTriangulator::triangulate(Triangles & tris)
{
  for ( ; makeDelaunay() > 0; );

  split();

  for ( ; makeDelaunay() > 0; );

  postbuild(tris);
}

void DelaunayTriangulator::split()
{
  EdgesSet to_split, to_exclude;

  for (OrEdgesList_shared::iterator i = container_.edges().begin(); i != container_.edges().end(); ++i)
  {
    OrEdge * e = i->get();
    OrEdge * a = e->get_adjacent();
    if ( !a || e->length() < splitThreshold_ )
      continue;

    if ( to_exclude.find(e) != to_exclude.end() )
      continue;

    to_split.insert(e);
    to_exclude.insert(a);
  }

  for ( ; !to_split.empty(); )
  {
    EdgesSet::iterator iter = to_split.begin();
    OrEdge * e = *iter;
    to_split.erase(iter);

    OrEdge * adj = e->get_adjacent();
    if ( !adj )
      continue;

    Vec3f p;
    if ( !getSplitPoint(e, p) )
      continue;

    int index = (int)container_.points().size();
    container_.points().push_back(p);

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

    EdgesList egs;
    egs.push_back(a1);
    egs.push_back(c1);
    egs.push_back(a2);
    egs.push_back(e);

    OrEdge * rnext = e->next();
    OrEdge * lprev = adj->prev();
    OrEdge * c2next = c2->next();

    EdgesSet to_delanay;
    to_delanay.insert(e);
    to_delanay.insert(c1);
    to_delanay.insert(b1);
    to_delanay.insert(rnext);
    to_delanay.insert(lprev);
    to_delanay.insert(c2next);

    makeDelaunay(to_delanay, to_split, to_exclude);

    // added edges could be changed while makeDelaunay, so we add them after
    for (EdgesList::iterator i = egs.begin(); i != egs.end(); ++i)
    {
      OrEdge * g = *i;
      OrEdge * a = g->get_adjacent();
      double L = g->length();
      if ( !a || L < splitThreshold_ )
        continue;

      if ( to_exclude.find(g) != to_exclude.end() )
        continue;

      to_split.insert(g);
      to_exclude.insert(a);
    }
  }
}

int DelaunayTriangulator::makeDelaunay()
{
  EdgesSet to_delanay, to_exclude;
  for (OrEdgesList_shared::iterator i = container_.edges().begin(); i != container_.edges().end(); ++i)
  {
    OrEdge * e = i->get();
    OrEdge * a = e->get_adjacent();
    if ( !a || to_exclude.find(e) != to_exclude.end() )
      continue;

    to_delanay.insert(e);
    to_exclude.insert(a);
  }

  int num = 0;
  for (EdgesSet::iterator i = to_delanay.begin(); i != to_delanay.end(); ++i)
  {
    OrEdge * e = *i;
    if ( !needRotate(e, cw_, rotateThreshold_) )
      continue;

    e->rotate();
    num++;
  }

  return num;
}

void DelaunayTriangulator::makeDelaunay(EdgesSet & to_delanay, EdgesSet & to_split, EdgesSet & to_exclude)
{
  EdgesList egs;
  for ( ; !to_delanay.empty(); )
  {
    EdgesSet::iterator i = to_delanay.begin();
    OrEdge * e = *i;
    to_delanay.erase(i);

    if ( !needRotate(e, cw_, rotateThreshold_) )
      continue;

    e->rotate();

    OrEdge * rnext = e->next();
    OrEdge * rprev = e->prev();

    OrEdge * lnext = e->get_adjacent()->next();
    OrEdge * lprev = e->get_adjacent()->prev();

    egs.push_back(rnext);
    egs.push_back(rprev);
    egs.push_back(lnext);
    egs.push_back(lprev);

    for (EdgesList::iterator j = egs.begin(); j != egs.end(); ++j)
    {
      OrEdge * g = *j;
      to_delanay.insert(g);

      OrEdge * a = g->get_adjacent();
      if ( !a || g->length() < splitThreshold_ )
        continue;

      if ( to_exclude.find(g) != to_exclude.end() )
        continue;

      to_split.insert(g);
      to_exclude.insert(a);
    }
  }
}

bool DelaunayTriangulator::getSplitPoint(const OrEdge * edge, Vec3f & p) const
{
  if ( !edge )
    return false;

  const OrEdge * adj = edge->get_adjacent();
  if ( !adj )
    return false;

  double l = edge->length();
  if ( l < splitThreshold_ )
    return false;

  const Vec3f & p0 = container_.points().at(edge->org());
  const Vec3f & p1 = container_.points().at(edge->dst());
  p = (p0 + p1) * 0.5;

  // thin V-pair of triangles?
  const Vec3f & q0 = container_.points().at(edge->next()->dst());
  const Vec3f & q1 = container_.points().at(adj->next()->dst());

  bool outside = false;
  double h = iMath::dist_to_line(p0, q0, p, outside).length();
  if ( h < thinThreshold_ && outside )
    return false;

  h = iMath::dist_to_line(p1, q0, p, outside).length();
  if ( h < thinThreshold_ && outside )
    return false;

  h = iMath::dist_to_line(p0, q1, p, outside).length();
  if ( h < thinThreshold_ && outside )
    return false;

  h = iMath::dist_to_line(p1, q1, p, outside).length();
  if ( h < thinThreshold_ && outside )
    return false;

  return true;
}

bool DelaunayTriangulator::needRotate(const OrEdge * edge, const Vec3f & cw, double threshold) const
{
  if ( !edge )
    return false;
  
  const OrEdge * adj = edge->get_adjacent();
  if ( !adj )
    return false;

  const Vec3f & po = container_.points().at(edge->org());
  const Vec3f & pd = container_.points().at(edge->dst());

  const Vec3f & pr = container_.points().at(edge->next()->dst());
  const Vec3f & pl = container_.points().at(adj->next()->dst());

  bool outside;
  Vec3f dist_r = iMath::dist_to_line(po, pd, pr, outside);
  if ( dist_r.length() < threshold )
    return false;

  Vec3f dist_l = iMath::dist_to_line(po, pd, pl, outside);
  if ( dist_l.length() < threshold )
    return false;

  Vec3f r1 = -edge->next()->dir();
  Vec3f r2 =  edge->prev()->dir();

  Vec3f r3 = -adj->next()->dir();
  Vec3f r4 =  adj->prev()->dir();

  Vec3f n1 = r1^r4;
  Vec3f n2 = r3^r2;

  // V-pair of triangles - don't rotate!
  if ( n1*cw <= 0 || n2*cw <= 0 )
    return false;

  // check Delaunay criteria
  double sa, ca;
  iMath::sincos(r1, r2, sa, ca);

  double sb, cb;
  iMath::sincos(r3, r4, sb, cb);

  double dln = sa*cb + sb*ca;
  return dln < 0;
}

void DelaunayTriangulator::postbuild(Triangles & tris)
{
  EdgesSet_const used;
  for (OrEdgesList_shared::const_iterator i = container_.edges().begin(); i != container_.edges().end(); ++i)
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

void DelaunayTriangulator::prebuild()
{
  OrEdge * curr = 0, * first = 0;
  for (size_t i = 0; i < container_.points().size(); ++i)
  {
    OrEdge * e = container_.new_edge((int)i, (int)((i+1) % container_.points().size()));
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

  rotateThreshold_ = edgeLength_*0.0001;
  splitThreshold_ = edgeLength_*1.5;
  thinThreshold_  = edgeLength_*0.1;

  intrusionPoint(curr);
}

//////////////////////////////////////////////////////////////////////////
void DelaunayTriangulator::intrusionPoint(OrEdge * from)
{
  OrEdge * cv_edge = findConvexEdge(from);
  if ( !cv_edge )
    return;

  // wrong topology!
  if ( !cv_edge->prev() )
    throw std::runtime_error("wrong topology given");

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

OrEdge * DelaunayTriangulator::findConvexEdge(OrEdge * from)
{
  if ( !from )
    return 0;

  for (OrEdge * curr = from->next(); curr != from; curr = curr->next())
  {
    int i = curr->prev()->org();
    int j = curr->org();
    int k = curr->dst();

    const Vec3f & p0 = container_.points().at(i);
    const Vec3f & p1 = container_.points().at(j);
    const Vec3f & p2 = container_.points().at(k);

    Vec3f v = (p1 - p0) ^ (p2 - p0);
    if ( cw_ * v > 0 ) // CW
      return curr;
  }

  // only singular triangles?
  return from;
}

OrEdge * DelaunayTriangulator::findIntrudeEdge(OrEdge * cv_edge)
{
  if ( !cv_edge )
    return 0;

  int i = cv_edge->prev()->org();
  int j = cv_edge->org();
  int k = cv_edge->dst();

  const Vec3f & p0 = container_.points().at(i);
  const Vec3f & p1 = container_.points().at(j);
  const Vec3f & p2 = container_.points().at(k);

  bool outside = false;
  Vec3f vdist_p1 = dist_to_line(p0, p2, p1, outside);
  double dist = 0;

  OrEdge * ir_edge = 0;
  OrEdge * last = cv_edge->prev()->prev();

  for (OrEdge * curr = cv_edge->next(); curr != last; curr = curr->next())
  {
    int n = curr->dst();
    const Vec3f & q = container_.points().at(n);
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
