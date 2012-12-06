#include "delaunay.h"
#include "imath.h"
#include <time.h>
#include <algorithm>
#include <fstream>
#include <limits>

using namespace iMath;

DelaunayTriangulator::DelaunayTriangulator(Vertices & verts) :
  container_(verts),
  edgeLength_(0), rotateThreshold_(0), splitThreshold_(0), thinThreshold_(0)
{
  if ( container_.verts().size() < 3 )
    throw std::logic_error("not enough points for triangulation");

  //cw_ = iMath::cw_dir(container_.verts());

  boundary_.resize(container_.verts().size());
  for (size_t i = 0; i < boundary_.size(); ++i)
    boundary_[i] = i;

  prebuild();
}

DelaunayTriangulator::~DelaunayTriangulator()
{
}

void DelaunayTriangulator::triangulate(Triangles & tris)
{
  save3d("D:\\Scenes\\3dpad\\intrusion.txt", "Mesh", "Boundary");

  for ( ;; )
  {
    if ( makeDelaunay() == 0 )
      break;
  }

  save3d("D:\\Scenes\\3dpad\\intrusion_delaunay.txt", "Mesh", "Boundary");

  split();

  for ( ;; )
  {
    if ( makeDelaunay() == 0 )
      break;
  }

  save3d("D:\\Scenes\\3dpad\\splitted_delaunay.txt", "Mesh", "Boundary");

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

  for (int n = 0; !to_split.empty(); ++n)
  {
    EdgesSet::iterator iter = to_split.begin();
    OrEdge * e = *iter;
    to_split.erase(iter);

    OrEdge * adj = e->get_adjacent();
    if ( !adj )
      continue;

    Vertex v;
    if ( !getSplitPoint(e, v) )
      continue;

    int index = (int)container_.verts().size();
    container_.verts().push_back(v);

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
    if ( !needRotate(e) )
      continue;

    if ( e->rotate() )
      num++;
  }

  return num;
}

void DelaunayTriangulator::makeDelaunay(EdgesSet & to_delanay, EdgesSet & to_split, EdgesSet & to_exclude)
{
  EdgesList egs;
  for (int n = 0; !to_delanay.empty(); ++n)
  {
    EdgesSet::iterator i = to_delanay.begin();
    OrEdge * e = *i;
    to_delanay.erase(i);

    if ( !needRotate(e) )
      continue;

    if ( !e->rotate() )
      continue;

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

Vec3f DelaunayTriangulator::calcPt(const Vec3f & p0, const Vec3f & p1, const Vec3f & n0, const Vec3f & n1, double t) const
{
  Vec3f dir = p1 - p0;
  if ( dir.length() > iMath::err )
    dir.normalize();
  else
    return p0;

  Vec3f x0 = n0 ^ dir;
  if ( x0.length() > iMath::err )
    x0.normalize();

  Vec3f x1 = n1 ^ dir;
  if ( x1.length() > iMath::err )
    x1.normalize();

  Vec3f r0 = x0 ^ n0;
  Vec3f r1 = x1 ^ n1;

  if ( r0.length() > iMath::err )
    r0.normalize();

  if ( r1.length() > iMath::err )
    r1.normalize();

  Vec3f q1 = p0 + r0;
  Vec3f q2 = p1 - r1;
  Vec3f p = p0*sqr3(1.-t) + q1 * t * sqr2(1.-t) + q2 * sqr2(t) * (1-t) + p1 * sqr3(t);

  return p;
}

bool DelaunayTriangulator::getSplitPoint(const OrEdge * edge, Vertex & v) const
{
  if ( !edge )
    return false;

  const OrEdge * adj = edge->get_adjacent();
  if ( !adj )
    return false;

  double l = edge->length();
  if ( l < splitThreshold_ )
    return false;

  const Vec3f & p0 = container_.verts().at(edge->org()).p();
  const Vec3f & p1 = container_.verts().at(edge->dst()).p();

  const Vec3f & n0 = container_.verts().at(edge->org()).n();
  const Vec3f & n1 = container_.verts().at(edge->dst()).n();

  Vec3f p = (p0 + p1)*0.5;
  Vec3f n = n0 + n1;
  n.normalize();

  // thin V-pair of triangles?
  const Vec3f & q0 = container_.verts().at(edge->next()->dst()).p();
  const Vec3f & q1 = container_.verts().at(adj->next()->dst()).p();

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

  v = Vertex(p, n);

  return true;
}

bool DelaunayTriangulator::needRotate(const OrEdge * edge) const
{
  if ( !edge )
    return false;
  
  const OrEdge * adj = edge->get_adjacent();
  if ( !adj )
    return false;

  const Vec3f & po = container_.verts().at(edge->org()).p();
  const Vec3f & pd = container_.verts().at(edge->dst()).p();

  const Vec3f & pr = container_.verts().at(edge->next()->dst()).p();
  const Vec3f & pl = container_.verts().at(adj->next()->dst()).p();

  bool outside = false;
  Vec3f dist_r = iMath::dist_to_line(po, pd, pr, outside);
  if ( dist_r.length() < rotateThreshold_ || outside )
    return false;

  Vec3f dist_l = iMath::dist_to_line(po, pd, pl, outside);
  if ( dist_l.length() < rotateThreshold_ || outside )
    return false;

  // now check Delaunay criteria
  Vec3f r1 = -edge->next()->dir();
  Vec3f r2 =  edge->prev()->dir();

  Vec3f r3 = -adj->next()->dir();
  Vec3f r4 =  adj->prev()->dir();

  double sa, ca;
  iMath::sincos(r1, r2, sa, ca);

  double sb, cb;
  iMath::sincos(r3, r4, sb, cb);

  double dln = sa*cb + sb*ca;
  return dln < 0;
}

void DelaunayTriangulator::postbuild(Triangles & tris) const
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
  for (size_t i = 0; i < container_.verts().size(); ++i)
  {
    OrEdge * e = container_.new_edge((int)i, (int)((i+1) % container_.verts().size()));
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
  thinThreshold_  = edgeLength_*0.2;

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

  if ( ir_edge )
  {
    OrEdge * cv_prev = cv_edge->prev();
    OrEdge * ir_next = ir_edge->next();
    if ( !cv_prev || !ir_next )
      return;

    OrEdge * e = container_.new_edge(ir_edge->dst(), cv_edge->org());
    OrEdge * a = e->create_adjacent();

    e->set_next(cv_edge);
    ir_edge->set_next(e);

    cv_prev->set_next(a);
    a->set_next(ir_next);

    intrusionPoint(e);
    intrusionPoint(a);
  }
  else
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
}

bool DelaunayTriangulator::isEdgeConvex(const OrEdge * edge) const
{
  if ( !edge )
    return false;

  int i = edge->prev()->org();
  int j = edge->org();
  int k = edge->dst();

  const Vertex & pre = container_.verts().at(i);
  const Vertex & org = container_.verts().at(j);
  const Vertex & dst = container_.verts().at(k);

  Vec3f dir = (pre.p() - org.p()) ^ (dst.p() - org.p());
  Vec3f cw = org.n() + pre.n() + dst.n();

  return cw * dir > 0;
}

OrEdge * DelaunayTriangulator::findConvexEdge(OrEdge * from)
{
  if ( !from )
    return 0;

  OrEdge * best = 0;
  double length_min = std::numeric_limits<double>::max();

  for (OrEdge * curr = from->next(); curr != from; curr = curr->next())
  {
    int i = curr->prev()->org();
    int j = curr->org();
    int k = curr->dst();

    const Vertex & pre = container_.verts().at(i);
    const Vertex & org = container_.verts().at(j);
    const Vertex & dst = container_.verts().at(k);

    if ( !isEdgeConvex(curr) )
      continue;

    double leng = (pre.p() - dst.p()).length();
    if ( leng < length_min || !best )
    {
      best = curr;
      length_min = leng;
    }
  }
  
  if ( best )
    return best;

  return from;
}

OrEdge * DelaunayTriangulator::findIntrudeEdge(OrEdge * cv_edge)
{
  if ( !cv_edge )
    return 0;

  int i = cv_edge->prev()->org();
  int j = cv_edge->org();
  int k = cv_edge->dst();

  const Vertex & v0 = container_.verts().at(i);
  const Vertex & v1 = container_.verts().at(j);
  const Vertex & v2 = container_.verts().at(k);

  Vec3f nor = v0.n() + v1.n() + v2.n();

  bool outside = false;
  Vec3f vdist_p1 = dist_to_line(v0.p(), v2.p(), v1.p(), outside);
  double dist_p1 = vdist_p1.length();
  double dist = 0;

  OrEdge * ir_edge = 0;
  OrEdge * last = cv_edge->prev()->prev();

  for (OrEdge * curr = cv_edge->next(); curr != last; curr = curr->next())
  {
    // intrude edge couldn't be convex
    if ( isEdgeConvex(curr) )
      continue;

    int k = curr->dst();
    const Vertex & q = container_.verts().at(k);
    Vec3f vd = dist_to_line(v0.p(), v2.p(), q.p(), outside);
    bool inside = inside_tri(v0.p(), v1.p(), v2.p(), q.p());
    if ( !inside || vd*vdist_p1 <= 0 )
      continue;

    double d = vd.length();
    double dist_icv = (q.p()-v1.p()).length();
    if ( d <= dist || d >= dist_p1 || dist_icv > dist_p1*2.0 )
      continue;

    if ( nor*q.n() < 0 )
      continue;

    ir_edge = curr;
    dist = d;
  }

  return ir_edge;
}

void DelaunayTriangulator::save3d(const char * fname, const char * meshName, const char * plineName) const
{
  if ( !fname || !meshName )
    return;

  std::ofstream ofs(fname);

  Triangles tris;
  postbuild(tris);

  const Vertices & verts = container_.verts();

  Vec3f color(0,1,0);

  ofs << "Mesh \"" << meshName << "\" {\n";

  ofs << "  Wireframe {\n";
  ofs << "    ( true )\n";
  ofs << "  }\n";

  ofs << "  Shaded {\n";
  ofs << "    ( true )\n";
  ofs << "  }\n";

  ofs << "  DefaultColor {\n";
  ofs << "    ( " << color.x << ", " << color.y << ", " << color.z << " )\n";
  ofs << "  }\n";

  ofs << "  Coords {\n";
  for (Vertices::const_iterator i = verts.begin(); i != verts.end(); ++i)
  {
    const Vec3f & p = i->p();
    ofs << "    ( " << p.x << ", " << p.y << ", " << p.z << " )\n";
  }
  ofs << "  }\n";


  ofs << "  Faces {\n";
  for (Triangles::const_iterator i = tris.begin(); i != tris.end(); ++i)
  {
    const Triangle & t = *i;
    ofs << "    ( " << t.x << ", " << t.y << ", " << t.z << " )\n";
  }
  ofs << "  }\n";

  ofs << "}\n";


  if ( plineName )
  {
    Vec3f lineColor(1,0,0);

    ofs << "Polyline \"" << plineName << "\" {\n";

    ofs << "  DefaultColor {\n";
    ofs << "    ( " << lineColor.x << ", " << lineColor.y << ", " << lineColor.z << " )\n";
    ofs << "  }\n";

    ofs << "  DrawPoints {\n";
    ofs << "    (true)\n";
    ofs << "  }\n";

    ofs << "  DrawLines {\n";
    ofs << "    (true)\n";
    ofs << "  }\n";

    ofs << "  Points {\n";

    for (size_t i = 0; i < boundary_.size(); ++i)
    {
      size_t j = boundary_[i];
      const Vec3f & p = verts.at(j).p();
      ofs << "    (" << p.x << ", " << p.y << ", " << p.z << ")\n";
    }
    ofs << "  }\n";

    ofs << "}\n";
  }
}


void DelaunayTriangulator::saveBoundary(const char * fname) const
{
  if ( !fname )
    return;

  std::ofstream ofs(fname);

  Triangles tris;
  postbuild(tris);

  const Vertices & verts = container_.verts();

  ofs << "{\n";

  for (size_t i = 0; i < boundary_.size(); ++i)
  {
    size_t j = boundary_[i];
    const Vec3f & p = verts.at(j).p();
    const Vec3f & n = verts.at(j).n();
    ofs << "  {" << p.x << ", " << p.y << ", " << p.z << "} {" << n.x << ", " << n.y << ", " << n.z << "}\n";
  }

  ofs << "}\n";
}
