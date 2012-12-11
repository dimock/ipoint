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

  Rect3f rect;
  boundary_.resize(container_.verts().size());
  for (size_t i = 0; i < boundary_.size(); ++i)
  {
    boundary_[i] = i;
    const Vec3f & p = verts[boundary_[i]].p();
    rect.add(p);
  }

  int depth = 5;
  octree_.reset( new OcTree<OrEdge>(rect, depth) );

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

  smooth(0.2, 1);

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

    octree_->remove(e);
    octree_->remove(adj);

    if ( !e->splitEdge(index) )
      throw std::runtime_error("couldn't split edge");

    OrEdge * a1 = e->prev();
    OrEdge * b1 = a1->get_adjacent();
    OrEdge * c1 = b1->prev();

    OrEdge * a2 = adj->next();
    OrEdge * b2 = a2->get_adjacent();
    OrEdge * c2 = b2->next();

    octree_->add(e);
    octree_->add(adj);
    octree_->add(a1);
    octree_->add(b1);
    octree_->add(c1);
    octree_->add(a2);
    octree_->add(b2);
    octree_->add(c2);

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

    OrEdge * a = e->get_adjacent();
    octree_->remove(e);
    octree_->remove(a);

    if ( e->rotate() )
      num++;

    octree_->add(e);
    octree_->add(a);
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

    OrEdge * a = e->get_adjacent();
    octree_->remove(e);
    octree_->remove(a);

    if ( !e->rotate() )
      continue;

    octree_->add(e);
    octree_->add(a);

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
  if ( dln > -iMath::err )
    return false;

  // self-intersections
  int i0 = edge->next()->dst();
  int i1 = adj->next()->dst();
  OrEdge temp(i0, i1, const_cast<EdgesContainer*>(&container_));
  if ( selfIsect(&temp) )
    return false;

  return true;
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
    
    octree_->add(e);

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
  splitThreshold_ = edgeLength_*2.0;
  thinThreshold_  = edgeLength_*0.25;

  intrusionPoint(curr);
}

//////////////////////////////////////////////////////////////////////////
void DelaunayTriangulator::intrusionPoint(OrEdge * from)
{
  EdgesList elist;
  elist.push_back(from);

  for ( ; !elist.empty(); )
  {
    OrEdge * curr = elist.back();
    elist.pop_back();

    OrEdge * cv_prev = 0;
    OrEdge * cv_edge = findConvexEdge(curr, cv_prev);

    // wrong topology!
    if ( !cv_prev || !cv_edge->next() )
      throw std::runtime_error("wrong topology given");

    // 1 triangle
    if ( cv_prev == cv_edge->next()->next() )
      continue;

    OrEdge * ir_edge = findIntrudeEdge(cv_edge);
    if ( ir_edge )
    {
      OrEdge * cv_next = cv_edge->next();
      OrEdge * ir_next = ir_edge->next();
      if ( !cv_next || !ir_next )
        return;

      OrEdge * e = container_.new_edge(ir_edge->dst(), cv_edge->dst());
      OrEdge * a = e->create_adjacent();

      octree_->add(e);
      octree_->add(a);

      e->set_next(cv_next);
      ir_edge->set_next(e);

      cv_edge->set_next(a);
      a->set_next(ir_next);

      elist.push_back(e);
      elist.push_back(a);
    }
    else
    {
      OrEdge * cv_next = cv_edge->next();
      if ( !cv_next )
         throw std::runtime_error("wrong topology given");;

      OrEdge * e = container_.new_edge(cv_edge->org(), cv_next->dst());
      OrEdge * a = e->create_adjacent();

      octree_->add(e);
      octree_->add(a);

      cv_prev->set_next(e);
      e->set_next(cv_next->next());

      cv_next->set_next(a);
      a->set_next(cv_edge);

      elist.push_back(e);
    }
  }
}

bool DelaunayTriangulator::isEdgeConvex(const OrEdge * edge) const
{
  if ( !edge )
    return false;

  const Vertex & pre = container_.verts().at( edge->org() );
  const Vertex & cur = container_.verts().at( edge->dst() );
  const Vertex & nxt = container_.verts().at( edge->next()->dst() );

  Vec3f dir = (pre.p() - cur.p()) ^ (nxt.p() - cur.p());
  Vec3f cw = cur.n() + pre.n() + nxt.n();

  return cw * dir > 0;
}

OrEdge * DelaunayTriangulator::findConvexEdge(OrEdge * from, OrEdge *& cv_prev)
{
  if ( !from )
    return 0;

  cv_prev = 0;
  OrEdge * best = 0, * prev = 0;

  double length_min = std::numeric_limits<double>::max();

  for ( OrEdge * curr = from;; )
  {
    OrEdge * next = curr->next();

    THROW_IF( !next, "bad topology" );

    const Vertex & pre = container_.verts().at( curr->org() );
    const Vertex & nxt = container_.verts().at( next->dst() );

    if ( isEdgeConvex(curr) )
    {
      double leng = (nxt.p() - pre.p()).length();
      if ( leng < length_min || !best )
      {
        best = curr;
        cv_prev = prev;
        length_min = leng;
      }
    }

    prev = curr;
    curr = next;

    if ( curr == from )
      break;
  }
  
  if ( !best )
    best = from;

  if ( !cv_prev )
    cv_prev = best->prev();

  return best;
}

OrEdge * DelaunayTriangulator::findIntrudeEdge(OrEdge * cv_edge)
{
  if ( !cv_edge )
    return 0;

  const Vertex & pre = container_.verts().at( cv_edge->org() );
  const Vertex & cur = container_.verts().at( cv_edge->dst() );
  const Vertex & nxt = container_.verts().at( cv_edge->next()->dst() );

  Vec3f nor = pre.n() + cur.n() + nxt.n();

  bool outside = false;
  Vec3f vdist_cur = dist_to_line(pre.p(), nxt.p(), cur.p(), outside);
  double dist_cur = vdist_cur.length();
  double dist = 0;

  OrEdge * ir_edge = 0;
  for ( OrEdge * curr = cv_edge->next()->next();
          curr != cv_edge && curr->next() != cv_edge;
          curr = curr->next())
  {
    // intrude edge couldn't be convex
    if ( isEdgeConvex(curr) )
      continue;

    const Vertex & iv = container_.verts().at( curr->dst() );
    Vec3f vd = dist_to_line(pre.p(), nxt.p(), iv.p(), outside);
    bool inside = inside_tri(pre.p(), cur.p(), nxt.p(), iv.p());

    if ( !inside || vd*vdist_cur <= 0 )
      continue;

    double d = vd.length();
    double dist_icv = (iv.p()-cur.p()).length();
    if ( d <= dist || d >= dist_cur || dist_icv > dist_cur*2.0 )
      continue;

    if ( nor*iv.n() < 0 )
      continue;

    OrEdge temp(cv_edge->dst(), curr->dst(), &container_);
    if ( selfIsect(&temp) )
      continue;

    ir_edge = curr;
    dist = d;
  }

  return ir_edge;
}

void DelaunayTriangulator::smooth(double coef, int itersN)
{
  for (int n = 0; n < itersN; ++n)
  for (OrEdgesList_shared::iterator i = container_.edges().begin(); i != container_.edges().end(); ++i)
  {
    OrEdge * edge = i->get();
    smoothPt(edge, coef);
  }
}

void DelaunayTriangulator::smoothPt(OrEdge * edge, double coef)
{
  if ( !edge )
    return;

  const Vertex & v0 = container_.verts().at(edge->org());
  const Vertex & v1 = container_.verts().at(edge->dst());

  Vec3f pnt = v0.p() + v1.p();
  Vec3f nor = v0.n() + v1.n();
  int counter = 2;

  THROW_IF( !edge->next() || !edge->next()->next(), "bad topology" );

  for (OrEdge * curr = edge; curr; )
  {
    THROW_IF( !curr->next() || !curr->next()->next(), "bad topology" );

    curr = curr->next()->next();
    const Vertex & v = container_.verts().at(curr->org());
    
    pnt += v.p();
    nor += v.n();
    counter++;

    curr = curr->get_adjacent();
    if ( curr == edge )
      break;
  }

  pnt *= 1.0 / counter;
  nor.normalize();
  Vec3f dp = pnt - v0.p();
  dp *= coef;

  pnt = v0.p() + dp;
  container_.verts().at(edge->org()) = Vertex(pnt, nor);
}
//////////////////////////////////////////////////////////////////////////
// Self-intersections
bool DelaunayTriangulator::selfIsect(OrEdge * edge) const
{
  EdgesSet_const items, used;
  octree_->collect(edge->rect(), items);

  for (EdgesSet_const::iterator i = items.begin(); i != items.end(); ++i)
  {
    const OrEdge * e = *i;
    if ( used.find(e) != used.end() )
      continue;

    THROW_IF ( !e->next() || !e->next()->next(), "bad topology" );

    // is triangle
    if ( e->next()->next()->next() != e )
      continue;

    // don't search triangle twice
    used.insert( e->next() );
    used.insert( e->next()->next() );

    Triangle tr = e->tri();
    if ( tr.x == edge->org() || tr.x == edge->dst() ||
         tr.y == edge->org() || tr.y == edge->dst() ||
         tr.z == edge->org() || tr.z == edge->dst() )
    {
      continue;
    }

    const Vec3f & ep0 = container_.verts().at(edge->org()).p();
    const Vec3f & ep1 = container_.verts().at(edge->dst()).p();

    const Vec3f & tp0 = container_.verts().at(tr.x).p();
    const Vec3f & tp1 = container_.verts().at(tr.y).p();
    const Vec3f & tp2 = container_.verts().at(tr.z).p();

    Vec3f ip;
    if ( iMath::edge_tri_isect(ep0, ep1, tp0, tp1, tp2, ip) )
      return true;
  }

  return false;
}

//////////////////////////////////////////////////////////////////////////
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
