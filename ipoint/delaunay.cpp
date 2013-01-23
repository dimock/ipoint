#include "delaunay.h"
#include "imath.h"
#include <time.h>
#include <algorithm>
#include <fstream>
#include <limits>

using namespace iMath;

DelaunayTriangulator::DelaunayTriangulator(Vertices & verts) :
  container_(verts),
  edgeLength_(0), rotateThreshold_(0), splitThreshold_(0), thinThreshold_(0),
  convexThreshold_(0.07), dimensionThreshold_(0)
{
  if ( container_.verts().size() < 3 )
    throw std::logic_error("not enough points for triangulation");

  //cw_ = iMath::cw_dir(container_.verts());

  boundary_.resize(container_.verts().size());
  for (size_t i = 0; i < boundary_.size(); ++i)
  {
    boundary_[i] = i;
    const Vec3f & p = verts[boundary_[i]].p();
    rect_.add(p);
  }

  dimensionThreshold_ = rect_.diagonal().length() * 0.3;

  int depth = 5;
  octree_.reset( new OcTree<OrEdge>(rect_, depth) );

  prebuild();
}

DelaunayTriangulator::~DelaunayTriangulator()
{
}

void DelaunayTriangulator::triangulate(Triangles & tris)
{
  save3d("D:\\Scenes\\3dpad\\intrusion.txt", "Mesh", "Boundary", "Normals");

  makeDelaunayRep(true);

  save3d("D:\\Scenes\\3dpad\\intrusion_delaunay.txt", "Mesh", "Boundary", "Normals");

  split();

  makeDelaunayRep(false);

  save3d("D:\\Scenes\\3dpad\\splitted_delaunay.txt", "Mesh", "Boundary", "Normals");

  smooth(2);

  save3d("D:\\Scenes\\3dpad\\splitted_delaunay_smooth.txt", "Mesh", 0, 0);

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

    //octree_->remove(e);
    //octree_->remove(adj);

    if ( !e->splitEdge(index) )
      throw std::runtime_error("couldn't split edge");

    OrEdge * a1 = e->prev();
    OrEdge * b1 = a1->get_adjacent();
    OrEdge * c1 = b1->prev();

    OrEdge * a2 = adj->next();
    OrEdge * b2 = a2->get_adjacent();
    OrEdge * c2 = b2->next();

    //octree_->add(e);
    //octree_->add(adj);
    //octree_->add(a1);
    //octree_->add(b1);
    //octree_->add(c1);
    //octree_->add(a2);
    //octree_->add(b2);
    //octree_->add(c2);

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

void DelaunayTriangulator::makeDelaunayRep(bool checkSI)
{
  int num = std::numeric_limits<int>::max(), repsN = 0;
  for ( ;; )
  {
    int n = makeDelaunay(checkSI);
    if ( n == 0 )
      break;

    if ( n >= num )
      repsN++;
    else
      repsN = 0;

    num = n;
    if ( repsN > 3 )
      break;
  }
}

int DelaunayTriangulator::makeDelaunay(bool checkSI)
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
    if ( !needRotate(e, checkSI) )
      continue;

    OrEdge * a = e->get_adjacent();

    if ( checkSI )
    {
      octree_->remove(e);
      octree_->remove(a);
    }

    if ( e->rotate() )
      num++;

    if ( checkSI )
    {
      octree_->add(e);
      octree_->add(a);
    }
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

    if ( !needRotate(e, false) )
      continue;

    //OrEdge * a = e->get_adjacent();
    //octree_->remove(e);
    //octree_->remove(a);

    if ( !e->rotate() )
      continue;

    //octree_->add(e);
    //octree_->add(a);

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

bool DelaunayTriangulator::needRotate(const OrEdge * edge, bool checkSI) const
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
  if ( checkSI )
  {
    int i0 = edge->next()->dst();
    int i1 = adj->next()->dst();
    OrEdge temp(i0, i1, const_cast<EdgesContainer*>(&container_));
    if ( selfIsect(&temp) )
      return false;

    Triangle tr0(edge->org(), i0, i1);
    if ( selfIsect(tr0) )
      return false;

    Triangle tr1(edge->dst(), i1, i0);
    if ( selfIsect(tr1) )
      return false;
  }

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

    if ( e->next()->next()->next() != e )
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
    {
      //Triangle tr(cv_edge->org(), cv_edge->dst(), cv_edge->next()->next()->dst());
      //if ( selfIsect(tr) )
      //{
      //  save3d("D:\\Scenes\\3dpad\\tri_isect.txt", "Mesh", "Boundary");
      //}
      continue;
    }

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

      
      //Triangle tr(cv_edge->org(), cv_edge->dst(), cv_next->dst());
      //bool found = selfIsect(tr);
      //if ( found )
      //{
      //  save3d("D:\\Scenes\\3dpad\\tri_isect2.txt", "Mesh", "Boundary");
      //  findIntrudeEdge(cv_edge);
      //}

      OrEdge * e = container_.new_edge(cv_edge->org(), cv_next->dst());
      OrEdge * a = e->create_adjacent();

      octree_->add(e);
      octree_->add(a);

      cv_prev->set_next(e);
      e->set_next(cv_next->next());

      cv_next->set_next(a);
      a->set_next(cv_edge);

      //if ( found )
      //{
      //  save3d("D:\\Scenes\\3dpad\\tri_isect2.txt", "Mesh", "Boundary");
      //}
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
  if ( dir.length() < err )
    return false;

  dir.normalize();
  Vec3f cw = cur.n();

  return cw * dir > convexThreshold_;
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
    const Vertex & cur = container_.verts().at( curr->dst() );
    const Vertex & nxt = container_.verts().at( next->dst() );

    if ( isEdgeConvex(curr) )
    {
      double leng = (nxt.p() - pre.p()).length();
      //leng += (nxt.p() - cur.p()).length();
      //leng += (pre.p() - cur.p()).length();
      if ( /*leng < dimensionThreshold_ && */leng < length_min )
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
    best = findConvexEdgeAlt(from, cv_prev);

  if ( !cv_prev )
    cv_prev = best->prev();
  
  return best;
}

OrEdge * DelaunayTriangulator::findConvexEdgeAlt(OrEdge * from, OrEdge *& cv_prev)
{
  if ( !from )
    return 0;

  cv_prev = 0;
  OrEdge * best = 0, * prev = 0;
  OrEdge * shortest = 0, * sprev = 0;
  double length_min = std::numeric_limits<double>::max();
  double length_min2 = std::numeric_limits<double>::max();

  for ( OrEdge * curr = from;; )
  {
    OrEdge * next = curr->next();

    THROW_IF( !next, "bad topology" );

    const Vertex & pre = container_.verts().at( curr->org() );
    const Vertex & cur = container_.verts().at( curr->dst() );
    const Vertex & nxt = container_.verts().at( next->dst() );

    double leng = (nxt.p() - pre.p()).length();
    //leng += (nxt.p() - cur.p()).length();
    //leng += (pre.p() - cur.p()).length();

    if ( !haveCrossSections(curr) )
    {
      if ( leng < length_min )
      {
        best = curr;
        cv_prev = prev;
        length_min = leng;
      }
    }
    else if ( leng < length_min2 )
    {
      shortest = curr;
      sprev = prev;
      length_min2 = leng;
    }

    prev = curr;
    curr = next;

    if ( curr == from )
      break;
  }

  if ( !best )
  {
    best = shortest;
    cv_prev = sprev;
  }

  if ( !cv_prev )
    cv_prev = best->prev();

  return best;
}

void writeTri(const char * fname, const Vec3f & p0, const Vec3f & p1, const Vec3f & p2, const Vec3f & q0, const Vec3f & q1)
{
  std::ofstream ofs(fname);

  ofs << "Mesh \"Tri\" {\n";

  ofs << "  Wireframe {\n";
  ofs << "    ( true )\n";
  ofs << "  }\n";

  ofs << "  Shaded {\n";
  ofs << "    ( true )\n";
  ofs << "  }\n";

  ofs << "  DefaultColor {\n";
  ofs << "    ( 0, 1, 0 )\n";
  ofs << "  }\n";

  ofs << "  Coords {\n";
  ofs << "    ( " << p0.x << ", " << p0.y << ", " << p0.z << " )\n";
  ofs << "    ( " << p1.x << ", " << p1.y << ", " << p1.z << " )\n";
  ofs << "    ( " << p2.x << ", " << p2.y << ", " << p2.z << " )\n";
  ofs << "  }\n";


  ofs << "  Faces {\n";
  ofs << "    ( 0, 1, 2 )\n";
  ofs << "  }\n";

  ofs << "}\n";

  ofs << "Edges \"Proj\" {\n";

  ofs << "  DefaultColor {\n";
  ofs << "    (0, 0, 1)\n";
  ofs << "  }\n";

  ofs << "  { (" << q0.x << ", " << q0.y << ", " << q0.z << ") (" << q1.x << ", " << q1.y << ", " << q1.z <<") (0, 0, 1) }\n";

  ofs << "}\n";
}

void DelaunayTriangulator::writeSomething(const char * fname, const Vec3f & p0, const Vec3f & p1, const Vec3f & p2, std::vector<int> & pline)
{
  std::ofstream ofs(fname);

  ofs << "Mesh \"Tri\" {\n";

  ofs << "  Wireframe {\n";
  ofs << "    ( true )\n";
  ofs << "  }\n";

  ofs << "  Shaded {\n";
  ofs << "    ( true )\n";
  ofs << "  }\n";

  ofs << "  DefaultColor {\n";
  ofs << "    ( 0, 1, 0 )\n";
  ofs << "  }\n";

  ofs << "  Coords {\n";
  ofs << "    ( " << p0.x << ", " << p0.y << ", " << p0.z << " )\n";
  ofs << "    ( " << p1.x << ", " << p1.y << ", " << p1.z << " )\n";
  ofs << "    ( " << p2.x << ", " << p2.y << ", " << p2.z << " )\n";
  ofs << "  }\n";


  ofs << "  Faces {\n";
  ofs << "    ( 0, 1, 2 )\n";
  ofs << "  }\n";

  ofs << "}\n";


  ofs << "Polyline \"pline\" {\n";

  ofs << "  DefaultColor {\n";
  ofs << "    ( 1,0,0 )\n";
  ofs << "  }\n";

  ofs << "  DrawPoints {\n";
  ofs << "    (true)\n";
  ofs << "  }\n";

  ofs << "  DrawLines {\n";
  ofs << "    (true)\n";
  ofs << "  }\n";

  ofs << "  Points {\n";

  for (size_t i = 0; i < pline.size(); ++i)
  {
    size_t j = pline[i];
    const Vec3f & p = container_.verts().at(j).p();
    ofs << "    (" << p.x << ", " << p.y << ", " << p.z << ")\n";
  }
  ofs << "  }\n";

  ofs << "}\n";
}

OrEdge * DelaunayTriangulator::findIntrudeEdge(OrEdge * cv_edge)
{
  if ( !cv_edge )
    return 0;

  const Vertex & pre = container_.verts().at( cv_edge->org() );
  const Vertex & cvv = container_.verts().at( cv_edge->dst() );
  const Vertex & nxt = container_.verts().at( cv_edge->next()->dst() );

  Vec3f nor = pre.n() + cvv.n() + nxt.n();

  bool outside = false;
  Vec3f vdist_cvv = dist_to_line(pre.p(), nxt.p(), cvv.p(), outside);
  double dist_cvv = vdist_cvv.length();
  double dist = 0;

  std::vector<int> pline;
  OrEdge * ir_edge = 0;
  for ( OrEdge * curr = cv_edge->next()->next();
          curr != cv_edge && curr->next() != cv_edge;
          curr = curr->next())
  {
    pline.push_back(curr->dst());

    const Vertex & iv = container_.verts().at( curr->dst() );
    Vec3f vd = dist_to_line(pre.p(), nxt.p(), iv.p(), outside);

    // project intrusion point to triangle plane
    Vec3f N = (cvv.p() - pre.p()) ^ (nxt.p() - pre.p());
    if ( N.length() < iMath::err )
      continue;
    N.normalize();
    double D = -cvv.p() * N;
    double ivd = iv.p() * N + D;
    Vec3f ivp = iv.p() - N*ivd;
    double x = ivp*N+D;

    bool inside = inside_tri(pre.p(), cvv.p(), nxt.p(), ivp);

    if ( !inside )
      continue;

    if ( vd*vdist_cvv <= 0 )
      continue;

   // writeTri("d:\\scenes\\3dpad\\tri.txt", pre.p(), cvv.p(), nxt.p(), iv.p(), ivp);

    double d = vd.length();
    double dist_icv = (iv.p()-cvv.p()).length();
    if ( d <= dist || d >= dist_cvv || dist_icv > 2.0*dist_cvv )
      continue;

    //if ( nor*iv.n() < 0 )
    //  continue;

    //OrEdge temp(cv_edge->dst(), curr->dst(), &container_);
    //if ( selfIsect(&temp) )
    //  continue;

    ir_edge = curr;
    dist = d;
  }

  //writeSomething("d:\\scenes\\3dpad\\something.txt", pre.p(), cvv.p(), nxt.p(), pline);

  return ir_edge;
}

void DelaunayTriangulator::smooth(int itersN)
{
  for (int n = 0; n < itersN; ++n)
  for (OrEdgesList_shared::iterator i = container_.edges().begin(); i != container_.edges().end(); ++i)
  {
    OrEdge * edge = i->get();
    smoothPt(edge);
  }
}

void DelaunayTriangulator::smoothPt(OrEdge * edge)
{
  if ( !edge )
    return;

  const Vertex & v0 = container_.verts().at(edge->org());
  const Vertex & v1 = container_.verts().at(edge->dst());

  Vec3f pnt = v0.p() + v1.p();
  Vec3f nor = v0.n() + v1.n();
  int counter = 2;

  THROW_IF( !edge->next() || !edge->next()->next(), "bad topology" );

  std::vector<Triangle> tris;
  for (OrEdge * curr = edge; curr; )
  {
    tris.push_back(curr->tri());

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

  double cosaMin = 1.0;
  for (size_t i = 0; i < tris.size(); ++i)
  {
    Triangle & tri0 = tris[i];
    const Vec3f & p0 = container_.verts().at(tri0.x).p();
    const Vec3f & p1 = container_.verts().at(tri0.y).p();
    const Vec3f & p2 = container_.verts().at(tri0.z).p();
    Vec3f n0 = (p1-p0) ^ (p2-p0);
    n0.normalize();
    for (size_t j = i+1; j < tris.size(); ++j)
    {
      Triangle & tri1 = tris[j];
      const Vec3f & q0 = container_.verts().at(tri1.x).p();
      const Vec3f & q1 = container_.verts().at(tri1.y).p();
      const Vec3f & q2 = container_.verts().at(tri1.z).p();
      Vec3f n1 = (q1-q0) ^ (q2-q0);
      n1.normalize();
      double cosa = n1 * n0;
      if ( cosa < cosaMin )
        cosaMin = cosa;
    }
  }

  double coef = (1.0 - cosaMin)*0.5;
  if ( coef < 0 )
    coef = 0;
  if ( coef > 1.0 )
    coef = 1.0;

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
  EdgesSet_const items, used, polyline;
  octree_->collect(edge->rect(), items);

  const Vec3f & ep0 = container_.verts().at(edge->org()).p();
  const Vec3f & ep1 = container_.verts().at(edge->dst()).p();

  for (EdgesSet_const::iterator i = items.begin(); i != items.end(); ++i)
  //for (OrEdgesList_shared::const_iterator i = container_.edges().begin(); i != container_.edges().end(); ++i)
  {
    const OrEdge * e = *i;
    if ( used.find(e) != used.end() )
      continue;

    THROW_IF ( !e->next() || !e->next()->next(), "bad topology" );

    // is triangle
    if ( e->next()->next()->next() != e )
    {
      polyline.insert(e);
      continue;
    }

    // don't search triangle twice
    used.insert( e->next() );
    used.insert( e->next()->next() );

    Triangle tr = e->tri();

    // touches
    if ( tr.x == edge->org() || tr.x == edge->dst() ||
         tr.y == edge->org() || tr.y == edge->dst() ||
         tr.z == edge->org() || tr.z == edge->dst() )
    {
      continue;
    }

    const Vec3f & tp0 = container_.verts().at(tr.x).p();
    const Vec3f & tp1 = container_.verts().at(tr.y).p();
    const Vec3f & tp2 = container_.verts().at(tr.z).p();

    Vec3f ip;
    if ( iMath::edge_tri_isect(ep0, ep1, tp0, tp1, tp2, ip) )
      return true;
  }

  if ( polyline.empty() )
    return false;

  for (EdgesSet_const::iterator i = polyline.begin(); i != polyline.end(); ++i)
  {
    const OrEdge * from = *i;
    if ( used.find(from) != used.end() )
      continue;

    for (const OrEdge * curr = from->next(); curr != from && curr->next() != from; curr = curr->next())
    {
      used.insert(curr);

      Triangle tr(from->org(), curr->org(), curr->dst());

      const Vec3f & tp0 = container_.verts().at(tr.x).p();
      const Vec3f & tp1 = container_.verts().at(tr.y).p();
      const Vec3f & tp2 = container_.verts().at(tr.z).p();

      // touches
      if ( tr.x == edge->org() || tr.x == edge->dst() ||
           tr.y == edge->org() || tr.y == edge->dst() ||
           tr.z == edge->org() || tr.z == edge->dst() )
      {
        continue;
      }

      Vec3f ip;
      if ( iMath::edge_tri_isect(ep0, ep1, tp0, tp1, tp2, ip) )
        return true;
    }
  }

  return false;
}

bool DelaunayTriangulator::selfIsect(const Triangle & tr) const
{
  const Vec3f & tp0 = container_.verts().at(tr.x).p();
  const Vec3f & tp1 = container_.verts().at(tr.y).p();
  const Vec3f & tp2 = container_.verts().at(tr.z).p();

  Rect3f rc;
  rc.add(tp0);
  rc.add(tp1);
  rc.add(tp2);

  OrEdge tedges[3] = { OrEdge(tr.x, tr.y, const_cast<EdgesContainer*>(&container_)),
    OrEdge(tr.y, tr.z, const_cast<EdgesContainer*>(&container_)),
    OrEdge(tr.z, tr.x, const_cast<EdgesContainer*>(&container_)) };

  EdgesSet_const items, used;
  octree_->collect(rc, items);

  for (EdgesSet_const::iterator i = items.begin(); i != items.end(); ++i)
  //for (OrEdgesList_shared::const_iterator i = container_.edges().begin(); i != container_.edges().end(); ++i)
  {
    const OrEdge * e = *i;
    if ( used.find(e) != used.end() )
      continue;

    // touches
    if ( tr.x == e->org() || tr.x == e->dst() ||
         tr.y == e->org() || tr.y == e->dst() ||
         tr.z == e->org() || tr.z == e->dst() )
    {
      continue;
    }

    const Vec3f & ep0 = container_.verts().at(e->org()).p();
    const Vec3f & ep1 = container_.verts().at(e->dst()).p();

    Vec3f ip;
    if ( iMath::edge_tri_isect(ep0, ep1, tp0, tp1, tp2, ip) )
      return true;

    // not a triangle
    if ( e->next()->next()->next() != e )
      continue;

    used.insert(e->next());
    used.insert(e->next()->next());

    Triangle etr = e->tri();

    const Vec3f & q0 = container_.verts().at(etr.x).p();
    const Vec3f & q1 = container_.verts().at(etr.y).p();
    const Vec3f & q2 = container_.verts().at(etr.z).p();

    for (int j = 0; j < 3; ++j)
    {
      OrEdge & oe = tedges[j];

      // touches
      if ( etr.x == oe.org() || etr.x == oe.dst() ||
           etr.y == oe.org() || etr.y == oe.dst() ||
           etr.z == oe.org() || etr.z == oe.dst() )
      {
        continue;
      }

      const Vec3f & x0 = container_.verts().at(e->org()).p();
      const Vec3f & x1 = container_.verts().at(e->dst()).p();

      if ( iMath::edge_tri_isect(x0, x1, q0, q1, q2, ip) )
        return true;
    }
  }

  return false;
}

bool DelaunayTriangulator::haveCrossSections(const OrEdge * edge) const
{
  EdgesSet_const items;
  octree_->collect(edge->rect(), items);

  const Vec3f & p0 = container_.verts().at(edge->org()).p();
  const Vec3f & p1 = container_.verts().at(edge->dst()).p();

  for (EdgesSet_const::iterator i = items.begin(); i != items.end(); ++i)
  {
    const OrEdge * e = *i;

    const Vec3f & q0 = container_.verts().at(e->org()).p();
    const Vec3f & q1 = container_.verts().at(e->dst()).p();

    Vec3f r;
    double dist = 0;
    if ( iMath::edges_isect(p0, p1, q0, q1, r, dist) )
      return true;
  }

  return false;
}
//////////////////////////////////////////////////////////////////////////
void DelaunayTriangulator::save3d(const char * fname, const char * meshName, const char * plineName, const char * edgesName) const
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

  if ( edgesName )
  {
    ofs << "Edges \"" << edgesName << "\" {\n";

    ofs << "  DefaultColor {\n";
    ofs << "    (0, 0, 1)\n";
    ofs << "  }\n";

    for (size_t i = 0; i < boundary_.size(); ++i)
    {
      size_t j = boundary_[i];
      const Vec3f & p = verts.at(j).p();
      Vec3f n = verts.at(j).n();
      n = p + n*edgeLength_;
      ofs << "  { (" << p.x << ", " << p.y << ", " << p.z << ") (" << n.x << ", " << n.y << ", " << n.z <<") (0, 0, 1) }\n";
    }

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
