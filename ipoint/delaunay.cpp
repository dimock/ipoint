#include "delaunay.h"
#include "imath.h"
#include <time.h>
#include <algorithm>

using namespace iMath;

DelanayTriangulator::DelanayTriangulator(Points3f & points) :
  points_(points), boundaryN_(points.size()), edgeLength_(0)
{
  //points_.clear();
  //load("d:\\scenes\\3dpad\\points.txt");

  if ( !iMath::cw(points_) )
    std::reverse(points_.begin(), points_.end());

  if ( points_.size() < 3 )
    throw std::logic_error("not enough points for triangulation");

  addPoints();

  int D = log((double)points_.size())/(2*log(2.0))+1;
  if ( D > 6 )
    D = 6;
  edgesTree_.reset( new OcTree<OrEdge>(&points_, D) );
  vertexTree_.reset( new OcTree<Vertex>(&points_, D) );

  for (size_t i = 0; i < points_.size(); ++i)
  {
    vertsList_.push_back( Vertex_shared(new Vertex(i, &points_)) );
    vertexTree_->add(vertsList_.back().get());
  }

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
  OrEdges edges;
  edgeLength_ = 0;
  for (size_t i = 0; i < boundaryN_; ++i)
  {
    OrEdge_shared e( new OrEdge(i, (i+1) % boundaryN_, &points_) );
    pushEdge(edges, e);
    edgeLength_ += e->length();
  }
  edgeLength_ *= 1.0/boundaryN_;

  for ( ; !edges.empty(); )
  {
    OrEdges::iterator iter = edges.begin();
    OrEdge * edge = iter->e_;
    edges.erase(iter);

    int i = findTri(*edge);
    if ( i < 0 )
    {
      //findTri(*edge);
      return false;
    }

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
  size_t n = boundaryN_*(boundaryN_);

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

  double err = (p1 - p0).length()*1e-10;
  Vec3f pc = (p0 + p1) * 0.5;

  double dim_min = edgeLength_*0.3;
  Rect3f rect = edge.rect();
  Vec3f infl(0, 0, 0);
  if ( rect.dimension().x < dim_min )
    infl.x = dim_min;
  if ( rect.dimension().y < dim_min )
    infl.y = dim_min;
  if ( rect.dimension().z < dim_min )
    infl.z = dim_min;

  rect.inflate(infl);
  
  Rect3f rect0 = rect;

  Vec3f tdim = vertexTree_->rect().dimension();

  std::set<const Vertex*> looked_up;
  for ( ; !rect.rectInside(vertexTree_->rect()); )
  {
    //Vec3f & rdim = rect.dimension();
    //if ( rdim.x > tdim.x*0.3 && rdim.y > tdim.y*0.3 && rdim.z > tdim.z*0.3 )
    //  break;

    std::set<const Vertex*> verts;
    vertexTree_->collect(rect, verts);

    for (std::set<const Vertex*>::iterator iter = verts.begin(); iter != verts.end(); ++iter)
    {
      if ( looked_up.find(*iter) != looked_up.end() )
        continue;

      looked_up.insert(*iter);

      const int & i = (*iter)->index_;
      if ( canUsePoint(err, i, edge, pc, bestt) )
        index = i;
    }

    if ( index >= 0 )
      break;

    Vec3f dim = rect.dimension()*0.5;
    rect.inflate(dim);
  }

  //if ( index < 0 )
  //{
  //  points_.push_back(Vec3f(0,0,0));
  //  rect = rect0;
  //  for (int step = 0; step < 3; ++step)
  //  {
  //    std::set<const Vertex*> verts;
  //    vertexTree_->collect(rect, verts);
  //    if ( verts.size() > 0 )
  //    {
  //      int i0 = (*verts.begin())->index_;
  //      Vec3f center = points_[i0];
  //      for (std::set<const Vertex*>::iterator iter = verts.begin(); iter != verts.end(); ++iter)
  //      {
  //        const Vec3f & p = points_[(*iter)->index_];
  //        center += p;
  //      }
  //      center *= 1.0/verts.size();

  //      int i = points_.size()-1;
  //      for (int j = 0; j < 10; ++j)
  //      {
  //        double x = ((double)rand())/RAND_MAX;
  //        double y = ((double)rand())/RAND_MAX;
  //        Vec3f p(x*rect.width()*0.3, y*rect.height()*0.3, 0);
  //        p += center;
  //        if ( canUsePoint(err, i, edge, pc, bestt) )
  //          index = i;
  //      }
  //    }

  //    if ( index >= 0 )
  //      break;

  //    Vec3f dim = rect.dimension()*0.5;
  //    rect.inflate(dim);
  //  }
  //}

  return index;
}

bool DelanayTriangulator::canUsePoint(const double & err, size_t i, const OrEdge & edge, const Vec3f & pc, double & bestt) const
{
  if ( i == edge.org() || i == edge.dst() )
    return false;

  const Vec3f & p0 = points_[edge.org()];
  const Vec3f & p1 = points_[edge.dst()];
  const Vec3f & q = points_[i];

  bool outside = false;
  double dist = dist_to_line(p0, p1, q, outside);
  if ( dist <= 0 )
    return false;

  Vec3f nor = (q - p0) ^ (p1 - p0);
  if ( nor.length() < err )
    return false;

  Vec3f rp = (p1 - p0) ^ nor;
  Vec3f rq = (p0 - q) ^ nor;
  Vec3f qc = (p0 + q) * 0.5;

  Vec3f r;
  if ( !line_line_isect(pc, rp, qc, rq, r, dist) )
    return false;

  double t = dist_to_line(p0, p1, r, outside);
  if ( t >= bestt )
    return false;

  OrEdge oe0(edge.org(), i, &points_);
  OrEdge oe1(edge.dst(), i, &points_);

  if ( hasIsect(oe0) || hasIsect(oe1) )
    return false;

  bestt = t;
  return true;
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
