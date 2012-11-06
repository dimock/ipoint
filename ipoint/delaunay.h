#pragma once

#include "rect.h"
#include <list>
#include <boost/shared_ptr.hpp>
#include <imath.h>

struct Vertex
{
  const Points3f * points_;
  int index_;

  Vertex(int i, const Points3f * points) : points_(points), index_(i)
  {
  }

  bool intersect(const Rect3f & r) const
  {
    const Vec3f & p = (*points_)[index_];
    return r.pointInside(p);
  }
};

class OrEdge
{
public:

  OrEdge(const Points3f * points) : org_(-1), dst_(-1), points_(points)
  {
  }

  OrEdge(int o, int d, Points3f * points) : org_(o), dst_(d), points_(points)
  {
    rect_.add((*points_)[o]);
    rect_.add((*points_)[d]);

    Vec3f dim  = rect_.dimension();
    double delta = (dim.x + dim.y + dim.z) * 0.001;
    rect_.inflate( Vec3f(delta, delta, delta) );
  }

  void flip()
  {
    std::swap(org_, dst_);
  }

  bool operator < (const OrEdge & other) const
  {
    return org_ < other.org_ || org_ == other.org_ && dst_ < other.dst_;
  }

  bool operator == (const OrEdge & other) const
  {
    return org_ == other.org_ && dst_ == other.dst_;
  }

  int org() const
  {
    return org_;
  }

  int dst() const
  {
    return dst_;
  }

  const Rect3f & rect() const
  {
    return rect_;
  }

  double length() const
  {
    return ((*points_)[org_] - (*points_)[dst_]).length();
  }

  bool intersect(const Rect3f & r) const
  {
    return rect_.intersecting(r);
  }

  bool isectEdge(const Vec3f & p0, const Vec3f & p1, Vec3f & r, double & dist) const
  {
    return iMath::edges_isect((*points_)[org_], (*points_)[dst_], p0, p1, r, dist);
  }

  bool isectEdge(const OrEdge & other, Vec3f & r, double & dist) const
  {
    return isectEdge((*other.points_)[other.org_], (*other.points_)[other.dst_], r, dist);
  }

  bool touches(const OrEdge & other) const
  {
    return org_ == other.org_ || org_ == other.dst_ || dst_ == other.org_ || dst_ == other.dst_;
  }

private:

  Rect3f rect_;
  const Points3f * points_;
  int org_, dst_;

};

typedef boost::shared_ptr<OrEdge> OrEdge_shared;
typedef boost::shared_ptr<Vertex> Vertex_shared;

struct OrEdgeWrp
{
  OrEdge * e_;

  OrEdgeWrp(OrEdge * e) : e_(e) {}

  bool operator < (const OrEdgeWrp & other) const
  {
    return *e_ < *other.e_;
  }
};

typedef std::set<OrEdgeWrp> OrEdges;

template <class T>
class OcTree
{
  const Points3f * points_;
  Rect3f rect_;
  int depth_;

  template <class Q>
  struct Node
  {
    Rect3f r_;

    Node(const Rect3f & r, int level) : r_(r), level_(level)
    {
    }

    bool intersect(const Q & q) const
    {
      return q.intersect(r_);
    }

    bool intersect(const Rect3f & rc) const
    {
      return r_.intersecting(rc);
    }

    boost::shared_ptr<Node> children_[8];

    int level_;
    std::list<const Q*> array_;
  };

  boost::shared_ptr<Node<T>> root_;

public:

  OcTree(const Points3f * points, int depth) : points_(points), depth_(depth)
  {
    for (size_t i = 0; i < points_->size(); ++i)
      rect_.add((*points_)[i]);

    root_.reset( new Node<T>(rect_, 0) );
  }

  void add(const T * t)
  {
    splitNode(root_.get(), t);
  }

  void collect(const Rect3f & rc, std::set<const T*> & items)
  {
    search(root_.get(), rc, items);
  }

  const Rect3f & rect() const
  {
    return rect_;
  }

private:

  void search(Node<T> * node, const Rect3f & rc, std::set<const T*> & items)
  {
    if ( !node || !node->intersect(rc) )
      return;

    if ( node->level_ >= depth_ )
    {
      for (std::list<const T*>::iterator i = node->array_.begin(); i != node->array_.end(); ++i)
        items.insert(*i);

      return;
    }

    for (int i = 0; i < 8; ++i)
      search(node->children_[i].get(), rc, items);
  }

  void splitNode(Node<T> * node, const T * t)
  {
    if ( node->level_ >= depth_ )
    {
      node->array_.push_back(t);
      return;
    }

    for (int i = 0; i < 8; ++i)
    {
      Rect3f rc = node->r_.octant(i);
      if ( !t->intersect(rc) )
        continue;

      if ( !node->children_[i].get() )
        node->children_[i].reset( new Node<T>(rc, node->level_+1) );

      splitNode(node->children_[i].get(), t);
    }
  }
};

typedef std::list<OrEdge_shared> OrEdgesList;
typedef std::list<Vertex_shared> VerticesList;

class DelanayTriangulator
{
public:
  
  DelanayTriangulator(Points3f & points);
  virtual ~DelanayTriangulator();

  bool triangulate(Triangles & tris);

  void save(const char * fname);
  void load(const char * fname);

private:

  void addPoints();
  bool pointInside(const Vec3f & q) const;
  
  int  findTri(const OrEdge & edge);
  void update(OrEdges & edges, int from, int to);

  bool hasIsect(const OrEdge & oe) const;

  void pushEdge(OrEdges & edges, OrEdge_shared e);

  bool canUsePoint(const double & err, size_t i, const OrEdge & edge, const Vec3f & pc, double & bestt) const;


  Points3f & points_;
  size_t boundaryN_;
  double edgeLength_;

  std::auto_ptr<OcTree<OrEdge>> edgesTree_;
  std::auto_ptr<OcTree<Vertex>> vertexTree_;

  OrEdgesList  edgesList_;
  VerticesList vertsList_;
};