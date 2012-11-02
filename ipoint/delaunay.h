#pragma once

#include "rect.h"
#include <list>

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

    Vec3f dim  =rect_.dimension();
    double maxd = (std::max(std::max(dim.x, dim.y), dim.z)) * 0.5;
    if ( dim.x < maxd )
    {
      double dx = (maxd - dim.x) * 0.5;
      rect_.vmin.x -= dx;
      rect_.vmax.x += dx;
    }
    if ( dim.y < maxd )
    {
      double dy = (maxd - dim.y) * 0.5;
      rect_.vmin.y -= dy;
      rect_.vmax.y += dy;
    }
    if ( dim.z < maxd )
    {
      double dz = (maxd - dim.x) * 0.5;
      rect_.vmin.z -= dz;
      rect_.vmax.z += dz;
    }
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

  bool intersect(const Rect3f & r) const
  {
    return rect_.intersecting(r);
  }

private:

  Rect3f rect_;
  const Points3f * points_;
  int org_, dst_;

};

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
      for (int i = 0; i < 8; ++i)
        children_[i] = 0;
    }

    ~Node()
    {
      for (int i = 0; i < 8; ++i)
        delete children_[i];
    }

    bool intersect(const Q & q) const
    {
      return q.intersect(r_);
    }

    Node * children_[8];

    int level_;
    std::list<const Q*> array_;
  };

  Node<T> * root_;

public:

  OcTree(const Points3f * points, int depth) : points_(points), depth_(depth)
  {
    for (size_t i = 0; i < points_->size(); ++i)
      rect_.add((*points_)[i]);

    root_ = new Node<T>(rect_, 0);
  }

  void add(const T * t)
  {
    Node<T> * node = findNode(root_, t);
    if ( !node )
      return;
    if ( node->level_ < depth_ )
      splitNode(node, t);
  }

private:

  Node<T> * findNode(Node<T> * node, const T * t)
  {
    if ( !node || !node->intersect(*t) )
      return 0;

    for (int i = 0; i < 8; ++i)
    {
      Node<T> * n = findNode(node->children_[i], t);
      if ( n )
        return n;
    }

    return node;
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

      if ( !node->children_[i] )
      {
        node->children_[i] = new Node<T>(rc, node->level_+1);
        splitNode(node->children_[i], t);
      }
    }
  }
};

class DelanayTriangulator
{
public:
  
  DelanayTriangulator(Points3f & points);
  ~DelanayTriangulator();

  bool triangulate(Triangles & tris);

private:

  void addPoints();
  bool pointInside(const Vec3f & q) const;
  
  int  findTri(const OrEdge & edge);
  void update(OrEdges & edges, int from, int to);

  bool isectEdge(const Vec3f & p0, const Vec3f & p1, size_t i0, size_t i1) const;

  Points3f & points_;
  size_t boundaryN_;

  std::auto_ptr<OcTree<OrEdge>> edgesTree_;
  std::auto_ptr<OcTree<Vertex>> vertexTree_;

  std::list<OrEdge*> edgesList_;
  std::list<Vertex*> vertexList_;
};