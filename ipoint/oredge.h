#pragma once

#include "rect.h"
#include <list>
#include <imath.h>
#include <boost/shared_ptr.hpp>

class EdgesContainer;

/**
    Oriented edge structure
        
                         **
                        /|^\    next
                       / || \
                      /  ||  \
                     /   ||   \
                    *    ||    *
       [adjacent]----\- >||   /
                      \  ||  /
                       \ || /
                        \||/    prev
                         **

*/

class OrEdge
{
public:

  OrEdge(EdgesContainer * container);
  OrEdge(int o, int d, EdgesContainer * container);


  // structure
  int org() const { return org_; }
  int dst() const { return dst_; }

  // topology
  OrEdge * get_adjacent();
  const OrEdge * get_adjacent() const;
  OrEdge * create_adjacent();
  void clear_adjacent();

  // rotate this & adjacent edges 90 deg CW
  void rotate();

  OrEdge * next() const;
  OrEdge * prev() const;
  
  // return old 'next'
  OrEdge * set_next(OrEdge * e);

  // insert point with index i
  bool splitTri(int i);
  bool splitEdge(int i);

  // get triangle representation
  Triangle tri() const;
  
  // comparison
  bool operator < (const OrEdge & other) const;
  bool operator == (const OrEdge & other) const;
  bool touches(const OrEdge & other) const;

  // geometry
  Rect3f rect() const;
  double length() const;
  Vec3f  dir() const;

  // math
  bool intersect(const Rect3f & r) const;
  bool isectEdge(const Vec3f & p0, const Vec3f & p1, Vec3f & r, double & dist) const;
  bool isectEdge(const OrEdge & other, Vec3f & r, double & dist) const;

  // data
private:
  
  int org_, dst_;
  OrEdge * next_, * adjacent_;
  EdgesContainer * container_;
};

typedef boost::shared_ptr<OrEdge> OrEdge_shared;
typedef std::list<OrEdge_shared> OrEdgesList_shared;

class EdgesContainer
{
public:

  EdgesContainer(Points3f & points) : points_(points)
  {}

  OrEdge * new_edge(int o, int d);
  const Points3f & points() const { return points_; }
  Points3f & points() { return points_; }
  const OrEdgesList_shared & edges() const { return edges_; }
  OrEdgesList_shared & edges() { return edges_; }

private:

  OrEdgesList_shared edges_;
  Points3f & points_;
};
