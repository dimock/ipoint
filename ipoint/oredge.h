#pragma once

#include "rect.h"
#include <boost/shared_ptr.hpp>

class DelanayTriangulator;

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

  OrEdge(DelanayTriangulator * container);
  OrEdge(int o, int d, DelanayTriangulator * container);


  // structure
  int org() const { return org_; }
  int dst() const { return dst_; }

  // topology
  OrEdge * adjacent();
  void clear_adjacent();

  // rotate this & adjacent edges 90 deg CW
  void rotate();
  OrEdge * next() const;
  OrEdge * prev() const;
  
  // return old 'next'
  OrEdge * set_next(OrEdge * e);

  // insert point with index i
  bool split(int i);

  // get triangle representation
  Triangle tri() const;
  
  // comparision
  bool operator < (const OrEdge & other) const;
  bool operator == (const OrEdge & other) const;
  bool touches(const OrEdge & other) const;

  // geometry
  const Rect3f & rect() const;
  double length() const;

  // math
  bool intersect(const Rect3f & r) const;
  bool isectEdge(const Vec3f & p0, const Vec3f & p1, Vec3f & r, double & dist) const;
  bool isectEdge(const OrEdge & other, Vec3f & r, double & dist) const;

private:

  Rect3f rect_;
  double length_;
  DelanayTriangulator * container_;
  int org_, dst_;
  OrEdge * next_, * adjacent_;
};

typedef boost::shared_ptr<OrEdge> OrEdge_shared;
