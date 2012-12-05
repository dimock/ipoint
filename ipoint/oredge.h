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

  // geometry
  double length() const;
  Vec3f  dir() const;

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

  EdgesContainer(Vertices & verts) : verts_(verts)
  {}

  OrEdge * new_edge(int o, int d);

  Vertices & verts()
  {
    return verts_;
  }

  const Vertices & verts() const
  {
    return verts_;
  }

  const OrEdgesList_shared & edges() const
  {
    return edges_;
  }

  OrEdgesList_shared & edges()
  {
    return edges_;
  }

private:

  OrEdgesList_shared edges_;
  Vertices & verts_;
};
