#pragma once

#include "oredge.h"
#include <queue>

class DelaunayTriangulator
{
  typedef std::set <OrEdge*> EdgesSet;
  typedef std::set <const OrEdge*> EdgesSet_const;
  typedef std::list<OrEdge*> EdgesList;

public:
  
  DelaunayTriangulator(Points3f & points);
  virtual ~DelaunayTriangulator();

  void triangulate(Triangles & tris);

private:

  void prebuild();
  bool needRotate(const OrEdge * e, const Vec3f & cw, double threshold) const;

  // returns number of edges rotated
  int  makeDelaunay();

  void makeDelaunay(EdgesSet & to_delanay, EdgesSet & to_split, EdgesSet & to_exclude);
  bool getSplitPoint(const OrEdge * , Vec3f & ) const;
  void split();
  void postbuild(Triangles &);
  void intrusionPoint(OrEdge * from);

  // edge->org() is convex point
  OrEdge * findConvexEdge(OrEdge * from);

  // edge->dst() is intrude point
  OrEdge * findIntrudeEdge(OrEdge * cv_edge);

  double edgeLength_;
  double rotateThreshold_;
  double splitThreshold_;
  double thinThreshold_;

  EdgesContainer container_;

  Vec3f cw_;
};