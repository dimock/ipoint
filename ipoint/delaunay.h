#pragma once

#include "oredge.h"

class DelanayTriangulator
{
  friend class OrEdge;

public:
  
  DelanayTriangulator(Points3f & points);
  virtual ~DelanayTriangulator();

  bool triangulate(Triangles & tris);

  void save(const char * fname);
  void load(const char * fname);

private:

  void prebuild();
  void intrusionPoint(OrEdge * from);

  // edge->org() is convex point
  OrEdge * findConvexEdge(OrEdge * from);

  // edge->dst() is intrude point
  OrEdge * findIntrudeEdge(OrEdge * cv_edge);

  Points3f & points_;
  double edgeLength_;

  EdgesContainer container_;

  Vec3f cw_;
};