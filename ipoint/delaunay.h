#pragma once

#include "oredge.h"
#include <queue>

class DelaunayTriangulator
{
  typedef std::set <OrEdge*> EdgesSet;
  typedef std::set <const OrEdge*> EdgesSet_const;
  typedef std::list<OrEdge*> EdgesList;

public:
  
  DelaunayTriangulator(Vertices & verts);
  virtual ~DelaunayTriangulator();

  void triangulate(Triangles & tris);
  void save3d(const char * fname, const char * meshName, const char * plineName) const;
  void saveBoundary(const char * fname) const;

private:

  void prebuild();
  bool needRotate(const OrEdge * e, double threshold) const;

  // returns number of edges rotated
  int  makeDelaunay();

  void makeDelaunay(EdgesSet & to_delanay, EdgesSet & to_split, EdgesSet & to_exclude);
  bool getSplitPoint(const OrEdge * , Vertex & ) const;
  void split();
  void intrusionPoint(OrEdge * from);

  void postbuild(Triangles &) const;

  // edge->org() is convex point
  OrEdge * findConvexEdge(OrEdge * from);

  // edge->dst() is intrude point
  OrEdge * findIntrudeEdge(OrEdge * cv_edge);

  double edgeLength_;
  double rotateThreshold_;
  double splitThreshold_;
  double thinThreshold_;

  EdgesContainer container_;
  std::vector<size_t> boundary_;
};