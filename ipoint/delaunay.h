#pragma once

#include "oredge.h"
#include "octree.h"

class DelaunayTriangulator
{
  typedef std::set <OrEdge*> EdgesSet;
  typedef std::set <const OrEdge*> EdgesSet_const;
  typedef std::list<OrEdge*> EdgesList;

public:
  
  DelaunayTriangulator(Vertices & verts);
  virtual ~DelaunayTriangulator();

  void triangulate(Triangles & tris);
  void save3d(const char * fname, const char * meshName, const char * plineName, const char * edgesName) const;
  void saveBoundary(const char * fname) const;
  void writeSomething(const char * fname, const Vec3f & p0, const Vec3f & p1, const Vec3f & p2, std::vector<int> &);

private:

  Vec3f calcPt(const Vec3f & p0, const Vec3f & p1, const Vec3f & n0, const Vec3f & n1, double t) const;

  void prebuild();
  bool needRotate(const OrEdge * e, bool checkSI) const;

  // returns number of edges rotated
  int  makeDelaunay(bool checkSI);
  void makeDelaunayRep(bool checkSI);

  void makeDelaunay(EdgesSet & to_delanay, EdgesSet & to_split, EdgesSet & to_exclude);
  bool getSplitPoint(const OrEdge * , Vertex & ) const;
  void split();
  void intrusionPoint(OrEdge * from);

  void postbuild(Triangles &) const;

  // edge->org() is convex point
  OrEdge * findConvexEdge(OrEdge * from, OrEdge *& cv_prev);
  OrEdge * findConvexEdgeAlt(OrEdge * from, OrEdge *& cv_prev);

  bool isEdgeConvex(const OrEdge * edge) const;

  // edge->dst() is intrude point
  OrEdge * findIntrudeEdge(OrEdge * cv_edge);

  void smooth(int itersN);
  void smoothPt(OrEdge * edge);

  // self-intersections
  bool selfIsect(OrEdge * edge) const;
  bool selfIsect(const Triangle & tr) const;
  bool haveCrossSections(const OrEdge * ) const;

  double edgeLength_;
  double rotateThreshold_;
  double splitThreshold_;
  double thinThreshold_;
  double convexThreshold_;
  double dimensionThreshold_;

  Rect3f rect_;
  EdgesContainer container_;
  std::vector<size_t> boundary_;

  boost::shared_ptr<OcTree<OrEdge>> octree_;
};