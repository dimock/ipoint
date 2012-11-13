#pragma once

#include "oredge.h"
#include <queue>

class EdgesCmpr
{
public:
  EdgesCmpr() {}

  bool operator () (const OrEdge * pe0, const OrEdge * pe1) const
  {
    return pe0->length() < pe1->length();
  }
};

class DelanayTriangulator
{
  friend class OrEdge;

public:
  
  DelanayTriangulator(Points3f & points);
  virtual ~DelanayTriangulator();

  void triangulate(Triangles & tris);

  void save(const char * fname);
  void load(const char * fname);

private:

  void prebuild();
  int  makeDelaunay();
  void makeDelaunay(std::set<OrEdge*> & edges, std::set<OrEdge*> & eg_list);
  bool getSplitPoint(OrEdge *, Vec3f & ) const;
  void split();
  void postbuild(Triangles &);
  void intrusionPoint(OrEdge * from);

  // edge->org() is convex point
  OrEdge * findConvexEdge(OrEdge * from);

  // edge->dst() is intrude point
  OrEdge * findIntrudeEdge(OrEdge * cv_edge);

  Points3f & points_;
  double edgeLength_;
  double rotateThreshold_;
  double splitThreshold_;
  double thinThreshold_;

  typedef std::priority_queue<OrEdge*, std::vector<OrEdge*>, EdgesCmpr> EdgesQueueSorted;

  EdgesContainer container_;

  Vec3f cw_;
};