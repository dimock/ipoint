#pragma once

#include "vec.h"

template <class T>
class OcTree
{
public:
  OcTree(Points3f & points);
};

class DelanayTriangulator
{
public:
  
  DelanayTriangulator(Points3f & points);

  bool triangulate(Triangles & tris);

private:

  void addPoints();
  bool pointInside(const Vec3f & q) const;
  
  int  findTri(const OrEdge & edge);
  void update(OrEdges & edges, int from, int to);

  bool isectEdge(const Vec3f & p0, const Vec3f & p1, size_t i0, size_t i1) const;

  Points3f & points_;
  size_t boundaryN_;
};