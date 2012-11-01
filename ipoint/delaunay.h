#pragma once

#include "vec.h"

class DelanayTriangulator
{
public:
  
  DelanayTriangulator(Points2f & points);

  bool triangulate(Triangles & tris);

private:

  void addPoints();
  bool pointInside(const Vec2f & q) const;
  
  int  findTri(const OrEdge & edge);
  void update(OrEdges & edges, int from, int to);

  bool isectEdge(const Vec2f & p0, const Vec2f & p1, size_t i0, size_t i1) const;

  Points2f & points_;
  size_t boundaryN_;
};