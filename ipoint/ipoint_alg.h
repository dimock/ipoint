#pragma once

#include "rect.h"
#include <vector>

class Triangulator;


class IntrusionPointAlgorithm
{

public:

  IntrusionPointAlgorithm();

  // polyline management
  void reset();
  void addPoint(const Vec2f & pt, bool close);
  void insertPoint(size_t idx, const Vec2f & pt);
  void removePoint(size_t idx);
  size_t pointsCount() const;
  Vec2f & operator [] (size_t idx);
  void setCursorPt(const Vec2f & );
  const Vec2f & getCursorPt() const;
  bool isCursorPtValid() const;
  const Rect2f & getRect() const;
  bool isClosed() const;
  bool haveSelfIsect(const Vec2f & pt, Vec2f & r) const;

  // triangulation
  bool triangulation(std::vector<Triangle> *& tris);

private:

  bool triangulate();

  Points2f points_;
  Vec2f cursorPt_;
  Rect2f rect_;
  bool closed_;
  Triangles tris_;
  size_t pointCount_;

  void recalc();

};
