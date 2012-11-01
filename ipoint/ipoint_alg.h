#pragma once

#include "rect.h"
#include <vector>

class Triangulator;

struct Triangle
{
  Triangle()
  {
    v[0] = v[1] = v[2] = -1;
  }

  Triangle(int v0, int v1, int v2)
  {
    v[0] = v0;
    v[1] = v1;
    v[2] = v2;
  }

  int v[3];
};

class IntrusionPointAlgorithm
{
public:

  typedef std::vector<Vec2f> Points2f;

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
  std::vector<Triangle> tris_;

  void recalc();

};
