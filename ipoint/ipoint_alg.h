#pragma once

#include "rect.h"
#include <vector>
#include <QObject>

class Triangulator;


class IntrusionPointAlgorithm : public QObject
{
  Q_OBJECT

public:

  IntrusionPointAlgorithm();

  // polyline management
  void reset();
  void addPoint(const Vec3f & pt, bool close);
  void insertPoint(size_t idx, const Vec3f & pt);
  void removePoint(size_t idx);
  size_t pointsCount() const;
  Vec3f & operator [] (size_t idx);
  void setCursorPt(const Vec3f & );
  const Vec3f & getCursorPt() const;
  bool isCursorPtValid() const;
  const Rect3f & getRect() const;
  bool isClosed() const;
  bool haveSelfIsect(const Vec3f & pt, Vec3f & r) const;

  // triangulation
  bool triangulation(std::vector<Triangle> *& tris);

signals:

  void trianglesChanged(size_t);

private:

  bool triangulate();

  Points3f points_;
  Vec3f cursorPt_;
  Rect3f rect_;
  bool closed_;
  Triangles tris_;
  size_t pointCount_;

  void recalc();

};
