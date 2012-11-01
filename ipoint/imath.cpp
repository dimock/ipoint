#include "imath.h"

bool edges_isect(const Vec2f & p0, const Vec2f & p1, const Vec2f & q0, const Vec2f & q1, Vec2f & r)
{
  Vec2f rp = p1 - p0;
  Vec2f rq = q1 - q0;
  double d = rp.x*rq.y-rp.y*rq.x;
  if ( d < 1e-20 && d > -1e-20 )
    return false;
  double t = (-p0.x*rq.y+q0.x*rq.y+rq.x*p0.y-rq.x*q0.y)/d; // p
  double u = (rp.x*p0.y-rp.x*q0.y-rp.y*p0.x+rp.y*q0.x)/d; // q
  if ( t < 0 || t > 1 || u < 0 || u > 1 )
    return false;
  r = p0 + rp*t;
  return true;
}

bool edge_halfline_isect(const Vec2f & p0, const Vec2f & p1, const Vec2f & q, const Vec2f & rq, Vec2f & r)
{
  Vec2f rp = p1 - p0;
  double d = rp.x*rq.y-rp.y*rq.x;
  if ( d < 1e-20 && d > -1e-20 )
    return false;
  double t = (-p0.x*rq.y+q.x*rq.y+rq.x*p0.y-rq.x*q.y)/d; // p
  double u = (rp.x*p0.y-rp.x*q.y-rp.y*p0.x+rp.y*q.x)/d; // q
  if ( t < 0 || t > 1 || u < 0 )
    return false;
  r = p0 + rp*t;
  return true;
}

bool line_line_isect(const Vec2f & p, const Vec2f & rp, const Vec2f & q, const Vec2f & rq, Vec2f & r)
{
  double d = rp.x*rq.y-rp.y*rq.x;
  if ( d < 1e-20 && d > -1e-20 )
    return false;
  double t = (-p.x*rq.y+q.x*rq.y+rq.x*p.y-rq.x*q.y)/d; // p
  r = p + rp*t;
  return true;
}

double dist_to_line(const Vec2f & p0, const Vec2f & p1, const Vec2f & q, bool & outside)
{
  Vec3f dir01(p1.x-p0.x, p1.y-p0.y, 0);
  double s = dir01.vecmod();
  dir01.norm();
  Vec3f dir0q(q.x-p0.x, q.y-p0.y, 0);
  Vec3f cp = dir01 ^ dir0q;
  double t = dir01*dir0q;
  outside = t < 0 || t > s;
  return -cp.z;
}

bool inside_tri(const Vec2f & p0, const Vec2f & p1, const Vec2f & p2, const Vec2f & q, bool cw)
{
  Vec3f pp0(p0.x, p0.y, 0);
  Vec3f pp1(p1.x, p1.y, 0);
  Vec3f pp2(p2.x, p2.y, 0);
  Vec3f qq(q.x, q.y, 0);

  Vec3f p01 = pp1 - pp0;
  Vec3f p12 = pp2 - pp1;
  Vec3f p20 = pp0 - pp2;

  Vec3f q0 = qq - pp0;
  Vec3f q1 = qq - pp1;
  Vec3f q2 = qq - pp2;

  Vec3f v0 = p01 ^ q0;
  Vec3f v1 = p12 ^ q1;
  Vec3f v2 = p20 ^ q2;

  if ( v0.z < 0 == cw && v1.z < 0 == cw && v2.z < 0 == cw )
    return true;

  return false;
}

bool cw(const Points2f & points)
{
  if ( points.size() < 3 )
    return false;

  double s = 0;
  Vec3f p0(points[0].x, points[0].y, 0);
  for (size_t i = 1; i < points.size(); ++i)
  {
    size_t j = i+1;
    if ( j >= points.size() )
      break;
    Vec3f p1(points[i].x, points[i].y, 0);
    Vec3f p2(points[j].x, points[j].y, 0);
    Vec3f v = (p1 - p0) ^ (p2 - p0);
    s += v.z;
  }
  return s < 0;
}
