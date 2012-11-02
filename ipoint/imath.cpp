#include "imath.h"


static const double err = 1e-20;

bool iMath::edges_isect(const Vec3f & p0, const Vec3f & p1, const Vec3f & q0, const Vec3f & q1, Vec3f & r)
{
  Vec3f rp = p1 - p0;
  Vec3f rq = q1 - q0;
  double lpq = rp.length()+rq.length();
  Vec3f n = rp ^ rq;
  if ( n.length() < err*lpq )
    return false;
  n.norm();
  double d = n*(q0-p0);
  Vec3f g0 = q0 - n*d;
  Vec3f g1 = q1 - n*d;
  d = rp.x*rq.y-rp.y*rq.x;
  if ( fabs(d) > err*lpq )
  {
    double t = (-p0.x*rq.y+g0.x*rq.y+rq.x*p0.y-rq.x*g0.y)/d; // p
    double u = (rp.x*p0.y-rp.x*g0.y-rp.y*p0.x+rp.y*g0.x)/d; // q
    if ( t < 0 || t > 1 || u < 0 || u > 1 )
      return false;
    r = p0 + rp*t;
    return true;
  }
  d = rp.x*rq.z-rp.z*rq.x;
  if ( fabs(d) < err*lpq )
    return false;
  double t = -(p0.x*rq.z-g0.x*rq.z-rq.x*p0.z+rq.x*g0.z)/d; // p
  double u = (rp.x*p0.z-rp.x*g0.z-rp.z*p0.x+rp.z*g0.x)/d; // q;
  if ( t < 0 || t > 1 || u < 0 || u > 1 )
    return false;
  r = p0 + rp*t;
  return true;
}

bool iMath::edge_halfline_isect(const Vec3f & p0, const Vec3f & p1, const Vec3f & q, const Vec3f & rq, Vec3f & r)
{
  Vec3f rp = p1 - p0;
  double lpq = rp.length()+rq.length();
  Vec3f n = rp ^ rq;
  if ( n.length() < err*lpq )
    return false;
  double d = n*(q-p0);
  Vec3f g = q-n*d;
  d = rp.x*rq.y-rp.y*rq.x;
  if ( fabs(d) > err*lpq )
  {
    double t = (-p0.x*rq.y+g.x*rq.y+rq.x*p0.y-rq.x*g.y)/d; // p
    double u = (rp.x*p0.y-rp.x*g.y-rp.y*p0.x+rp.y*g.x)/d; // q
    if ( t < 0 || t > 1 || u < 0 )
      return false;
    r = p0 + rp*t;
    return true;
  }
  d = rp.x*rq.z-rp.z*rq.x;
  if ( fabs(d) < err*lpq )
    return false;
  double t = -(p0.x*rq.z-g.x*rq.z-rq.x*p0.z+rq.x*g.z)/d; // p
  double u = (rp.x*p0.z-rp.x*g.z-rp.z*p0.x+rp.z*g.x)/d; // q;
  if ( t < 0 || t > 1 || u < 0 )
    return false;
  r = p0 + rp*t;
  return true;
}

bool iMath::line_line_isect(const Vec3f & p, const Vec3f & rp, const Vec3f & q, const Vec3f & rq, Vec3f & r)
{
  double lpq = rp.length() + rq.length();
  Vec3f n = rp ^ rq;
  if ( n.length() < err*lpq )
    return false;
  double d = n*(q-p);
  Vec3f g = q-n*d;
  d = rp.x*rq.y-rp.y*rq.x;
  if ( fabs(d) > err*lpq )
  {
    double t = (-p.x*rq.y+g.x*rq.y+rq.x*p.y-rq.x*g.y)/d; // p
    r = p + rp*t;
    return true;
  }
  d = rp.x*rq.z-rp.z*rq.x;
  if ( fabs(d) < err*lpq )
    return false;
  double t = -(p.x*rq.z-g.x*rq.z-rq.x*p.z+rq.x*g.z)/d; // p
  r = p + rp*t;
  return true;
}

double iMath::dist_to_line(const Vec3f & p0, const Vec3f & p1, const Vec3f & q, bool & outside)
{
  Vec3f dir01 = p1 - p0;
  double s = dir01.length();
  dir01.norm();
  Vec3f dir0q = q - p0;
  Vec3f cp = dir01 ^ dir0q;
  double t = dir01*dir0q;
  outside = t < 0 || t > s;
  return -cp.z;
}

bool iMath::inside_tri(const Vec3f & p0, const Vec3f & p1, const Vec3f & p2, const Vec3f & q, bool cw)
{
  Vec3f p01 = p1 - p0;
  Vec3f p12 = p2 - p1;
  Vec3f p20 = p0 - p2;

  Vec3f q0 = q - p0;
  Vec3f q1 = q - p1;
  Vec3f q2 = q - p2;

  Vec3f v0 = p01 ^ q0;
  Vec3f v1 = p12 ^ q1;
  Vec3f v2 = p20 ^ q2;

  if ( v0.z < 0 == cw && v1.z < 0 == cw && v2.z < 0 == cw )
    return true;

  return false;
}

bool iMath::cw(const Points3f & points)
{
  if ( points.size() < 3 )
    return false;

  double s = 0;
  const Vec3f & p0 = points[0];
  for (size_t i = 1; i < points.size(); ++i)
  {
    size_t j = i+1;
    if ( j >= points.size() )
      break;
    const Vec3f & p1 = points[i];
    const Vec3f & p2 = points[j];
    Vec3f v = (p1 - p0) ^ (p2 - p0);
    s += v.z;
  }
  return s < 0;
}
