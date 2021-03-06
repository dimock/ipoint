#pragma once

#include "rect.h"

namespace iMath
{

extern const double err;

bool edges_isect(const Vec3f & p0, const Vec3f & p1, const Vec3f & q0, const Vec3f & q1, Vec3f & r, double & dist);

bool edge_halfline_isect(const Vec3f & p0, const Vec3f & p1, const Vec3f & q, const Vec3f & rq, Vec3f & r, double & dist);

bool line_line_isect(const Vec3f & p, const Vec3f & rp, const Vec3f & q, const Vec3f & rq, Vec3f & r, double & dist);

bool edge_tri_isect(const Vec3f & ep0, const Vec3f & ep1, const Vec3f & tp0, const Vec3f & tp1, const Vec3f & tp2, Vec3f & ip);


// returns vectorized distance, v.length() = abs(dist), dir * cw > 0 means q is right to line
Vec3f dist_to_line(const Vec3f & p0, const Vec3f & p1, const Vec3f & q, bool & outside);

bool inside_tri(const Vec3f & p0, const Vec3f & p1, const Vec3f & p2, const Vec3f & q);

Vec3f cw_dir(const Vertices & verts);

void sincos(const Vec3f & r1, const Vec3f & r2, double & s, double & c);

inline double sqr2(double t)
{
  return t*t;
}

inline double sqr3(double t)
{
  return t*t*t;
}
}