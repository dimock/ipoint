#pragma once

#include "rect.h"

namespace iMath
{

bool edges_isect(const Vec3f & p0, const Vec3f & p1, const Vec3f & q0, const Vec3f & q1, Vec3f & r, double & dist);

bool edge_halfline_isect(const Vec3f & p0, const Vec3f & p1, const Vec3f & q, const Vec3f & rq, Vec3f & r, double & dist);

bool line_line_isect(const Vec3f & p, const Vec3f & rp, const Vec3f & q, const Vec3f & rq, Vec3f & r, double & dist);


// returns signed distance, dist > 0 means q-line is on right side of p-line
double dist_to_line(const Vec3f & p0, const Vec3f & p1, const Vec3f & q, bool & outside);

bool inside_tri(const Vec3f & p0, const Vec3f & p1, const Vec3f & p2, const Vec3f & q, bool cw);

bool cw(const Points3f & points);

}