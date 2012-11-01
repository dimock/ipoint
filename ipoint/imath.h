#pragma once

#include "rect.h"

bool edges_isect(const Vec2f & p0, const Vec2f & p1, const Vec2f & q0, const Vec2f & q1, Vec2f & r);

bool edge_halfline_isect(const Vec2f & p0, const Vec2f & p1, const Vec2f & q, const Vec2f & rq, Vec2f & r);

bool line_line_isect(const Vec2f & p, const Vec2f & rp, const Vec2f & q, const Vec2f & rq, Vec2f & r);


// returns signed distance, dist > 0 means q-line is on right side of p-line
double dist_to_line(const Vec2f & p0, const Vec2f & p1, const Vec2f & q, bool & outside);

bool inside_tri(const Vec2f & p0, const Vec2f & p1, const Vec2f & p2, const Vec2f & q, bool cw);

bool cw(const Points2f & points);