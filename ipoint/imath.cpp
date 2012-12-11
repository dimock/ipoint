#include "imath.h"


extern const double iMath::err = 1e-10;

bool iMath::edges_isect(const Vec3f & p0, const Vec3f & p1, const Vec3f & q0, const Vec3f & q1, Vec3f & r, double & dist)
{
  Vec3f rp = p1 - p0;
  Vec3f rq = q1 - q0;
  double lpq = rp.length()+rq.length();
  Vec3f n = rp ^ rq;
  if ( n.length() < err*lpq )
    return false;
  n.normalize();
  dist = n*(q0-p0);
  Vec3f g0 = q0 - n*dist;
  Vec3f g1 = q1 - n*dist;
  double d = rp.x*rq.y-rp.y*rq.x;
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

bool iMath::edge_halfline_isect(const Vec3f & p0, const Vec3f & p1, const Vec3f & q, const Vec3f & rq, Vec3f & r, double & dist)
{
  Vec3f rp = p1 - p0;
  double lpq = rp.length()+rq.length();
  Vec3f n = rp ^ rq;
  if ( n.length() < err*lpq )
    return false;
  dist = n*(q-p0);
  Vec3f g = q-n*dist;
  double d = rp.x*rq.y-rp.y*rq.x;
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

bool iMath::line_line_isect(const Vec3f & p, const Vec3f & rp, const Vec3f & q, const Vec3f & rq, Vec3f & r, double & dist)
{
  double lpq = rp.length() + rq.length();
  Vec3f n = rp ^ rq;
  if ( n.length() < err*lpq )
    return false;
  dist = n*(q-p);
  Vec3f g = q-n*dist;
  double d = rp.x*rq.y-rp.y*rq.x;
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

Vec3f iMath::dist_to_line(const Vec3f & p0, const Vec3f & p1, const Vec3f & q, bool & outside)
{
  outside = false;
  Vec3f dir01 = p1 - p0;
  double s = dir01.length();
  if ( s < err )
    return Vec3f(0, 0, (p0-q).length());

  dir01.normalize();
  Vec3f dir0q = q - p0;
  Vec3f cp = dir01 ^ dir0q;
  double t = dir01*dir0q;
  outside = t < 0 || t > s;
  return cp;
}

bool iMath::inside_tri(const Vec3f & p0, const Vec3f & p1, const Vec3f & p2, const Vec3f & q)
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

  bool b0 = v0 * v1 > 0;
  bool b2 = v0 * v2 > 0;

  return b0 && b2;
}

Vec3f iMath::cw_dir(const Vertices & verts)
{
  if ( verts.size() < 3 )
    return Vec3f();

  Vec3f cw_dir;
  const Vec3f & p0 = verts[0].p();
  for (size_t i = 1; i < verts.size(); ++i)
  {
    size_t j = i+1;
    if ( j >= verts.size() )
      break;
    const Vec3f & p1 = verts[i].p();
    const Vec3f & p2 = verts[j].p();
    Vec3f v = (p1 - p0) ^ (p2 - p0);
    cw_dir += v;
  }
  return cw_dir;
}

void iMath::sincos(const Vec3f & r1, const Vec3f & r2, double & s, double & c)
{
  c = r1 * r2;
  s = sqrt(1.0 - c*c);
}

#include <fstream>

bool iMath::edge_tri_isect(const Vec3f & ep0, const Vec3f & ep1, const Vec3f & tp0, const Vec3f & tp1, const Vec3f & tp2, Vec3f & ip)
{
  Vec3f r = ep1 - ep0;
  if ( r.length() < err )
    return false;

  Vec3f n = (tp1 - tp0) ^ (tp2 - tp0);
  if ( n.length() < err )
    return false;
  n.normalize();

  double d = -n*tp0;
  double w = r*n;
  if ( fabs(w) < err )
    return false;

  double t = -(d + ep0*n)/w;
  ip = r*t + ep0;
  if ( t < 0.0 || t > 1.0 )
    return false;

  bool ok = inside_tri(tp0, tp1, tp2, ip);
  if ( ok )
  {
    std::ofstream ofs("D:\\Scenes\\3dpad\\isect.txt");
    Vec3f color(0,1,0);
    const char * meshName = "EdgeIsect";

    ofs << "Mesh \"" << meshName << "\" {\n";

    ofs << "  Wireframe {\n";
    ofs << "    ( true )\n";
    ofs << "  }\n";

    ofs << "  Shaded {\n";
    ofs << "    ( true )\n";
    ofs << "  }\n";

    ofs << "  DefaultColor {\n";
    ofs << "    ( " << color.x << ", " << color.y << ", " << color.z << " )\n";
    ofs << "  }\n";

    ofs << "  Coords {\n";
    {
      ofs << "    ( " << tp0.x << ", " << tp0.y << ", " << tp0.z << " )\n";
      ofs << "    ( " << tp1.x << ", " << tp1.y << ", " << tp1.z << " )\n";
      ofs << "    ( " << tp2.x << ", " << tp2.y << ", " << tp2.z << " )\n";
    }
    ofs << "  }\n";


    ofs << "  Faces {\n";
    {
      ofs << "    ( 0, 1, 2 )\n";
    }
    ofs << "  }\n";

    ofs << "}\n";

    ofs << "Edges \"Edge\" {\n";

    ofs << "  { (" << ep0.x << ", " << ep0.y << ", " << ep0.z << ") (" << ep1.x << ", " << ep1.y << ", " << ep1.z <<") (1, 0, 0) }\n";

    ofs << "}\n";
  }
  return ok;
}
