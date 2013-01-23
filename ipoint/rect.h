/********************************************************************
	created:	2004/09/29
	created:	29:9:2004   19:39
	file   :	rect
	author :	Dimock
	
	purpose:	Rectangle
*********************************************************************/

#pragma once

#include "vec.h"
#include <xutility>
#include "float.h"

#define MinSizes 1e-10

struct Rect3f
{
  Vec3f vmin, vmax;

  Rect3f()
  {
    makeInvalid();
  }

  Rect3f(const Vec3f & vmin, const Vec3f & vmax)
  {
    this->vmin = vmin;
    this->vmax = vmax;
    validate();
  }

  Rect3f(const Rect3f & rect)
  {
    this->vmin = rect.vmin;
    this->vmax = rect.vmax;
    validate();
  }

  void makeInvalid()
  {
    vmin.x = vmin.y = vmin.z = DBL_MAX;
    vmax.x = vmax.y = vmax.z = -DBL_MAX;
  }

  void set(const Vec3f & orig, const Vec3f & dim)
  {
    vmin = orig;
    vmax = vmin + dim;
  }

  Rect3f & operator = (const Rect3f & rect)
  {
    this->vmin = rect.vmin;
    this->vmax = rect.vmax;
    validate();
    return *this;
  }

  bool isValid() const
  {
    return width() > MinSizes && height() > MinSizes;
  }

  void validate()
  {
    if ( vmin.x > vmax.x )
      std::swap(vmin.x, vmax.x);

    if ( vmin.y > vmax.y )
      std::swap(vmin.y, vmax.y);

    if ( vmin.z > vmax.z )
      std::swap(vmin.z, vmax.z);
  }

  bool pointInside(const Vec3f & p) const
  {
    return (vmin.x <= p.x && p.x <= vmax.x) &&
           (vmin.y <= p.y && p.y <= vmax.y) &&
           (vmin.z <= p.z && p.z <= vmax.z);
  }

  bool rectInside(const Rect3f & r) const
  {
    return pointInside(r.vmin) && pointInside(r.vmax);
  }

  bool intersecting(const Rect3f & r) const
  {
    return ( r.vmin.x <= vmax.x && r.vmax.x >= vmin.x ) &&
           ( r.vmin.y <= vmax.y && r.vmax.y >= vmin.y ) &&
           ( r.vmin.z <= vmax.z && r.vmax.z >= vmin.z );
  }

  Rect3f octant(int i) const
  {
    Vec3f c = center();

    switch ( i )
    {
    case 0:
      return Rect3f(vmin, c);

    case 1:
      return Rect3f(Vec3f(c.x, vmin.y, vmin.z), Vec3f(vmax.x, c.y, c.z));

    case 2:
      return Rect3f(Vec3f(c.x, c.y, vmin.z), Vec3f(vmax.x, vmax.y, c.z));

    case 3:
      return Rect3f(Vec3f(vmin.x, c.y, vmin.z), Vec3f(c.x, vmax.y, c.z));

    case 4:
      return Rect3f(Vec3f(vmin.x, vmin.y, c.z), Vec3f(c.x, c.y, vmax.z));

    case 5:
      return Rect3f(Vec3f(c.x, vmin.y, c.z), Vec3f(vmax.x, c.y, vmax.z));

    case 6:
      return Rect3f(Vec3f(c.x, c.y, c.z), Vec3f(vmax.x, vmax.y, vmax.z));

    case 7:
      return Rect3f(Vec3f(vmin.x, c.y, c.z), Vec3f(c.x, vmax.y, vmax.z));
    }

    return Rect3f();
  }

  void add(const Vec3f & v)
  {
    if ( v.x < vmin.x )
      vmin.x = v.x;
    if ( v.x > vmax.x )
      vmax.x = v.x;

    if ( v.y < vmin.y )
      vmin.y = v.y;
    if ( v.y > vmax.y )
      vmax.y = v.y;

    if ( v.z < vmin.z )
      vmin.z = v.z;
    if ( v.z > vmax.z )
      vmax.z = v.z;
  }

  void add(const Rect3f & rect)
  {
    add(rect.vmin);
    add(rect.vmax);
  }

  double width() const
  {
    return vmax.x - vmin.x;
  }

  double height() const
  {
    return vmax.y - vmin.y;
  }

  double depth() const
  {
    return vmax.z - vmin.z;
  }

  Vec3f dimension() const
  {
    return Vec3f(width(), height(), depth());
  }

  Vec3f diagonal() const
  {
    return vmax - vmin;
  }

  Vec3f origin() const
  {
    return vmin;
  }

  Vec3f center() const
  {
    Vec3f c = vmin + vmax;
    return c *= 0.5;
  }

  void scale(const Vec3f & s)
  {
    Vec3f c = center();
    Vec3f d = dimension();

    d.scale(s);
    d *= 0.5;

    vmin = c - d;
    vmax = c + d;
  }

  void inflate(const Vec3f & d)
  {
    vmin -= d;
    vmax += d;
  }

  void move(const Vec3f & d)
  {
    vmin += d;
    vmax += d;
  }
};

struct Screen
{
  // positions of the screen in world space
  Rect3f rect;

  // size in screen units (ex. pixels)
  Vec3f  size;

  // if TRUE - 0 is upper point (as default bitmap)
  bool topbottom;

  Screen() : topbottom(true)
  {
  }

  Screen(const Rect3f & rect, const Vec3f & size, bool topbottom = true)
  {
    this->rect = rect;
    this->size = size;
    this->topbottom = topbottom;
  }

  bool isValid() const
  {
    return rect.isValid() && size.x > MinSizes && size.y > MinSizes;
  }

  Vec3f toScreen(const Vec3f & v) const
  {
    if ( !isValid() )
         return Vec3f();

    Vec3f pt = v - rect.origin();
    pt.scale(rect.dimension().rcpr());

    if ( topbottom )
      pt.y = 1.0 - pt.y;

    return pt.scale(size);
  }

  Vec3f toWorld(const Vec3f & v) const
  {
    if ( !isValid() )
         return rect.origin();

    Vec3f pt = v;

    pt.scale(size.rcpr());
   
    if ( topbottom )
      pt.y = 1.0 - pt.y;

    pt.scale(rect.dimension());

    pt += rect.origin();

    return pt;
  }

  Vec3f deltaToScreen(const Vec3f & d) const
  {
    if ( !isValid() )
      return Vec3f();

    Vec3f dpt = d;
    dpt.scale(rect.dimension().rcpr());

    if ( topbottom )
      dpt.y = 1.0 - dpt.y;

    return dpt.scale(size);
  }

  Vec3f deltaToWorld(const Vec3f & d) const
  {
    if ( !isValid() )
      return Vec3f();

    Vec3f dv = d;
    dv.scale(size.rcpr());

    if ( topbottom )
      dv.y = - dv.y;

    return dv.scale(rect.dimension());
  }

  void moveInWorld(const Vec3f & wd)
  {
    if ( !isValid() )
      return;

    rect.move(wd);
  }

  void moveInScreen(const Vec3f & sd)
  {
    if ( !isValid() )
      return;

    rect.move(deltaToWorld(sd));
  }
};
