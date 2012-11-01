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

struct Rect2f
{
  Vec2f vmin, vmax;

  Rect2f()
  {
    makeInvalid();
  }

  Rect2f(const Vec2f & vmin, const Vec2f & vmax)
  {
    this->vmin.set(vmin);
    this->vmax = vmax;
    validate();
  }

  Rect2f(const Rect2f & rect)
  {
    this->vmin.set(rect.vmin);
    this->vmax.set(rect.vmax);
    validate();
  }

  void makeInvalid()
  {
    vmin.x = vmin.y = vmin.x = DBL_MAX;
    vmax.x = vmax.y = vmax.x = -DBL_MAX;
  }

  void set(const Vec2f & orig, const Vec2f & dim)
  {
    vmin = orig;
    vmax = vmin + dim;
  }

  Rect2f & operator = (const Rect2f & rect)
  {
    this->vmin.set(rect.vmin);
    this->vmax.set(rect.vmax);
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

    if ( width() < MinSizes && height() > MinSizes )
    {
      vmin.x -= height() * .5;
      vmax.x += height() * .5;
    }
    else if ( width() > MinSizes && height() < MinSizes )
    {
      vmin.y -= width() *.5;
      vmax.y += width() *.5;
    }
  }

  void add(const Vec2f & v)
  {
    if ( v.x < vmin.x )
      vmin.x = v.x;
    if ( v.x > vmax.x )
      vmax.x = v.x;
    if ( v.y < vmin.y )
      vmin.y = v.y;
    if ( v.y > vmax.y )
      vmax.y = v.y;
  }

  void add(const Rect2f & rect)
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

  Vec2f dimension() const
  {
    return Vec2f(width(), height());
  }

  Vec2f origin() const
  {
    return vmin;
  }

  Vec2f center() const
  {
    Vec2f c = vmin + vmax;
    return c.scale(Vec2f(0.5, 0.5));
  }

  void scale(const Vec2f & s)
  {
    Vec2f c = center();
    Vec2f d = dimension();

    d.scale(s);
    d.scale(Vec2f(0.5, 0.5));

    vmin = c - d;
    vmax = c + d;
  }

  void inflate(const Vec2f & d)
  {
    vmin -= d;
    vmax += d;
  }

  void move(const Vec2f & d)
  {
    vmin += d;
    vmax += d;
  }
};

struct Screen2f
{
  // positions of the screen in world space
  Rect2f rect;

  // size in screen units (ex. pixels)
  Vec2f  size;

  // if TRUE - 0 is upper point (as default bitmap)
  bool topbottom;

  Screen2f(const Screen2f & screen)
  {
    this->rect = screen.rect;
    this->size = screen.size;
    this->topbottom = screen.topbottom;
  }

  Screen2f()
  {
    this->topbottom = true;
  }

  Screen2f(const Rect2f & rect, const Vec2f & size, bool topbottom = true)
  {
    this->rect = rect;
    this->size = size;
    this->topbottom = topbottom;
  }

  bool isValid() const
  {
    return rect.isValid() && size.x > MinSizes && size.y > MinSizes;
  }

  Vec2f toScreen(const Vec2f & v) const
  {
    if ( !isValid() )
         return Vec2f();

    Vec2f pt = v - rect.origin();
    pt.scale(rect.dimension().rcpr());

    if ( topbottom )
      pt.y = 1.0 - pt.y;

    return pt.scale(size);
  }

  Vec2f toWorld(const Vec2f & v) const
  {
    if ( !isValid() )
         return rect.origin();

    Vec2f pt = v;

    pt.scale(size.rcpr());
   
    if ( topbottom )
      pt.y = 1.0 - pt.y;

    pt.scale(rect.dimension());

    pt += rect.origin();

    return pt;
  }

  Vec2f deltaToScreen(const Vec2f & d) const
  {
    if ( !isValid() )
      return Vec2f();

    Vec2f dpt = d;
    dpt.scale(rect.dimension().rcpr());

    if ( topbottom )
      dpt.y = 1.0 - dpt.y;

    return dpt.scale(size);
  }

  Vec2f deltaToWorld(const Vec2f & d) const
  {
    if ( !isValid() )
      return Vec2f();

    Vec2f dv = d;
    dv.scale(size.rcpr());

    if ( topbottom )
      dv.y = - dv.y;

    return dv.scale(rect.dimension());
  }

  void moveInWorld(const Vec2f & wd)
  {
    if ( !isValid() )
      return;

    rect.move(wd);
  }

  void moveInScreen(const Vec2f & sd)
  {
    if ( !isValid() )
      return;

    rect.move(deltaToWorld(sd));
  }
};
