/********************************************************************
	created:	2004/08/27
	created:	27:8:2004   11:31
	filename: 	vec.h
	author:		Dimock
	
	purpose:	2D/3D vectors
*********************************************************************/

#pragma once

#include <math.h>

/************************************************************************/
/* 2D Vector                                                            */
/************************************************************************/

struct Vec2f
{
    double x, y;

    Vec2f() : x(0), y(0)
    {}

    Vec2f(const double x, const double y)
    {
        this->x = x;
        this->y = y;
    }

    void set(const double x, const double y)
    {
        this->x = x;
        this->y = y;
    }

    void set(const Vec2f & v)
    {
      set(v.x, v.y);
    }

    Vec2f operator - () const
    {
        return Vec2f(-x, -y);
    }

    Vec2f operator + (const Vec2f & v) const
    {
        return Vec2f(x + v.x, y + v.y);
    }


    Vec2f operator - (const Vec2f & v) const
    {
        return Vec2f(x - v.x, y - v.y);
    }

    Vec2f & operator += (const Vec2f & v)
    {
        x += v.x;
        y += v.y;
        return *this;
    }

    Vec2f & operator -= (const Vec2f & v)
    {
        x -= v.x;
        y -= v.y;
        return *this;
    }

    double operator * (const Vec2f & v) const
    {
        return x*v.x + y*v.y;
    }

    Vec2f operator * (const double f) const
    {
        return Vec2f(x*f, y*f);
    }

    Vec2f & operator *= (const double f)
    {
        x *= f; y *= f;
        return *this;
    }

    double vecmod() const
    {
        return sqrt(x*x + y*y);
    }

    double vecmod2() const
    {
      return x*x + y*y;
    }

    Vec2f & norm()
    {
        double r = 1.0f/vecmod();
        x *= r; y *= r;
        return *this;
    }

    Vec2f & scale(const Vec2f & s)
    {
        this->x *= s.x;
        this->y *= s.y;
        return *this;
    }

    Vec2f rcpr() const
    {
        return Vec2f(1.0f/x, 1.0f/y);
    }
};

/************************************************************************/
/* 3D Vector                                                            */
/************************************************************************/

struct Vec3f
{
    double x, y, z;

    Vec3f() : x(0), y(0), z(0)
    {}

    Vec3f(const double x, const double y, const double z)
    {
        this->x = x;
        this->y = y;
        this->z = z;
    }

    void set(const double x, const double y, const double z)
    {
        this->x = x;
        this->y = y;
        this->z = z;
    }

    void set(const Vec3f & v)
    {
      set(v.x, v.y, v.z);
    }

    Vec3f operator + (const Vec3f & v) const
    {
        return Vec3f(x + v.x, y + v.y, z + v.z);
    }


    Vec3f operator - (const Vec3f & v) const
    {
        return Vec3f(x - v.x, y - v.y, z - v.z);
    }

    Vec3f & operator += (const Vec3f & v)
    {
        x += v.x;
        y += v.y;
        z += v.z;
        return *this;
    }

    Vec3f & operator -= (const Vec3f & v)
    {
        x -= v.x;
        y -= v.y;
        z -= v.z;
        return *this;
    }

    double operator * (const Vec3f & v) const
    {
        return x*v.x + y*v.y + z*v.z;
    }

    Vec3f operator ^ (const Vec3f & v) const
    {
        return Vec3f(y*v.z-z*v.y, z*v.x-x*v.z, x*v.y-y*v.x);
    }

    Vec3f operator * (const double f) const
    {
        return Vec3f(x*f, y*f, z*f);
    }

    Vec3f & operator *= (const double f)
    {
        x *= f; y *= f; z *= f;
        return *this;
    }

    double vecmod() const
    {
        return sqrt(x*x + y*y + z*z);
    }

    Vec3f & norm()
    {
        double r = 1.0f / vecmod();
        x *= r; y *= r; z *= r;
        return *this;
    }

    Vec3f & scale(Vec3f & s)
    {
        this->x *= s.x;
        this->y *= s.y;
        this->z *= s.z;
        return *this;
    }

    Vec3f rcpr() const
    {
        return Vec3f(1.0f/x, 1.0f/y, 1.0f/z);
    }
};
