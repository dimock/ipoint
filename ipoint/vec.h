/********************************************************************
	created:	2004/08/27
	created:	27:8:2004   11:31
	filename: 	vec.h
	author:		Dimock
	
	purpose:	2D/3D vectors
*********************************************************************/

#pragma once

#include <math.h>
#include <vector>
#include <set>

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

    double length() const
    {
        return sqrt(x*x + y*y + z*z);
    }

    Vec3f & norm()
    {
        double r = 1.0f / length();
        x *= r; y *= r; z *= r;
        return *this;
    }

    Vec3f & scale(const Vec3f & s)
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


struct Triangle
{
  Triangle()
  {
    v[0] = v[1] = v[2] = -1;
  }

  Triangle(int v0, int v1, int v2)
  {
    v[0] = v0;
    v[1] = v1;
    v[2] = v2;
  }

  union
  {
    int v[3];
    struct
    {
      int x, y, z;
    };
  };
};


typedef std::vector<Vec3f> Points3f;
typedef std::vector<Triangle> Triangles;


