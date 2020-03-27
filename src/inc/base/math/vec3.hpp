
#pragma once

#ifndef VEC3_HPP
#define VEC3_HPP

#include<cmath>

///////////////////////////////////////////////////////////////////////////////////////////////////
//forward declaration
///////////////////////////////////////////////////////////////////////////////////////////////////

class vec3;
using col3 = vec3;

///////////////////////////////////////////////////////////////////////////////////////////////////
//vec3
///////////////////////////////////////////////////////////////////////////////////////////////////

class vec3
{
public:

	//constructors
	vec3() : x(), y(), z()
	{
	}
	vec3(const float s) : x(s), y(s), z(s)
	{
	}
	vec3(const float x, const float y, const float z) : x(x), y(y), z(z)
	{
	}
	
	//assignment operators
	vec3 &operator+=(const vec3 &v)
	{
		x += v.x; y += v.y; z += v.z; return *this;
	}
	vec3 &operator-=(const vec3 &v)
	{
		x -= v.x; y -= v.y; z -= v.z; return *this;
	}
	vec3 &operator*=(const vec3 &v)
	{
		x *= v.x; y *= v.y; z *= v.z; return *this;
	}
	vec3 &operator/=(const vec3 &v)
	{
		x /= v.x; y /= v.y; z /= v.z; return *this;
	}
	vec3 &operator*=(const float s)
	{
		x *= s; y *= s; z *= s; return *this;
	}
	vec3 &operator/=(const float s)
	{
		x /= s; y /= s; z /= s; return *this;
	}
	
	//binary operators
	vec3 operator+(const vec3 &v) const
	{
		return vec3(x + v.x, y + v.y, z + v.z);
	}
	vec3 operator-(const vec3 &v) const
	{
		return vec3(x - v.x, y - v.y, z - v.z);
	}
	vec3 operator*(const vec3 &v) const
	{
		return vec3(x * v.x, y * v.y, z * v.z);
	}
	vec3 operator/(const vec3 &v) const
	{
		return vec3(x / v.x, y / v.y, z / v.z);
	}
	vec3 operator*(const float s) const
	{
		return vec3(x * s, y * s, z * s);
	}
	vec3 operator/(const float s) const
	{
		return vec3(x / s, y / s, z / s);
	}
	friend vec3 operator*(const float s, const vec3 &v)
	{
		return v * s;
	}

	//unitary operators
	vec3 operator+() const
	{
		return *this;
	}
	vec3 operator-() const
	{
		return vec3(-x, -y, -z);
	}

	//accessor
	float &operator[](const size_t i)
	{
		return vals[i];
	}
	const float &operator[](const size_t i) const
	{
		return vals[i];
	}

public:

	union{
		struct{ 
			float x, y, z;
		};
		float vals[3];
	};
};

///////////////////////////////////////////////////////////////////////////////////////////////////
//function definitions
///////////////////////////////////////////////////////////////////////////////////////////////////

//dot product (inner product)
inline float dot(const vec3 &v1, const vec3 &v2)
{
	return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

//cross product (outer product)
inline vec3 cross(const vec3 &v1, const vec3 &v2)
{
	return vec3(v1.y * v2.z - v1.z * v2.y, v1.z * v2.x - v1.x * v2.z, v1.x * v2.y - v1.y * v2.x);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

//squared L2 norm
inline float squared_norm(const vec3 &v)
{
	return dot(v, v);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

//L2 norm
inline float norm(const vec3 &v)
{
	return sqrt(squared_norm(v));
}

///////////////////////////////////////////////////////////////////////////////////////////////////

//normalization
inline vec3 normalize(const vec3 &v)
{
	return v / norm(v);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

//convert RGB color into luminance
inline float luminance(const col3 &c)
{
	return 0.2126f * c[0] + 0.7152f * c[1] + 0.0722f * c[2];
}

///////////////////////////////////////////////////////////////////////////////////////////////////

#endif
