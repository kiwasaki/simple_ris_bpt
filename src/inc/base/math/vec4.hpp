
#pragma once

#ifndef VEC4_HPP
#define VEC4_HPP

#include"vec3.hpp"

///////////////////////////////////////////////////////////////////////////////////////////////////
//vec4 (four-dimensional vector class)
///////////////////////////////////////////////////////////////////////////////////////////////////

class vec4
{
public:

	//constructors
	vec4() : x(), y(), z(), w()
	{
	}
	vec4(const float s) : x(s), y(s), z(s), w(s)
	{
	}
	vec4(const vec3 &v, const float w) : x(v.x), y(v.y), z(v.z), w(w)
	{
	}
	vec4(const float x, const float y, const float z, const float w) : x(x), y(y), z(z), w(w)
	{
	}
	
	//assignment operators
	vec4 &operator+=(const vec4 &v)
	{
		x += v.x; y += v.y; z += v.z; w += v.w; return *this;
	}
	vec4 &operator-=(const vec4 &v)
	{
		x -= v.x; y -= v.y; z -= v.z; w -= v.w; return *this;
	}
	vec4 &operator*=(const vec4 &v)
	{
		x *= v.x; y *= v.y; z *= v.z; w *= v.w; return *this;
	}
	vec4 &operator/=(const vec4 &v)
	{
		x /= v.x; y /= v.y; z /= v.z; w /= v.w; return *this;
	}
	vec4 &operator*=(const float s)
	{
		x *= s; y *= s; z *= s; w *= s; return *this;
	}
	vec4 &operator/=(const float s)
	{
		x /= s; y /= s; z /= s; w /= s; return *this;
	}
	
	//binary operators
	vec4 operator+(const vec4 &v) const
	{
		return vec4(x + v.x, y + v.y, z + v.z, w + v.w);
	}
	vec4 operator-(const vec4 &v) const
	{
		return vec4(x - v.x, y - v.y, z - v.z, w - v.w);
	}
	vec4 operator*(const vec4 &v) const
	{
		return vec4(x * v.x, y * v.y, z * v.z, w * v.w);
	}
	vec4 operator/(const vec4 &v) const
	{
		return vec4(x / v.x, y / v.y, z / v.z, w / v.w);
	}
	vec4 operator*(const float s) const
	{
		return vec4(x * s, y * s, z * s, w * s);
	}
	vec4 operator/(const float s) const
	{
		return vec4(x / s, y / s, z / s, w / s);
	}
	friend vec4 operator*(const float s, const vec4 &v)
	{
		return v * s;
	}

	//unitary operator
	vec4 operator+() const
	{
		return *this;
	}
	vec4 operator-() const
	{
		return vec4(-x, -y, -z, -w);
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

	//cast to three-dimensional vector class vec3
	explicit operator vec3() const
	{
		return vec3(x, y, z);
	}

public:

	union{
		struct{ 
			float x, y, z, w;
		};
		float vals[4];
	};
};

///////////////////////////////////////////////////////////////////////////////////////////////////
//function definitions
///////////////////////////////////////////////////////////////////////////////////////////////////

//dot product (inner product)
inline float dot(const vec4 &v1, const vec4 &v2)
{
	return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z + v1.w * v2.w;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

//squared L2 norm
inline float squared_norm(const vec4 &v)
{
	return dot(v, v);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

//L2 norm
inline float norm(const vec4 &v)
{
	return sqrt(squared_norm(v));
}

///////////////////////////////////////////////////////////////////////////////////////////////////

//normalization
inline vec4 normalize(const vec4 &v)
{
	return v / norm(v);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

#endif
