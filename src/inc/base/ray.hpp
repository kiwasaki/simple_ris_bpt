
#pragma once

#ifndef RAY_HPP
#define RAY_HPP

#include"math.hpp"

///////////////////////////////////////////////////////////////////////////////////////////////////
//ray
///////////////////////////////////////////////////////////////////////////////////////////////////

class ray
{
public:

	//constructor (o: origin, d: direction (unit vector), t: distance from origin to intersection point)
	ray(const vec3 &o, const vec3 &d, const float t = FLT_MAX) : m_o(o), m_d(d), m_t(t)
	{
	}

	const vec3 &o() const
	{
		return m_o;
	}
	const vec3 &d() const
	{
		return m_d;
	}

	float t() const
	{
		return m_t;
	}
	float &t()
	{
		return m_t;
	}

	float t_min() const
	{
		return 1e-3f;
	}

private:

	vec3 m_o;
	vec3 m_d;
	float m_t;
};

///////////////////////////////////////////////////////////////////////////////////////////////////

#endif
