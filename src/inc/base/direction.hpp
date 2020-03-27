
#pragma once

#ifndef DIRECTION_HPP
#define DIRECTION_HPP

#include"math.hpp"

///////////////////////////////////////////////////////////////////////////////////////////////////
//direction
///////////////////////////////////////////////////////////////////////////////////////////////////

class direction
{
public:

	//constructor
	direction() : m_is_valid()
	{
	}
	direction(const vec3 &w, const vec3 &n) : m_w(w), m_cos(dot(w, n)), m_abs_cos(abs(m_cos))
	{
		//invalidate directions of grazing angle
		m_is_valid = (m_abs_cos >= 1e-6f);
	}
	explicit direction(const vec3 &w) : m_w(w), m_cos(1), m_abs_cos(1), m_is_valid(true)
	{
	}

	//return direction
	operator vec3() const
	{
		return assert(is_valid()), m_w;
	}
	vec3 operator-() const
	{
		return assert(is_valid()), -m_w;
	}

	//return cosine
	float cos() const
	{
		return assert(is_valid()), m_cos;
	}
	float abs_cos() const
	{
		return assert(is_valid()), m_abs_cos;
	}

	bool in_upper_hemisphere() const
	{
		return assert(is_valid()), (m_cos > 0);
	}
	bool in_lower_hemisphere() const
	{
		return !(in_upper_hemisphere());
	}

	bool is_valid() const
	{
		return m_is_valid;
	}
	bool is_invalid() const
	{
		return !(is_valid());
	}

private:

	vec3 m_w;
	float m_cos;
	float m_abs_cos;
	bool m_is_valid;
};

///////////////////////////////////////////////////////////////////////////////////////////////////

#endif
