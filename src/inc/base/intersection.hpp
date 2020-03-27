
#pragma once

#ifndef INTERSECTION_HPP
#define INTERSECTION_HPP

#include"math.hpp"

///////////////////////////////////////////////////////////////////////////////////////////////////
//forward declaration
///////////////////////////////////////////////////////////////////////////////////////////////////

class material;

///////////////////////////////////////////////////////////////////////////////////////////////////
//intersection
///////////////////////////////////////////////////////////////////////////////////////////////////

class intersection
{
public:

	intersection() : m_is_valid(), mp_mtl()
	{
	}

	//p: intersection point, n: normal at p, p_mtl: address of mtl for p
	intersection(const vec3 &p, const vec3 &n, const material *p_mtl) : m_p(p), m_n(n), m_is_valid(true), mp_mtl(p_mtl)
	{
	}
	
	//return position/normal
	const vec3 &p() const
	{
		return assert(is_valid()), m_p;
	}
	const vec3 &n() const
	{
		return assert(is_valid()), m_n;
	}

	//return material
	const material &material() const
	{
		return assert(is_valid() && (mp_mtl != nullptr)), *mp_mtl;
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
	
	vec3 m_p;
	vec3 m_n;
	bool m_is_valid;
	const ::material *mp_mtl;
};

///////////////////////////////////////////////////////////////////////////////////////////////////
//sample_point
///////////////////////////////////////////////////////////////////////////////////////////////////

class sample_point : public intersection
{
public:

	sample_point() : m_pdf()
	{
	}

    //p: sampled position, n: normal at p, p_mtl: address of material at p, pdf: sampling pdf
	sample_point(const vec3 &p, const vec3 &n, const ::material *p_mtl, const float pdf) : intersection(p, n, p_mtl), m_pdf(pdf)
	{
		assert(pdf > 0);
	}

	float pdf() const
	{
		return m_pdf;
	}

private:

	float m_pdf;
};

///////////////////////////////////////////////////////////////////////////////////////////////////

#endif
