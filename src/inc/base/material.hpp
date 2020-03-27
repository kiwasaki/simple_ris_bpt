
#pragma once

#ifndef MATERIAL_HPP
#define MATERIAL_HPP

#include"rng.hpp"
#include"math.hpp"
#include"direction.hpp"
#include"intersection.hpp"

//brdf & material classes
//this implementation handles only diffuse BRDF and diffuse light

///////////////////////////////////////////////////////////////////////////////////////////////////
//brdf_sample
///////////////////////////////////////////////////////////////////////////////////////////////////

class brdf_sample
{
public:

	brdf_sample() : m_pdf()
	{
	}

	brdf_sample(const direction &w, const col3 &f, const float pdf) : m_w(w), m_f(f), m_pdf(pdf)
	{
		assert(pdf > 0);
	}

	const direction &w() const
	{
		return assert(is_valid()), m_w;
	}

	const col3 &f() const
	{
		return assert(is_valid()), m_f;
	}

	float pdf() const
	{
		return assert(is_valid()), m_pdf;
	}

	bool is_valid() const
	{
		return (m_pdf > 0);
	}
	bool is_invalid() const
	{
		return !(m_pdf > 0);
	}

private:

	direction m_w;
	col3 m_f;
	float m_pdf;
};

///////////////////////////////////////////////////////////////////////////////////////////////////
//brdf
///////////////////////////////////////////////////////////////////////////////////////////////////

class brdf
{
public:

	brdf(const intersection &x, const direction &w, const col3 &kd) : m_f(kd / PI()), m_n(x.n())
	{
		if(abs(m_n.x) < abs(m_n.y)){
			m_t = normalize(vec3(0, m_n.z, -m_n.y));
		}else{
			m_t = normalize(vec3(-m_n.z, 0, m_n.x));
		}
		m_b = cross(m_n, m_t);
	}
	brdf() = default;

	//return brdf
	//w: incident direction for path tracing, outgoing direction for light tracing
	col3 f(const direction &w) const
	{
		return assert(w.in_upper_hemisphere()), m_f;
	}

	//sample direction
	//sample incident direction for path tracing, outgoing direction for light tracing
	brdf_sample sample(random_number_generator &rng) const
	{
		const float u1 = rng.generate_uniform_real();
		const float u2 = rng.generate_uniform_real();
		const float ph = (2 * PI()) * u1;
		const float ct = sqrt(1 - u2);
		const float st = sqrt(u2);
		const float cp = cos(ph);
		const float sp = sin(ph);

		const direction w(
			m_t * (st * cp) + m_b * (st * sp) + m_n * (ct), m_n
		);
		return brdf_sample(w, f(w), pdf(w));
	}

	float pdf(const direction &w) const
	{
		return assert(w.in_upper_hemisphere()), w.abs_cos() / PI();
	}

private:

	col3 m_f;
	vec3 m_t;
	vec3 m_b;
	vec3 m_n;
};

///////////////////////////////////////////////////////////////////////////////////////////////////
//material
///////////////////////////////////////////////////////////////////////////////////////////////////

class material
{
public:

	material(const col3 &col, const bool is_emissive) : m_col(col), m_is_emissive(is_emissive)
	{
	}

	brdf make_brdf(const intersection &x, const direction &w) const
	{
		return is_emissive() ? brdf(x, w, col3(1)) : brdf(x, w, m_col);
	}

	col3 Le(const intersection &x, const direction &w) const
	{
		return assert(is_emissive()), m_col * brdf(x, w, col3(1)).f(w);
	}
	col3 Me() const
	{
		return assert(is_emissive()), m_col;
	}

	bool is_emissive() const
	{
		return m_is_emissive;
	}

private:

	col3 m_col;
	bool m_is_emissive;
};

///////////////////////////////////////////////////////////////////////////////////////////////////

#endif
