
#pragma once

#ifndef SPHERE_HPP
#define SPHERE_HPP

#include"ray.hpp"
#include"rng.hpp"
#include"intersection.hpp"

///////////////////////////////////////////////////////////////////////////////////////////////////
//sphere
///////////////////////////////////////////////////////////////////////////////////////////////////

class sphere
{
public:

	//constructor c : center of sphere, r : radius of sphere
	sphere(const vec3 &c, const float r) : m_c(c), m_r(r)
	{
	}

	//calculate intersection point (isect) between ray (o,d) and sphere
	bool calc_intersection(const vec3 &o, const vec3 &d, float &t_max, const float t_min, intersection &isect) const
	{
		if(intersect(o, d, t_max, t_min) == false){
			return false;
		}

		const vec3 p = o + d * t_max;

		//direct normal toward origin o
		vec3 n = normalize(p - m_c);
		if(dot(n, d) > 0){
			n = -n;
		}
		isect = intersection(p, n, nullptr);
		return true;
	}

	//ray sphere intersection test
	//if ray intersects sphere, distance from origin o to intersection point is stored in t_max
	bool intersect(const vec3 &o, const vec3 &d, float &t_max, const float t_min) const
	{
		const vec3 co = o - m_c;

		const float A = dot(d, d);
		const float B = dot(d, co);
		const float C = dot(co, co) - m_r * m_r;

		const float D = B * B - A * C;
		if(D <= 0){
			return false;
		}

		const float sqrt_D = sqrt(D);
		const float inv_A = 1 / A;
		const float t1 = (-B - sqrt_D) * inv_A;
		const float t2 = (-B + sqrt_D) * inv_A;

		float t;
		if(t1 > t_min){
			t = t1;
		}else if(t2 > t_min){
			t = t2;
		}else{
			return false;
		}
		if(t >= t_max){
			return false;
		}else{
			t_max = t;
			return true;
		}
	}

	//uniform sampling of sphere
	sample_point sample(random_number_generator &rng) const
	{
		const float u1 = rng.generate_uniform_real();
		const float u2 = rng.generate_uniform_real();

		const float ph = u1 * 2 * PI();
		const float ct = u2 * 2 - 1;
		const float st = sqrt(1 - ct * ct);
		const float cp = cos(ph);
		const float sp = sin(ph);
		const vec3 n(st * cp, st * sp, ct);
		return sample_point(n * m_r + m_c, n, nullptr, 1 / area());
	}

	//calculate surface area of sphere
	float area() const
	{
		return 4 * PI() * m_r * m_r;
	}

private:

	vec3 m_c;
	float m_r;
};

///////////////////////////////////////////////////////////////////////////////////////////////////

#endif
