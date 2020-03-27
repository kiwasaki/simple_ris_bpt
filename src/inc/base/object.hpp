
#pragma once

#ifndef OBJECT_HPP
#define OBJECT_HPP

#include"sphere.hpp"
#include"material.hpp"

///////////////////////////////////////////////////////////////////////////////////////////////////
//object
///////////////////////////////////////////////////////////////////////////////////////////////////

class object
{
public:

	object(const sphere &sph, const material &mtl) : m_sph(sph), m_mtl(mtl)
	{
	}

	bool calc_intersection(ray &r, intersection &isect) const
	{
		if(m_sph.calc_intersection(r.o(), r.d(), r.t(), r.t_min(), isect)){
			isect = intersection(isect.p(), isect.n(), &m_mtl);
			return true;
		}else{
			return false;
		}
	}

	bool intersect(const ray &r) const
	{
		float t_max = r.t();
		return m_sph.intersect(r.o(), r.d(), t_max, r.t_min());
	}

	sample_point sample(random_number_generator &rng) const
	{
		sample_point sample = m_sph.sample(rng);
		return sample_point(sample.p(), sample.n(), &m_mtl, sample.pdf());
	}

	float light_power() const
	{
		return m_mtl.is_emissive() ? luminance(m_mtl.Me()) * m_sph.area() : 0;
	}

private:

	sphere m_sph;
	material m_mtl;
};

///////////////////////////////////////////////////////////////////////////////////////////////////

#endif
