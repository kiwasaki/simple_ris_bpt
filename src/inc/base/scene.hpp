
#pragma once

#ifndef SCENE_HPP
#define SCENE_HPP

#include"object.hpp"
#include"distribution.hpp"

///////////////////////////////////////////////////////////////////////////////////////////////////
//scene
///////////////////////////////////////////////////////////////////////////////////////////////////

class scene
{
public:

	scene(std::vector<object> objs)
	{
		//construct distribution to sample points on light sources
		m_objs = distribution<object>(std::move(objs), [](const object &obj){ return obj.light_power(); });
	}

	//calculate intersection
	intersection calc_intersection(ray &r) const
	{
		intersection isect;
		for(const auto &obj : m_objs){
			obj.calc_intersection(r, isect);
		}
		return isect;
	}

	//visibility test
	bool intersect(const ray &r) const
	{
		const ray r_(r.o(), r.d(), r.t() * (1 - 1e-3f));
		for(const auto &obj : m_objs){
			if(obj.intersect(r_)){
				return true;
			}
		}
		return false;
	}

	//point sampling of light sources in the scene
	sample_point sample_light(random_number_generator &rng) const
	{
		//sample sphere proportional to areaxflux
		const auto s1 = m_objs.sample(rng);

		//uniformly sampling point on sphere
		const sample_point s2 = s1.p_elem->sample(rng);

		const float pdf = s1.pmf * s2.pdf();
		return sample_point(s2.p(), s2.n(), &s2.material(), pdf);
	}

	//calculate pdf of intersection point x
	float pdf_light(const intersection &x) const
	{
		return luminance(x.material().Me()) / m_objs.normalization_constant();
	}

private:

	distribution<object> m_objs;
};

///////////////////////////////////////////////////////////////////////////////////////////////////

#endif
