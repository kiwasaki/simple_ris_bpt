
#pragma once

#ifndef DISTRIBUTION_HPP
#define DISTRIBUTION_HPP

#include<vector>
#include<algorithm>

#include"rng.hpp"

///////////////////////////////////////////////////////////////////////////////////////////////////
//distribution
///////////////////////////////////////////////////////////////////////////////////////////////////

template<class T> class distribution
{
public:

	//elems: set of elements to construct distribution, weight: function object that returns weight
	//distribution::sample samples element proportional to weight
	//distribution::normalization_constant returns sum of weight
	template<class Weight> distribution(std::vector<T> elems, Weight weight) : m_cdf(elems.size() + 1)
	{
		double sum = 0;
		for(size_t i = 0, n = elems.size(); i < n; i++){
			m_cdf[i] = float(sum); sum += weight(elems[i]);
		}

		const float inv_sum = float(1 / sum);
		for(size_t i = 0, n = elems.size(); i < n; i++){
			m_cdf[i] *= inv_sum;
		}
		m_cdf.back() = 1;
		m_elems = std::move(elems);
		m_normalization_constant = float(sum);
	}
	distribution() : m_normalization_constant()
	{
	}


	//p_elem : address of sampled element, pmf: sampling probability
	struct sample_t{
		const T *p_elem; float pmf;
	};
	sample_t sample(random_number_generator &rng) const
	{
		const size_t idx = std::upper_bound(m_cdf.begin(), m_cdf.end(), rng.generate_uniform_real()) - m_cdf.begin() - 1; //二分探索
		return sample_t{ &m_elems[idx], m_cdf[idx + 1] - m_cdf[idx] };
	}

	//return pmf to sample idx-th element
	float pmf(const size_t idx) const
	{
		return assert(idx < m_elems.size()), m_cdf[idx + 1] - m_cdf[idx];
	}

	float normalization_constant() const
	{
		return m_normalization_constant;
	}

	typename std::vector<T>::iterator begin()
	{
		return m_elems.begin();
	}
	typename std::vector<T>::iterator end()
	{
		return m_elems.end();
	}
	typename std::vector<T>::const_iterator begin() const
	{
		return m_elems.begin();
	}
	typename std::vector<T>::const_iterator end() const
	{
		return m_elems.end();
	}

private:

	std::vector<T> m_elems;
	std::vector<float> m_cdf;
	float m_normalization_constant;
};

///////////////////////////////////////////////////////////////////////////////////////////////////

#endif
