
#pragma once

#ifndef RANDOM_NUMBER_GENERATOR_HPP
#define RANDOM_NUMBER_GENERATOR_HPP

#include<random>

///////////////////////////////////////////////////////////////////////////////////////////////////
//random_number_generator
///////////////////////////////////////////////////////////////////////////////////////////////////

class random_number_generator
{
public:

	//constructor
	random_number_generator(const size_t seed = std::mt19937_64::default_seed) : m_engine(seed)
	{
	}

	//generate uniform random variable [0,1)
	float generate_uniform_real()
	{
		const float tmp = std::generate_canonical<float, size_t( -1 )>(m_engine);
		if(tmp < 1){
			return tmp;
		}else{
			return 1 - FLT_EPSILON * 0.5f;
		}
	}

	//generate uniform random variable of integers in [min,max]
	size_t generate_uniform_int(const size_t min, const size_t max)
	{
		return min + (m_engine() % (max - min + 1));
	}

private:

	std::mt19937_64 m_engine;
};

///////////////////////////////////////////////////////////////////////////////////////////////////

#endif
