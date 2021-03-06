
#pragma once

#ifndef MATH_HPP
#define MATH_HPP

#include<cfloat>
#include<cassert>

#include"math/vec3.hpp"
#include"math/vec4.hpp"
#include"math/mat4.hpp"

///////////////////////////////////////////////////////////////////////////////////////////////////
//function definitions
///////////////////////////////////////////////////////////////////////////////////////////////////

inline float PI()
{
	return 3.14159265358979323846f;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

//convert radian to degree
inline float conv_rad_to_deg(const float rad)
{
	return rad * (180 / PI());
}

///////////////////////////////////////////////////////////////////////////////////////////////////

//convert degree to radian
inline float conv_deg_to_rad(const float deg)
{
	return deg * (PI() / 180);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

//clamp val in [min,max]
inline float clamp(const float val, const float min, const float max)
{
	return (val < min) ? min : (val > max) ? max : val;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

#endif
