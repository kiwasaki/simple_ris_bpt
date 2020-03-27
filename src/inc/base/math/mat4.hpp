
#pragma once

#ifndef MAT4_HPP
#define MAT4_HPP

#include"vec4.hpp"

///////////////////////////////////////////////////////////////////////////////////////////////////
//mat4
/*/////////////////////////////////////////////////////////////////////////////////////////////////
column-major 4x4 matrix class
/////////////////////////////////////////////////////////////////////////////////////////////////*/

class mat4
{
public:

	//constructors
	mat4() : m_c1(), m_c2(), m_c3(), m_c4()
	{
	}
	mat4(const vec4 &c1, const vec4 &c2, const vec4 &c3, const vec4 &c4) : m_c1(c1), m_c2(c2), m_c3(c3), m_c4(c4)
	{
	}
	mat4(
		const float _11, const float _12, const float _13, const float _14, 
		const float _21, const float _22, const float _23, const float _24, 
		const float _31, const float _32, const float _33, const float _34, 
		const float _41, const float _42, const float _43, const float _44) : m_c1(_11, _21, _31, _41), m_c2(_12, _22, _32, _42), m_c3(_13, _23, _33, _43), m_c4(_14, _24, _34, _44)
	{
	}
	
	//assignment operators
	mat4 &operator+=(const mat4 &m)
	{
		m_c1 += m.m_c1; m_c2 += m.m_c2; m_c3 += m.m_c3; m_c4 += m.m_c4; return *this;
	}
	mat4 &operator-=(const mat4 &m)
	{
		m_c1 -= m.m_c1; m_c2 -= m.m_c2; m_c3 -= m.m_c3; m_c4 -= m.m_c4; return *this;
	}
	mat4 &operator*=(const mat4 &m)
	{
		const vec4 c1 = (*this) * m.m_c1;
		const vec4 c2 = (*this) * m.m_c2;
		const vec4 c3 = (*this) * m.m_c3;
		const vec4 c4 = (*this) * m.m_c4;
		m_c1 = c1; m_c2 = c2; m_c3 = c3; m_c4 = c4; return *this;
	}
	mat4 &operator*=(const float s)
	{
		m_c1 *= s; m_c2 *= s; m_c3 *= s; m_c4 *= s; return *this;
	}
	mat4 &operator/=(const float s)
	{
		m_c1 /= s; m_c2 /= s; m_c3 /= s; m_c4 /= s; return *this;
	}
	
	//binary operators
	mat4 operator+(const mat4 &m) const
	{
		return mat4(m_c1 + m.m_c1, m_c2 + m.m_c2, m_c3 + m.m_c3, m_c4 + m.m_c4);
	}
	mat4 operator-(const mat4 &m) const
	{
		return mat4(m_c1 - m.m_c1, m_c2 - m.m_c2, m_c3 - m.m_c3, m_c4 - m.m_c4);
	}
	mat4 operator*(const mat4 &m) const
	{
		return mat4((*this) * m.m_c1, (*this) * m.m_c2, (*this) * m.m_c3, (*this) * m.m_c4);
	}
	vec4 operator*(const vec4 &v) const
	{
		return vec4(
			m_c1.x * v.x + m_c2.x * v.y + m_c3.x * v.z + m_c4.x * v.w, 
			m_c1.y * v.x + m_c2.y * v.y + m_c3.y * v.z + m_c4.y * v.w, 
			m_c1.z * v.x + m_c2.z * v.y + m_c3.z * v.z + m_c4.z * v.w, 
			m_c1.w * v.x + m_c2.w * v.y + m_c3.w * v.z + m_c4.w * v.w
		);
	}
	mat4 operator*(const float s) const
	{
		return mat4(m_c1 * s, m_c2 * s, m_c3 * s, m_c4 * s);
	}
	mat4 operator/(const float s) const
	{
		return mat4(m_c1 / s, m_c2 / s, m_c3 / s, m_c4 / s);
	}
	friend mat4 operator*(const float s, const mat4 &m)
	{
		return m * s;
	}

	//unitary operators
	mat4 operator+() const
	{
		return *this;
	}
	mat4 operator-() const
	{
		return mat4(-m_c1, -m_c2, -m_c3, -m_c4);
	}

private:
	
	vec4 m_c1;
	vec4 m_c2;
	vec4 m_c3;
	vec4 m_c4;
};

///////////////////////////////////////////////////////////////////////////////////////////////////
//function definition
///////////////////////////////////////////////////////////////////////////////////////////////////


inline vec3 mat_mul_vec_div_w(const mat4 &m, const vec4 &v)
{
	const vec4 tmp = m * v; 
	return vec3(tmp) / tmp.w;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

//calculate view matrix
inline mat4 look_at(const vec3 &eye, const vec3 &center, const vec3 &up)
{
	const vec3 axis_z = normalize(eye - center);
	const vec3 axis_x = normalize(cross(up, axis_z));
	const vec3 axis_y = cross(axis_z, axis_x);

	return mat4(
		axis_x.x, axis_x.y, axis_x.z, -dot(eye, axis_x),
		axis_y.x, axis_y.y, axis_y.z, -dot(eye, axis_y),
		axis_z.x, axis_z.y, axis_z.z, -dot(eye, axis_z), 0, 0, 0, 1
	);
}


//calculate inverse matrix of view matrix
inline mat4 inv_look_at(const vec3 &eye, const vec3 &center, const vec3 &up)
{
	const vec3 axis_z = normalize(eye - center);
	const vec3 axis_x = normalize(cross(up, axis_z));
	const vec3 axis_y = cross(axis_z, axis_x);

	return mat4(
		axis_x.x, axis_y.x, axis_z.x, eye.x, 
		axis_x.y, axis_y.y, axis_z.y, eye.y, 
		axis_x.z, axis_y.z, axis_z.z, eye.z, 0, 0, 0, 1
	);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

//calculate perspective projection matrix
inline mat4 perspective(const float fovy, const float aspect, const float z_near, const float z_far)
{
	const float pi = 2 * acos(0.0f);
	const float rad = fovy * (pi / 180);
	const float cot = 1 / tan(rad * 0.5f);
	const float inv_f_n = 1 / (z_far - z_near);

	return mat4(
		cot / aspect,    0,                           0,                               0,
			       0,  cot,                           0,                               0,
			       0,    0, -(z_far + z_near) * inv_f_n, -(2 * z_far * z_near) * inv_f_n,
			       0,    0,                          -1,                               0
	);
}


//calculate inverse matrix of perspective-projection matrix
inline mat4 inv_perspective(const float fovy, const float aspect, const float z_near, const float z_far)
{
	const float pi = 2 * acos(0.0f);
	const float rad = fovy * (pi / 180);
	const float tan_ = tan(rad * 0.5f);
	const float mP_33 = -(z_far + z_near) / (z_far - z_near);
	const float inv_mP_34 = -(z_far - z_near) / (2 * z_far * z_near);

	return mat4(
		aspect * tan_,    0,         0,                 0,
		            0, tan_,         0,                 0, 
		            0,    0,         0,                -1,
		            0,    0, inv_mP_34, mP_33 * inv_mP_34
	);
}

///////////////////////////////////////////////////////////////////////////////////////////////////


//calculate viewport transformation matrix
inline mat4 viewport(const int x, const int y, const int width, const int height)
{
	const float sx = 0.5f * width;
	const float sy = 0.5f * height;
	const float sz = 0.5f;

	return mat4(
		sx,  0,  0, sx + x,
		 0, sy,  0, sy + y,
		 0,  0, sz, sz + 0,
		 0,  0,  0,      1
	);
}

//calculate inverse matrix of viewport transformation matrix
inline mat4 inv_viewport(const int x, const int y, const int width, const int height)
{
	const float sx = 2.0f / width;
	const float sy = 2.0f / height;
	const float sz = 2.0f;

	return mat4(
		sx,  0,  0, -x * sx - 1, 
		 0, sy,  0, -y * sy - 1,
		 0,  0, sz,         - 1,
		 0,  0,  0,           1
	);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

#endif
