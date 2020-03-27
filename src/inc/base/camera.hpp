
#include<iostream>

#pragma once

#ifndef CAMERA_HPP
#define CAMERA_HPP

#include"ray.hpp"
#include"rng.hpp"

///////////////////////////////////////////////////////////////////////////////////////////////////
//camera
///////////////////////////////////////////////////////////////////////////////////////////////////

class camera
{
public:

	camera(const vec3 &eye, const vec3 &center, const int res_x, const int res_y, const float fovy, const float lens_radius = 1e-1f) : m_p(eye), m_res_x(res_x), m_res_y(res_y), m_fovy(fovy), m_lens_radius(lens_radius)
	{
		m_n = center - eye;
		m_focus = norm(m_n);
		m_n /= m_focus;

		const float theta = asin(m_n.y);
		const float phi = atan2(-m_n.x, -m_n.z);
		const float st = sin(theta);
		const float ct = cos(theta);
		const float sp = sin(phi);
		const float cp = cos(phi);
		m_b = vec3(st * sp, ct, st * cp);
		m_t = normalize(cross(m_n, m_b));

		const float aspect = res_x / float(res_y);
		m_screen_size_y = 2 * m_focus * tan(conv_deg_to_rad(fovy * 0.5f));
		m_screen_size_x = m_screen_size_y * aspect;
		m_screen_min = center - m_t * (m_screen_size_x * 0.5f) - m_b * (m_screen_size_y * 0.5f);

		m_stow = inv_look_at(eye, center, m_b) * inv_perspective(fovy, aspect, m_focus, m_focus * 1.1f) * inv_viewport(0, 0, res_x, res_y);

		if(lens_radius > 0){
			m_pdf_lens = 1 / (PI() * lens_radius * lens_radius);
		}else{
			m_pdf_lens = 1;
		}

		m_pdf_pixel = res_x * res_y * m_focus * m_focus / (m_screen_size_x * m_screen_size_y);
	}

	ray sample(const int x, const int y, random_number_generator &rng) const
	{
		const float u1 = rng.generate_uniform_real();
		const float u2 = rng.generate_uniform_real();
		const float u3 = rng.generate_uniform_real();
		const float u4 = rng.generate_uniform_real();

		const float r = m_lens_radius * sqrt(u1);
		const float th = 2 * PI() * u2;
		const float st = sin(th);
		const float ct = cos(th);
		const vec3 org = m_t * (r * ct) + m_b * (r * st) + m_p;

		const vec3 dst = mat_mul_vec_div_w(m_stow, vec4(x + u3, y + u4, 0, 1));
		const vec3 dir = normalize(dst - org);
		return ray(org, dir);
	}

	struct intersection{
		int x, y; bool is_valid;
	};
	intersection calc_intersection(const vec3 &lens_p, const direction &lens_wi) const
	{
		const float t = m_focus / lens_wi.cos();
		const vec3 screen_p = lens_p + vec3(lens_wi) * t;
		const int pixel_x = int(dot(screen_p - m_screen_min, m_t) * m_res_x / m_screen_size_x);
		const int pixel_y = int(dot(screen_p - m_screen_min, m_b) * m_res_y / m_screen_size_y);

		if((0 <= pixel_x) && (pixel_x < m_res_x)){
			if((0 <= pixel_y) && (pixel_y < m_res_y)){
				return intersection{pixel_x, pixel_y, true};
			}
		}
		return intersection{-1, -1, false};
	}

	float We(const direction &w) const
	{
		const float cos2 = pow(w.abs_cos(), 2);
		return m_pdf_lens * m_pdf_pixel / (cos2 * cos2);
	}

	float pdf_o() const
	{
		return m_pdf_lens;
	}

	float pdf_d(const direction &w) const
	{
		return m_pdf_pixel / (w.cos() * w.cos() * w.cos());
	}

	const vec3 &p() const
	{
		return m_p;
	}
	const vec3 &d() const
	{
		return m_n;
	}

	int res_x() const
	{
		return m_res_x;
	}
	int res_y() const
	{
		return m_res_y;
	}

	float fovy() const
	{
		return m_fovy;
	}

	float lens_radius() const
	{
		return m_lens_radius;
	}

private:

	vec3 m_p;
	vec3 m_t;
	vec3 m_b;
	vec3 m_n;
	mat4 m_stow;
	vec3 m_screen_min;
	int m_res_x;
	int m_res_y;
	float m_fovy;
	float m_focus;
	float m_pdf_lens;
	float m_pdf_pixel;
	float m_lens_radius;
	float m_screen_size_x;
	float m_screen_size_y;
};

///////////////////////////////////////////////////////////////////////////////////////////////////

#endif
