
#pragma once

#ifndef OUR_PATH_HPP
#define OUR_PATH_HPP

#include"path_vertex.hpp"

///////////////////////////////////////////////////////////////////////////////////////////////////

namespace our{

///////////////////////////////////////////////////////////////////////////////////////////////////
//constant parameters
///////////////////////////////////////////////////////////////////////////////////////////////////

//parameter for russian roulette
//generation of (rr_threshold+1)-th vertex of sub-path may be terminated.
const size_t rr_threshold = 5;

//clamping parameter for geometry term G in cache point
const float G_max = 1e6f;

//clamping parameter epsilon in Sec. 5.1
const float mis_threshold = 1e-3f;

///////////////////////////////////////////////////////////////////////////////////////////////////
//forward declaration
///////////////////////////////////////////////////////////////////////////////////////////////////

class light_path;
class camera_path;

///////////////////////////////////////////////////////////////////////////////////////////////////
//light_path
///////////////////////////////////////////////////////////////////////////////////////////////////

class light_path
{
public:

	void construct(const scene &scene, random_number_generator &rng, const kd_tree<cache> &caches);

	//zi : z(i), zip1: z(i+1), FGVc: array to store F(brdf)*GV at neighbor cache points of z(i)
	static std::tuple<float, float, col3> pdfs_FG(const scene &scene, const camera_path_vertex &zi, const camera_path_vertex &zip1, std::array<col3, Nc> &FGVc);

	//return sampling pdf of z(i) from z(i+1) (n: z(i) is n-th vertex from light source)
	static float pdf(const camera_path_vertex &zi, const camera_path_vertex &zip1, const size_t n);

	//ztm1: z(t-1), ysm1: y(s-1), n: z(t-1) is n-th vertex from light source, zy: direction from z(t-1) to y(s-1), yz: direction from y(s-1) to z(t-1)
	static std::tuple<float, col3> pdf_FG(const camera_path_vertex &ztm1, const light_path_vertex &ysm1, const size_t n, const direction &zy, const direction &yz);

	//ztm2: z(t-2), ztm1: z(t-1), n: z(t-2) is n-th vertex from light source, zy: direction from z(t-1) to y(s-1), FGVc: array to store FGV
	static std::tuple<float, col3> pdf_FG(const scene &scene, const camera_path_vertex &ztm2, const camera_path_vertex &ztm1, const size_t n, const direction &zy, std::array<col3, Nc> &FGVc);

	//return MIS partial weight (yz : direction from y(s-1) to z(t-1), zy: direction from z(t-1) to y(s-1), Qp : normalization factor for virtual cache point)
	static float mis_partial_weight(const light_path &y, const size_t s, const camera_path &z, const size_t t, const direction &yz, const direction &zy, const float M, const float Qp);

	//return number of vertices
	size_t num_vertices() const
	{
		return m_vertices.size() - 1; //decrement to exclude dummy vertex
	}

	//return path vertex
	light_path_vertex &operator()(const size_t i)
	{
		return m_vertices[i + 1]; //increment to exclude dummy vertex
	}
	const light_path_vertex &operator()(const size_t i) const
	{
		return m_vertices[i + 1];
	}

private:

	std::vector<light_path_vertex> m_vertices;
};

///////////////////////////////////////////////////////////////////////////////////////////////////
//camera_path
///////////////////////////////////////////////////////////////////////////////////////////////////

class camera_path
{
public:

	//x,y: pixel coordinate
	void construct(const scene &scene, const camera &camera, const int x, const int y, random_number_generator &rng);

	//x,y: pixel coordinate, caches: cache points
	void construct(const scene &scene, const camera &camera, const int x, const int y, random_number_generator &rng, const kd_tree<cache> &caches);

	//return sampling pdfs (with RR and without RR) of y(i) from y(i+1)
	static std::tuple<float, float> pdfs(const light_path_vertex &yi, const light_path_vertex &yip1);

	//return sampling pdf of y(i) from y(i+1) (n: y(i) is n-th vertex from eye)
	static float pdf(const light_path_vertex &yi, const light_path_vertex &yip1, const size_t n);

	//return sampling pdf of y(s-1) from z(t-1) (n: y(s-1) is n-th vertex from eye)
	static float pdf(const light_path_vertex &ysm1, const camera_path_vertex &ztm1, const size_t n, const direction &yz, const direction &zy);

	//return sampling pdf of y(i+1) from y(i) (n: y(s-2) is n-th vertex from eye, yz: direction from y(s-1) to z(t-1))
	static float pdf(const light_path_vertex &ysm2, const light_path_vertex &ysm1, const size_t n, const direction &yz);

	//return MIS partial weight (yz/zy directions from y(s-1)/z(t-1) to z(t-1)/y(s-1), Qp: normalization factor for virtual cache point)
	static float mis_partial_weight(const scene &scene, const light_path &y, const size_t s, const camera_path &z, const size_t t, const direction &yz, const direction &zy, const float M, const float Qp);

	size_t num_vertices() const
	{
		return m_vertices.size();
	}

	//return i-th vertex
	camera_path_vertex &operator()(const size_t i)
	{
		return m_vertices[i];
	}
	const camera_path_vertex &operator()(const size_t i) const
	{
		return m_vertices[i];
	}

private:

	size_t m_ns1; //number of samples for strategies (s>=1,t=1) (i.e., widthxheight)
	std::vector<camera_path_vertex> m_vertices;
};

///////////////////////////////////////////////////////////////////////////////////////////////////
//candidate (each pre-sampled light sub-path in ¥hat{Y})
///////////////////////////////////////////////////////////////////////////////////////////////////

class candidate
{
public:

    //i: index of vertex
	candidate(const light_path &path, const size_t i) : mp_path(&path), m_i(i)
	{
	}
	candidate() : mp_path()
	{
	}

	size_t s() const
	{
		return assert(mp_path != nullptr), m_i + 1;
	}
	const light_path &path() const
	{
		return assert(mp_path != nullptr), *mp_path;
	}
	const light_path_vertex &vertex() const
	{
		return assert(mp_path != nullptr), (*mp_path)(m_i);
	}

private:

	const light_path *mp_path; 
	size_t m_i;
};

///////////////////////////////////////////////////////////////////////////////////////////////////
//cache
///////////////////////////////////////////////////////////////////////////////////////////////////

class cache : public distribution<candidate>, protected camera_path_vertex
{
public:

	//v: eye sub-path vertex, first_iteration: flag to detect whether first iteration or not
	cache(const camera_path_vertex &v, const bool first_iteration);

	//construct resampling pmf (candidates: pre-sampled light sub-paths)
	void calc_distribution(const scene &scene, const std::vector<candidate> &candidates, const size_t M);

	//calculate F(brdf)*G(geo term)*V(visibility) at cache point
	col3 calc_FGV(const scene &scene, const ::intersection &x, const ::brdf &brdf) const;

	//return estimate of Q (normalization factor of target distribution)
	float Q() const
	{
		return m_Q;
	}

	using camera_path_vertex::intersection;

private:

	float m_Z; //normalization factor estimated using light sub-paths in current iteration
	float m_Q; //normalization factor estimated using light sub-paths in previous iteration
};

inline float rr_probability(const col3 &f, const float cos, const float pdf)
{
	return std::min(luminance(f) * cos / pdf, 1.0f);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

} //namespace our

///////////////////////////////////////////////////////////////////////////////////////////////////

#include"path/cache-impl.hpp"
#include"path/light_path-impl.hpp"
#include"path/camera_path-impl.hpp"

///////////////////////////////////////////////////////////////////////////////////////////////////

#endif
