
#pragma once

#ifndef OUR_HPP
#define OUR_HPP

#include"our/path.hpp"

///////////////////////////////////////////////////////////////////////////////////////////////////

namespace our{

///////////////////////////////////////////////////////////////////////////////////////////////////
//renderer
//To simplify the implementation, we do not use the strategies (s>=2, t=0)
//////////////////////////////////////////////////////////////////////////////////////////////////

class renderer
{
public:

	//constructor ( M : number of pre-sampled light sub-paths, nt : number of threads )
	renderer(const scene &scene, const camera &camera, const size_t M, const size_t nt = std::thread::hardware_concurrency());

	//rendering
	imagef render(const scene &scene, const camera &camera);

private:

	//calculate radiance for pixel (x,y)
	col3 radiance(const int x, const int y, const scene &scene, const camera &camera, random_number_generator &rng);

	//calculate contributions of strategies (s=0,t>=2) (i.e., unidirectional path tracing from eye) for Line 10 of Algorithm1
	col3 calculate_0t(const scene &scene, const light_path &y, const camera_path &z);

	//calculate resampling estimators (i.e., strategy (s>=1, t>=2)) in Eq. (6) (Lines 11 to 23 of Algorithm1)
	col3 calculate_st(const scene &scene, const camera_path &z, random_number_generator &rng);

	//calculate contributions of strategies (s>=1,t=1) (i.e., light tracing) for Line 10 of Algorithm1
	void calculate_s1(const scene &scene, const camera &camera, const light_path &y, const camera_path &z, random_number_generator &rng);

private:

	size_t m_M;
	size_t m_nt;
	size_t m_ns1; //number of samples for strategy (s>=1,t=1), i.e., widthxheight of the image
	float m_Qp;   //normalization factor for virtual cache point (uniform distribution) in Sec. 5.2
	double m_sum; //sum of Qp for each iteration
	double m_ite; //number of iterations
	imagef m_buf_s1; //buffer to store contributions of strategy (s>=1,t=1) (i.e., light tracing)
	kd_tree<cache> m_caches; //cache points. we store cache points in the previous iteration to calculate the normalization factor Q
	std::unique_ptr<spinlock[]> m_locks; //spinlock for exclusive access to m_buf_s1
	std::vector<candidate> m_candidates; //pre-sampled light sub-paths ¥hat{Y} for resampling
	std::vector<light_path> m_light_paths; //light sub-paths for strategies handled by BPT
};

///////////////////////////////////////////////////////////////////////////////////////////////////

} //namespace our

///////////////////////////////////////////////////////////////////////////////////////////////////

#include"our/renderer-impl.hpp"

///////////////////////////////////////////////////////////////////////////////////////////////////

#endif
