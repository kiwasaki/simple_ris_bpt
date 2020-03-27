
///////////////////////////////////////////////////////////////////////////////////////////////////

namespace our{

///////////////////////////////////////////////////////////////////////////////////////////////////
//cache
///////////////////////////////////////////////////////////////////////////////////////////////////

//constructor (v: eye sub-path vertex, first_iteration: flag (true for 1st iteration, false otherwise)
inline cache::cache(const camera_path_vertex &v, const bool first_iteration) : camera_path_vertex(v)
{
	if(first_iteration){
		m_Q = -1;//for first iteration, normalization factor Q will be estimated in calc_distribution
	}
	else{
		//estimate of Q is approximated using Q estimated in previous iteration
		m_Q = 0;
		for(size_t i = 0; i < Nc; i++){
			m_Q += v.neighbor_cache(i).m_Z; //m_Z is estimate of Q using ¥bar{Y}_{n-1} stored at neighbor cache points
		}
		m_Q /= Nc;
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////

//construct resampling pmf
inline void cache::calc_distribution(const scene &scene, const std::vector<candidate> &candidates, const size_t M)
{
	//construct resampling pmf (q*/p) (Line 5 in Algorithm1)
	auto weight = [&](const candidate &c){ 
		return luminance(c.vertex().Le_throughput() * calc_FGV(scene, c.vertex().intersection(), c.vertex().brdf())); 
	};
	distribution<candidate>::operator=(
		distribution<candidate>(candidates, weight)
	);

	//estimate Q using M pre-sampled light sub-paths in current iteration
	//m_Z is used in the next iteration (Line 6 in Algorithm1)
	m_Z = normalization_constant() / M; //normalization_constant=sum(q*/p)

	//for first iteration ¥hat{Y}_1 is used
	if(m_Q == -1){
		m_Q = m_Z;
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////

//calculate F(brdf)*G(geo. term)*V(visibility) at cache point
inline col3 cache::calc_FGV(const scene &scene, const ::intersection &x, const ::brdf &brdf) const
{
	const auto &c_isect = camera_path_vertex::intersection();

	const vec3 tmp_wo = c_isect.p() - x.p();
	const float dist2 = squared_norm(tmp_wo);
	const float dist = sqrt(dist2);
	const direction wo(tmp_wo / dist, x.n());
	if(wo.is_invalid() || wo.in_lower_hemisphere()){
		return col3();
	}

	const direction wi(-wo, c_isect.n());
	if(wi.is_invalid() || wi.in_lower_hemisphere()){
		return col3();
	}
	
	//visibility test for V
	if(scene.intersect(ray(c_isect.p(), wi, dist)) == false){
		//clamp G term to avoid unstable estimation of Q
		//(for glossy BRDFs, it would be better to clamp F*G instead of G only)
		return brdf.f(wo) * std::min(wi.abs_cos() * wo.abs_cos() / dist2, G_max);
	}
	return col3();
}

///////////////////////////////////////////////////////////////////////////////////////////////////

} //namespace our

///////////////////////////////////////////////////////////////////////////////////////////////////
