
///////////////////////////////////////////////////////////////////////////////////////////////////

namespace our{

///////////////////////////////////////////////////////////////////////////////////////////////////
//light_path
///////////////////////////////////////////////////////////////////////////////////////////////////

//construct light sub-path
inline void light_path::construct(const scene &scene, random_number_generator &rng, const kd_tree<cache> &caches)
{
	m_vertices.clear();

	//initialize m_vertices using "dummy" path vertex to avoid out of range access when MIS is calculated, ptr to scene is stored in material
	m_vertices.emplace_back(intersection(vec3(), vec3(), reinterpret_cast<const material*>(&scene)), brdf(), direction(), direction(), col3(), 0.0f);

	//sample point on light source
	const sample_point lsample = scene.sample_light(rng);
	if(lsample.is_invalid()){
		return;
	}

	//sample outgoing direction
	const brdf lbrdf = lsample.material().make_brdf(lsample, direction(lsample.n()));
	const brdf_sample bsample = lbrdf.sample(rng);

	//add path vertex
	col3 Le_throughput = lsample.material().Me() / lsample.pdf();
	if(bsample.is_invalid()){
		m_vertices.emplace_back(lsample, lbrdf, direction(lsample.n()), direction(), Le_throughput, lsample.pdf());
		return;
	}else{
		m_vertices.emplace_back(lsample, lbrdf, direction(lsample.n()), bsample.w(), Le_throughput, lsample.pdf());
	}

	//generate path
	float pdf = bsample.pdf();
	ray r(lsample.p(), bsample.w());
	while(true){
	
		//intersection test
		const intersection isect = scene.calc_intersection(r);
		if(isect.is_invalid()){
			break;
		}

		const direction wi(-r.d(), isect.n());
		if(wi.is_invalid()){
			break;
		}
	
		//convert to area measure
		pdf *= wi.abs_cos() / (r.t() * r.t());
	
		//if isect is on light source terminate tracing
		if(isect.material().is_emissive()){
			break;
		}
	
		//sample direction
		const brdf brdf = isect.material().make_brdf(isect, wi);
		const brdf_sample sample = brdf.sample(rng);

		//add path vertex
		if(sample.is_invalid()){
			m_vertices.emplace_back(isect, brdf, wi, direction(), Le_throughput, pdf);
			break;
		}else{
			m_vertices.emplace_back(isect, brdf, wi, sample.w(), Le_throughput, pdf);
		}

		//russian roulette
		if(num_vertices() >= rr_threshold){
			
			const float q = rr_probability(sample.f(), sample.w().abs_cos(), sample.pdf());
			if(rng.generate_uniform_real() < q){
				pdf = sample.pdf() * q;
			}else{
				break;
			}
		}else{
			pdf = sample.pdf();
		}

		//update ray/throughput weight
		r = ray(isect.p(), sample.w());
		Le_throughput *= sample.f() * sample.w().abs_cos() / pdf;
	}

	//precompute variables for MIS weights
	{
		//search neighbor cache points
		thread_local std::vector<neighbor<cache>> neighbors;
		for(size_t i = 1, n = num_vertices(); i < n; i++){

			auto &yi = operator()(i);
			caches.find_nearest(yi.intersection().p(), FLT_MAX, Nc, neighbors);

			for(size_t j = 0; j < Nc; j++){
				yi.set_neighbor_cache(j, *neighbors[j]);
			}
		}

		//calculate backward pdfs from eye
		for(size_t i = 0, n = num_vertices(); i + 2 < n; i++){
			auto &yi = operator()(i);
			auto &yip1 = operator()(i + 1);
			auto pdfs = camera_path::pdfs(yi, yip1);
			yi.set_pdf_bwd(std::get<0>(pdfs)); //RRなしのPDF
			yi.set_pdf_bwd_rr(std::get<1>(pdfs)); //RRありのPDF
		}

		//set q*/p
		for(size_t i = 1, n = num_vertices(); i < n; i++){
		
			auto &yi = operator()(i);
			auto &yim1 = operator()(i - 1);

			for(size_t j = 0; j < Nc; j++){
				yi.set_Le_throughput_FGVc(j, yim1.Le_throughput() * yi.neighbor_cache(j).calc_FGV(scene, yim1.intersection(), yim1.brdf()));
			}
		}
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////}

//calculate MIS partial weight (yz: direction from y(s-1) to z(t-1), zy: direction from z(t-1) to y(s-1), Qp: normalization factor for virtual cache point)
inline float light_path::mis_partial_weight(const light_path &y, const size_t s, const camera_path &z, const size_t t, const direction &yz, const direction &zy, const float M, const float Qp)
{
	float w = 0;
	for(size_t i = 0; i < s; i++){

		if(i == 0){
			w += 1;
		}else{
			for(size_t j = 0; j < Nc; j++){

				const float Q = y(i).neighbor_cache(j).Q();
				const float Le_throughput_FGVc = y(i).Le_throughput_FGVc(j);
					
				if(Le_throughput_FGVc > 0){ 
					w += (1 / float(Nc + 1)) * M / ((M - 1) * std::max(mis_threshold, Q / Le_throughput_FGVc) + 1);
				}
			}
			w += (1 / float(Nc + 1)) * M / ((M - 1) * Qp + 1);
		}
		if(i == s - 1){
			w *= camera_path::pdf(y(s - 1), z(t - 1), (s - i) + t, yz, zy);
		}else if(i == s - 2){
			w *= camera_path::pdf(y(s - 2), y(s - 1), (s - i) + t, yz);
		}else{
			w *= camera_path::pdf(y(i), y(i + 1), (s - i) + t);
		}
		w /= y(i).pdf_fwd();
	}
	return w;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

//return pdfs (without/with RR) and FG, and calculate FGV at neighbor cache points of z(i)
//zi: z(i), zip1: z(i+1), FGVc: array to store FGVs
inline std::tuple<float, float, col3> light_path::pdfs_FG(const scene &scene, const camera_path_vertex &zi, const camera_path_vertex &zip1, std::array<col3, Nc> &FGVc)
{
	auto &zi_isect = zi.intersection();
	auto &zip1_isect = zip1.intersection();

	//BRDF at z(i+1)
	const auto brdf = zip1_isect.material().make_brdf(zip1_isect, zip1.wi());

	//solid angle pdf
	const float pdf_w = brdf.pdf(zip1.wo());
	const float pdf_w_rr = pdf_w * rr_probability(brdf.f(zip1.wo()), zip1.wo().abs_cos(), pdf_w);

	//convert to area measure
	const float J = zi.wi().abs_cos() / squared_norm(zi_isect.p() - zip1_isect.p());
	const float pdf_A = pdf_w * J;
	const float pdf_A_rr = pdf_w_rr * J;

	//calculate FG
	const col3 FG = brdf.f(zip1.wo()) * (zip1.wo().abs_cos() * J);

	//calculate FGV at neighbor cache points of z(i)
	for(size_t i = 0; i < Nc; i++){
		FGVc[i] = zi.neighbor_cache(i).calc_FGV(scene, zip1_isect, brdf);
	}
	return std::make_tuple(pdf_A, pdf_A_rr, FG);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

//return sampling pdf of z(i) from z(i+1) (n: z(i) is n-th vertex from light source)
inline float light_path::pdf(const camera_path_vertex &zi, const camera_path_vertex &zip1, const size_t n)
{
	return (void)zip1, (n > rr_threshold) ? zi.pdf_bwd_rr() : zi.pdf_bwd();
}

///////////////////////////////////////////////////////////////////////////////////////////////////

//return pdf and FG and calculate FGV at cache points neighbor to z(t-2)
//ztm2: z(t-2), ztm1: z(t-1), n: z(i) is n-th vertex from light source, zy: direction from z(t-1) to y(s-1), FGVc: array to store FGV
inline std::tuple<float, col3> light_path::pdf_FG(const scene &scene, const camera_path_vertex &ztm2, const camera_path_vertex &ztm1, const size_t n, const direction &zy, std::array<col3, Nc> &FGVc)
{
	//BRDF at z(t-1)
	const auto &ztm1_isect = ztm1.intersection();
	const auto brdf = ztm1_isect.material().make_brdf(ztm1_isect, zy);

	//solid angle pdf
	float pdf_w = brdf.pdf(ztm1.wo());
	if(n > rr_threshold){
		pdf_w *= rr_probability(brdf.f(ztm1.wo()), ztm1.wo().abs_cos(), pdf_w);
	}

	//convert to area measure
	const float J = ztm2.wi().abs_cos() / squared_norm(ztm2.intersection().p() - ztm1_isect.p());
	const float pdf_A = pdf_w * J;

	//calculate FG
	const col3 FG = brdf.f(ztm1.wo()) * (ztm1.wo().abs_cos() * J);

	//calculate FGV at cache points neighbor to z(t-2)
	for(size_t i = 0; i < Nc; i++){
		FGVc[i] = ztm2.neighbor_cache(i).calc_FGV(scene, ztm1_isect, brdf);
	}
	return std::make_tuple(pdf_A, FG);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

//return pdf and FG
//ztm1: z(t-1), ysm1: y(s-1), n: z(i) is n-th vertex from light source, zy: direction from z(t-1) to y(s-1), yz: direction from y(s-1) to z(t-1)
inline std::tuple<float, col3> light_path::pdf_FG(const camera_path_vertex &ztm1, const light_path_vertex &ysm1, const size_t n, const direction &zy, const direction &yz)
{
	const auto &ztm1_isect = ztm1.intersection();
	const auto &ysm1_isect = ysm1.intersection();

	col3 FG;
	float pdf_A;

	if(ztm1_isect.material().is_emissive()){ //z(t-1) is on light source
		//intersection::material in y(-1) stores ptr to scene
		pdf_A = reinterpret_cast<const scene&>(ysm1_isect.material()).pdf_light(ztm1_isect);
		FG = ztm1_isect.material().Me();
	}
	//z(t-1) is on (non-emissive) surface
	else{
		//calculate solid angle pdf
		float pdf_w = ysm1.brdf().pdf(yz);
		if(n > rr_threshold){
			pdf_w *= rr_probability(ysm1.brdf().f(yz), yz.abs_cos(), pdf_w);
		}

		//convert to area measure
		const float J = zy.abs_cos() / squared_norm(ztm1_isect.p() - ysm1_isect.p());
		pdf_A = pdf_w * J;

		//calculate FG
		FG = ysm1.brdf().f(yz) * yz.abs_cos() * J;
	}
	return std::make_tuple(pdf_A, FG);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

} //namespace our

///////////////////////////////////////////////////////////////////////////////////////////////////
