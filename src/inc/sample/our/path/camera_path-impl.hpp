
///////////////////////////////////////////////////////////////////////////////////////////////////

namespace our{

///////////////////////////////////////////////////////////////////////////////////////////////////
//camera_path
///////////////////////////////////////////////////////////////////////////////////////////////////

//construct eye sub-paths
inline void camera_path::construct(const scene &scene, const camera &camera, const int x, const int y, random_number_generator &rng)
{
	m_vertices.clear();

	//number of samples for strategies (s>=1,t=1) (used in MIS weights)
	m_ns1 = camera.res_x() * camera.res_y();

	ray r = camera.sample(x, y, rng);

	//generate path vertex on lens, store camera to material
	m_vertices.emplace_back(intersection(r.o(), camera.d(), reinterpret_cast<const material*>(&camera)), brdf(), direction(), direction(), col3(1), camera.pdf_o());

	float pdf = camera.pdf_d(direction(r.d(), camera.d()));

	//initialize throughput*We
	col3 throughput_We(1);

	//generate path vertices
	while(true){

		const intersection isect = scene.calc_intersection(r);
		if(isect.is_invalid()){
			break;
		}

		const direction wo(-r.d(), isect.n());
		if(wo.is_invalid()){
			break;
		}

		pdf *= wo.abs_cos() / (r.t() * r.t());

		//if isect is on light source, add isect and terminate tracing
		if(isect.material().is_emissive()){
			m_vertices.emplace_back(isect, brdf(), wo, direction(isect.n()), throughput_We, pdf);
			break;
		}

		const brdf brdf = isect.material().make_brdf(isect, wo);
		const brdf_sample sample = brdf.sample(rng);

		//add path vertex
		if(sample.is_invalid()){
			m_vertices.emplace_back(isect, brdf, wo, direction(), throughput_We, pdf);
			break;
		}else{
			m_vertices.emplace_back(isect, brdf, wo, sample.w(), throughput_We, pdf);
		}

		//russian roulette
		if(num_vertices() >= rr_threshold){
			
			const float q = rr_probability(sample.f(),  sample.w().abs_cos(), sample.pdf());
			if(rng.generate_uniform_real() < q){
				pdf = sample.pdf() * q;
			}else{
				break;
			}
		}else{
			pdf = sample.pdf();
		}

		//update ray & throughput weight
		r = ray(isect.p(), sample.w());
		throughput_We *= sample.f() * sample.w().abs_cos() / pdf;
	}
}

//construct eye sub-path
inline void camera_path::construct(const scene &scene, const camera &camera, const int x, const int y, random_number_generator &rng, const kd_tree<cache> &caches)
{
	//construct path
	construct(scene, camera, x, y, rng);

	//precompute variables used in MIS weights
	{
		//search nearest cache points
		thread_local std::vector<neighbor<cache>> neighbors;
		for(size_t i = 1, n = num_vertices(); i < n; i++){

			auto &zi = operator()(i);
			caches.find_nearest(zi.intersection().p(), FLT_MAX, Nc, neighbors);

			for(size_t j = 0; j < Nc; j++){
				zi.set_neighbor_cache(j, *neighbors[j]);
			}
		}

		//calculate backward pdfs and FGV at neighbor cache points
		for(size_t i = 1, n = num_vertices(); i + 2 < n; i++){
		
			auto &zi = operator()(i);
			auto &zip1 = operator()(i + 1);
			auto pdfs_FG = light_path::pdfs_FG(scene, zi, zip1, zi.FGVc());
			zi.set_pdf_bwd(std::get<0>(pdfs_FG));
			zi.set_pdf_bwd_rr(std::get<1>(pdfs_FG));
			zi.set_FG_bwd(std::get<2>(pdfs_FG));
		}
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////

//calculate MIS partial weight
inline float camera_path::mis_partial_weight(const scene &scene, const light_path &y, const size_t s, const camera_path &z, const size_t t, const direction &yz, const direction &zy, const float M, const float Qp)
{
	float w = 0;
	{
		auto recurse = [&](const size_t i, const col3 &Le_throughput, const float pdf_L_zi, auto *This) -> void
		{
			//calculate FGV
			std::array<col3, Nc> FGVc;
			if(i > 1){
				
				col3 FG_zim1;
				float pdf_L_zim1;
				if(i == t - 1){
					const auto pdf_FG = light_path::pdf_FG(scene, z(t - 2), z(t - 1), s + (t - (i - 1)), zy, FGVc);
					pdf_L_zim1 = std::get<0>(pdf_FG);
					FG_zim1 = std::get<1>(pdf_FG);

				}else{
					FGVc = z(i - 1).FGVc();
					FG_zim1 = z(i - 1).FG_bwd();
					pdf_L_zim1 = light_path::pdf(z(i - 1), z(i), s + (t - (i - 1)));
				}
				(*This)(i - 1, Le_throughput * FG_zim1 / pdf_L_zim1, pdf_L_zim1, This);
			}
			
			if(i == 1){
				w += z.m_ns1;
			}else{
				for(size_t j = 0; j < Nc; j++){
			
					const float Q = z(i - 1).neighbor_cache(j).Q();
					const float Le_throughput_FGVc = luminance(Le_throughput * FGVc[j]);
			
					if(Le_throughput_FGVc > 0){
						w += (1 / float(Nc + 1)) * M / ((M - 1) * std::max(mis_threshold, Q / Le_throughput_FGVc) + 1);
					}
				}
				w += (1 / float(Nc + 1)) * M / ((M - 1) * Qp + 1);
			}
			w *= pdf_L_zi / z(i).pdf_fwd();
		};
	
		const col3 Le_throughput = (
			(s > 0) ? y(s - 1).Le_throughput() : col3(1)
		);
		const auto pdf_FG = light_path::pdf_FG(z(t - 1), y(s - 1), s + 1, zy, yz);
		const auto pdf = std::get<0>(pdf_FG);
		const auto &FG = std::get<1>(pdf_FG);
		recurse(t - 1, Le_throughput * FG / pdf, pdf, &recurse);
	}
	return w;
}

///////////////////////////////////////////////////////////////////////////////////////////////////


//sampling pdfs (without/with RR) of y(i) from y(i+1)
inline std::tuple<float, float> camera_path::pdfs(const light_path_vertex &yi, const light_path_vertex &yip1)
{
	auto &yi_isect = yi.intersection();
	auto &yip1_isect = yip1.intersection();

	//BRDF at y(i+1)
	const auto brdf = yip1_isect.material().make_brdf(yip1_isect, yip1.wo());

	//pdf of solid angle measure
	const float pdf_w = brdf.pdf(yip1.wi());
	const float pdf_w_rr = pdf_w * rr_probability(brdf.f(yip1.wi()), yip1.wi().abs_cos(), pdf_w);

	//convert to area measure
	const float J = yi.wo().abs_cos() / squared_norm(yi_isect.p() - yip1_isect.p());
	const float pdf_A = pdf_w * J;
	const float pdf_A_rr = pdf_w_rr * J;
	return std::make_tuple(pdf_A, pdf_A_rr);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

//sampling pdf of y(i) (yi) from y(i+1) (yip1) (n: y(i) is n-th vertex from eye)
inline float camera_path::pdf(const light_path_vertex &yi, const light_path_vertex &yip1, const size_t n)
{
	return (void)yip1, (n > rr_threshold) ? yi.pdf_bwd_rr() : yi.pdf_bwd();
}

///////////////////////////////////////////////////////////////////////////////////////////////////

//sampling pdf of y(s-1) (ysm1) from z(t-1) (ztm1), (yz: direction from y(s-1) to z(t-1), zy: direction from z(t-1) to y(s-1))
inline float camera_path::pdf(const light_path_vertex &ysm1, const camera_path_vertex &ztm1, const size_t n, const direction &yz, const direction &zy)
{
	float pdf_w;

	if(n==1){
		pdf_w = reinterpret_cast<const class camera&>(ztm1.intersection().material()).pdf_d(zy);
	}
	else{ //z(t-1) on surfaces
		pdf_w = ztm1.brdf().pdf(zy);
		if(n > rr_threshold){
			pdf_w *= rr_probability(ztm1.brdf().f(zy), zy.abs_cos(), pdf_w);
		}
	}

    //convert to area measure
	return pdf_w * yz.abs_cos() / squared_norm(ztm1.intersection().p() - ysm1.intersection().p());
}

///////////////////////////////////////////////////////////////////////////////////////////////////

//sampling pdf of y(s-2) (ysm2) from y(s-1) (ysm1)
inline float camera_path::pdf(const light_path_vertex &ysm2, const light_path_vertex &ysm1, const size_t n, const direction &yz)
{
	//BRDF at y(s-1)
	const auto &ysm1_isect = ysm1.intersection();
	const auto brdf = ysm1_isect.material().make_brdf(ysm1_isect, yz);

	//solid angle pdf
	float pdf_w = brdf.pdf(ysm1.wi());
	if(n > rr_threshold){
		pdf_w *= rr_probability(brdf.f(ysm1.wi()), ysm1.wi().abs_cos(), pdf_w);
	}

	//convert to area measure
	return pdf_w * ysm2.wo().abs_cos() / squared_norm(ysm2.intersection().p() - ysm1_isect.p());
}

///////////////////////////////////////////////////////////////////////////////////////////////////

} //namespace our

///////////////////////////////////////////////////////////////////////////////////////////////////
