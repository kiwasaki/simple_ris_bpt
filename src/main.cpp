/**
 *  a simple implementation of
 *  "Resampling-aware Weighting Functions for BPT Using Multiple Light Sub-paths"
 *  by K. Nabata et al. (ACM TOG Vol. 39, No. 2, Article No. 15, pp. 1-11, 2020)
 */

#include"inc/sample/our.hpp"

#include<chrono>
#include<random>
#include<vector>
#include<thread>
#include<fstream>
#include<iostream>
#include<algorithm>

///////////////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv)
{
	//scene setup
	const scene scene(std::vector<object>{
		object(sphere(vec3(1 - 1e+3f, 0, 0), 1e+3f), material(col3(0.14f, 0.45f, 0.091f), false)), //+X
		object(sphere(vec3(1e+3f - 1, 0, 0), 1e+3f), material(col3(0.63f, 0.065f, 0.05f), false)), //-X
		object(sphere(vec3(0, 1 - 1e+3f, 0), 1e+3f), material(col3(0.725f, 0.71f, 0.68f), false)), //+Y
		object(sphere(vec3(0, 1e+3f - 1, 0), 1e+3f), material(col3(0.725f, 0.71f, 0.68f), false)), //-Y
		object(sphere(vec3(0, 0, 1e+3f - 1), 1e+3f), material(col3(0.725f, 0.71f, 0.68f), false)), //-Z

		object(sphere(vec3(0, 0.9f, 0), 0.1f), material(col3(170, 120, 40), true)), //Light
        /*
		object(sphere(vec3(0.89f, -0.89f, -0.89f), 0.1f), material(col3(170, 120, 40) * 10, true)), //Light
		object(sphere(vec3(0.89f, -0.89f + 0.31f, -0.89f), 0.2f), material(col3(0.1f), false)),
		object(sphere(vec3(0.89f - 0.31f, -0.89f, -0.89f), 0.2f), material(col3(0.1f), false)),
		object(sphere(vec3(0.89f, -0.89f, -0.89f + 0.31f), 0.2f), material(col3(0.1f), false)),
        */
	});

	//camera setup
	const float fovy = 40;
	const camera camera(vec3(0, 0, 1 / tan(conv_deg_to_rad(fovy / 2)) + 1), vec3(0, 0, 0), 512, 512, fovy, 0.0);

	//parameter setup
	const size_t M = 200; //the number of pre-sampled light sub-paths
	our::renderer renderer(scene, camera, M);

	//buffer for storing rendering results
	const int w = camera.res_x();
	const int h = camera.res_y();
	imaged sum(w, h);

	//rendering algorithm shown in Algorithm 1 on Page 6
	const size_t max_iterations = 256;
	for(size_t n = 0; n < max_iterations; n++){

		std::cout << "iteration = " << n << std::endl;

		const imagef result = renderer.render(scene, camera);

		for(int i = 0, npixel = 3 * w * h; i < npixel; i++){
			sum(0,0)[i] += result(0,0)[i];
		}
	}

	//save image as test.bmp
	image result(w, h);
	for(int i = 0, n = 3 * w * h; i < n; i++){
		result(0,0)[i] = (unsigned char) (clamp(pow(float(sum(0,0)[i] / max_iterations ), 1.0f / 2.2f), 0, 1) * 255 ); //gamma_correction
	}
	save_as_bmp(result, "test.bmp");
	return 0;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
