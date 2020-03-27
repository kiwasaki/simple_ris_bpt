
#pragma once

#ifndef UTILITY_HPP
#define UTILITY_HPP

#include<mutex>
#include<thread>
#include<vector>
#include<atomic>

///////////////////////////////////////////////////////////////////////////////////////////////////
//spinlock
///////////////////////////////////////////////////////////////////////////////////////////////////

class spinlock
{
public:

	void lock()
	{
		while(m_state.test_and_set(std::memory_order_acquire)){
		}
	}

	void unlock()
	{
		m_state.clear(std::memory_order_release);
	}

private:

	std::atomic_flag m_state = ATOMIC_FLAG_INIT;
};

///////////////////////////////////////////////////////////////////////////////////////////////////
//function definitions
///////////////////////////////////////////////////////////////////////////////////////////////////


//evaluate func( x, y ) in parallel with nt threads
template<class Func> inline void in_parallel(const int nx, const int ny, Func func, const size_t nt = std::thread::hardware_concurrency())
{
	std::atomic<int> idx = 0;
	std::vector<std::thread> threads(nt);

	for(auto &thread : threads){

		thread = std::thread([&](){
			for(int i = idx.fetch_add(1); i < nx * ny; i = idx.fetch_add(1)){
				const int y = i / nx;
				const int x = i - nx * y;
				func(x, y);
			}
		});
	}
	for(auto &thread : threads){
		thread.join();
	}
}

//evaluate func(i) in parallel with nt threads
template<class Func> inline void in_parallel(const int nx, Func func, const size_t nt = std::thread::hardware_concurrency())
{
	std::atomic<int> idx = 0;
	std::vector<std::thread> threads(nt);

	for(auto &thread : threads){

		thread = std::thread([&](){
			for(int i = idx.fetch_add(1); i < nx; i = idx.fetch_add(1)){
				func(i);
			}
		});
	}
	for(auto &thread : threads){
		thread.join();
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////

#endif
