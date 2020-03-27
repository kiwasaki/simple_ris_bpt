
#pragma once

#ifndef KD_TREE_HPP
#define KD_TREE_HPP

#include<queue>
#include<vector>
#include"math.hpp"

///////////////////////////////////////////////////////////////////////////////////////////////////
//neighbor
///////////////////////////////////////////////////////////////////////////////////////////////////

template<class T> class neighbor
{
public:

	neighbor(const T &elem, const float d2) : mp_elem(&elem), m_d2(d2)
	{
	}

	float d2() const
	{
		return m_d2;
	}

	const T &operator*() const
	{
		return *mp_elem;
	}
	const T *operator->() const
	{
		return mp_elem;
	}

private:

	const T *mp_elem; 
	float m_d2;
};

///////////////////////////////////////////////////////////////////////////////////////////////////
//kd_tree
///////////////////////////////////////////////////////////////////////////////////////////////////

template<class T> class kd_tree
{
public:

	struct node{
		node() = default;
		node(node&&) = default;
		node(const node&) = delete;
		node &operator=(node&&) = default;
		node &operator=(const node&) = delete;
		node(const vec3 &p, const int k, T &&elem) : p(p), k(k){
			new (&storage) T(std::move(elem));
		}
		~node(){
			reinterpret_cast<T&>(storage).~T();
		}
		operator const T&() const{
			return reinterpret_cast<const T&>(storage);
		}
		vec3 p; int k; std::aligned_storage_t<sizeof(T), alignof(T)> storage;
	};

	//elems: set of elements, point: function object that returns position
	template<class Point> kd_tree(std::vector<T> elems, Point point) : m_nodes(elems.size())
	{
		auto implement = [&](const size_t idx, auto first, auto last, const int depth, auto *This) -> void
		{
			const size_t num = last - first;
			if(num == 1){
				m_nodes[idx] = node(point(*first), -1, std::move(*first));
			} else{
				const int k = depth % 3;

				const size_t subtree_height = size_t(ceil(log2(num + 1))) - 1;
				auto mid = first + std::min((size_t(1) << subtree_height) - 1, num - (size_t(1) << (subtree_height - 1)));

				std::nth_element(first, mid, last, [&, k](const T &a, const T &b){
					return (point(a)[k] < point(b)[k]);
				});

				m_nodes[idx] = node(point(*mid), k, std::move(*mid));

				{
					(*This)(2 * idx + 1, first, mid, depth + 1, This);
				}
				if(++mid != last){
					(*This)(2 * idx + 2, mid,  last, depth + 1, This);
				}
			}
		};
		implement(0, elems.begin(), elems.end(), 0, &implement);
	}
	kd_tree() = default;

	//p: query point, r: query radius, n: number of elements, neighbors: store neighbor elements
	void find_nearest(const vec3 &p, const float r, const size_t n, std::vector<neighbor<T>> &neighbors) const
	{
		float r2 = r * r;
		auto implement = [&, p, n, this](const size_t idx, auto *This) -> void
		{
			if(idx >= m_nodes.size()){
				return;
			}
			const node &node = m_nodes[idx];

			const vec3 diff(
				p - node.p
			);
			if(2 * idx + 1 < m_nodes.size()){
				const float diff1_k = diff[node.k];
				const float diff2_k = diff[node.k] * diff[node.k];

				if(diff1_k < 0){
					(*This)(2 * idx + 1, This); if(diff2_k < r2){ (*This)(2 * idx + 2, This); }
				} else{
					(*This)(2 * idx + 2, This); if(diff2_k < r2){ (*This)(2 * idx + 1, This); }
				}
			}

			const float d2 = squared_norm(diff);

			if(d2 < r2){

				neighbors.emplace_back(node, d2);

				auto pred = [](const auto &a, const auto &b){
					return (a.d2() < b.d2());
				};
				if(neighbors.size() > n){
					std::push_heap(neighbors.begin(), neighbors.end(), pred);
					std:: pop_heap(neighbors.begin(), neighbors.end(), pred); neighbors.pop_back();
					r2 = neighbors.front().d2();
				}
				else if(neighbors.size() == n){
					std::make_heap(neighbors.begin(), neighbors.end(), pred);
					r2 = neighbors.front().d2();
				}
			}
		};
		neighbors.clear();
		implement(0, &implement);

		if(neighbors.size() < n){
			std::make_heap(neighbors.begin(), neighbors.end(), [](const auto &a, const auto &b){ return (a.d2() < b.d2()); });
		}
	}

	typename std::vector<node>::const_iterator begin() const
	{
		return m_nodes.begin();
	}
	typename std::vector<node>::const_iterator end() const
	{
		return m_nodes.end();
	}

private:

	std::vector<node> m_nodes;
};

///////////////////////////////////////////////////////////////////////////////////////////////////

#endif
