
#pragma once

#ifndef OUR_PATH_VERTEX_HPP
#define OUR_PATH_VERTEX_HPP

#include<array>
#include"../../base.hpp"

///////////////////////////////////////////////////////////////////////////////////////////////////

namespace our{

///////////////////////////////////////////////////////////////////////////////////////////////////
//forward declaration
///////////////////////////////////////////////////////////////////////////////////////////////////

class cache;

///////////////////////////////////////////////////////////////////////////////////////////////////
//constant parameters
///////////////////////////////////////////////////////////////////////////////////////////////////

//number of nearest cache points (Nc in Sec 5.2)
static const size_t Nc = 3;

///////////////////////////////////////////////////////////////////////////////////////////////////
//light_path_vertex
///////////////////////////////////////////////////////////////////////////////////////////////////

class light_path_vertex
{
public:

	//isect: intersection point, brdf: BRDF at isect, wi/wo: incident/outgoing directions, Le_throughput: emittance Le*throughput_weight
	light_path_vertex(const intersection &isect, const brdf &brdf, const direction &wi, const direction &wo, const col3 &Le_throughput, const float pdf) : m_brdf(brdf), m_isect(isect), m_wi(wi), m_wo(wo), m_Le_throughput(Le_throughput), m_pdf_fwd(pdf)
	{
		for(size_t i = 0; i < Nc; i++){
			m_cache_ptrs[i] = nullptr; m_Le_throughput_FGVc[i] = -1;
		}
		m_pdf_bwd = -1;
		m_pdf_bwd_rr = -1;
	}

	//set cache point c
	void set_neighbor_cache(const size_t i, const cache &c)
	{
		m_cache_ptrs[i] = &c;
	}

	//return i-th nearest cache point
	const cache &neighbor_cache(const size_t i) const
	{
		return assert(m_cache_ptrs[i] != nullptr), *m_cache_ptrs[i];
	}

	//set q*/p at i-th cache point  (q* in Eq. (15))
	void set_Le_throughput_FGVc(const size_t i, const col3 &Le_throughput_FGVc)
	{
		m_Le_throughput_FGVc[i] = luminance(Le_throughput_FGVc);
	}

	//return q*/p at i-th cache point
	float Le_throughput_FGVc(const size_t i) const
	{
		return assert(m_Le_throughput_FGVc[i] != -1), m_Le_throughput_FGVc[i];
	}

	void set_pdf_bwd(const float pdf_bwd)
	{
		m_pdf_bwd = pdf_bwd;
	}
	void set_pdf_bwd_rr(const float pdf_bwd_rr)
	{
		m_pdf_bwd_rr = pdf_bwd_rr;
	}

	//return Le*throughput
	const col3 &Le_throughput() const
	{
		return m_Le_throughput;
	}

	//return pdf in forward direction (from light source)
	float pdf_fwd() const
	{
		return m_pdf_fwd;
	}
	//return pdf in backward direction (from eye)
	float pdf_bwd() const
	{
		return assert(m_pdf_bwd != -1), m_pdf_bwd;
	}
	float pdf_bwd_rr() const
	{
		return assert(m_pdf_bwd_rr != -1), m_pdf_bwd_rr;
	}

	const direction &wi() const
	{
		return m_wi;
	}
	const direction &wo() const
	{
		return m_wo;
	}

	const brdf &brdf() const
	{
		return m_brdf;
	}

	const intersection &intersection() const
	{
		return m_isect;
	}

private:

	::brdf         m_brdf;
	::intersection m_isect;
	direction      m_wi;
	direction      m_wo;
	col3           m_Le_throughput;
	float          m_Le_throughput_FGVc[Nc];
	float          m_pdf_fwd;
	float          m_pdf_bwd;
	float          m_pdf_bwd_rr;
	const cache   *m_cache_ptrs[Nc];
};

///////////////////////////////////////////////////////////////////////////////////////////////////
//camera_path_vertex
///////////////////////////////////////////////////////////////////////////////////////////////////

class camera_path_vertex
{
public:

	//isect: intersection point, brdf: brdf at isect, wo&wi outgoing&incident directions, throughput_We: throughput * importance / PDF,
	camera_path_vertex(const intersection &isect, const brdf &brdf, const direction &wo, const direction &wi, const col3 &throughput_We, const float pdf) : m_brdf(brdf), m_isect(isect), m_wo(wo), m_wi(wi), m_throughput_We(throughput_We), m_pdf_fwd(pdf)
	{
		//initialize m_cache_ptrs & m_FGVc
		for(size_t i = 0; i < Nc; i++){
			m_cache_ptrs[i] = nullptr; m_FGVc[i][0] = -1;
		}
		m_FG_bwd[0] = -1;
		m_pdf_bwd = -1;
		m_pdf_bwd_rr = -1; //pdf including russian roulette probability
	}

	//set cache point c
	void set_neighbor_cache(const size_t i, const cache &c)
	{
		m_cache_ptrs[i] = &c;
	}

	//return i-th nearest cache point
	const cache &neighbor_cache(const size_t i) const
	{
		return assert(m_cache_ptrs[i] != nullptr), *m_cache_ptrs[i];
	}

	//return F(brdf) x G(geo term) x V(visibility) stored at cache points
	const std::array<col3, Nc> &FGVc() const
	{
		return m_FGVc;
	}
	std::array<col3, Nc> &FGVc()
	{
		return m_FGVc;
	}

	void set_FG_bwd(const col3 &FG_bwd)
	{
		m_FG_bwd = FG_bwd;
	}

	const col3 &FG_bwd() const
	{
		return assert(m_FG_bwd[0] != -1), m_FG_bwd;
	}

	void set_pdf_bwd(const float pdf_bwd)
	{
		m_pdf_bwd = pdf_bwd;
	}
	void set_pdf_bwd_rr(const float pdf_bwd_rr)
	{
		m_pdf_bwd_rr = pdf_bwd_rr;
	}

	//return throughput x importance(W_e)
	const col3 &throughput_We() const
	{
		return m_throughput_We;
	}

	//return pdfs
	float pdf_fwd() const
	{
		return m_pdf_fwd;
	}
	float pdf_bwd() const
	{
		return assert(m_pdf_bwd != -1), m_pdf_bwd;
	}
	float pdf_bwd_rr() const
	{
		return assert(m_pdf_bwd_rr != -1), m_pdf_bwd_rr;
	}

	const direction &wi() const
	{
		return m_wi;
	}
	const direction &wo() const
	{
		return m_wo;
	}

	const brdf &brdf() const
	{
		return m_brdf;
	}

	const intersection &intersection() const
	{
		return m_isect;
	}

private:

	::brdf               m_brdf;
	::intersection       m_isect;
	direction            m_wo;
	direction            m_wi;
	col3                 m_FG_bwd;
	col3                 m_throughput_We;
	float                m_pdf_fwd;
	float                m_pdf_bwd;
	float                m_pdf_bwd_rr;
	const cache         *m_cache_ptrs[Nc];
	std::array<col3, Nc> m_FGVc;
};

///////////////////////////////////////////////////////////////////////////////////////////////////

} //namespace our

///////////////////////////////////////////////////////////////////////////////////////////////////

#endif
