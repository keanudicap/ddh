#ifndef WARTHOG_JPSPLUS_EXPANSION_POLICY_H
#define WARTHOG_JPSPLUS_EXPANSION_POLICY_H

// jpsplus_expansion_policy.h
//
// An experimental variation of warthog::jpsplus_expansion_policy,
// this version is designed for efficient offline jps.
//
// @author: dharabor
// @created: 06/01/2010

#include "blocklist2.h"
#include "gridmap.h"
#include "helpers.h"
#include "jps.h"
#include "offline_jump_point_locator.h"
#include "problem_instance.h"
#include "search_node.h"

#include "stdint.h"

namespace warthog
{

class jps_record;
class jpsplus_expansion_policy 
{
	public:
		jpsplus_expansion_policy(warthog::gridmap* map);
		~jpsplus_expansion_policy();

		// create a warthog::search_node object from a state description
		// (in this case, an id)
		inline warthog::search_node*
		generate(uint32_t node_id)
		{
			return nodepool_->generate(node_id);
		}


		// reset the policy and discard all generated nodes
		inline void
		clear()
		{
			reset();
			nodepool_->clear();
		}


		void 
		expand(warthog::search_node*, warthog::problem_instance*);

		inline void
		first(warthog::search_node*& ret, warthog::cost_t& cost)
		{
			which_ = 0;
			ret = neighbours_[which_];
			cost = costs_[which_];
		}

		inline bool
		has_next()
		{
			if((which_+1) < num_neighbours_) { return true; }
			return false;
		}

		inline void
		n(warthog::search_node*& ret, warthog::cost_t& cost)
		{
			ret = neighbours_[which_];
			cost = costs_[which_];
		}

		inline void
		next(warthog::search_node*& ret, warthog::cost_t& cost)
		{
			if(which_ < num_neighbours_)
			{
				which_++;
			}
			ret = neighbours_[which_];
			cost = costs_[which_];
		}

		uint32_t 
		mapwidth()
		{
			return map_->width();
		}

		void
		to_xy(uint32_t padded_id, uint32_t& x, uint32_t& y)
		{
			map_->to_unpadded_xy(padded_id, x, y);
		}

		inline bool
		get_verbose() { return verbose_; }

		void
		set_verbose(bool verbose) 
		{ 
			verbose_ = verbose;
		   	jpl_->set_verbose(verbose);
		} 

		inline uint32_t
		mem()
		{
			return sizeof(*this) + map_->mem() + nodepool_->mem() + 
				jpl_->mem();
		}


	private:
		warthog::gridmap* map_;
		warthog::blocklist2* nodepool_;
		offline_jump_point_locator* jpl_;
		uint32_t which_;
		uint32_t num_neighbours_;
		std::vector<warthog::search_node*> neighbours_;
		std::vector<warthog::cost_t> costs_;

		warthog::jps_record* start_node_;
		warthog::jps_record* goal_node_;
		uint32_t search_id_;
		bool verbose_;

		inline void
		reset()
		{
			which_ = 0;
			num_neighbours_ = 0;
			neighbours_.clear();
			costs_.clear();
		}

};

}

#endif

