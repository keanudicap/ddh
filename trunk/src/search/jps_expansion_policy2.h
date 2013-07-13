#ifndef WARTHOG_JPS_EXPANSION_POLICY2_H
#define WARTHOG_JPS_EXPANSION_POLICY2_H

// jps_expansion_policy2.h
//
// An experimental variation of warthog::jps_expansion_policy,
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

class jps_expansion_policy2 
{
	public:
		jps_expansion_policy2(warthog::gridmap* map);
		~jps_expansion_policy2();

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

		inline uint32_t
		mem()
		{
			return sizeof(*this) + map_->mem() + nodepool_->mem() + jpl_->mem();
		}

		uint32_t 
		mapwidth()
		{
			return map_->width();
		}

	private:
		warthog::gridmap* map_;
		warthog::blocklist2* nodepool_;
		offline_jump_point_locator* jpl_;
		uint32_t which_;
		uint32_t num_neighbours_;
		std::vector<warthog::search_node*> neighbours_;
		std::vector<warthog::cost_t> costs_;
		std::vector<uint32_t> jp_ids_;

		inline void
		reset()
		{
			which_ = 0;
			num_neighbours_ = 0;
			neighbours_.clear();
			costs_.clear();
			jp_ids_.clear();
		}

};

}

#endif

