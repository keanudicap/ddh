#ifndef WARTHOG_GRIDMAP_EXPANSION_POLICY_H
#define WARTHOG_GRIDMAP_EXPANSION_POLICY_H

// gridmap_expansion_policy.h
//
// An ExpansionPolicy for square grids (weighted or uniform-cost).
//
// @author: dharabor
// @created: 28/10/2010
//

#include "gridmap.h"
#include "search_node.h"

#include <memory>

namespace warthog
{

class problem_instance;
class gridmap_expansion_policy 
{
	public:
		gridmap_expansion_policy(warthog::gridmap* map);
		~gridmap_expansion_policy();

		void 
		expand(unsigned int, warthog::problem_instance*);

		// fetches the first neighbour of (cx_, cy_). 
		// also resets the current neigbour iterator
		inline unsigned int
		first()
		{
			which_ = 0;
			return this->n();
		}

		// @return the id of the current neighbour of (cx_, cy_) -- the node
		// being expanded. 
		// ::end is returned if all neighbours have been processed.
		inline unsigned int 
		n()
		{
			return neis_[which_];
		}


		// @return the node id of the next neighbour of (cx_, cy_)
		// ::end() is returned if there is no next neighbour
		inline unsigned int
		next()
		{
			if(which_ < num_neis_) { which_++; }
			return this->n();
		}

		// @return true if (cx_, cy_) has more neighbours to process.
		// false otherwise.
		bool 
		has_next()
		{
			if((which_+1) < num_neis_)
			{
				return true;
			}
			return false;
		}

		inline double
		cost_to_n()
		{
			return costs_[which_];
		}

		inline unsigned int
		get_max_node_id()
	   	{
			return map_->height() * map_->width();
		}

		unsigned int
		mem()
		{
			return map_->mem() + 
			sizeof(*this);
		}
	
	private:
		warthog::gridmap* map_;
		unsigned int cx_, cy_;
		// 3x3 square of tiles + one terminal element
		unsigned int neis_[10]; 
		double costs_[10];
		unsigned int which_; // current neighbour
		unsigned int num_neis_; // might have < 9 (some could be blocked)
};

}

#endif

