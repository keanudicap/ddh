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
		gridmap_expansion_policy(std::shared_ptr<warthog::gridmap> map);
		~gridmap_expansion_policy();

		void 
		expand(unsigned int, warthog::problem_instance*);

		unsigned int
		end() 
		{ 
			return warthog::UNDEF; 
		}

		// fetches the first neighbour of (cx_, cy_). 
		// also resets the current neigbour iterator
		inline unsigned int
		first()
		{
			which_ = 0;
			if(tiles_[which_])
			{
				return this->n();
			}
			return this->next();
		}

		// @return the id of the current neighbour of (cx_, cy_) -- the node
		// being expanded. 
		// ::end is returned if all neighbours have been processed.
		inline unsigned int 
		n()
		{
			int nx = cx_;
			int ny = cy_;
			if(which_ < 9)
			{
				switch(which_)
				{
					case 0: // (x-1, y-1)
						nx += -1;
						ny += -1;
						break;
					case 1: // (x-1 y)
						nx += -1;
						ny += 0;
						break;
					case 2: // (x-1, y+1)
						nx += -1; 
						ny += 1;
						break;
					case 3: // (x, y-1)
						nx += 0;
						ny += -1;
						break;
					case 4: // (x, y)
						nx += 0;
						ny += 0;
						break;
					case 5: // (x, y+1)
						nx += 0;
						ny += 1;
						break;
					case 6: // (x+1, y-1)
						nx += 1;
						ny += -1;
						break;
					case 7: // (x+1, y)
						nx += 1;
						ny += 0;
						break;
					case 8: // (x+1, y+1)
						nx += 1;
						ny += 1;
						break;
					default:
						break;
				}
				if(nx >= 0 && nx < (int)map_->width() && ny >= 0 && ny < (int)map_->height())	
				{
					return (ny*map_->width())+nx;
				}
			}
			return this->end();
		}


		// @return the node id of the next neighbour of (cx_, cy_)
		// ::end() is returned if there is no next neighbour
		inline unsigned int
		next()
		{
			for(++which_; which_ < 9; which_++)
			{
				if(tiles_[which_])
				{
					return n();
				}
			}
			return this->end();
		}

		// @return true if (cx_, cy_) has more neighbours to process.
		// false otherwise.
		bool 
		has_next()
		{
			for(int i=which_+1; i < 9; i++)
			{
				if(tiles_[i])
				{
					return true;
				}
			}
			return false;
		}

		inline double
		cost_to_n()
		{
			if(which_ < 9)
			{
				return costs_[which_];
			}
			return warthog::INF;
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
		std::shared_ptr<warthog::gridmap> map_;
		unsigned int cx_, cy_;
		warthog::dbword tiles_[9]; // 3x3 square of tiles around node (cx_, cy_)
		double costs_[9];
		unsigned int which_; // current neighbour
};

}

#endif

