#ifndef WARTHOG_OCTILE_HEURISTIC_H
#define WARTHOG_OCTILE_HEURISTIC_H

// octile_heuristic.h
//
// @author: dharabor
// @created: 21/08/2012
//

#include "constants.h"
#include "helpers.h"

#include <cstdlib>

namespace warthog
{

class octile_heuristic
{
	public:
		octile_heuristic(uint32_t mapwidth, uint32_t mapheight) 
	    	: mapwidth_(mapwidth), mapheight_(mapheight) { }
		~octile_heuristic() { }

		inline double
		h(uint32_t x, uint32_t y, 
				uint32_t x2, uint32_t y2)
		{
			double dx = abs(x-x2);
			double dy = abs(y-y2);
			if(dx < dy)
			{
				return dx * warthog::ROOT_TWO + (dy - dx) * warthog::ONE;
			}
			return dy * warthog::ROOT_TWO + (dx - dy) * warthog::ONE;
		}

		inline double
		h(uint32_t id, uint32_t id2)
		{
			uint32_t x, x2;
			uint32_t y, y2;
			warthog::helpers::index_to_xy(id, mapwidth_, x, y);
			warthog::helpers::index_to_xy(id2,mapwidth_, x2, y2);
			return this->h(x, y, x2, y2);
		}

		inline void
		hsteps(uint32_t id, uint32_t id2, uint32_t& ssteps, uint32_t& dsteps)
		{
			uint32_t x, x2;
			uint32_t y, y2;
			warthog::helpers::index_to_xy(id, mapwidth_, x, y);
			warthog::helpers::index_to_xy(id2,mapwidth_, x2, y2);

			double dx = abs(x-x2);
			double dy = abs(y-y2);
			if(dx < dy)
			{
				dsteps = dx;
				ssteps = (dy - dx);
			}
			else
			{
				dsteps = dy;
				ssteps = dx - dy;
			}
		}

	private:
		uint32_t mapwidth_;
		uint32_t mapheight_;
};

}

#endif

