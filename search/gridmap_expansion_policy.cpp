#include "gridmap_expansion_policy.h"
#include "helpers.h"
#include "problem_instance.h"

warthog::gridmap_expansion_policy::gridmap_expansion_policy(
		std::shared_ptr<warthog::gridmap> map) : map_(std::move(map))
{
}

warthog::gridmap_expansion_policy::~gridmap_expansion_policy()
{
}

void 
warthog::gridmap_expansion_policy::expand(unsigned int nodeid,
		warthog::problem_instance* problem)
{
	warthog::helpers::index_to_xy(nodeid, map_->width(), cx_, cy_);

	// get terrain type of each tile in the 3x3 square around (x, y)
	tiles_[0] = map_->get_label(cx_-1, cy_-1);
	tiles_[1] = map_->get_label(cx_-1, cy_);
	tiles_[2] = map_->get_label(cx_-1, cy_+1);
	tiles_[3] = map_->get_label(cx_, cy_-1);
	tiles_[4] = map_->get_label(cx_, cy_);
	tiles_[5] = map_->get_label(cx_, cy_+1);
	tiles_[6] = map_->get_label(cx_+1, cy_-1);
	tiles_[7] = map_->get_label(cx_+1, cy_);
	tiles_[8] = map_->get_label(cx_+1, cy_+1);

	// calculate transition costs; from (x, y) to each adjacent tile
	// this value is just an average of terrain costs weighted by the 
	// transition type (straight or diagonal)
	if(!tiles_[4])
	{
		// if current tile is an obstacle, it infinite cost transitions
		costs_[0] = costs_[1] = costs_[2] = costs_[3] = warthog::INF;
		costs_[4] = costs_[5] = costs_[6] = costs_[7] = warthog::INF;
		costs_[8] = warthog::INF;
		return;
	}
	costs_[4] = 0;  // zero cost to move from (cx, cy) to (cx, cy)

	// cost to step in direction N 
	costs_[3] = tiles_[3] ? ((tiles_[3] + tiles_[4]) * 0.5) : warthog::INF;
	// cost to step in direction E
	costs_[7] = tiles_[7] ? ((tiles_[7] + tiles_[4]) * 0.5) : warthog::INF;
	// cost to step in direction S
	costs_[5] = tiles_[5] ? ((tiles_[5] + tiles_[4]) * 0.5) : warthog::INF;
	// cost to step in direction W
	costs_[1] = tiles_[1] ? ((tiles_[1] + tiles_[4]) * 0.5) : warthog::INF;
	
	// cost to step in direction NE
	costs_[6] = (tiles_[6] & tiles_[3] & tiles_[7]) ? 
		((tiles_[6] + tiles_[3] + tiles_[7] + tiles_[4]) 
		 * 0.25 * warthog::ROOT_TWO ) : (warthog::INF);
	
	// cost to step in direction SE
	costs_[8] = (tiles_[8] & tiles_[7] & tiles_[5]) ? 
		((tiles_[8] + tiles_[7] + tiles_[5] + tiles_[4]) 
		 * 0.25 * warthog::ROOT_TWO ) : (warthog::INF);

	// cost to step in direction SW
	costs_[2] = (tiles_[2] & tiles_[1] & tiles_[5]) ? 
		((tiles_[2] + tiles_[1] + tiles_[5] + tiles_[4]) 
		 * 0.25 * warthog::ROOT_TWO ) : (warthog::INF);
	
	// cost to step in direction NW
	costs_[0] = (tiles_[0] & tiles_[1] & tiles_[3]) ? 
		((tiles_[0] + tiles_[1] + tiles_[3] + tiles_[4]) 
		 * 0.25 * warthog::ROOT_TWO ) : (warthog::INF);
	
	// set index of first neighbour
	which_ = 0;
}

