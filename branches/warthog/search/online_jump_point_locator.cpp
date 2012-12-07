#include "gridmap.h"
#include "jps.h"
#include "online_jump_point_locator.h"

#include <climits>

warthog::online_jump_point_locator::online_jump_point_locator(warthog::gridmap* map)
	: map_(map), jumplimit_(UINT32_MAX)
{
}

warthog::online_jump_point_locator::~online_jump_point_locator()
{
}


// Finds a jump point successor of node (x, y) in Direction d.
// Also given is the location of the goal node (goalx, goaly) for a particular
// search instance. If encountered, the goal node is always returned as a 
// jump point successor.
//
// @return: the id of a jump point successor or warthog::INF if no jp exists.
void
warthog::online_jump_point_locator::jump(warthog::jps::direction d,
	   	uint32_t node_id, uint32_t goal_id, uint32_t& jumpnode_id, 
		double& jumpcost)
{
	jumpcost = 0;
	double cost_per_step = 1;
	if(d > 8) { cost_per_step = warthog::ROOT_TWO; }

	// first 3 bits of first 3 bytes represent a 3x3 cell of tiles
	// from the grid. Assume little endian format.
	uint32_t neis;
	uint32_t next_id = node_id;
	uint32_t mapw = map_->width();
	for(uint32_t steps=1; steps <= jumplimit_; steps++)
	{
		// jump a single step in direction @param d. we check that
		// the step is to a non-obstacle node and that no corners are cut.
		bool jump_ok = false;
		switch(d)
		{
			case warthog::jps::NORTH:
				next_id -= mapw;
				map_->get_neighbours2(next_id, (uint8_t*)&neis);
				jump_ok = (neis & 131584) == 131584; // bits 9 and 17
				break;
			case warthog::jps::SOUTH:
				next_id += mapw;
				map_->get_neighbours2(next_id, (uint8_t*)&neis);
				jump_ok = (neis & 514) == 514; // bits 9 and 4
				break;
			case warthog::jps::EAST:
				next_id++;
				map_->get_neighbours2(next_id, (uint8_t*)&neis);
				jump_ok = (neis & 768) == 768; // bits 9 and 8
				break;
			case warthog::jps::WEST:
				next_id--;
				map_->get_neighbours2(next_id, (uint8_t*)&neis);
				jump_ok = (neis & 1536) == 1536; // bits 9 and 10
				break;
			case warthog::jps::NORTHEAST:
				next_id = next_id - mapw + 1;
				map_->get_neighbours2(next_id, (uint8_t*)&neis);
				jump_ok = (neis & 197376) == 197376; // bits 9, 8, 16, 17
				break;
			case warthog::jps::SOUTHEAST:
				next_id = next_id + mapw + 1;
				map_->get_neighbours2(next_id, (uint8_t*)&neis);
				jump_ok = (neis & 771) == 771; // bits 9, 8, 0, 1
				break;
			case warthog::jps::NORTHWEST:
				next_id = next_id - mapw - 1;
				map_->get_neighbours2(next_id, (uint8_t*)&neis);
				jump_ok = (neis & 394752) == 394752; // bits 9, 10, 17, 18
				break;
			case warthog::jps::SOUTHWEST:
				next_id = next_id + mapw - 1;
				map_->get_neighbours2(next_id, (uint8_t*)&neis);
				jump_ok = (neis & 1542) == 1542; // bits 9, 1, 2, 10
				break;
			default:
				break;
		}

		// stop jumping if we hit an obstacle or take an invalid step
		if(!jump_ok)
		{
			jumpcost = warthog::INF;
			next_id = warthog::INF;
			break;
		}
		jumpcost += cost_per_step;

		// the goal is always a jump point
		if(next_id == goal_id) { break; }


		// next_id is a jump node if it has >=1 forced neighbours2.
		if(warthog::jps::compute_forced(d, neis)) { break; }

		// recurse straight before stepping diagonally;
		// next_id is a jump point if the recursion finds a node
		// with a forced neighbour (cannot step beyond next_id; it might
		// be a potential turning point on the optimal path).
		uint32_t jp_id; 
		double jp_cost;
		if(d == warthog::jps::NORTHEAST)
		{
			jump(warthog::jps::NORTH, next_id, goal_id, jp_id, jp_cost);
			if(jp_id != warthog::INF) { break; }
			jump(warthog::jps::EAST, next_id, goal_id, jp_id, jp_cost);
			if(jp_id != warthog::INF) { break; }
		}
		else if(d == warthog::jps::NORTHWEST)
		{
			jump(warthog::jps::NORTH, next_id, goal_id, jp_id, jp_cost);
			if(jp_id != warthog::INF) { break; }
			jump(warthog::jps::WEST, next_id, goal_id, jp_id, jp_cost);
			if(jp_id != warthog::INF) { break; }
		}
		else if(d == warthog::jps::SOUTHEAST)
		{
			jump(warthog::jps::SOUTH, next_id, goal_id, jp_id, jp_cost);
			if(jp_id != warthog::INF) { break; }
			jump(warthog::jps::EAST, next_id, goal_id, jp_id, jp_cost);
			if(jp_id != warthog::INF) { break; }
		}
		else if(d == warthog::jps::SOUTHWEST)
		{
			jump(warthog::jps::SOUTH, next_id, goal_id, jp_id, jp_cost);
			if(jp_id != warthog::INF) { break; }
			jump(warthog::jps::WEST, next_id, goal_id, jp_id, jp_cost);
			if(jp_id != warthog::INF) { break; }
		}
	}
	jumpnode_id = next_id;
}

