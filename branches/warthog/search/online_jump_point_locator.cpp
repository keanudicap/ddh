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
	char tiles[9];
	uint32_t next_id = node_id;
	uint32_t mapw = map_->width();
	for(uint32_t steps=1; steps <= jumplimit_; steps++)
	{
		// TODO: get triples instead of neighbours
		// TODO: optimised function for uniform cost maps
		switch(d)
		{
			case warthog::jps::NORTH:
				next_id -= mapw;
				map_->get_neighbours(next_id, tiles);
				jumpcost = tiles[7] ? (jumpcost + (tiles[4] + tiles[7]) *
						 warthog::ONE_OVER_TWO) : warthog::INF;
				break;
			case warthog::jps::SOUTH:
				next_id += mapw;
				map_->get_neighbours(next_id, tiles);
				jumpcost = tiles[1] ? (jumpcost + (tiles[4] + tiles[1]) *
					   	warthog::ONE_OVER_TWO) : warthog::INF;
				break;
			case warthog::jps::EAST:
				next_id++;
				map_->get_neighbours(next_id, tiles);
				jumpcost = tiles[3] ? (jumpcost + (tiles[4] + tiles[3]) *
					   	warthog::ONE_OVER_TWO) : warthog::INF;
				break;
			case warthog::jps::WEST:
				next_id--;
				map_->get_neighbours(next_id, tiles);
				jumpcost = tiles[5] ? (jumpcost + (tiles[4] + tiles[5]) *
					   	warthog::ONE_OVER_TWO) : warthog::INF;
				break;
			case warthog::jps::NORTHEAST:
				next_id = next_id - mapw + 1;
				map_->get_neighbours(next_id, tiles);
				jumpcost = (tiles[3] && tiles[6] && tiles[7]) ? 
					(jumpcost + (tiles[3] + tiles[6] + tiles[7] + tiles[4]) *
					warthog::ROOT_TWO_OVER_FOUR) : warthog::INF; 
				break;
			case warthog::jps::SOUTHEAST:
				next_id = next_id + mapw + 1;
				map_->get_neighbours(next_id, tiles);
				jumpcost = (tiles[3] && tiles[0] && tiles[1]) ? 
					(jumpcost + (tiles[3] + tiles[0] + tiles[1] + tiles[4]) *
					warthog::ROOT_TWO_OVER_FOUR) : warthog::INF; 
				break;
			case warthog::jps::NORTHWEST:
				next_id = next_id - mapw - 1;
				map_->get_neighbours(next_id, tiles);
				jumpcost = (tiles[5] && tiles[8] && tiles[7]) ? 
					(jumpcost + (tiles[5] + tiles[8] + tiles[7] + tiles[4]) *
					warthog::ROOT_TWO_OVER_FOUR) : warthog::INF; 
				break;
			case warthog::jps::SOUTHWEST:
				next_id = next_id + mapw - 1;
				map_->get_neighbours(next_id, tiles);
				jumpcost = (tiles[5] && tiles[2] && tiles[1]) ? 
					(jumpcost + (tiles[5] + tiles[2] + tiles[1] + tiles[4]) *
					warthog::ROOT_TWO_OVER_FOUR) : warthog::INF; 
				break;
			default:
				break;
		}

		// stop jumping if we hit an obstacle or take an invalid step
		if(jumpcost == warthog::INF || tiles[4] == 0)
		{
			jumpcost = warthog::INF;
			next_id = warthog::INF;
			break;
		}

		// the goal is always a jump point
		if(next_id == goal_id) { break; }


		// next_id is a jump node if it has >=1 forced neighbours.
		if(warthog::jps::compute_forced(d, tiles)) { break; }

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


