#include "gridmap.h"
#include "jps.h"
#include "online_jump_point_locator.h"

#include <cassert>
#include <climits>

warthog::online_jump_point_locator::online_jump_point_locator(warthog::gridmap* map)
	: map_(map), jumplimit_(UINT32_MAX)
{
	rmap_ = create_rmap();
}

warthog::online_jump_point_locator::~online_jump_point_locator()
{
	delete rmap_;
}

// create a copy of the grid map which is rotated by 90 degrees clockwise.
// also create a corresponding JPL object for jumping around using this map.
warthog::gridmap*
warthog::online_jump_point_locator::create_rmap()
{
	uint32_t maph = map_->header_height();
	uint32_t mapw = map_->header_width();
	uint32_t rmaph = mapw;
	uint32_t rmapw = maph;
	warthog::gridmap* rmap = new warthog::gridmap(rmaph, rmapw);

	for(uint32_t x = 0; x < mapw; x++) 
	{
		for(uint32_t y = 0; y < maph; y++)
		{
			uint32_t label = map_->get_label(map_->to_padded_id(x, y));
			uint32_t rx = ((rmapw-1) - y);
			uint32_t ry = x;
			uint32_t rid = rmap->to_padded_id(rx, ry);
			rmap->set_label(rid, label);
		}
	}
	return rmap;
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
	switch(d)
	{
		case warthog::jps::NORTH:
			jump_north(node_id, goal_id, jumpnode_id, jumpcost);
			break;
		case warthog::jps::SOUTH:
			jump_south(node_id, goal_id, jumpnode_id, jumpcost);
			break;
		case warthog::jps::EAST:
			jump_east(node_id, goal_id, jumpnode_id, jumpcost);
			break;
		case warthog::jps::WEST:
			jump_west(node_id, goal_id, jumpnode_id, jumpcost);
			break;
		case warthog::jps::NORTHEAST:
			jump_northeast(node_id, goal_id, jumpnode_id, jumpcost);
			break;
		case warthog::jps::NORTHWEST:
			jump_northwest(node_id, goal_id, jumpnode_id, jumpcost);
			break;
		case warthog::jps::SOUTHEAST:
			jump_southeast(node_id, goal_id, jumpnode_id, jumpcost);
			break;
		case warthog::jps::SOUTHWEST:
			jump_southwest(node_id, goal_id, jumpnode_id, jumpcost);
			break;
		default:
			break;
	}
}

void
warthog::online_jump_point_locator::jump_north(uint32_t node_id, 
		uint32_t goal_id, uint32_t& jumpnode_id, double& jumpcost)
{
	node_id = this->map_id_to_rmap_id(node_id);
	goal_id = this->map_id_to_rmap_id(goal_id);
	__jump_north(node_id, goal_id, jumpnode_id, jumpcost, rmap_);
	jumpnode_id = this->rmap_id_to_map_id(jumpnode_id);
}

void
warthog::online_jump_point_locator::__jump_north(uint32_t node_id, 
		uint32_t goal_id, uint32_t& jumpnode_id, double& jumpcost,
		warthog::gridmap* mymap)
{
	// jumping north in the original map is the same as jumping
	// east when we use a version of the map rotated 90 degrees.
	__jump_east(node_id, goal_id, jumpnode_id, jumpcost, rmap_);
}

void
warthog::online_jump_point_locator::jump_south(uint32_t node_id, 
		uint32_t goal_id, uint32_t& jumpnode_id, double& jumpcost)
{
	node_id = this->map_id_to_rmap_id(node_id);
	goal_id = this->map_id_to_rmap_id(goal_id);
	__jump_south(node_id, goal_id, jumpnode_id, jumpcost, rmap_);
	jumpnode_id = this->rmap_id_to_map_id(jumpnode_id);
}

void
warthog::online_jump_point_locator::__jump_south(uint32_t node_id, 
		uint32_t goal_id, uint32_t& jumpnode_id, double& jumpcost,
		warthog::gridmap* mymap)
{
	// jumping north in the original map is the same as jumping
	// west when we use a version of the map rotated 90 degrees.
	__jump_west(node_id, goal_id, jumpnode_id, jumpcost, rmap_);
}

void
warthog::online_jump_point_locator::jump_east(uint32_t node_id, 
		uint32_t goal_id, uint32_t& jumpnode_id, double& jumpcost)
{
	__jump_east(node_id, goal_id, jumpnode_id, jumpcost, map_);
}


void
warthog::online_jump_point_locator::__jump_east(uint32_t node_id, 
		uint32_t goal_id, uint32_t& jumpnode_id, double& jumpcost, 
		warthog::gridmap* mymap)
{
	jumpnode_id = node_id;

	uint32_t neis[3] = {0, 0, 0};
	bool deadend = false;
	uint32_t num_steps = 0;

	while(true)
	{
		// read in tiles from 3 adjacent rows. the curent node 
		// is in the low byte of the middle row
		mymap->get_neighbours_32bit(node_id + num_steps, neis);

		// identity forced neighbours and deadend tiles. 
		// forced neighbours are found in the top or bottom row. they 
		// can be identified as a non-obstacle tile that follows
		// immediately  after an obstacle tile. A dead-end tile is
		// an obstacle found  on the middle row; 
		uint32_t 
		forced_bits = (~neis[0] << 1) & neis[0];
		forced_bits |= (~neis[2] << 1) & neis[2];
		uint32_t 
		deadend_bits = ~neis[1];

		// stop if we found any forced or dead-end tiles
		int stop_bits = (forced_bits | deadend_bits);
		if(stop_bits)
		{
			uint32_t stop_pos = __builtin_ffs(stop_bits)-1; // returns idx+1
			num_steps += stop_pos; 
			deadend = deadend_bits & (1 << stop_pos);
			break;
		}

		// jump to the last position in the cache. we do not jump past the end
		// in case the last tile from the row above or below is an obstacle.
		// Such a tile, followed by a non-obstacle tile, would yield a forced 
		// neighbour that we don't want to miss.
		num_steps += 31; 
	}

	jumpnode_id = node_id + num_steps;

	uint32_t goal_dist = goal_id - node_id;
	if(num_steps > goal_dist)
	{
		jumpnode_id = goal_id;
		jumpcost = goal_id - node_id;
		return;
	}

	if(deadend)
	{
		if(num_steps > 0) { num_steps--; }
		jumpnode_id = warthog::INF;
	}
	jumpcost = num_steps;
	
}

// analogous to ::jump_east 
void
warthog::online_jump_point_locator::jump_west(uint32_t node_id, 
		uint32_t goal_id, uint32_t& jumpnode_id, double& jumpcost)
{
	__jump_west(node_id, goal_id, jumpnode_id, jumpcost, map_);
}

void
warthog::online_jump_point_locator::__jump_west(uint32_t node_id, 
		uint32_t goal_id, uint32_t& jumpnode_id, double& jumpcost, 
		warthog::gridmap* mymap)
{
	uint32_t num_steps = 0;
	bool deadend = false;
	uint32_t neis[3] = {0, 0, 0};

	while(true)
	{
		// cache 32 tiles from three adjacent rows.
		// current tile is in the high byte of the middle row
		mymap->get_neighbours_upper_32bit(node_id - num_steps, neis);

		// identify forced and dead-end nodes
		uint32_t 
		forced_bits = (~neis[0] >> 1) & neis[0];
		forced_bits |= (~neis[2] >> 1) & neis[2];
		uint32_t 
		deadend_bits = ~neis[1];

		// stop if we encounter any forced or deadend nodes
		uint32_t stop_bits = (forced_bits | deadend_bits);
		if(stop_bits)
		{
			uint32_t stop_pos = __builtin_clz(stop_bits);
			num_steps += stop_pos;
			deadend = deadend_bits & (0x80000000 >> stop_pos);
			break;
		}
		// jump to the end of cache. jumping +32 involves checking
		// for forced neis between adjacent sets of contiguous tiles
		num_steps += 31;
	
	}
	jumpnode_id = node_id - num_steps;

	uint32_t goal_dist = node_id - goal_id;
	if(num_steps > goal_dist)
	{
		jumpnode_id = goal_id;
		jumpcost = node_id - goal_id;
		return;
	}

	if(deadend)
	{
		if(num_steps > 0) { num_steps--; }
		jumpnode_id = warthog::INF;
	}
	jumpcost = num_steps;
}

void
warthog::online_jump_point_locator::jump_northeast(uint32_t node_id,
	   	uint32_t goal_id, uint32_t& jumpnode_id, double& jumpcost)
{
	uint32_t num_steps = 0;

	// first 3 bits of first 3 bytes represent a 3x3 cell of tiles
	// from the grid. next_id at centre. Assume little endian format.
	uint32_t next_id = node_id;
	uint32_t mapw = map_->width();

	// early return if the first diagonal step is invalid
	// (validity of subsequent steps is checked by straight jump functions)
	uint32_t neis;
	map_->get_neighbours(next_id, (uint8_t*)&neis);
	if((neis & 1542) != 1542) { jumpnode_id = warthog::INF; return; }

	// jump a single step at a time (no corner cutting)
	while(true)
	{
		num_steps++;
		next_id = next_id - mapw + 1;

		// recurse straight before stepping again diagonally;
		// (ensures we do not miss any optimal turning points)
		uint32_t jp_id; 
		double cost1, cost2;
		jump_north(next_id, goal_id, jp_id, cost1);
		if(jp_id != warthog::INF) { break; }
		jump_east(next_id, goal_id, jp_id, cost2);
		if(jp_id != warthog::INF) { break; }

		// couldn't move in either straight dir; node_id is an obstacle
		if(!(cost1 && cost2)) { next_id = warthog::INF; break; }

	}
	jumpnode_id = next_id;
	jumpcost = num_steps*warthog::ROOT_TWO;
}

void
warthog::online_jump_point_locator::jump_northwest(uint32_t node_id, 
		uint32_t goal_id, uint32_t& jumpnode_id, double& jumpcost)
{
	uint32_t num_steps = 0;

	// first 3 bits of first 3 bytes represent a 3x3 cell of tiles
	// from the grid. next_id at centre. Assume little endian format.
	uint32_t next_id = node_id;
	uint32_t mapw = map_->width();

	// early termination (invalid first step)
	uint32_t neis;
	map_->get_neighbours(next_id, (uint8_t*)&neis);
	if((neis & 771) != 771) { jumpnode_id = warthog::INF; return; }

	// jump a single step at a time (no corner cutting)
	while(true)
	{
		num_steps++;
		next_id = next_id - mapw - 1;

		// recurse straight before stepping again diagonally;
		// (ensures we do not miss any optimal turning points)
		uint32_t jp_id; 
		double cost1, cost2;
		jump_north(next_id, goal_id, jp_id, cost1);
		if(jp_id != warthog::INF) { break; }
		jump_west(next_id, goal_id, jp_id, cost2);
		if(jp_id != warthog::INF) { break; }

		// couldn't move in either straight dir; node_id is an obstacle
		if(!(cost1 && cost2)) { next_id = warthog::INF; break; }
	}
	jumpnode_id = next_id;
	jumpcost = num_steps*warthog::ROOT_TWO;
}

void
warthog::online_jump_point_locator::jump_southeast(uint32_t node_id, 
		uint32_t goal_id, uint32_t& jumpnode_id, double& jumpcost)
{
	uint32_t num_steps = 0;

	// first 3 bits of first 3 bytes represent a 3x3 cell of tiles
	// from the grid. next_id at centre. Assume little endian format.
	uint32_t next_id = node_id;
	uint32_t mapw = map_->width();
	
	// early return if the first diagonal step is invalid
	// (validity of subsequent steps is checked by straight jump functions)
	uint32_t neis;
	map_->get_neighbours(next_id, (uint8_t*)&neis);
	if((neis & 394752) != 394752) { jumpnode_id = warthog::INF; return; }

	// jump a single step at a time (no corner cutting)
	while(true)
	{
		num_steps++;
		next_id = next_id + mapw + 1;

		// recurse straight before stepping again diagonally;
		// (ensures we do not miss any optimal turning points)
		uint32_t jp_id; 
		double cost1, cost2;
		jump_south(next_id, goal_id, jp_id, cost1);
		if(jp_id != warthog::INF) { break; }
		jump_east(next_id, goal_id, jp_id, cost2);
		if(jp_id != warthog::INF) { break; }

		// couldn't move in either straight dir; node_id is an obstacle
		if(!(cost1 && cost2)) { next_id = warthog::INF; break; }
	}
	jumpnode_id = next_id;
	jumpcost = num_steps*warthog::ROOT_TWO;
}

void
warthog::online_jump_point_locator::jump_southwest(uint32_t node_id, 
		uint32_t goal_id, uint32_t& jumpnode_id, double& jumpcost)
{
	uint32_t num_steps = 0;

	// first 3 bits of first 3 bytes represent a 3x3 cell of tiles
	// from the grid. next_id at centre. Assume little endian format.
	uint32_t neis;
	uint32_t next_id = node_id;
	uint32_t mapw = map_->width();

	// early termination (first step is invalid)
	map_->get_neighbours(next_id, (uint8_t*)&neis);
	if((neis & 197376) != 197376) { jumpnode_id = warthog::INF; return; }

	// jump a single step (no corner cutting)
	while(true)
	{
		num_steps++;
		next_id = next_id + mapw - 1;

		// recurse straight before stepping again diagonally;
		// (ensures we do not miss any optimal turning points)
		uint32_t jp_id; 
		double cost1, cost2;
		jump_south(next_id, goal_id, jp_id, cost1);
		if(jp_id != warthog::INF) { break; }
		jump_west(next_id, goal_id, jp_id, cost2);
		if(jp_id != warthog::INF) { break; }

		// couldn't move in either straight dir; node_id is an obstacle
		if(!(cost1 && cost2)) { next_id = warthog::INF; break; }
	}
	jumpnode_id = next_id;
	jumpcost = num_steps*warthog::ROOT_TWO;
}
