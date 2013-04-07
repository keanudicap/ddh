#include "gridmap.h"
#include "jps.h"
#include "online_jump_point_locator.h"

#include <cassert>
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
	jumpcost = 0;

	// first 3 bits of first 3 bytes represent a 3x3 cell of tiles
	// from the grid. next_id at centre. Assume little endian format.
	uint32_t neis;
	uint32_t next_id = node_id;
	uint32_t mapw = map_->width();
	uint32_t bit_offset = (node_id & warthog::DBWORD_BITS_MASK);
	uint32_t dbindex = node_id >> warthog::LOG2_DBWORD_BITS;
	uint32_t dbw = map_->width() >> warthog::LOG2_DBWORD_BITS;
	map_->get_neighbours(next_id, (uint8_t*)&neis);

	while(true)
	{
		uint8_t row0 = ((uint8_t*)&neis)[0];
		uint8_t row1 = ((uint8_t*)&neis)[1];
		bool stop = 
			((~row1 & row0) & 0x5) || ((neis & 0x202) != 0x202) || (next_id == goal_id);
		if(stop)
		{ 
			// we hit the goal
			if(next_id == goal_id)
			{
				jumpnode_id = next_id;
				break;
			}

			// deadend
			if((neis & 0x202) != 0x202)
			{
				jumpnode_id = warthog::INF;
				break;
			}

			// forced nei
			jumpcost++;
			jumpnode_id = next_id - mapw;
			break; 
		}
		jumpcost++;
		next_id -= mapw;
		dbindex -= dbw;
		neis <<= 8;
		map_->get_tripleh(dbindex-dbw, bit_offset, ((uint8_t*)&neis)[0]);
	}
}

void
warthog::online_jump_point_locator::jump_south(uint32_t node_id, 
		uint32_t goal_id, uint32_t& jumpnode_id, double& jumpcost)
{
	jumpcost = 0;

	// first 3 bits of first 3 bytes represent a 3x3 cell of tiles
	// from the grid. next_id at centre. Assume little endian format.
	uint32_t neis;
	uint32_t next_id = node_id;
	uint32_t mapw = map_->width();
	uint32_t bit_offset = (node_id & warthog::DBWORD_BITS_MASK);
	uint32_t dbindex = node_id >> warthog::LOG2_DBWORD_BITS;
	uint32_t dbw = map_->width() >> warthog::LOG2_DBWORD_BITS;
	map_->get_neighbours(next_id, (uint8_t*)&neis);

	while(true)
	{
		uint8_t row1 = ((uint8_t*)&neis)[1];
		uint8_t row2 = ((uint8_t*)&neis)[2];
		bool stop = 
			((~row1 & row2) & 0x5) || ((neis & 0x20200) != 0x20200) || (next_id == goal_id);
		if(stop)
		{ 
			// we hit the goal
			if(next_id == goal_id)
			{
				jumpnode_id = next_id;
				break;
			}

			// deadend
			if((neis & 0x20200) != 0x20200)
			{
				jumpnode_id = warthog::INF;
				break;
			}

			// forced nei
			jumpcost++;
			jumpnode_id = next_id + mapw;
			break; 
		}
		jumpcost++;
		next_id += mapw;
		dbindex += dbw;
		neis >>= 8;
		map_->get_tripleh(dbindex+dbw, bit_offset, ((uint8_t*)&neis)[2]);
	}
}

void
warthog::online_jump_point_locator::jump_east(uint32_t node_id, 
		uint32_t goal_id, uint32_t& jumpnode_id, double& jumpcost)
{
	jumpcost = 0;
	jumpnode_id = node_id;

	// first 3 bits of first 3 bytes represent a 3x3 cell of tiles
	// from the grid. next_id at centre. Assume little endian format.
	uint64_t neis = 0;
	bool deadend = false;

	while(true)
	{
		// read in 16 tiles from 3 adjacent rows
		// NB: curent node is at index 1 (not 0) in this cache
		map_->get_neighbours_16bit(node_id + jumpcost, (uint16_t*)&neis);
	
		// jump ahead if the cache contains no obstacles
		if((neis & 0xffffffffffff) == 0xffffffffffff)
		{
			jumpcost += 16; 
			continue;
		}

		// jump ahead if first half of the cache contains no obstacles
		if((neis & 0x00ff00ff00ff) == 0x00ff00ff00ff)
		{
			jumpcost += 8;
			neis >>= 8;
		}

		// jump ahead if the next four locations have no obstacle neis
		if((neis & 0x000f000f000f) == 0x000f000f000f)
		{
			jumpcost += 4;
			neis >>= 4;
		}

		// jump step by step (NB: neis not shifted; bitmasks change instead)
		if((neis & 0x30000) != 0x30000)
		{
			deadend = true;
			break;
		}
		uint64_t forced = (~neis << 1) & neis;
		if(forced & 0x200000002) { jumpcost++; break; }

		// step again
		if((neis & 0x60000) != 0x60000) 
		{
			deadend = true;
			jumpcost++;
			break;
		}
		if(forced & 0x400000004) { jumpcost+=2; break; }

		jumpcost += 2;
	}

	jumpnode_id = node_id + jumpcost;
	if(node_id < goal_id && jumpnode_id >= goal_id)
	{
		jumpnode_id = goal_id;
		jumpcost = goal_id - node_id;
	}
	else if(deadend)
	{
		jumpnode_id = warthog::INF;
	}
	
}

void
warthog::online_jump_point_locator::jump_west(uint32_t node_id, 
		uint32_t goal_id, uint32_t& jumpnode_id, double& jumpcost)
{
	jumpcost = 0;
	bool deadend = false;

	uint64_t neis = 0;
	jumpcost = 0; // begin from the node we want to step to

	while(true)
	{
		// read in 16 tiles from 3 adjacent rows
		// NB: curent node is at index 1 (not 0) in this cache
		map_->get_neighbours_upper_16bit(node_id - jumpcost, (uint16_t*)&neis);

		// jump ahead if all cached tiles are traversable
		if((neis & 0xffffffffffff) == 0xffffffffffff)
		{
			jumpcost += 16;
			continue;
		}

		// jump ahead if all tiles in first half of cache are traversable
		if((neis & 0xff00ff00ff00) == 0xff00ff00ff00)
		{
			jumpcost += 8;
			neis <<= 8;
		}

		// jump ahead if the next few tiles in the cache are not obstacles
		if((neis & 0xf000f000f000) == 0xf000f000f000)
		{
			jumpcost += 4;
			neis <<= 4;
		}

		// jump step by step (NB: neis not shifted; bitmasks change instead)
		if((neis & 0xC0000000) != 0xC0000000) 
		{
			deadend = true;
			break;
		}
		uint64_t forced = (~neis >> 1) & neis;
		if(forced & 0x400000004000) { jumpcost++; break; }

		if((neis & 0x60000000) != 0x60000000) 
		{
			deadend = true;
			jumpcost++;
			break;
		}
		if(forced & 0x200000002000) { jumpcost+=2; break; }

		jumpcost += 2;
	}
	jumpnode_id = node_id - jumpcost;
	if(node_id > goal_id && jumpnode_id <= goal_id)
	{
		jumpnode_id = goal_id;
		jumpcost = node_id - goal_id;
	}
	else if(deadend)
	{
		jumpnode_id = warthog::INF;
	}
}

void
warthog::online_jump_point_locator::jump_northeast(uint32_t node_id,
	   	uint32_t goal_id, uint32_t& jumpnode_id, double& jumpcost)
{
	jumpcost = 0;

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
		jumpcost += warthog::ROOT_TWO;
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
}

void
warthog::online_jump_point_locator::jump_northwest(uint32_t node_id, 
		uint32_t goal_id, uint32_t& jumpnode_id, double& jumpcost)
{
	jumpcost = 0;

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
		jumpcost += warthog::ROOT_TWO;
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
}

void
warthog::online_jump_point_locator::jump_southeast(uint32_t node_id, 
		uint32_t goal_id, uint32_t& jumpnode_id, double& jumpcost)
{
	jumpcost = 0;

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
		jumpcost += warthog::ROOT_TWO;
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
}

void
warthog::online_jump_point_locator::jump_southwest(uint32_t node_id, 
		uint32_t goal_id, uint32_t& jumpnode_id, double& jumpcost)
{
	jumpcost = 0;

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
		jumpcost += warthog::ROOT_TWO;
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
}
