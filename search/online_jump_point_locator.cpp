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
	// from the grid. Assume little endian format.
	uint32_t neis;
	uint32_t next_id = node_id;
	uint32_t mapw = map_->width();
	map_->get_neighbours(next_id, (uint8_t*)&neis);

	while(true)
	{
		// jump step by step 
		if((neis & 0x202) != 0x202)
		{
			jumpnode_id = warthog::INF;
			jumpcost = warthog::INF;
			break;
		}
		jumpcost++;
		next_id -= mapw;

		if(next_id == goal_id)
		{
			jumpnode_id = next_id;
			break;
		}

		// compute forced
		uint8_t row0 = ((uint8_t*)&neis)[0];
		uint8_t row1 = ((uint8_t*)&neis)[1];
		if((~row1 & row0) & 0x5) { jumpnode_id = next_id; break; }

		neis <<= 8;
		map_->get_tripleh(next_id-mapw, ((uint8_t*)&neis)[0]);
	}
}

void
warthog::online_jump_point_locator::jump_south(uint32_t node_id, 
		uint32_t goal_id, uint32_t& jumpnode_id, double& jumpcost)
{
	jumpcost = 0;

	// first 3 bits of first 3 bytes represent a 3x3 cell of tiles
	// from the grid. Assume little endian format.
	uint32_t neis;
	uint32_t next_id = node_id;
	uint32_t mapw = map_->width();
	map_->get_neighbours(next_id, (uint8_t*)&neis);

	while(true)
	{
		// jump step by step 
		if((neis & 0x20200) != 0x20200)
		{
			jumpnode_id = warthog::INF;
			jumpcost = warthog::INF;
			break;
		}
		jumpcost++;
		next_id += mapw;

		if(next_id == goal_id)
		{
			jumpnode_id = next_id;
			break;
		}

		// compute forced
		uint8_t row1 = ((uint8_t*)&neis)[1];
		uint8_t row2 = ((uint8_t*)&neis)[2];
		if((~row1 & row2) & 0x5) { jumpnode_id = next_id; break; }

		neis >>= 8;
		map_->get_tripleh(next_id+mapw, ((uint8_t*)&neis)[2]);
	}
}

void
warthog::online_jump_point_locator::jump_east(uint32_t node_id, 
		uint32_t goal_id, uint32_t& jumpnode_id, double& jumpcost)
{
	jumpcost = 0;
	jumpnode_id = node_id;

	// first 3 bits of first 3 bytes represent a 3x3 cell of tiles
	// from the grid. Assume little endian format.
	uint64_t neis;
	bool deadend = false;

	while(true)
	{
		// read in 16 tiles from 3 adjacent rows
		map_->get_neighbours_16bit(node_id + jumpcost, (uint16_t*)&neis);
		if((neis & 0xfffefffefffe) == 0xfffefffefffe)
		{
			jumpcost += 14;
			continue;
		}

		if((neis & 0x00fe00fe00fe) == 0x00fe00fe00fe)
		{
			jumpcost += 6;
			neis >>= 6;
		}

		// jump step by step (NB: neis not shifted; bitmasks change instead)
		if((neis & 0x60000) != 0x60000)
		{
			deadend = true;
			break;
		}
		uint64_t forced = (~neis << 1) & neis;
		if(forced & 0x400000004) { jumpcost++; break; }

		if((neis & 0xC0000) != 0xC0000) 
		{
			deadend = true;
			jumpcost++;
			break;
		}
		if(forced & 0x800000008) { jumpcost+=2; break; }

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
		jumpcost = warthog::INF;
	}
	
}

void
warthog::online_jump_point_locator::jump_west(uint32_t node_id, 
		uint32_t goal_id, uint32_t& jumpnode_id, double& jumpcost)
{
	jumpcost = 0;
	bool deadend = false;

	uint64_t neis;
	jumpcost = 0; // begin from the node we want to step to

	// last 3 bits of these integers hold node_id and
	// its immediate neighbours (assumes little endian format)
	//uint16_t& row0 = ((uint16_t*)&neis)[0];
	//uint16_t& row1 = ((uint16_t*)&neis)[1];
	//uint16_t& row2 = ((uint16_t*)&neis)[2];

	while(true)
	{
		// read in 2 bytes from each of 3 adjacent rows
		map_->get_neighbours_upper_16bit(node_id - jumpcost, (uint16_t*)&neis);

		// jump ahead if all tiles are traversable
		if((neis & 0x7fff7fff7fff) == 0x7fff7fff7fff)
		{
			jumpcost += 14;
			continue;
		}

		// jump ahead if all tiles in first byte are traversable
		if((neis & 0x7f007f007f00) == 0x7f007f007f00)
		{
			jumpcost += 6;
			neis <<= 6;
		}

		// jump step by step (NB: neis not shifted; bitmasks change instead)
		if((neis & 0x60000000) != 0x60000000) 
		{
			deadend = true;
			break;
		}
		uint64_t forced = (~neis >> 1) & neis;
		if(forced & 0x200000002000) { jumpcost++; break; }

		if((neis & 0x30000000) != 0x30000000) 
		{
			deadend = true;
			jumpcost++;
			break;
		}
		if(forced & 0x100000001000) { jumpcost+=2; break; }

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
		jumpcost = warthog::INF;
	}
}

void
warthog::online_jump_point_locator::jump_northeast(uint32_t node_id,
	   	uint32_t goal_id, uint32_t& jumpnode_id, double& jumpcost)
{
	jumpcost = 0;

	// first 3 bits of first 3 bytes represent a 3x3 cell of tiles
	// from the grid. Assume little endian format.
	uint32_t neis;
	uint32_t next_id = node_id;
	uint32_t mapw = map_->width();
	map_->get_neighbours(next_id, (uint8_t*)&neis);

	// jump a single step at a time (no corner cutting)
	while(true)
	{
		// stop jumping if we hit an obstacle or take an invalid step
		if((neis & 1542) != 1542) // bits 9, 10, 1 and 2
		{
			jumpcost = warthog::INF;
			next_id = warthog::INF;
			break;
		}
		jumpcost += warthog::ROOT_TWO;
		next_id = next_id - mapw + 1;
		map_->get_neighbours(next_id, (uint8_t*)&neis);

		// stop jumping if we hit the goal
		if(next_id == goal_id) { break; }

		// recurse straight before stepping again diagonally;
		// (ensures we do not miss any optimal turning points)
		uint32_t jp_id; 
		double jp_cost;
		jump_north(next_id, goal_id, jp_id, jp_cost);
		if(jp_id != warthog::INF) { break; }
		jump_east(next_id, goal_id, jp_id, jp_cost);
		if(jp_id != warthog::INF) { break; }
	}
	jumpnode_id = next_id;
}

void
warthog::online_jump_point_locator::jump_northwest(uint32_t node_id, 
		uint32_t goal_id, uint32_t& jumpnode_id, double& jumpcost)
{
	jumpcost = 0;

	// first 3 bits of first 3 bytes represent a 3x3 cell of tiles
	// from the grid. Assume little endian format.
	uint32_t neis;
	uint32_t next_id = node_id;
	uint32_t mapw = map_->width();
	map_->get_neighbours(next_id, (uint8_t*)&neis);

	// jump a single step at a time (no corner cutting)
	while(true)
	{
		// stop jumping if we hit an obstacle or take an invalid step
		if((neis & 771) != 771) // bits 9, 8, 0 and 1
		{
			jumpcost = warthog::INF;
			next_id = warthog::INF;
			break;
		}
		jumpcost += warthog::ROOT_TWO;
		next_id = next_id - mapw - 1;
		map_->get_neighbours(next_id, (uint8_t*)&neis);

		// stop jumping if we hit the goal
		if(next_id == goal_id) { break; }

		// recurse straight before stepping again diagonally;
		// (ensures we do not miss any optimal turning points)
		uint32_t jp_id; 
		double jp_cost;
		jump_north(next_id, goal_id, jp_id, jp_cost);
		if(jp_id != warthog::INF) { break; }
		jump_west(next_id, goal_id, jp_id, jp_cost);
		if(jp_id != warthog::INF) { break; }
	}
	jumpnode_id = next_id;
}

void
warthog::online_jump_point_locator::jump_southeast(uint32_t node_id, 
		uint32_t goal_id, uint32_t& jumpnode_id, double& jumpcost)
{
	jumpcost = 0;

	// first 3 bits of first 3 bytes represent a 3x3 cell of tiles
	// from the grid. Assume little endian format.
	uint32_t neis;
	uint32_t next_id = node_id;
	uint32_t mapw = map_->width();
	map_->get_neighbours(next_id, (uint8_t*)&neis);

	// jump a single step at a time (no corner cutting)
	while(true)
	{
		// stop jumping if we hit an obstacle or take an invalid step
		if((neis & 394752) != 394752) // bits 9, 10, 17 and 18
		{
			jumpcost = warthog::INF;
			next_id = warthog::INF;
			break;
		}
		jumpcost += warthog::ROOT_TWO;
		next_id = next_id + mapw + 1;
		map_->get_neighbours(next_id, (uint8_t*)&neis);

		// stop jumping if we hit the goal
		if(next_id == goal_id) { break; }

		// recurse straight before stepping again diagonally;
		// (ensures we do not miss any optimal turning points)
		uint32_t jp_id; 
		double jp_cost;
		jump_south(next_id, goal_id, jp_id, jp_cost);
		if(jp_id != warthog::INF) { break; }
		jump_east(next_id, goal_id, jp_id, jp_cost);
		if(jp_id != warthog::INF) { break; }
	}
	jumpnode_id = next_id;
}

void
warthog::online_jump_point_locator::jump_southwest(uint32_t node_id, 
		uint32_t goal_id, uint32_t& jumpnode_id, double& jumpcost)
{
	jumpcost = 0;

	// first 3 bits of first 3 bytes represent a 3x3 cell of tiles
	// from the grid. Assume little endian format.
	uint32_t neis;
	uint32_t next_id = node_id;
	uint32_t mapw = map_->width();
	map_->get_neighbours(next_id, (uint8_t*)&neis);

	// jump a single step (no corner cutting)
	while(true)
	{
		// stop jumping if we hit an obstacle or take an invalid step
		if((neis & 197376) != 197376) // bits 9, 8, 16 and 17
		{
			jumpcost = warthog::INF;
			next_id = warthog::INF;
			break;
		}
		jumpcost += warthog::ROOT_TWO;
		next_id = next_id + mapw - 1;
		map_->get_neighbours(next_id, (uint8_t*)&neis);

		// stop jumping if we hit the goal
		if(next_id == goal_id) { break; }

		// recurse straight before stepping again diagonally;
		// (ensures we do not miss any optimal turning points)
		uint32_t jp_id; 
		double jp_cost;
		jump_south(next_id, goal_id, jp_id, jp_cost);
		if(jp_id != warthog::INF) { break; }
		jump_west(next_id, goal_id, jp_id, jp_cost);
		if(jp_id != warthog::INF) { break; }
	}
	jumpnode_id = next_id;
}
