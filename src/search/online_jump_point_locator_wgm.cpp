#include "weighted_gridmap.h"
#include "jps.h"
#include "online_jump_point_locator_wgm.h"

#include <cassert>
#include <climits>

warthog::online_jump_point_locator_wgm::online_jump_point_locator_wgm(
        warthog::weighted_gridmap* map) : map_(map), jumplimit_(UINT32_MAX)
{
	rmap_ = create_rmap();
}

warthog::online_jump_point_locator_wgm::~online_jump_point_locator_wgm()
{
	delete rmap_;
}

// create a copy of the grid map which is rotated by 90 degrees clockwise.
// this version will be used when jumping North or South. 
warthog::weighted_gridmap*
warthog::online_jump_point_locator_wgm::create_rmap()
{
	uint32_t maph = map_->header_height();
	uint32_t mapw = map_->header_width();
	uint32_t rmaph = mapw;
	uint32_t rmapw = maph;
	warthog::weighted_gridmap* rmap = new warthog::weighted_gridmap(rmaph, rmapw);

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
warthog::online_jump_point_locator_wgm::jump(warthog::jps::direction d,
	   	uint32_t node_id, uint32_t goal_id, uint32_t& jumpnode_id, 
		warthog::cost_t& jumpcost)
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
warthog::online_jump_point_locator_wgm::jump_north(uint32_t node_id, 
		uint32_t goal_id, uint32_t& jumpnode_id, warthog::cost_t& jumpcost)
{
	node_id = this->map_id_to_rmap_id(node_id);
	goal_id = this->map_id_to_rmap_id(goal_id);
	__jump_north(node_id, goal_id, jumpnode_id, jumpcost, rmap_);
	jumpnode_id = this->rmap_id_to_map_id(jumpnode_id);
}

void
warthog::online_jump_point_locator_wgm::__jump_north(uint32_t node_id, 
		uint32_t goal_id, uint32_t& jumpnode_id, warthog::cost_t& jumpcost,
		warthog::weighted_gridmap* mymap)
{
	// jumping north in the original map is the same as jumping
	// east when we use a version of the map rotated 90 degrees.
	__jump_east(node_id, goal_id, jumpnode_id, jumpcost, rmap_);
}

void
warthog::online_jump_point_locator_wgm::jump_south(uint32_t node_id, 
		uint32_t goal_id, uint32_t& jumpnode_id, warthog::cost_t& jumpcost)
{
	node_id = this->map_id_to_rmap_id(node_id);
	goal_id = this->map_id_to_rmap_id(goal_id);
	__jump_south(node_id, goal_id, jumpnode_id, jumpcost, rmap_);
	jumpnode_id = this->rmap_id_to_map_id(jumpnode_id);
}

void
warthog::online_jump_point_locator_wgm::__jump_south(uint32_t node_id, 
		uint32_t goal_id, uint32_t& jumpnode_id, warthog::cost_t& jumpcost,
		warthog::weighted_gridmap* mymap)
{
	// jumping north in the original map is the same as jumping
	// west when we use a version of the map rotated 90 degrees.
	__jump_west(node_id, goal_id, jumpnode_id, jumpcost, rmap_);
}

void
warthog::online_jump_point_locator_wgm::jump_east(uint32_t node_id, 
		uint32_t goal_id, uint32_t& jumpnode_id, warthog::cost_t& jumpcost)
{
	__jump_east(node_id, goal_id, jumpnode_id, jumpcost, map_);
}


void
warthog::online_jump_point_locator_wgm::__jump_east(uint32_t node_id, 
		uint32_t goal_id, uint32_t& jumpnode_id, warthog::cost_t& jumpcost, 
		warthog::weighted_gridmap* mymap)
{
    
	
}

// analogous to ::jump_east 
void
warthog::online_jump_point_locator_wgm::jump_west(uint32_t node_id, 
		uint32_t goal_id, uint32_t& jumpnode_id, warthog::cost_t& jumpcost)
{
	__jump_west(node_id, goal_id, jumpnode_id, jumpcost, map_);
}

void
warthog::online_jump_point_locator_wgm::__jump_west(uint32_t node_id, 
		uint32_t goal_id, uint32_t& jumpnode_id, warthog::cost_t& jumpcost, 
		warthog::weighted_gridmap* mymap)
{

}

void
warthog::online_jump_point_locator_wgm::jump_northeast(uint32_t node_id,
	   	uint32_t goal_id, uint32_t& jumpnode_id, warthog::cost_t& jumpcost)
{

}

void
warthog::online_jump_point_locator_wgm::jump_northwest(uint32_t node_id, 
		uint32_t goal_id, uint32_t& jumpnode_id, warthog::cost_t& jumpcost)
{

}

void
warthog::online_jump_point_locator_wgm::jump_southeast(uint32_t node_id, 
		uint32_t goal_id, uint32_t& jumpnode_id, warthog::cost_t& jumpcost)
{

}

void
warthog::online_jump_point_locator_wgm::jump_southwest(uint32_t node_id, 
		uint32_t goal_id, uint32_t& jumpnode_id, warthog::cost_t& jumpcost)
{

}
