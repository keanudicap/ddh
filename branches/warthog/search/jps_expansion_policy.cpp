#include "jps_expansion_policy.h"

warthog::jps_expansion_policy::jps_expansion_policy(warthog::gridmap* map)
{
	map_ = map;
	nodepool_ = new warthog::blocklist(map->height(), map->width());
	jpl_ = new warthog::online_jump_point_locator(map);

	rmap_ = create_rmap();
	rjpl_ = new warthog::online_jump_point_locator(rmap_);

	reset();
}

// create a copy of the grid map which is rotated by 90 degrees clockwise.
// also create a corresponding JPL object for jumping around using this map.
warthog::gridmap*
warthog::jps_expansion_policy::create_rmap()
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

warthog::jps_expansion_policy::~jps_expansion_policy()
{
	delete rjpl_;
	delete rmap_;
	delete jpl_;
	delete nodepool_;
}

void 
warthog::jps_expansion_policy::expand(
		warthog::search_node* current, warthog::problem_instance* problem)
{
	reset();

	// compute the direction of travel used to reach the current node.
	warthog::jps::direction dir_c =
	   	this->compute_direction(current->get_parent(), current);

	// get the tiles around the current node c
	uint32_t c_tiles;
	uint32_t current_id = current->get_id();
	map_->get_neighbours(current_id, (uint8_t*)&c_tiles);

	// look for jump points in the direction of each natural 
	// and forced neighbour
	uint32_t succ_dirs = warthog::jps::compute_successors(dir_c, c_tiles);
	uint32_t goal_id = problem->get_goal();
	for(uint32_t i = 0; i < 8; i++)
	{
		warthog::jps::direction d = (warthog::jps::direction) (1 << i);
		if(succ_dirs & d)
		{
			double jumpcost;
			uint32_t succ_id;

			switch(d)
			{
				case warthog::jps::SOUTH:
				{
					uint32_t rcurrent = this->map_id_to_rmap_id(current_id);
					uint32_t rgoal = this->map_id_to_rmap_id(goal_id);
					uint32_t rsucc;
					rjpl_->jump(warthog::jps::WEST, rcurrent, rgoal, rsucc, jumpcost);
					succ_id = rsucc==warthog::INF?rsucc:rmap_id_to_map_id(rsucc);
					break;
				}
				case warthog::jps::NORTH:
				{
					uint32_t rcurrent = this->map_id_to_rmap_id(current_id);
					uint32_t rgoal = this->map_id_to_rmap_id(goal_id);
					uint32_t rsucc;
					rjpl_->jump(warthog::jps::EAST, rcurrent, rgoal, rsucc, jumpcost);
					succ_id = rsucc==warthog::INF?rsucc:rmap_id_to_map_id(rsucc);
					break;
				}
				default:
				{
					jpl_->jump(d, current_id, goal_id, succ_id, jumpcost);
					break;
				}
			}

			if(succ_id != warthog::INF)
			{
				neighbours_[num_neighbours_] = nodepool_->generate(succ_id);
				costs_[num_neighbours_] = jumpcost;
				// move terminator character as we go
				neighbours_[++num_neighbours_] = 0;
			}
		}
	}
}

