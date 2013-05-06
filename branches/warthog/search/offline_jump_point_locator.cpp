#include "gridmap.h"
#include "online_jump_point_locator.h"
#include "offline_jump_point_locator.h"

warthog::offline_jump_point_locator::offline_jump_point_locator(
		warthog::gridmap* map) : map_(map)
{
	dbsize_ = 8*map_->height() * map_->width();
	db_ = new uint8_t[dbsize_];
	for(uint32_t i=0; i < dbsize_; i++) db_[i] = 0;
	preproc();
}

warthog::offline_jump_point_locator::~offline_jump_point_locator()
{
	delete [] db_;
}

void
warthog::offline_jump_point_locator::preproc()
{
	warthog::online_jump_point_locator jpl(map_);
	for(uint32_t y = 0; y < map_->header_height(); y++)
	{
		for(uint32_t x = 0; x < map_->header_width(); x++)
		{
			uint32_t mapid = map_->to_padded_id(x, y);
			//uint32_t mapid = y * map_->width() + x;
			//uint32_t tmp = map_->to_padded_id(mapid);
			std::cout << mapid << " ";
			for(int i = 0; i < 8; i++)
			{
				warthog::jps::direction dir = 
					(warthog::jps::direction)(1 << i);
				std::cout << dir << ": ";
				uint32_t jumpnode_id;
				double jumpcost;
				jpl.jump(dir, mapid,
						warthog::INF, jumpnode_id, jumpcost);
				
				// convert from cost to number of steps
				if(dir > 8)
				{
					jumpcost = (jumpcost / warthog::ROOT_TWO);
				}
				uint32_t num_steps = (uint8_t)floor((jumpcost + 0.5));
				std::cout << (jumpnode_id == warthog::INF ? 0 : num_steps) << " ";

				// truncate result so we can fit the label into a single byte
				if(num_steps > 127)
				{
					num_steps = 127;
					jumpnode_id = 0; 
				}

				// store the label and set the leading bit if the 
				// jump leads to a dead-end
				db_[mapid*8 + i] = num_steps;
				if(jumpnode_id == warthog::INF)
				{
					db_[mapid*8 + i] |= 128;
				}
			}
			std::cout << std::endl;
		}
	}
}

void
warthog::offline_jump_point_locator::jump(warthog::jps::direction d, 
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
warthog::offline_jump_point_locator::jump_northwest(uint32_t node_id,
	  	uint32_t goal_id, uint32_t& jumpnode_id, double& jumpcost)
{
	uint32_t mapw = map_->width();
	uint8_t label = db_[8*node_id + 5];
	uint8_t num_steps = label & 127;

	// goal test (so many div ops! and branches! how ugly!)
	uint32_t id_delta = (mapw + 1) * num_steps;
	uint32_t goal_delta = node_id - goal_id;
	if(id_delta > goal_delta)
	{
		uint32_t gx, gy, nx, ny;
		map_->to_padded_xy(goal_id, gx, gy);
		map_->to_padded_xy(node_id, nx, ny);

		uint32_t ydelta = (ny - gy);
		uint32_t xdelta = (nx - gx); 
		uint32_t nid, steps_to_nid;
		if(ydelta < xdelta)
		{
			steps_to_nid = ydelta;
			nid = node_id - (mapw + 1) * steps_to_nid;
			jump_west(nid, goal_id, jumpnode_id, jumpcost);
		}
		else
		{
			steps_to_nid = xdelta;
			nid = node_id - (mapw + 1) * xdelta;
			jump_north(nid, goal_id, jumpnode_id, jumpcost);
		}

		// return the goal instead of the jump point
		if(jumpnode_id == goal_id) 
		{ 
			jumpcost = steps_to_nid * warthog::ROOT_TWO + jumpcost;
			return; 
		}
	}

	// return the jump point; but only if it isn't sterile
	jumpnode_id = node_id - id_delta;
	jumpcost = num_steps * warthog::ROOT_TWO;
	if(label & 128) { jumpnode_id = warthog::INF; }
}

void
warthog::offline_jump_point_locator::jump_northeast(uint32_t node_id,
	  	uint32_t goal_id, uint32_t& jumpnode_id, double& jumpcost)
{
	uint8_t label = db_[8*node_id + 4];
	uint8_t num_steps = label & 127;
	uint32_t mapw = map_->width();

	// goal test (so many div ops! and branches! how ugly!)
	uint32_t id_delta = (mapw - 1) * num_steps;
	uint32_t goal_delta = node_id - goal_id;
	if(id_delta > goal_delta)
	{
		uint32_t gx, gy, nx, ny;
		map_->to_padded_xy(goal_id, gx, gy);
		map_->to_padded_xy(node_id, nx, ny);

		uint32_t ydelta = (ny - gy);
		uint32_t xdelta = (gx - nx); 
		uint32_t nid, steps_to_nid;
		if(ydelta < xdelta)
		{
			steps_to_nid = ydelta;
			nid = node_id - (mapw - 1) * steps_to_nid;
			jump_east(nid, goal_id, jumpnode_id, jumpcost);
		}
		else
		{
			steps_to_nid = xdelta;
			nid = node_id - (mapw - 1) * xdelta;
			jump_north(nid, goal_id, jumpnode_id, jumpcost);
		}

		// return the goal instead of the jump point
		if(jumpnode_id == goal_id) 
		{ 
			jumpcost = steps_to_nid * warthog::ROOT_TWO + jumpcost;
			return; 
		}
	}

	// return the jump point; but only if it isn't sterile
	jumpnode_id = node_id + id_delta;
	jumpcost = num_steps * warthog::ROOT_TWO;
	if(label & 128) { jumpnode_id = warthog::INF; }
}

void
warthog::offline_jump_point_locator::jump_southwest(uint32_t node_id,
	  	uint32_t goal_id, uint32_t& jumpnode_id, double& jumpcost)
{
	uint8_t label = db_[8*node_id + 7];
	uint8_t num_steps = label & 127;
	uint32_t mapw = map_->width();


	// goal test (so many div ops! and branches! how ugly!)
	uint32_t id_delta = (mapw - 1) * num_steps;  
	uint32_t goal_delta = goal_id - node_id;
	if(id_delta > goal_delta)
	{
		uint32_t gx, gy, nx, ny;
		map_->to_padded_xy(goal_id, gx, gy);
		map_->to_padded_xy(node_id, nx, ny);

		uint32_t ydelta = (gy - ny);
		uint32_t xdelta = (nx - gx); 
		uint32_t nid, steps_to_nid;
		if(ydelta < xdelta)
		{
			steps_to_nid = ydelta;
			nid = node_id + (mapw - 1) * steps_to_nid;
			jump_west(nid, goal_id, jumpnode_id, jumpcost);
		}
		else
		{
			steps_to_nid = xdelta;
			nid = node_id + (mapw - 1) * xdelta;
			jump_south(nid, goal_id, jumpnode_id, jumpcost);
		}

		// return the goal instead of the jump point
		if(jumpnode_id == goal_id) 
		{ 
			jumpcost = steps_to_nid * warthog::ROOT_TWO + jumpcost;
			return; 
		}
	}

	// return the jump point; but only if it isn't sterile
	jumpnode_id = node_id + id_delta;
	jumpcost = num_steps * warthog::ROOT_TWO;
	if(label & 128) { jumpnode_id = warthog::INF; }
}

void
warthog::offline_jump_point_locator::jump_southeast(uint32_t node_id,
	  	uint32_t goal_id, uint32_t& jumpnode_id, double& jumpcost)
{
	uint8_t label = db_[8*node_id + 6];
	uint8_t num_steps = label & 127;
	uint32_t mapw = map_->width();
	
	// goal test (so many div ops! and branches! how ugly!)
	uint32_t id_delta = (mapw + 1) * num_steps;  
	uint32_t goal_delta = goal_id - node_id;
	if(id_delta > goal_delta)
	{
		uint32_t gx, gy, nx, ny;
		map_->to_padded_xy(goal_id, gx, gy);
		map_->to_padded_xy(node_id, nx, ny);

		uint32_t ydelta = (gy - ny);
		uint32_t xdelta = (gx - nx); 
		uint32_t nid, steps_to_nid;
		if(ydelta < xdelta)
		{
			steps_to_nid = ydelta;
			nid = node_id + (mapw + 1) * steps_to_nid;
			jump_east(nid, goal_id, jumpnode_id, jumpcost);
		}
		else
		{
			steps_to_nid = xdelta;
			nid = node_id + (mapw + 1) * xdelta;
			jump_south(nid, goal_id, jumpnode_id, jumpcost);
		}

		// return the goal instead of the jump point
		if(jumpnode_id == goal_id) 
		{ 
			jumpcost = steps_to_nid * warthog::ROOT_TWO + jumpcost;
			return; 
		}
	}


	jumpcost = num_steps * warthog::ROOT_TWO;
	jumpnode_id = node_id + (mapw + 1) * num_steps;
	if(label & 128) { jumpnode_id = warthog::INF; }
}

void
warthog::offline_jump_point_locator::jump_north(uint32_t node_id,
	  	uint32_t goal_id, uint32_t& jumpnode_id, double& jumpcost)
{
	uint8_t label = db_[8*node_id];
	uint8_t num_steps = label & 127;

	// do not jump over the goal
	uint32_t id_delta = num_steps * map_->width();
	uint32_t goal_delta = node_id - goal_id;
	if(id_delta > goal_delta)
	{
		uint32_t gx = goal_id % map_->width();
		uint32_t nx = goal_id % map_->width();
		if(nx == gx) 
		{ 
			jumpnode_id = goal_id; 
			jumpcost = goal_delta / map_->width();
			return;
		}
	}

	// return the jump point at hand
	jumpnode_id = node_id - id_delta;
	jumpcost = num_steps;
	if(label & 128) { jumpnode_id = warthog::INF; }
}

void
warthog::offline_jump_point_locator::jump_south(uint32_t node_id,
	  	uint32_t goal_id, uint32_t& jumpnode_id, double& jumpcost)
{
	uint8_t label = db_[8*node_id + 1];
	uint8_t num_steps = label & 127;
	
	// do not jump over the goal
	uint32_t id_delta = jumpcost * map_->width();
	uint32_t goal_delta = goal_id - node_id;
	if(id_delta > goal_delta)
	{
		uint32_t gx = goal_id % map_->width();
		uint32_t nx = goal_id % map_->width();
		if(nx == gx) 
		{ 
			jumpnode_id = goal_id; 
			jumpcost = goal_delta / map_->width();
			return;
		}
	}

	// return the jump point at hand
	jumpnode_id = node_id + id_delta;
	jumpcost = num_steps;
	if(label & 128) { jumpnode_id = warthog::INF; }
}

void
warthog::offline_jump_point_locator::jump_east(uint32_t node_id,
	  	uint32_t goal_id, uint32_t& jumpnode_id, double& jumpcost)
{
	uint8_t label = db_[8*node_id + 2];

	// do not jump over the goal
	uint32_t id_delta = label & 127;
	uint32_t goal_delta = goal_id - node_id;
	if(id_delta > goal_delta)
	{
		jumpnode_id = goal_id;
		jumpcost = goal_delta;
		return;
	}

	// return the jump point at hand
	jumpnode_id = node_id + id_delta;
	jumpcost = id_delta;
	if(label & 128) { jumpnode_id = warthog::INF; }
}

void
warthog::offline_jump_point_locator::jump_west(uint32_t node_id,
	  	uint32_t goal_id, uint32_t& jumpnode_id, double& jumpcost)
{
	uint8_t label = db_[8*node_id + 3];

	// do not jump over the goal
	uint32_t id_delta = label & 127;
	uint32_t goal_delta = node_id - goal_id;
	if(id_delta > goal_delta)
	{
		jumpnode_id = goal_id;
		jumpcost = goal_delta;
		return;
	}

	// return the jump point at hand
	jumpnode_id = node_id - id_delta;
	jumpcost = id_delta;
	if(label & 128) { jumpnode_id = warthog::INF; }
}

