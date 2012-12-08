#ifndef WARTHOG_ONLINE_JUMP_POINT_LOCATOR_H
#define WARTHOG_ONLINE_JUMP_POINT_LOCATOR_H

// online_jump_point_locator.h
//
// A class wrapper around some code that finds, online, jump point
// successors of an arbitrary nodes in a uniform-cost grid map.
//
// For theoretical details see:
// [Harabor D. and Grastien A, 2011, 
// Online Graph Pruning Pathfinding on Grid Maps, AAAI]
//
// @author: dharabor
// @created: 03/09/2012
//


class warthog::gridmap;
namespace warthog
{

class online_jump_point_locator 
{
	public: 
		online_jump_point_locator(warthog::gridmap* map);
		~online_jump_point_locator();

		void
		jump(warthog::jps::direction d, uint32_t node_id, uint32_t goalid, 
				uint32_t& jumpnode_id, double& jumpcost);

	private:
		void
		jump_northwest(uint32_t node_id, uint32_t goal_id, 
				uint32_t& jumpnode_id, double& jumpcost);
		void
		jump_northeast(uint32_t node_id, uint32_t goal_id, 
				uint32_t& jumpnode_id, double& jumpcost);
		void
		jump_southwest(uint32_t node_id, uint32_t goal_id, 
				uint32_t& jumpnode_id, double& jumpcost);
		void
		jump_southeast(uint32_t node_id, uint32_t goal_id, 
				uint32_t& jumpnode_id, double& jumpcost);
		void
		jump_north(uint32_t node_id, uint32_t goal_id, 
				uint32_t& jumpnode_id, double& jumpcost);
		void
		jump_south(uint32_t node_id, uint32_t goal_id, 
				uint32_t& jumpnode_id, double& jumpcost);
		void
		jump_east(uint32_t node_id, uint32_t goal_id, 
				uint32_t& jumpnode_id, double& jumpcost);
		void
		jump_west(uint32_t node_id, uint32_t goal_id, 
				uint32_t& jumpnode_id, double& jumpcost);

		warthog::gridmap* map_;
		uint32_t jumplimit_;
};

}

#endif

