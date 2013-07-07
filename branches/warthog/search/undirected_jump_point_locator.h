#ifndef WARTHOG_UNDIRECTED_JUMP_POINT_LOCATOR_H
#define WARTHOG_UNDIRECTED_JUMP_POINT_LOCATOR_H

// undirected_jump_point_locator.h
//
// Identifies nodes that are jump points in a given cardinal
// travel direction as well as jump points in the opposite 
// cardinal direction.
//
// Similar implementation to warthog::online_jump_point_locator.
//
// @author: dharabor
// @created: 03/09/2012
//

#include "arraylist.h"
#include "jps.h"
#include "jps_record.h"

class warthog::gridmap;
namespace warthog
{

class undirected_jump_point_locator
{
	public: 
		undirected_jump_point_locator(warthog::gridmap* map);
		~undirected_jump_point_locator();

		void
		jump(warthog::jps::direction d, uint32_t node_id, uint32_t goalid, 
				warthog::arraylist<warthog::jps_label>& jpoints);

		uint32_t 
		mem()
		{
			return sizeof(this) + rmap_->mem();
		}

	private:
		void
		jump_north(warthog::arraylist<warthog::jps_label>& jpoints);

		void
		jump_south(warthog::arraylist<warthog::jps_label>& jpoints); 

		void
		jump_east(warthog::arraylist<warthog::jps_label>& jpoints);

		void
		jump_west(warthog::arraylist<warthog::jps_label>& jpoints);

		void
		jump_northeast(warthog::arraylist<warthog::jps_label>& jpoints);

		void
		jump_northwest(warthog::arraylist<warthog::jps_label>& jpoints);

		void
		jump_southeast(warthog::arraylist<warthog::jps_label>& jpoints);

		void
		jump_southwest(warthog::arraylist<warthog::jps_label>& jpoints);

		// these versions can be passed a map parameter to
		// use when jumping. they allow switching between
		// map_ and rmap_ (a rotated counterpart).
		void
		__jump_north(uint32_t node_id, uint32_t goal_id, 
				uint32_t& jumpnode_id, warthog::cost_t& jumpcost,
				warthog::gridmap* mymap);
		void
		__jump_south(uint32_t node_id, uint32_t goal_id, 
				uint32_t& jumpnode_id, warthog::cost_t& jumpcost, 
				warthog::gridmap* mymap);
		void
		__jump_east(uint32_t node_id, uint32_t goal_id, 
				uint32_t& jumpnode_id, warthog::cost_t& jumpcost, 
				warthog::gridmap* mymap);
		void
		__jump_west(uint32_t node_id, uint32_t goal_id, 
				uint32_t& jumpnode_id, warthog::cost_t& jumpcost, 
				warthog::gridmap* mymap);

		// these versions perform a single diagonal jump, returning
		// the intermediate diagonal jump point and the straight 
		// jump points that caused the jumping process to stop
		void
		__jump_northeast(
				uint32_t& node_id, uint32_t& rnode_id, 
				uint32_t goal_id, uint32_t rgoal_id,
				uint32_t& jumpnode_id, warthog::cost_t& jumpcost,
				uint32_t& jp1_id, warthog::cost_t& jp1_cost,
				uint32_t& jp2_id, warthog::cost_t& jp2_cost);
		void
		__jump_northwest(
				uint32_t& node_id, uint32_t& rnode_id, 
				uint32_t goal_id, uint32_t rgoal_id,
				uint32_t& jumpnode_id, warthog::cost_t& jumpcost,
				uint32_t& jp1_id, warthog::cost_t& jp1_cost,
				uint32_t& jp2_id, warthog::cost_t& jp2_cost);
		void
		__jump_southeast(
				uint32_t& node_id, uint32_t& rnode_id, 
				uint32_t goal_id, uint32_t rgoal_id,
				uint32_t& jumpnode_id, warthog::cost_t& jumpcost,
				uint32_t& jp1_id, warthog::cost_t& jp1_cost,
				uint32_t& jp2_id, warthog::cost_t& jp2_cost);
		void
		__jump_southwest(
				uint32_t& node_id, uint32_t& rnode_id, 
				uint32_t goal_id, uint32_t rgoal_id,
				uint32_t& jumpnode_id, warthog::cost_t& jumpcost,
				uint32_t& jp1_id, warthog::cost_t& jp1_cost,
				uint32_t& jp2_id, warthog::cost_t& jp2_cost);

		// functions to convert map indexes to rmap indexes
		inline uint32_t
		map_id_to_rmap_id(uint32_t mapid)
		{
			if(mapid == warthog::INF) { return mapid; }

			uint32_t x, y;
			uint32_t rx, ry;
			map_->to_unpadded_xy(mapid, x, y);
			ry = x;
			rx = map_->header_height() - y - 1;
			return rmap_->to_padded_id(rx, ry);
		}

		// convert rmap indexes to map indexes
		inline uint32_t
		rmap_id_to_map_id(uint32_t rmapid)
		{
			if(rmapid == warthog::INF) { return rmapid; }

			uint32_t x, y;
			uint32_t rx, ry;
			rmap_->to_unpadded_xy(rmapid, rx, ry);
			x = ry;
			y = rmap_->header_width() - rx - 1;
			return map_->to_padded_id(x, y);
		}

		warthog::gridmap*
		create_rmap();

		warthog::gridmap* map_;
		warthog::gridmap* rmap_;
		uint32_t jumplimit_;

		uint32_t current_goal_id_;
		uint32_t current_rgoal_id_;
		uint32_t current_node_id_;
		uint32_t current_rnode_id_;

		bool undirected_;
};

}

#endif

