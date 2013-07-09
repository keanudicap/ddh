#ifndef WARTHOG_OFFLINE_JUMP_POINT_LOCATOR_H
#define WARTHOG_OFFLINE_JUMP_POINT_LOCATOR_H

// offline_jump_point_locator.h
//
// Identifies jump points using a pre-computed database that stores
// distances from each node to jump points in every direction.
//
// @author: dharabor
// @created: 05/05/2013
//

#include "arraylist.h"
#include "jps.h"
#include "jps_graph.h"
#include "jps_record.h"
#include "undirected_jump_point_locator.h"

namespace warthog
{

class gridmap;
class offline_jump_point_locator
{
	public:
		offline_jump_point_locator(warthog::gridmap* map);
		~offline_jump_point_locator();

		warthog::arraylist<warthog::jps_label> const *
		jump(warthog::jps::direction d, uint32_t node_id, uint32_t goal_id);

		void
		insert(warthog::jps_record* start_node, 
				warthog::jps_record* goal_node);

		void
		undo_prior_insertions();

		inline bool
		get_verbose() { return verbose_; }

		inline void
		set_verbose(bool verbose) { verbose_ = verbose; std::cerr << "new verbose: "<<verbose_ << std::endl;  } 

		uint32_t
		mem();

	private:
		void
		preproc_identify();

		void
		preproc_build_graph();

		void
		init_jpmap();

		bool
		load(const char* filename);

		void 
		save(const char* filename);

		void
		scan_up(warthog::jps_record* source);

		void
		scan_down(warthog::jps_record* source);

		void
		scan_left(warthog::jps_record* source);

		void
		scan_right(warthog::jps_record* source);

		void
		insert_nongoal(warthog::jps_record* source, warthog::jps_record* goal);

		warthog::gridmap* map_;
		warthog::gridmap* jpmap_;
		warthog::undirected_jump_point_locator* jpl_;
		warthog::jps_graph* graph_;
		warthog::arraylist<warthog::arraylist<warthog::jps_label>*>* modified_lists_;

		warthog::jps_record* start_;
		warthog::jps_record* goal_;
		warthog::jps_record* shadow_goal_;
		bool verbose_;
};

}

#endif

