#include "jpsplus_expansion_policy.h"
#include "jps_record.h"

warthog::jpsplus_expansion_policy::jpsplus_expansion_policy(warthog::gridmap* map)
{
	map_ = map;
	nodepool_ = new warthog::blocklist2(map->height(), map->width());
	jpl_ = new warthog::offline_jump_point_locator(map);
	start_node_ = new warthog::jps_record(UINT32_MAX);
	goal_node_ = new warthog::jps_record(UINT32_MAX);

	neighbours_.reserve(100);
	costs_.reserve(100);

	search_id_ = warthog::INF;
	verbose_ = 0;

	reset();
}

warthog::jpsplus_expansion_policy::~jpsplus_expansion_policy()
{
	delete jpl_;
	delete nodepool_;
	delete start_node_;
	delete goal_node_;
}

void 
warthog::jpsplus_expansion_policy::expand(
		warthog::search_node* current, warthog::problem_instance* problem)
{
	reset();

	// insert start + goal into the graph
	// NB: start_node_ and goal_node_ are pre-allocated
	if(search_id_ != problem->get_searchid())
	{
		search_id_ = problem->get_searchid();
		jpl_->undo_prior_insertions();
		start_node_->set_id(problem->get_start());
		goal_node_->set_id(problem->get_goal());
		jpl_->insert(start_node_, goal_node_);
	}

	// compute the direction of travel used to reach the current node.
	warthog::jps::direction dir_c = current->get_pdir();

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
			warthog::arraylist<jps_label> const * labels = 
			jpl_->jump(d, current_id, goal_id);

			for(uint32_t j = 0; j < labels->size(); j++)
			{
				warthog::jps_label label = labels->at(j);

				warthog::jps::direction pdir = label.get_dir();
				warthog::search_node* mynode = nodepool_->generate(label.get_id());
				warthog::cost_t cost_to_nei = (label.dsteps_ * warthog::ROOT_TWO) +
				   	(label.ssteps_ * warthog::ONE);
				neighbours_.push_back(mynode);
				costs_.push_back(cost_to_nei);

				if(mynode->get_searchid() != search_id_) { mynode->reset(search_id_); }

				// TODO: fix this stupid hack; this update should be performed during node
				// relaxation (ie. invoked by flexible_astar, not here).
				if((current->get_g() + cost_to_nei) < mynode->get_g())
				{
					mynode->set_pdir(pdir);
				}
			}
		}
	}

	num_neighbours_ = neighbours_.size();

	// terminator (historical; yeah, this code is stupid)
	neighbours_.push_back(0);
	costs_.push_back(0);
}

