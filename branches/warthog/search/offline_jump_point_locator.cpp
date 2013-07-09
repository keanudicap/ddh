#define __STDC_FORMAT_MACROS
#include "gridmap.h"
#include "octile_heuristic.h"
#include "offline_jump_point_locator.h"

#include <algorithm>
#include <assert.h>
#include <inttypes.h>
#include <stdio.h>


warthog::offline_jump_point_locator::offline_jump_point_locator(
		warthog::gridmap* map) :
   	map_(map), 
	jpl_(new undirected_jump_point_locator(map_)), 
	modified_lists_(new warthog::arraylist<
			warthog::arraylist<warthog::jps_label>*>(1, 1, 1)),
	verbose_(0)
{
	if(map_->padded_mapsize() > ((1 << 23)-1)) 
	{
		// search nodes are stored as 32bit quantities.
		// bit 0 stores the expansion status
		// bits 1-23 store the unique node id
		// bits 24-31 store the direction from the parent
		std::cerr << "map size too big for this implementation of JPS+."
			<< " aborting."<< std::endl;
		exit(1);
	}


	// try to load a preprocessed database;
	// if no such database can be found, we create a new one.
	graph_ = new warthog::jps_graph(map_->height() * map_->width());
	if(!load(map_->filename()))
	{
		preproc_identify();
		std::cout << "graph size: "<<graph_->size() << " jp nodes.\n";
		preproc_build_graph();
		save(map_->filename());
	}
	init_jpmap();
}

warthog::offline_jump_point_locator::~offline_jump_point_locator()
{
	undo_prior_insertions();

	delete graph_;
	delete jpl_;
	delete modified_lists_;
	delete jpmap_;
}

void
warthog::offline_jump_point_locator::init_jpmap()
{
	jpmap_ = new gridmap(map_->header_height(), map_->header_width());
	for(uint32_t y = 0; y < jpmap_->header_height(); y++)
	{
		for(uint32_t x = 0; x < jpmap_->header_width(); x++)
		{
			uint32_t mapid = jpmap_->to_padded_id(x, y);
			bool label = 0;
			if(graph_->find(mapid))
			{
				label = 1;
				assert(mapid == graph_->find(mapid)->get_id());
			}
			jpmap_->set_label(mapid, label);
		}
	}
}

void
warthog::offline_jump_point_locator::preproc_build_graph()
{
	// generate all successors of every jump point in the graph
	for(uint32_t index = 0; index < map_->width() * map_->height(); index++)
	{
		warthog::jps_record* tmp = graph_->find(index);
		if(tmp == 0) { continue; }
	//	warthog::jps_record* tmp = (*graph_->find(916523)).second;
	//	uint32_t x, y;
	//	map_->to_unpadded_xy(tmp->get_id(), x, y);
	//	std::cerr << "node: "<<tmp->get_id() << " ("<< x << ", "<<y<<")"<<std::endl;

		for(int i = 0; i < 8; i++)
		{
			warthog::jps::direction dir = 
				(warthog::jps::direction)(1 << i);
		//	warthog::jps::direction 
		//		dir = warthog::jps::NORTHWEST;
			
			warthog::arraylist<jps_label>* labels = tmp->get_list(dir);
			jpl_->jump(dir, tmp->get_id(), UINT32_MAX, *labels);
			for(uint32_t j=0; j < labels->size(); j++)
			{
				//map_->to_unpadded_xy(labels->at(j).get_id(), x, y);
				//std::cerr << "\tsucc: "<<tmp->get_id() << " ("<< x << ", "<<y<<")"
				//	<< "gpdir: "<<dir<<" pdir: "<< labels->at(i).get_dir() <<std::endl;
				
				uint32_t succ_id = labels->at(j).get_id();
				if(graph_->find(succ_id) == 0)
				{
					std::cerr << "err. fatal. trying to connect a node to a successor that "
						"doesn't exist in the preprocessed graph.\n";
					uint32_t x, y;
					map_->to_unpadded_xy(tmp->get_id(), x, y);
					std::cerr << "source: "<<tmp->get_id() << " ("<< x << ", "<<y<<")"<<std::endl;
					map_->to_unpadded_xy(succ_id, x, y);
					std::cerr << "succ: " << succ_id << " ("<< x << ", "<<y<<")"<<std::endl;
					exit(1);
				}
			}

//			std::cout << "\t" << tmp->get_id() << " has "<<
//				labels->size() << " successors in dir "<<dir<<std::endl;
		}
//		std::cout << tmp->get_id() << " has "<<tmp->num_successors() << 
//				" successors in total"<<std::endl;
		if(tmp->num_successors() == 0)
		{
			std::cerr << "err. fatal. jp with zero successors\n";
			uint32_t x, y;
			map_->to_unpadded_xy(tmp->get_id(), x, y);
			std::cerr << "source: "<<tmp->get_id() << " ("<< x << ", "<<y<<")"<<std::endl;
			exit(1);
		}
		assert(tmp->num_successors() > 0);
	}	
}

void
warthog::offline_jump_point_locator::preproc_identify()
{
	// identify the set of jump points that comprise the graph
	warthog::arraylist<warthog::jps_label> labels(1, 1, 1);

	for(uint32_t y = 0; y < map_->header_height(); y++)
	{
		for(uint32_t x = 0; x < map_->header_width(); x++)
		{
			uint32_t mapid = map_->to_padded_id(x, y);
			for(int i = 0; i < 4; i++)
			{
				labels.clear();
				warthog::jps::direction dir = 
					(warthog::jps::direction)(1 << i);

				jpl_->jump(dir, mapid, warthog::INF, labels);
				if(labels.size() == 0) { continue; }
				
				uint32_t jumpnode_id = labels.at(0).get_id();
				if(graph_->find(jumpnode_id) == 0)
				{
					jps_record* tmp = new jps_record(jumpnode_id);
					graph_->insert(tmp);
					assert(graph_->find(jumpnode_id));
					//std::cout << "jumpnode: "<<jumpnode_id << " added " << std::endl;
				}
			}
		}
	}
}


bool
warthog::offline_jump_point_locator::load(const char* filename)
{
	char fname[256];
	strcpy(fname, filename);
	strcat(fname, ".jps+2");
	FILE* f = fopen(fname, "rb");
	std::cerr << "loading "<<fname << "... ";
	if(f == NULL) 
	{
		std::cerr << "no dice. oh well. keep going.\n"<<std::endl;
		return false;
	}

	while(!ferror(f))
	{
		uint32_t id;
		fread(&id, sizeof(id), 1, f);
		if(feof(f)) { break; }

#ifndef NDEBUG
		if(verbose_)
		{
			std::cerr << "load: node "<<id<<std::endl;
		}
#endif
		
		jps_record* tmp = new jps_record(id);
		for(uint32_t i = 0; i < 8; i++)
		{
			uint32_t list_size;
			uint8_t dir;
			fread(&dir, sizeof(dir), 1, f);
			fread(&list_size, sizeof(list_size), 1, f);

			warthog::arraylist<warthog::jps_label>* labels =
				tmp->get_list((warthog::jps::direction)(dir));
			for(uint32_t j = 0; j < list_size; j++)
			{
				warthog::jps_label label;
				fread(&label, sizeof(label), 1, f);
				assert(label.get_dir() > 0);
				assert(label.get_dir() <= 128);
				labels->push_back(label);

#ifndef NDEBUG
				if(verbose_)
				{
					std::cerr << "\tsuccessor "<<label.get_id()<<
						" dir "<< (warthog::jps::direction)dir << std::endl;
				}
#endif
				if(feof(f)) 
				{ 
					std::cerr << "load failed; unexpected eof. abort.\n";
					exit(1);
				}
			}
		}
		graph_->insert(tmp);
	}

	if(ferror(f))
	{
		std::cerr << "read error loading search graph. aborting\n";
		exit(1);
	}

#ifndef NDEBUG
	if(verbose_)
	{
		std::cerr << "load graph; size: "<<graph_->size()<<std::endl;
		std::cerr << "verbose: "<<verbose_<<std::endl;
	}
#endif


	fclose(f);
	return true;
}

void 
warthog::offline_jump_point_locator::save(const char* filename)
{
	char fname[256];
	strcpy(fname, filename);
	strcat(fname, ".jps+2");

	FILE* f = fopen(fname, "wb");
	if(f == NULL) 
	{
		std::cerr << "err; cannot write jump-point graph to file "
			<<fname<<". oh well. try to keep going.\n"<<std::endl;
		return;
	}

	for(uint32_t index = 0; index < map_->width() * map_->height(); index++)
	{
		warthog::jps_record* record = graph_->find(index);
		if(record == 0) { continue; }

		uint32_t jp_id = record->get_id();
		fwrite(&jp_id, sizeof(jp_id), 1, f);

#ifndef NDEBUG
		uint32_t num_successors = record->num_successors();
		assert(num_successors > 0);

		if(verbose_)
		{
			std::cerr << "save: node "<<jp_id<< " #successors: " 
				<<record->num_successors()<<std::endl;
		}
#endif
		for(uint32_t i = 0; i < 8; i++)
		{
			uint8_t dir = (1 << i);
			fwrite(&dir, sizeof(uint8_t), 1, f);

			warthog::arraylist<warthog::jps_label>* labels = 
				record->get_list((warthog::jps::direction)dir);
			uint32_t list_size = labels->size();
			fwrite(&list_size, sizeof(list_size), 1, f);

			for(uint32_t j = 0; j < labels->size(); j++)
			{
				warthog::jps_label label = labels->at(j);
				fwrite(&label, sizeof(label), 1, f);
#ifndef NDEBUG
				if(verbose_)
				{
					std::cerr << "\tsuccessor "<<label.get_id()<<
						" dir "<< (warthog::jps::direction)dir << std::endl;
				}
#endif
			}
		}
	}

	fclose(f);
	std::cerr << "jump-point graph (n="<<graph_->size()<<") saved to disk. file="<<fname<<std::endl;
}

warthog::arraylist<warthog::jps_label> const *
warthog::offline_jump_point_locator::jump(
		warthog::jps::direction d, uint32_t node_id, uint32_t goal_id)
{
	warthog::jps_record* record = graph_->find(node_id);	
	assert(record);
	return record->get_list(d);
}

void
warthog::offline_jump_point_locator::undo_prior_insertions()
{
	if(start_)
	{
		graph_->remove(start_->get_id());
		start_ = 0;
	}
	if(goal_)
	{
		if(shadow_goal_)
		{
			graph_->insert(shadow_goal_);
		}
		else
		{
			graph_->remove(goal_->get_id());
		}
	}

	// remove any references to previously inserted nodes
	for(uint32_t i=0; i < modified_lists_->size(); i++)
	{
		warthog::arraylist<warthog::jps_label>* list = modified_lists_->at(i);
		list->pop_back();
	}

	modified_lists_->clear();
	start_ = goal_ = shadow_goal_ = 0;
}

void
warthog::offline_jump_point_locator::insert(
		warthog::jps_record* start_node,
		warthog::jps_record* goal_node)
{
	// NB: need both start and goal in the graph before we call
	// connect!!
	bool connect_start = false;
	bool connect_goal = false;
	if(jpmap_->get_label(start_node->get_id()) == 0)
	{
		assert(graph_->find(start_node->get_id()) == 0);
		graph_->insert(start_node);
		start_ = start_node;
		connect_start = true;
	}

	// always insert the goal; even if it exists in the graph.
	// the reason is that even if the goal node exists in the graph 
	// it may not be a jump point in every direction. 
	if(jpmap_->get_label(goal_node->get_id()) == 0)
	{
		assert(graph_->find(goal_node->get_id()) == 0);
		graph_->insert(goal_node);
		goal_ = goal_node;
		connect_goal = true;
	}
	else
	{
		assert(graph_->find(start_node->get_id()));
		shadow_goal_ = graph_->find(goal_node->get_id());
		graph_->insert(goal_node);
		goal_ = goal_node;
		connect_goal = true;
	}

	if(connect_start)
	{
		insert_nongoal(start_node, goal_node);
	}
	if(connect_goal)
	{
		scan_right(goal_node);
		scan_left(goal_node);
		scan_up(goal_node);
		scan_down(goal_node);
	}
}

uint32_t
warthog::offline_jump_point_locator::mem()
{
	uint32_t total_mem = sizeof(this);
	total_mem += sizeof(void*) * modified_lists_->size();
	total_mem += graph_->mem();
	return total_mem;
}

// scans grid nodes to the left of @param source in order to identify which jump points
// the node needs to be connected to.
void
warthog::offline_jump_point_locator::scan_left(warthog::jps_record* source)
{
	uint32_t source_id = source->get_id();
	uint32_t node_id = source_id;
	uint32_t ssteps, dsteps;
	uint32_t neis;
	ssteps = dsteps = 0;
	map_->get_neighbours(node_id, (uint8_t*)&neis);
	while((neis & 512) == 512) 
	{
		// scan northwest
		uint32_t current_id = node_id;
		uint32_t current_neis = neis;
		dsteps = 0;
		while((current_neis & 771) == 771) 
		{
			dsteps++;
			current_id -= (map_->width() + 1);

			if(jpmap_->get_label(current_id))
			{
				warthog::arraylist<warthog::jps_label>* labels = 0;
				warthog::jps_label label;
				label.dir_id_ = source_id;
				*(((uint8_t*)&label.dir_id_)+3) = warthog::jps::SOUTHEAST;
				warthog::jps_record* jp_node = graph_->find(current_id);
				labels = jp_node->get_list(warthog::jps::SOUTHEAST);
				modified_lists_->push_back(labels);
				label.ssteps_ = ssteps;
				label.dsteps_ = dsteps;
				labels->push_back(label);
			}
			map_->get_neighbours(current_id, (uint8_t*)&current_neis);
		}

		// scan southwest
		current_id = node_id;
		current_neis = neis;
		dsteps = 0;
		while((current_neis & 197376) == 197376)
		{
			dsteps++;
			current_id += (map_->width() - 1);

			if(jpmap_->get_label(current_id))
			{
				warthog::arraylist<warthog::jps_label>* labels = 0;
				warthog::jps_label label;
				label.dir_id_ = source_id;
				*(((uint8_t*)&label.dir_id_)+3) = warthog::jps::NORTHEAST;
				warthog::jps_record* jp_node = graph_->find(current_id);
				labels = jp_node->get_list(warthog::jps::NORTHEAST);
				modified_lists_->push_back(labels);
				label.ssteps_ = ssteps;
				label.dsteps_ = dsteps;
				labels->push_back(label);
			}
			map_->get_neighbours(current_id, (uint8_t*)&current_neis);
		}

		// scan west
		if(jpmap_->get_label(node_id))
		{
			uint32_t forced_neis = 0;
			warthog::arraylist<warthog::jps_label>* labels = 0;

			warthog::jps_label label;
			label.dir_id_ = source_id;
			*(((uint8_t*)&label.dir_id_)+3) = warthog::jps::EAST;
			forced_neis = warthog::jps::compute_forced(warthog::jps::EAST, neis);
			warthog::jps_record* jp_node = graph_->find(node_id);
			labels = jp_node->get_list(warthog::jps::EAST);
			modified_lists_->push_back(labels);
			label.ssteps_ = ssteps;
			label.dsteps_ = 0;
			labels->push_back(label);

			// early termination
			if(forced_neis) { break; }
		}
		ssteps++;
		node_id--;
		map_->get_neighbours(node_id, (uint8_t*)&neis);
	}
}

void
warthog::offline_jump_point_locator::scan_right(warthog::jps_record* source)
{
	uint32_t source_id = source->get_id();
	uint32_t node_id = source_id;
	uint32_t ssteps, dsteps;
	uint32_t neis;
	ssteps = dsteps = 0;
	map_->get_neighbours(node_id, (uint8_t*)&neis);
	while((neis & 512) == 512) 
	{
		// scan northeast
		uint32_t current_id = node_id;
		uint32_t current_neis = neis;
		dsteps = 0;
		while((current_neis & 1542) == 1542) 
		{
			dsteps++;
			current_id -= (map_->width() - 1);

			if(jpmap_->get_label(current_id))
			{
				warthog::arraylist<warthog::jps_label>* labels = 0;
				warthog::jps_label label;
				label.dir_id_ = source_id;
				*(((uint8_t*)&label.dir_id_)+3) = warthog::jps::SOUTHWEST;
				warthog::jps_record* jp_node = graph_->find(current_id);
				labels = jp_node->get_list(warthog::jps::SOUTHWEST);
				modified_lists_->push_back(labels);
				label.ssteps_ = ssteps;
				label.dsteps_ = dsteps;
				labels->push_back(label);
			}
			map_->get_neighbours(current_id, (uint8_t*)&current_neis);
		}

		// scan southeast
		current_id = node_id;
		current_neis = neis;
		dsteps = 0;
		while((current_neis & 394752) == 394752)
		{
			dsteps++;
			current_id += (map_->width() + 1);

			if(jpmap_->get_label(current_id))
			{
				warthog::arraylist<warthog::jps_label>* labels = 0;
				warthog::jps_label label;
				label.dir_id_ = source_id;
				*(((uint8_t*)&label.dir_id_)+3) = warthog::jps::NORTHWEST;
				warthog::jps_record* jp_node = graph_->find(current_id);
				labels = jp_node->get_list(warthog::jps::NORTHWEST);
				modified_lists_->push_back(labels);
				label.ssteps_ = ssteps;
				label.dsteps_ = dsteps;
				labels->push_back(label);
			}
			map_->get_neighbours(current_id, (uint8_t*)&current_neis);
		}

		// scan east
		if(jpmap_->get_label(node_id))
		{
			uint32_t forced_neis = 0;
			warthog::arraylist<warthog::jps_label>* labels = 0;

			warthog::jps_label label;
			label.dir_id_ = source_id;
			*(((uint8_t*)&label.dir_id_)+3) = warthog::jps::WEST;
			forced_neis = warthog::jps::compute_forced(warthog::jps::WEST, neis);
			warthog::jps_record* jp_node = graph_->find(node_id);
			labels = jp_node->get_list(warthog::jps::WEST);
			modified_lists_->push_back(labels);
			label.ssteps_ = ssteps;
			label.dsteps_ = 0;
			labels->push_back(label);

			// early termination
			if(forced_neis) { break; }
		}
		ssteps++;
		node_id++;
		map_->get_neighbours(node_id, (uint8_t*)&neis);
	}
}

void
warthog::offline_jump_point_locator::scan_up(warthog::jps_record* source)
{
	uint32_t ssteps, dsteps, neis;
	uint32_t source_id = source->get_id() - map_->width(); 
	uint32_t node_id = source_id;
	ssteps = 1; // no oevrlap with scan_left/scan_right

	map_->get_neighbours(node_id, (uint8_t*)&neis);
	while((neis & 512) == 512) 
	{
		// scan northeast
		uint32_t current_id = node_id;
		uint32_t current_neis = neis;
		dsteps = 0;
		while((current_neis & 1542) == 1542) 
		{
			dsteps++;
			current_id -= (map_->width() - 1);

			if(jpmap_->get_label(current_id))
			{
				warthog::arraylist<warthog::jps_label>* labels = 0;
				warthog::jps_label label;
				label.dir_id_ = source->get_id();
				*(((uint8_t*)&label.dir_id_)+3) = warthog::jps::SOUTHWEST;
				warthog::jps_record* jp_node = graph_->find(current_id);
				labels = jp_node->get_list(warthog::jps::SOUTHWEST);
				modified_lists_->push_back(labels);
				label.ssteps_ = ssteps;
				label.dsteps_ = dsteps;
				labels->push_back(label);
			}
			map_->get_neighbours(current_id, (uint8_t*)&current_neis);
		}

		// scan northwest
		current_id = node_id;
		current_neis = neis;
		dsteps = 0;
		while((current_neis & 771) == 771) 
		{
			dsteps++;
			current_id -= (map_->width() + 1);

			if(jpmap_->get_label(current_id))
			{
				warthog::arraylist<warthog::jps_label>* labels = 0;
				warthog::jps_label label;
				label.dir_id_ = source->get_id();
				*(((uint8_t*)&label.dir_id_)+3) = warthog::jps::SOUTHEAST;
				warthog::jps_record* jp_node = graph_->find(current_id);
				labels = jp_node->get_list(warthog::jps::SOUTHEAST);
				modified_lists_->push_back(labels);
				label.ssteps_ = ssteps;
				label.dsteps_ = dsteps;
				labels->push_back(label);
			}
			map_->get_neighbours(current_id, (uint8_t*)&current_neis);
		}


		// scan north
		if(jpmap_->get_label(node_id))
		{
			uint32_t forced_neis = 0;
			warthog::arraylist<warthog::jps_label>* labels = 0;

			warthog::jps_label label;
			label.dir_id_ = source->get_id();
			*(((uint8_t*)&label.dir_id_)+3) = warthog::jps::SOUTH;
			forced_neis = warthog::jps::compute_forced(warthog::jps::SOUTH, neis);
			warthog::jps_record* jp_node = graph_->find(node_id);
			labels = jp_node->get_list(warthog::jps::SOUTH);
			modified_lists_->push_back(labels);
			label.ssteps_ = ssteps;
			label.dsteps_ = 0;
			labels->push_back(label);

			// early termination
			if(forced_neis) { break; }
		}

		ssteps++;
		node_id = node_id - map_->width();
		map_->get_neighbours(node_id, (uint8_t*)&neis);
	}
}

void
warthog::offline_jump_point_locator::scan_down(warthog::jps_record* source)
{
	uint32_t ssteps, dsteps, neis;
	uint32_t source_id = source->get_id() + map_->width(); 
	uint32_t node_id = source_id;
	ssteps = 1; // no overlap with scan_right/scan_left

	map_->get_neighbours(node_id, (uint8_t*)&neis);
	while((neis & 512) == 512) 
	{
		// scan southwest
		uint32_t current_id = node_id;
		uint32_t current_neis = neis;
		dsteps = 0;
		while((current_neis & 197376) == 197376)
		{
			dsteps++;
			current_id += (map_->width() - 1);

			if(jpmap_->get_label(current_id))
			{
				warthog::arraylist<warthog::jps_label>* labels = 0;
				warthog::jps_label label;
				label.dir_id_ = source->get_id();
				*(((uint8_t*)&label.dir_id_)+3) = warthog::jps::NORTHEAST;
				warthog::jps_record* jp_node = graph_->find(current_id);
				labels = jp_node->get_list(warthog::jps::NORTHEAST);
				modified_lists_->push_back(labels);
				label.ssteps_ = ssteps;
				label.dsteps_ = dsteps;
				labels->push_back(label);
			}
			map_->get_neighbours(current_id, (uint8_t*)&current_neis);
		}

		// scan southeast
		current_id = node_id;
		current_neis = neis;
		dsteps = 0;
		while((current_neis & 394752) == 394752)
		{
			dsteps++;
			current_id += (map_->width() + 1);

			if(jpmap_->get_label(current_id))
			{
				warthog::arraylist<warthog::jps_label>* labels = 0;
				warthog::jps_label label;
				label.dir_id_ = source->get_id();
				*(((uint8_t*)&label.dir_id_)+3) = warthog::jps::NORTHWEST;
				warthog::jps_record* jp_node = graph_->find(current_id);
				labels = jp_node->get_list(warthog::jps::NORTHWEST);
				modified_lists_->push_back(labels);
				label.ssteps_ = ssteps;
				label.dsteps_ = dsteps;
				labels->push_back(label);
			}
			map_->get_neighbours(current_id, (uint8_t*)&current_neis);
		}

		// scan south
		if(jpmap_->get_label(node_id))
		{
			uint32_t forced_neis = 0;
			warthog::arraylist<warthog::jps_label>* labels = 0;

			warthog::jps_label label;
			label.dir_id_ = source->get_id();
			*(((uint8_t*)&label.dir_id_)+3) = warthog::jps::NORTH;
			forced_neis = warthog::jps::compute_forced(warthog::jps::NORTH, neis);
			warthog::jps_record* jp_node = graph_->find(node_id);
			labels = jp_node->get_list(warthog::jps::NORTH);
			modified_lists_->push_back(labels);
			label.ssteps_ = ssteps;
			label.dsteps_ = 0;
			labels->push_back(label);

			// early termination
			if(forced_neis) { break; }
		}

		ssteps++;
		node_id = node_id + map_->width();
		map_->get_neighbours(node_id, (uint8_t*)&neis);
	}
}

void
warthog::offline_jump_point_locator::insert_nongoal(
		warthog::jps_record* source, warthog::jps_record* goal)
{
	uint32_t source_id = source->get_id();
	uint32_t goal_id = goal->get_id();

	for(uint32_t i = 0; i < 8; i++)
	{
		warthog::jps::direction d = (warthog::jps::direction) (1 << i);
		warthog::arraylist<jps_label>* labels = source->get_list(d);
		jpl_->jump(d, source_id, goal_id, *labels);
	}
}
