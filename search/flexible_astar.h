#ifndef FLEXIBLE_ASTAR_H
#define FLEXIBLE_ASTAR_H

// flexible_astar.h
//
// A* implementation that allows arbitrary combinations of 
// heuristic functions and node expansion policies.
// This implementation uses a binary heap for the open_ list
// and a bit array for the closed_ list.
// 
// @author: dharabor
// @created: 21/08/2012
//

#include "heap.h"
#include "nodemap.h"
#include "problem_instance.h"

#include <iostream>
#include <memory>
#include <stack>

namespace warthog
{

// H is a heuristic function
// E is an expansion policy
template <class H, class E>
class flexible_astar 
{
//	using H::h;
//	using E::first;
//	using E::expand;
//	using E::next;
//	using E::n;
//	using E::new_node;
//	using E::get_max_node_id;

	public:
		flexible_astar(std::shared_ptr<H> heuristic, std::shared_ptr<E> expander)
			: heuristic_(heuristic), expander_(expander)
		{
			open_ = new warthog::heap(1024, true);
			closed_ = new warthog::nodemap(expander_->get_max_node_id());
			verbose_ = false;
		}

		~flexible_astar()
		{
			delete open_;
			delete closed_;
		}

		inline bool
		get_verbose() { return verbose_; }

		inline void
		set_verbose(bool verbose) { verbose_ = verbose; } 

		inline std::stack<unsigned int>
		get_path(unsigned int startid, unsigned int goalid)
		{
			std::stack<unsigned int> path;
			double len = search(startid, goalid);
			if(len != warthog::INF)
			{
				for(unsigned int id = goalid;
					   	id != warthog::UNDEF;
					   	id = closed_[id])
				{
					path.push(id);
				}
				assert(path.top() == startid);
			}
			return path;
		}

		double
		get_length(unsigned int startid, unsigned int goalid)
		{
			return search(startid, goalid);
		}

	private:
		std::shared_ptr<H> heuristic_;
		std::shared_ptr<E> expander_;
		warthog::heap* open_;
		warthog::nodemap* closed_;
		bool verbose_;

		// no copy
		flexible_astar(const flexible_astar& other) { } 
		flexible_astar& 
		operator=(const flexible_astar& other) { return *this; }

		double 
		search(unsigned int startid, unsigned int goalid)
		{
			warthog::problem_instance instance;
			instance.set_goal(goalid);
			instance.set_start(startid);

			double len = DBL_MAX;
			warthog::search_node* start = new warthog::search_node(startid);
			start->update(0, heuristic_->h(startid, goalid), warthog::UNDEF);
			open_->push(start);

			while(open_->size())
			{
				if(open_->peek()->id() == goalid)
				{
					len = open_->peek()->g();
					break;
				}

				warthog::search_node* current = open_->pop();
				#ifndef NDEBUG
				if(verbose_)
				{
					current->print(std::cerr << "expanding...");
					std::cerr << std::endl;
				}
				#endif
				closed_->set_value(current->id(),current->pid());
				assert(closed_->get_value(current->id() == current->pid()));
				expander_->expand(current->id(), &instance);
				for(unsigned int nid = expander_->first(); 
						nid != expander_->end();
					   	nid = expander_->next())
				{
					if(!(closed_->get_value(nid) == warthog::UNDEF))
					{
						// skip neighbours already expanded
						continue;
					}

					warthog::search_node* n = open_->find(nid);
					if(n)
					{
						assert(n->id() == nid);
						// update a node from the fringe
						double gVal = current->g() + expander_->cost_to_n();
						if(gVal < n->g())
						{
							n->update(gVal, current->id());
							open_->decrease_key(n);
							#ifndef NDEBUG
							if(verbose_)
							{
								n->print(std::cerr << "  updating...");
								std::cerr << std::endl;
							}
							#endif
						}
						else
						{
							#ifndef NDEBUG
							if(verbose_)
							{
								n->print(std::cerr << "  ignoring...");
								std::cerr << std::endl;
							}
							#endif
						}
					}
					else
					{
						// add a new node to the fringe
						n = new warthog::search_node(nid);
						n->update(current->g() + expander_->cost_to_n(), 
								heuristic_->h(nid, goalid), current->id());
						open_->push(n);
						#ifndef NDEBUG
						if(verbose_)
						{
							n->print(std::cerr << "  generating...");
							std::cerr << std::endl;
						}
						#endif
					}
				}
				#ifndef NDEBUG
				if(verbose_)
				{
					current->print(std::cerr << "closing...");
					std::cerr << std::endl;
				}
				#endif
				delete current;
			}

			// cleanup
			while(open_->size())
			{
				delete open_->pop_tail();
			}
			open_->clear();
			closed_->clear();

			return len;
		}

};

}

#endif

