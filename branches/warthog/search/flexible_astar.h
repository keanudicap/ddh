#ifndef FLEXIBLE_ASTAR_H
#define FLEXIBLE_ASTAR_H

// flexible_astar.h
//
// A* implementation that allows arbitrary combinations of 
// heuristic functions and node expansion policies.
// This implementation uses a binary heap for the open list
// and a bit array for the closed list.
// 
// @author: dharabor
// @created: 21/08/2012
//

#include "heap.h"
#include "nodemap.h"

#include <memory>
#include <stack>

namespace warthog
{

// H is a heuristic function
// E is an expansion policy
template <class H, class E>
class flexible_astar 
{
	using H::h;
	using E::first;
	using E::expand;
	using E::next;
	using E::n;
	using E::new_node;
	using E::get_max_node_id;

	public:
		flexible_astar(std::shared_ptr<H> heuristic, std::shared_ptr<E> expander)
			: heuristic_(heuristic), expander_(expander) { }
		~flexible_astar() { }

		std::stack<unsigned int>
		get_path(unsigned int startid, unsigned int goalid)
		{
			warthog::heap open(1024, true);
			warthog::nodemap closed(expander_->get_max_node_id());

			std::stack<unsigned int> path;
			double len = search(startid, goalid, open, closed);
			if(len != warthog::INF)
			{
				for(unsigned int id = goalid;
					   	id != startid;
					   	id = closed[id])
				{
					path.push(id);
				}
			}
			return path;
		}

		double
		get_length(unsigned int startid, unsigned int goalid)
		{
			warthog::heap open(1024, true);
			warthog::nodemap closed(expander_->get_max_node_id());
			return search(startid, goalid, open, closed);
		}

	private:
		std::shared_ptr<H> heuristic_;
		std::shared_ptr<E> expander_;

		double 
		search(unsigned int startid, unsigned int goalid, 
				warthog::heap& open, warthog::nodemap& closed)
		{
			double len = DBL_MAX;
			warthog::search_node* start = new warthog::search_node(startid);
			start->update(0, heuristic_->h(startid, goalid), warthog::UNDEF);
			open.push(start);
			closed[startid] = startid;

			while(open.size())
			{
				if(open.peek()->id() == goalid)
				{
					len = open.peek()->g();
					break;
				}

				warthog::search_node* current = open.pop();
				closed[current->id()] = current->pid();
				expander_->expand(current->id());
				for(unsigned int nid = expander_->first(); 
						nid != expander_->end();
					   	nid = expander_->next())
				{
					if(!(closed.get_value(nid) == warthog::UNDEF))
					{
						// skip neighbours already expanded
						continue;
					}

					warthog::search_node* n = open.find(nid);
					if(n)
					{
						// update a node from the fringe
						double gVal = current->g() + expander_->cost_to_n();
						if(gVal < n->g())
						{
							n->update(gVal, current->id());
							open.decrease_key(n);
						}
					}
					else
					{
						// add a new node to the fringe
						n = new warthog::search_node(nid);
						n->update(current->g() + expander_->cost_to_n(), 
								heuristic_->h(nid, goalid), current->id());
						open.push(n);
					}
				}
				delete current;
			}

			// cleanup
			while(open.size())
			{
				delete open.pop_tail();
			}
			closed.reset();

			return len;
		}

		flexible_astar(const flexible_astar& other) { } 
		flexible_astar& 
		operator=(const flexible_astar& other) { return *this; }


};

}

#endif

