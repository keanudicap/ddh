#ifndef FLEXIBLE_ASTAR_H
#define FLEXIBLE_ASTAR_H

// flexible_astar.h
//
// A* implementation that allows arbitrary combinations of 
// heuristic functions and node expansion policies.
// This implementation uses a binary heap for the open_ list
// and a bit array for the closed_ list.
//
// TODO: is it better to store a separate closed list and ungenerate nodes
// or use more memory and not ungenerate until the end of search??
// 32bytes vs... whatever unordered_map overhead is a two integer key/value pair
// 
// @author: dharabor
// @created: 21/08/2012
//

#include "cpool.h"
#include "pqueue.h"
#include "problem_instance.h"
#include "search_node.h"

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
	public:
		flexible_astar(H* heuristic, E* expander)
			: heuristic_(heuristic), expander_(expander)
		{
			open_ = new warthog::pqueue(1024, true);
			verbose_ = false;
		}

		~flexible_astar()
		{
			cleanup();
			delete open_;
		}

		inline size_t
		mem()
		{
			size_t bytes = 
				// memory for the priority quete
				open_->mem() + 
				// gridmap size and other stuff needed to expand nodes
				expander_->mem() +
				// misc
				sizeof(*this);
			return bytes;
		}

		inline bool
		get_verbose() { return verbose_; }

		inline void
		set_verbose(bool verbose) { verbose_ = verbose; } 

		inline std::stack<unsigned int>
		get_path(unsigned int startid, unsigned int goalid)
		{
			std::stack<unsigned int> path;
			warthog::search_node* goal = search(startid, goalid);
			if(goal)
			{
				// follow backpointers to extract the path
				assert(goal->get_id() == goalid);
				for(warthog::search_node* cur = goal;
						cur != 0;
					    cur = cur->get_parent())
				{
					path.push(cur->get_id());
				}
				assert(path.top() == startid);
			}
			cleanup();
			return path;
		}

		double
		get_length(unsigned int startid, unsigned int goalid)
		{
			warthog::search_node* goal = search(startid, goalid);
			double len = warthog::INF;
			if(goal)
			{
				assert(goal->get_id() == goalid);
				len = goal->get_g();
			}
			cleanup();
			return len;
		}

	private:
		H* heuristic_;
		E* expander_;
		warthog::pqueue* open_;
		bool verbose_;
		static unsigned int searchid_;

		// no copy
		flexible_astar(const flexible_astar& other) { } 
		flexible_astar& 
		operator=(const flexible_astar& other) { return *this; }

		warthog::search_node*
		search(unsigned int startid, unsigned int goalid)
		{
			#ifndef NDEBUG
			if(verbose_)
			{
				std::cerr << "search: startid="<<startid<<" goalid=" <<goalid
					<< std::endl;
			}
			#endif

			warthog::problem_instance instance;
			instance.set_goal(goalid);
			instance.set_start(startid);

			warthog::search_node* goal = 0;
			warthog::search_node* start = expander_->generate(startid);
			start->set_g(0);
			start->set_f(heuristic_->h(startid, goalid));
			open_->push(start);

			while(open_->size())
			{
				if(open_->peek()->get_id() == goalid)
				{
					goal = open_->peek();
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
				current->set_expanded(true); // NB: set this before calling expander_ 
				assert(current->get_expanded());
				expander_->expand(current, &instance);

				warthog::search_node* n = 0;
				double cost_to_n = warthog::INF;
				for(expander_->first(n, cost_to_n); 
						n != 0;
					   	expander_->next(n, cost_to_n))
				{
					if(n->get_expanded())
					{
						// skip neighbours already expanded
						continue;
					}

					if(open_->contains(n))
					{
						// update a node from the fringe
						double gval = current->get_g() + cost_to_n;
						if(gval < n->get_g())
						{
							n->relax(gval, current);
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
						double gval = current->get_g() + cost_to_n;
						assert(gval != warthog::INF);
						n->set_g(gval);
						n->set_f(gval + heuristic_->h(n->get_id(), goalid));
					   	n->set_parent(current);
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
			}
			return goal;
		}

		void
		cleanup()
		{
			open_->clear();
			expander_->clear();
		}


};

template <class H, class E>
unsigned int warthog::flexible_astar<H, E>::searchid_ = warthog::FNV32_prime;

}

#endif

