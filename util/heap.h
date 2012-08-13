#ifndef WARTHOG_HEAP_H
#define WARTHOG_HEAP_H

// heap.h
//
// A min priority queue. Loosely based on an implementation from HOG
// by Nathan Sturtevant.
//
// TODO: Custom hash table that maps node ids to key values without
// the iterator/pair overhead of unordered_map.
//
// @author: dharabor
// @created: 09/08/2012
//

#include "search_node.h"

#include <cassert>
#include <iostream>
#include <tr1/unordered_map>

namespace warthog
{

class search_node;
class heap 
{
	typedef std::tr1::unordered_map<unsigned int, unsigned int> node_list;

	public:
		heap(unsigned int size, bool minheap);
		virtual ~heap();

		// removes all elements from the heap
		void clear();

		// reprioritise the specified element (up or down)
		void decrease_key(warthog::search_node* val);
		void increase_key(warthog::search_node* val);

		// add a new element to the heap
		void push(warthog::search_node* val);

		// remove the top element from the heap
		warthog::search_node* pop();
		
		// check if the heap contains the specified element
		inline bool contains(warthog::search_node *val)
		{
			if(this->keymap_.find(val->id()) != this->keymap_.end())
			{
				return true;
			}
			return false;
		}

		// retrieve the top element without removing it
		inline warthog::search_node* peek()
		{
			if(heapsize_ > 0)
			{
				return this->elts_[0];
			}
			return 0;
		}

		inline unsigned int size()
		{
			return heapsize_;
		}

		inline bool is_minheap() 
		{ 
			return minheap_; 
		} 
		
		void print(std::ostream& out);

	private:
		// reorders the subheap containing elts_[index]
		void heapify_up(unsigned int index);
		
		// reorders the subheap under elts_[index]
		void heapify_down(unsigned int index);

		// allocates more memory so the heap can grow
		void resize(unsigned int newsize);
	
		// returns true if:
		//   minheap is true and the priority of second < first
		//   minheap is false and the priority of second > first
		inline bool rotate(warthog::search_node& first,
				warthog::search_node& second)
		{
			if(minheap_)
			{
				if(second < first)
				{
					return true;
				}
			}
			else
			{
				if(second > first)
				{
					return true;
				}
			}
			return false;
		}

		// swap the positions of two nodes in the underlying array
		inline void swap(unsigned int index1, unsigned int index2)
		{
			assert(index1 < heapsize_ && index2 < heapsize_);

			warthog::search_node* tmp = elts_[index1];
			elts_[index1] = elts_[index2];
			keymap_[elts_[index2]->id()] = index1;

			elts_[index2] = tmp;
			keymap_[tmp->id()] = index2;
		}

		unsigned int maxsize_;
		bool minheap_;
		unsigned int heapsize_;
		warthog::search_node** elts_;
		node_list keymap_;
};

}

#endif

