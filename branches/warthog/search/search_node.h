#ifndef WARTHOG_SEARCH_NODE_H
#define WARTHOG_SEARCH_NODE_H

// search_node.h
//
// @author: dharabor
// @created: 10/08/2012
//

#include "constants.h"
#include "cpool.h"

#include <iostream>

namespace warthog
{
	
class search_node
{
	public:
		search_node(unsigned int id) 
			: id_and_status_(id << 1), h_(0), g_(0), parent_(0), 
			priority_(warthog::INF)
		{
			assert(this->get_id() < ((1ul<<31)-1));
			refcount_++;
		}

		~search_node()
		{
			refcount_--;
		}

		inline unsigned int 
		get_id() const { return (id_and_status_ >> 1); }

		inline void
		set_id(unsigned int id) 
		{ 
			id_and_status_ = (id << 1) | (id_and_status_ & 1); 
		} 

		inline bool
		get_expanded() const { return (id_and_status_ & 1); }

		inline void
		set_expanded(bool expanded) 
		{ 
			id_and_status_ = ((id_and_status_ >> 1) << 1) |
			   	(unsigned int)(expanded?1:0);
		}

		inline warthog::search_node* 
		get_parent() const { return parent_; }

		inline void
		set_parent(warthog::search_node* parent) { parent_ = parent; } 

		inline unsigned int
		get_priority() const { return priority_; }

		inline void
		set_priority(unsigned int priority) { priority_ = priority; } 

		inline double
		get_g() const { return g_; }

		inline void
		set_g(double g) { g_ = g; }

		inline double
		get_h() const { return h_; }

		inline void
		set_h(double h) { h_ = h; }

		inline double 
		get_f() const { return g_ + h_; }

		inline void 
		relax(double g, warthog::search_node* parent)
		{
			assert(g < g_);
			g_ = g;
			parent_ = parent;
		}

		//inline void* 
		//operator new(size_t s)
		//{
		//	return (void*) warthog::search_node::mem_.allocate();
		//}

		//inline void  
		//operator delete(void* addr, size_t bytes)
		//{
		//	warthog::search_node::mem_.deallocate((char*)addr);
		//}

		inline bool
		operator<(const warthog::search_node& other) const
		{
			double myf = g_ + h_ + warthog::EPSILON;
			double otherf = other.g_ + other.h_;
			if(myf < otherf)
			{
				return true;
			}
			if(myf > otherf)
			{
				return false;
			}

			// break ties in favour of smaller g
			if((g_ + warthog::EPSILON) < other.g_)
			{
				return true;
			}
			return false;
		}

		inline bool
		operator>(const warthog::search_node& other) const
		{
			if((this->get_f() - warthog::EPSILON) > other.get_f())
			{
				return true;
			}
			if(!((other.get_f() - warthog::EPSILON) > this->get_f()))
			{
				// break ties in favour of smaller g
				if((g_ - warthog::EPSILON) > other.g_)
				{
					return true;
				}
			}
			return false;
		}

		inline bool
		operator==(const warthog::search_node& other) const
		{
			if( !(*this < other) && !(*this > other))
			{
				return true;
			}
			return false;
		}

		inline bool
		operator<=(const warthog::search_node& other) const
		{
			if(*this < other)
			{
				return true;
			}
			if(!(*this > other))
			{
				return true;
			}
			return false;
		}

		inline bool
		operator>=(const warthog::search_node& other) const
		{
			if(*this > other)
			{
				return true;
			}
			if(!(*this < other))
			{
				return true;
			}
			return false;
		}

		inline void 
		print(std::ostream&  out) const
		{
			out << "search_node id:" << get_id() << " g: "<<g_
				<<" h: "<<h_<<" f: "<<this->get_f()
				<< " expanded: " << get_expanded() << " ";
		}

		static unsigned int 
		get_refcount() { return refcount_; }

		unsigned int
		mem()
		{
			return sizeof(*this);
		}

	private:
		unsigned int id_and_status_; // bit 0 is expansion status; 1-31 are id
		double h_;
		double g_;
		warthog::search_node* parent_;
		unsigned int priority_; // expansion priority

		static unsigned int refcount_;
		static warthog::mem::cpool mem_;
};

}

#endif

