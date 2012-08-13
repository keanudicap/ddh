#ifndef WARTHOG_SEARCH_NODE_H
#define WARTHOG_SEARCH_NODE_H

// search_node.h
//
// @author: dharabor
// @created: 10/08/2012
//

#include "constants.h"

#include <iostream>

namespace warthog
{
	
class search_node
{
	public:
		search_node(unsigned int id) : id_(id), f_(0), g_(0), backpointer_(0)
		{
			refcount_++;
		}

		virtual ~search_node()
		{
			refcount_--;
		}

		inline void print(std::ostream&  out)
		{
			out << "search_node id:" << id_ << " f: "<<f_<<" g: "<<g_<<" ";
		}

		inline unsigned int id() { return id_; }
		inline double f() { return f_; }
		inline double g() { return g_; }
		inline warthog::search_node* backpointer() { return backpointer_; }

		inline void update(double f, double g, warthog::search_node* ptr)
		{
			f_ = f;
			g_ = g;
			backpointer_ = ptr;
		}

		inline bool operator<(const warthog::search_node& other)
		{
			if((f_ + warthog::EPSILON) < other.f_)
			{
				return true;
			}
			if(!((other.f_ + warthog::EPSILON) < f_))
			{
				// break ties in favour of smaller g
				if((g_ + warthog::EPSILON) < other.g_)
				{
					return true;
				}
			}
			return false;
		}

		inline bool operator>(const warthog::search_node& other)
		{
			if((f_ - warthog::EPSILON) > other.f_)
			{
				return true;
			}
			if(!((other.f_ - warthog::EPSILON) > f_))
			{
				// break ties in favour of smaller g
				if((g_ - warthog::EPSILON) > other.g_)
				{
					return true;
				}
			}
			return false;
		}

		inline bool operator==(const warthog::search_node& other)
		{
			if( !(*this < other) && !(*this > other))
			{
				return true;
			}
			return false;
		}

		inline bool operator<=(const warthog::search_node& other)
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

		inline bool operator>=(const warthog::search_node& other)
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

		static unsigned int get_refcount() { return refcount_; }

	private:
		unsigned int id_;
		double f_;
		double g_;
		warthog::search_node* backpointer_;
		static unsigned int refcount_;
};


}

#endif

