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
		search_node(unsigned int id) 
			: id_(id), h_(0), g_(0), pid_(warthog::UNDEF)
		{
			refcount_++;
		}

		~search_node()
		{
			refcount_--;
		}

		inline unsigned int 
		id() const { return id_; }

		inline unsigned int
		pid() { return pid_; }

		inline double 
		f() const { return g_ + h_; }

		inline double
		g() const { return g_; }

		inline double
		h() const { return h_; }


		inline void 
		update(double g, double h, unsigned int pid)
		{
			g_ = g;
			h_ = h;
			pid_ = pid;
		}

		// use this version if heuristic value is unchanged
		inline void 
		update(double g, unsigned int pid)
		{
			g_ = g;
			pid_ = pid;
		}

		inline bool
		operator<(const warthog::search_node& other) const
		{
			if((this->f() + warthog::EPSILON) < other.f())
			{
				return true;
			}
			if(!((other.f() + warthog::EPSILON) < this->f()))
			{
				// break ties in favour of smaller g
				if((g_ + warthog::EPSILON) < other.g_)
				{
					return true;
				}
			}
			return false;
		}

		inline bool
		operator>(const warthog::search_node& other) const
		{
			if((this->f() - warthog::EPSILON) > other.f())
			{
				return true;
			}
			if(!((other.f() - warthog::EPSILON) > this->f()))
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
			out << "search_node id:" << id_ << " g: "<<g_
				<<" h: "<<h_<<" f: "<<this->f() << " ";
		}

		static unsigned int 
		get_refcount() { return refcount_; }

		unsigned int
		mem()
		{
			return sizeof(*this);
		}

	private:
		unsigned int id_;
		double h_;
		double g_;
		unsigned int pid_; // parent id
		static unsigned int refcount_;
};

}

#endif

