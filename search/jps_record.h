#ifndef WARTHOG_JPS_RECORD_H
#define WARTHOG_JPS_RECORD_H

// jps_record.h
//
// Describes a jump point and all reachable non-intermediate
// successors. 
// ::id_ is the identifier associated with the current node
// ::neis_ is an array of lists that describes the jump operations
// that need to be performed in order to reach each successor.
// There are usually 8 lists. One list for successors in each of the
// 8 compass directions.
//
// @author: dharabor
// @created: 31/05/2013
//

#include "arraylist.h"
#include "jps.h"
#include <cassert>
#include <iostream>

namespace warthog
{

// describes a jump from a given (known, but here unspecified) node.
// dsteps_ is the number of diagonal steps to take. 
// ssteps_ is the number of straight steps to take.
// dir_id_ is the combination of the successor node identifer (lower 3 bytes)
// and the direction of travel used to reach it (upper byte).
// the direction is always one of the four cardinal directions {N, S, E, W}
// (the corresponding diagonal direction is not stored explicitly).
class jps_label
{
	public:

		void
		set_id(uint32_t id)
		{
			dir_id_ = 
				(dir_id_ & ~warthog::jps::id_mask) | id;
		}

		uint32_t 
		get_id()
		{
			return dir_id_ & warthog::jps::id_mask;	
		}

		void
		set_dir(warthog::jps::direction d)
		{
			uint32_t dir = d << 24;
			dir_id_ = (dir_id_ & warthog::jps::id_mask) | dir;
		}

		warthog::jps::direction
		get_dir()
		{
			return (warthog::jps::direction)
				*(((uint8_t*)&dir_id_)+3);
		}

		void
		print(std::ostream& out)
		{
			out << "label id: "<<get_id() << " sdir: "<<get_dir() 
				<< " dsteps: "<<dsteps_ << " ssteps: " << ssteps_ 
				<< std::endl;
		}

		uint32_t dir_id_;
		uint16_t ssteps_;
		uint16_t dsteps_;
};
	
class jps_record
{
	public:
		jps_record(uint32_t id);
		~jps_record();

		inline uint32_t
		get_id() { return id_; }

		inline void
		set_id(uint32_t id) { id_ = id; clear(); }


		// returns a list of elements, each of which describes how
		// to reach the successors of node ::id_
		// there are 8 such lists; one for each element 
		// of warthog::jps::direction (except warthog::jps::NONE).
		inline warthog::arraylist<jps_label>*
		get_list(warthog::jps::direction pdir)
		{
			assert(pdir != warthog::jps::NONE);
			uint32_t index = __builtin_ffs(pdir) - 1;
			return neis_[index];
		}

		inline void
		clear()
		{
			for(uint32_t i = 0; i < 8; i++)
			{
				neis_[i]->clear();
			}
		}
		
		inline uint32_t 
		num_successors()
		{
			uint32_t total = 0;
			for(uint32_t i = 0; i < 8; i++)
			{
				total += neis_[i]->size();
			}
			return total;
		}


		uint32_t
		mem();


	private:
		uint32_t id_;
		warthog::arraylist<jps_label>** neis_;
};

}

#endif

