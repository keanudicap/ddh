#ifndef WARTHOG_JPSP_RECORD_H
#define WARTHOG_JPSP_RECORD_H

// jpsp_record.h
//
// Describes a jump point and all reachable non-intermediate
// successors. 
//
// @author: dharabor
// @created: 31/05/2013
//

#include "arraylist.h"
#include "jps.h"
#include <cassert>

namespace warthog
{
	
class jpsp_record
{
	public:
		jpsp_record(uint32_t id);
		~jpsp_record();

		void
		add_successor(warthog::jps::direction pdir, 
				uint16_t xdelta, uint16_t ydelta);

		void
		get_successor(warthog::jps::direction pdir, 
				size_t nei_index, uint16_t& xdelta, uint16_t& ydelta);

		inline size_t
		num_successors(warthog::jps::direction pdir)
		{
			assert(pdir != warthog::jps::NONE);
			uint32_t index = __builtin_ffs(pdir);
			return num_neis_[index];
		}

		inline uint32_t
		get_id() { return id_; }

	private:
		uint32_t id_;

		// one set of neis for each of the 8 possible 
		// parent jps::directions
		warthog::arraylist<uint32_t>** neis_;
		uint32_t* num_neis_;

};

}

#endif

