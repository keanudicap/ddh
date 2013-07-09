#ifndef WARTHOG_JPS_GRAPH_H
#define WARTHOG_JPS_GRAPH_H

// jps_graph.h
//
// A jump point graph implemented as a sparse array. The array comprises a of list-of-lists. 
// Each list is a contiguous array of type warthog::jps_record. Each list is created 
// on-the-fly at the point of the first insertion of an element into an index that resides 
// in the list.
// 
// @author: dharabor
// @created: 08/07/2013
//

#include "jps.h"
#include "jps_record.h"
#include <stdint.h>

namespace warthog
{

class jps_graph
{
	public:
		jps_graph(uint32_t size)
		{
			blocksize_ = 64;
			log2_blocksize_ = 6;
			blockmask_ = blocksize_ - 1;
			num_blocks_ = (size / blocksize_) + 1;
			assigned_ = new uint64_t[num_blocks_];

			blocks_ = new warthog::jps_record**[num_blocks_];
			for(uint32_t i = 0; i < num_blocks_; i++)
			{
				assigned_[i] = 0;
				blocks_[i] = 0;
			}
		}

		~jps_graph()
		{
			for(uint32_t i = 0; i < num_blocks_; i++)
			{
				if(blocks_[i] == 0) { continue; }
				for(uint32_t j = 0; j < blocksize_; j++)
				{
					uint64_t mask = (uint64_t)1 << j;
					if(assigned_[i] & mask);
					{
						delete blocks_[i][j];
					}
				}
				delete [] blocks_[i];
			}
			delete [] blocks_;
			delete [] assigned_;
		}

		warthog::jps_record*
		find(uint32_t index)
		{
			uint32_t blockid = index >> log2_blocksize_;
			uint32_t elementid = index & blockmask_;
			uint64_t mask = (uint64_t)1 << elementid;
			if(assigned_[blockid] & (mask))
			{
				return blocks_[blockid][elementid];
			}
			return 0;
		}

		inline void
		insert(warthog::jps_record* value)
		{
			assert(value);
			uint32_t nodeid = value->get_id();
			uint32_t blockid = nodeid >> log2_blocksize_;	
			uint32_t elementid = nodeid & blockmask_;
			uint64_t set_mask = (uint64_t)1 << elementid;

			if(blockid >= num_blocks_)
			{
				grow(blockid+1);

				warthog::jps_record** block = new warthog::jps_record*[blocksize_];
				blocks_[blockid] = block;
				block[elementid] = value;
				assigned_[blockid] |= set_mask;
				return;
			}

			if(assigned_[blockid])
			{
				blocks_[blockid][elementid] = value;
				assigned_[blockid] |= set_mask;
			}
			else
			{
				warthog::jps_record** block = new warthog::jps_record*[blocksize_];
				blocks_[blockid] = block;
				block[elementid] = value;
				assigned_[blockid] |= set_mask;
			}
			assert(assigned_[blockid] & set_mask);
			assert(blocks_[blockid][elementid]->get_id() == nodeid);
		}
		
		inline void 
		grow(size_t newsize)
		{
			if(newsize <= num_blocks_) { return; }

			warthog::jps_record*** bigcollection =
			   	new warthog::jps_record**[newsize];
			uint64_t* bigassigned = new uint64_t[newsize];

			for(size_t i = 0; i < num_blocks_; i++)
			{
				bigcollection[i] = blocks_[i];
				bigassigned[i] = assigned_[i];
			}
			delete [] blocks_;
			delete [] assigned_;

			for(size_t i = num_blocks_; i < newsize; i++)
			{
				bigcollection[i] = 0;
				bigassigned[i] = 0;
			}
			blocks_ = bigcollection;
			assigned_ = bigassigned;
			num_blocks_ = newsize;
		}

		inline warthog::jps_record*
		remove(uint32_t index)
		{
			warthog::jps_record* ret = 0;
			uint32_t blockid = index >> log2_blocksize_;
			if(blockid <= num_blocks_)
			{
				uint32_t elementid = index & blockmask_;
				uint64_t clear_mask = ~((uint64_t)1 << elementid);
				assigned_[blockid] &= clear_mask;
				ret = blocks_[blockid][elementid];
				assert((assigned_[blockid] & ~clear_mask) == 0);

				if(!assigned_[blockid])
				{
					delete blocks_[blockid];
					blocks_[blockid] = 0;
				}
			}
			return ret;
		}

		uint32_t
		mem()
		{
			uint32_t total_mem = sizeof(this);
			total_mem += sizeof(*assigned_)*num_blocks_;
			for(uint32_t i = 0; i < num_blocks_; i++)
			{
				if(assigned_[i])
				{
					total_mem += sizeof(blocks_[i]) * blocksize_;
				}
				for(uint32_t j = 0; j < blocksize_; j++)
				{
					uint64_t mask = (uint64_t)1 << j;
					if(assigned_[i] & mask)
					{
						total_mem += blocks_[i][j]->mem();
					}
				}
				
			}
			return total_mem;
		}

	private:
		uint32_t blocksize_;
		uint32_t log2_blocksize_;

		uint64_t blockmask_;
		uint64_t* assigned_;
		uint32_t num_blocks_;
		warthog::jps_record*** blocks_;

};

}

#endif

