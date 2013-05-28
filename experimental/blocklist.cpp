#include "blocklist.h"
#include "helpers.h"
#include "search_node.h"

#include <cassert>

warthog::blocklist::blocklist(uint32_t mapheight, uint32_t mapwidth)
	: blocks_(0), pool_(0)
{
	assert(warthog::blocklist_ns::NBS == 64);
	num_blocks_ = ((mapwidth*mapheight) >> warthog::blocklist_ns::LOG2_NBS)+1;
	blocks_ = new warthog::nodeblock[num_blocks_];

	blockspool_ = 
		new warthog::mem::cpool(sizeof(void*)*warthog::blocklist_ns::NBS, 1);
	pool_ = new warthog::mem::cpool(sizeof(warthog::search_node));

	for(uint32_t i=0; i < num_blocks_; i++)
	{
		blocks_[i].nodes_ = 0;
		blocks_[i].isallocated_ = 0;
	}
}

warthog::blocklist::~blocklist()
{
	clear();
	delete pool_;
	delete blockspool_;
}

warthog::search_node*
warthog::blocklist::generate(uint32_t node_id)
{
	uint32_t block_id = node_id >> warthog::blocklist_ns::LOG2_NBS;
	uint32_t list_id = node_id &  warthog::blocklist_ns::NBS_MASK;
	assert(block_id <= num_blocks_);

	if(!blocks_[block_id].nodes_)
	{
		// allocate space for a new block of nodes
		//std::cerr << "generating block: "<<block_id<<std::endl;
		blocks_[block_id].nodes_ = new (blockspool_->allocate())
		   	warthog::search_node*[warthog::blocklist_ns::NBS];
		blocks_[block_id].isallocated_ = 0;
		//for(int i = 0; i < 64; i++)
		//{
		//	blocks_[block_id][i] = 0;
		//}
	}

	uint64_t flag = 1;
	flag <<= list_id;
	if(blocks_[block_id].isallocated_ & flag)
	{
		//assert(blocks_[block_id][list_id] != 0);
		// return previously allocated node
		return blocks_[block_id].nodes_[list_id];
	}

	// allocate a new node
	//assert(blocks_[block_id][list_id] == 0);
	warthog::search_node* mynode = 
		new (pool_->allocate()) warthog::search_node(node_id);
	blocks_[block_id].nodes_[list_id] = mynode;
	blocks_[block_id].isallocated_ |= flag;
	return mynode;
}

void
warthog::blocklist::clear()
{
	for(uint32_t i=0; i < num_blocks_; i++)
	{
		if(blocks_[i].nodes_ != 0)
		{
			//std::cerr << "deleting block: "<<i<<std::endl;
			blocks_[i].nodes_ = 0;
			blocks_[i].isallocated_ = 0;
		}
	}
	pool_->reclaim();
	blockspool_->reclaim();
}

uint32_t
warthog::blocklist::mem()
{
	uint32_t bytes = 
		sizeof(*this) + 
		blockspool_->mem() + pool_->mem() + 
		num_blocks_ * sizeof(warthog::nodeblock);

	return bytes;
}
