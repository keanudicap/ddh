#include "jpsp_record.h"

warthog::jpsp_record::jpsp_record(uint32_t id) : id_(id)
{
	num_neis_ = new uint32_t[8];
	neis_ = new warthog::arraylist<uint32_t>*[8];
	for(int i = 0; i < 8; i++)
	{
		neis_[i] = 0;
		num_neis_[i] = 0;
	}
}

warthog::jpsp_record::~jpsp_record()
{
	for(int i = 0; i < 8; i++)
	{
		delete neis_[i];
	}
	delete [] neis_;
	delete [] num_neis_;
}

void
warthog::jpsp_record::add_successor(warthog::jps::direction pdir, 
		uint16_t xdelta, uint16_t ydelta)
{
	assert(pdir != warthog::jps::NONE);
	uint32_t index = pdir == __builtin_ffs(pdir) - 1;
	if(num_neis_[index] == 0)
	{
		assert(neis_[index] == 0);
		neis_[index] = new warthog::arraylist<uint32_t>(1, 1, 1);
	}

	uint32_t succ = (((uint32_t)xdelta) << 16) | ydelta;
	neis_[index]->push_back(succ);
	num_neis_[index]++;
}

void
warthog::jpsp_record::get_successor(warthog::jps::direction pdir, 
		size_t nei_index, uint16_t& xdelta, uint16_t& ydelta)
{
	assert(pdir != warthog::jps::NONE);
	uint32_t index = __builtin_ffs(pdir) - 1;
	assert(nei_index < num_neis_[pdir]);
	uint32_t succ = neis_[index]->at(nei_index);
	ydelta = succ & UINT16_MAX;
	xdelta = (succ >> 16) & UINT16_MAX;
}
