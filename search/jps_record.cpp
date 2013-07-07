#include "jps_record.h"

warthog::jps_record::jps_record(uint32_t id) : id_(id)
{
	neis_ = new warthog::arraylist<jps_label>*[8];
	for(uint32_t i = 0; i < 8; i++)
	{
		neis_[i] = new warthog::arraylist<jps_label>(1, 1, 1);
	}
}

warthog::jps_record::~jps_record()
{
	for(uint32_t i = 0; i < 8; i++)
	{
		delete neis_[i];
	}
	delete [] neis_;
}

uint32_t
warthog::jps_record::mem()
{
	uint32_t total_mem = sizeof(this);
	for(uint32_t i = 0; i < 8; i++)
	{
		total_mem += sizeof(warthog::jps_label) * neis_[i]->size();
	}
	return total_mem;
}
