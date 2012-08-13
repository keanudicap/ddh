#include "heap.h"

warthog::heap::heap(unsigned int s, bool minheap) 
	: maxsize_(s), minheap_(minheap), heapsize_(0), elts_(0)
{
	resize(s);
}

warthog::heap::~heap()
{
	keymap_.clear();
	delete [] elts_;
}

void 
warthog::heap::push(warthog::search_node *val)
{
	if(contains(val))
	{
		return;
	}

	if(heapsize_+1 > maxsize_)
	{
		resize(maxsize_*2);
	}
	elts_[heapsize_] = val;
	keymap_[val->id()] = heapsize_;
	heapsize_++;
	heapify_up(heapsize_-1);
}

warthog::search_node*
warthog::heap::pop()
{
	if (heapsize_ == 0)
	{
		return 0;
	}

	warthog::search_node *ans = elts_[0];
	heapsize_--;
	keymap_.erase(ans->id());

	if(heapsize_ > 0)
	{
		elts_[0] = elts_[heapsize_];
		heapify_down(0);
	}
	return ans;
}

void 
warthog::heap::heapify_up(unsigned int index)
{
	while(index > 0)
	{
		unsigned int parent = (index-1) >> 1;
		if(rotate(*elts_[parent], *elts_[index]))
		{
			swap(parent, index);
			index = parent;
		}
		else
		{
			break;
		}
	}
}

void 
warthog::heap::heapify_down(unsigned int index)
{
	while(index < (heapsize_ >> 1))
	{
		// find smallest (or largest, depending on heap type) child
		unsigned int child1 = index*2+1;
		unsigned int child2 = index*2+2;
		unsigned int which = child1;
		if ((child2 < heapsize_) && rotate(*elts_[child1], *elts_[child2]))
		{
			which = child2;
		}

		// swap child with parent if necessary
		if (rotate(*elts_[index], *elts_[which]))
		{
			swap(index, which);
			index = which;
		}
		else
		{
			break;
		}
	}
}

void 
warthog::heap::decrease_key(warthog::search_node* val)
{	
 	warthog::heap::node_list::const_iterator it = keymap_.find(val->id());
	if(it == keymap_.end())
	{
		return;
	}

	if(minheap_)
	{
		heapify_up((*it).second);
	}
	else
	{
		heapify_down((*it).second);
	}
}

void 
warthog::heap::increase_key(warthog::search_node* val)
{
	warthog::heap::node_list::const_iterator it = keymap_.find(val->id());
	if(it == keymap_.end())
	{
		return;
	}

	if(minheap_)
	{
		heapify_down((*it).second);
	}
	else
	{
		heapify_up((*it).second);
	}
}

void
warthog::heap::resize(unsigned int newsize)
{
//	std::cout << "heap::resize oldsize: "<<heapsize_<<" newsize " << newsize<<std::endl;
	if(newsize < heapsize_)
	{
		std::cerr << "err; heap::resize newsize < heapsize " << std::endl;
		exit(1);
	}

	keymap_.rehash(newsize);
 	warthog::search_node** tmp = new search_node*[newsize];
	for(unsigned int i=0; i < heapsize_; i++)
	{
		tmp[i] = elts_[i];
	}
	delete [] elts_;
	elts_ = tmp;
	maxsize_ = newsize;
}

void
warthog::heap::clear()
{
	keymap_.clear();
	for(unsigned int i=0; i < heapsize_; i++)
	{
		elts_[i] = 0;
	}
}

void 
warthog::heap::print(std::ostream& out)
{
	for(unsigned int i=0; i < heapsize_; i++)
	{
		elts_[i]->print(out);
		out << std::endl;
	}
}

