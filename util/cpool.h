#ifndef WARTHOG_CPOOL_H
#define WARTHOG_CPOOL_H

// cpool.h
//
// A pool of pre-allocated memory specialised for the construction of
// single structs of a fixed size.
//
// @author: dharabor
// @created: 23/08/2012
//

#include <algorithm>
#include <cassert>
#include <iostream>

namespace warthog
{

namespace mem
{

const int DEFAULT_CHUNK_SIZE = 1024*256; // 256K

class cchunk
{
	public:

		cchunk(size_t obj_size, size_t pool_size) :
			obj_size_(obj_size), pool_size_(pool_size)
		{
			mem_ = new char[obj_size_*pool_size_];
			next_addr_ = mem_;
			max_addr_ = mem_ + obj_size_*pool_size_;

			freed_stack_ = new char*[pool_size_];
			tofs_index_ = -1; // initially empty
		}

		~cchunk() 
		{
			delete [] mem_;
			delete [] freed_stack_;
		}

		inline char*
		allocate()
		{
			if(tofs_index_ >=0)
			{
				char* retval = freed_stack_[tofs_index_];
				tofs_index_--;
				return retval;
			}

			if((next_addr_ + obj_size_) < max_addr_)
			{
				char* retval = next_addr_;
				next_addr_ += obj_size_;
				return retval;
			}
			return 0;
		}

		inline void
		deallocate(char* addr)
		{
			if(addr < mem_ || (addr+obj_size_) >= max_addr_)
			{
				std::cerr << "err; warthog::mem::cchunk; freeing memory outside"
					" range of the chunk at addr: "<<&mem_ << "\n";
			}

			if((addr + obj_size_) > next_addr_)
			{
				std::cerr << "err; warthog::mem::cchunk freeing "
					<< "unallocated memory in chunk at addr "<<&mem_<<"\n";
			}

			++tofs_index_;
			assert(tofs_index_ < (int)pool_size_);
			if(tofs_index_ == (pool_size_-1))
			{
				// mark the entire chunk as free
				next_addr_ = mem_;
				tofs_index_ = -1;
			}
			else
			{
				// mark just the object at addr as free
				freed_stack_[tofs_index_] = addr;
			}
		}

		inline bool
		contains(char* addr)
		{
			if(addr >= mem_ && (addr + obj_size_) < max_addr_)
			{
				return true;
			}
			return false;
		}

	private:
		char* mem_;
		char* next_addr_;
		char* max_addr_;

		size_t obj_size_;  
		size_t pool_size_; 

		// keep a stack of freed objects
		char** freed_stack_;
		int tofs_index_; // top of freed stack
};

class cpool
{
	public:
		cpool(size_t obj_size) :
		   	num_chunks_(0), max_chunks_(1), obj_size_(obj_size)
		{
			chunks_ = new cchunk*[max_chunks_];
			for(int i = 0; i < (int) num_chunks_; i++)
			{
				add_chunk(warthog::mem::DEFAULT_CHUNK_SIZE);
			}
		}

		~cpool()
		{
			for(size_t i=0; i < num_chunks_; i++)
			{
				delete chunks_[i];
			}
			delete [] chunks_;
		}

		void
		add_chunk(size_t pool_size)
		{
			if(num_chunks_ < max_chunks_)
			{
				chunks_[num_chunks_] = new cchunk(obj_size_, pool_size);
				num_chunks_++;
			}
			else
			{
				// make room for a new chunk
				size_t big_max= max_chunks_*2;
				cchunk** big_chunks = new cchunk*[big_max];
				for(unsigned int i = 0; i < max_chunks_; i++)
				{
					big_chunks[i] = chunks_[i];
				}
				delete [] chunks_;
				chunks_ = big_chunks;
				max_chunks_ = big_max;

				// finally; add a new chunk
				chunks_[num_chunks_] = new cchunk(obj_size_, pool_size);
				num_chunks_++;
			}
		}

		inline char*
		allocate()
		{
			// look for space in an existing chunk
			// NB: linear-time search! increase DEFAULT_CHUNK_SIZE if
			// number of chunks grows too large
			char* mem_ptr = 0;
			for(unsigned int i=0; i < num_chunks_; i++)
			{
				mem_ptr = chunks_[i]->allocate();
				if(mem_ptr)
				{
					break;
				}
			}

			if(!mem_ptr)
			{
				// not enough space in any existing chunk; make a new one
				add_chunk(warthog::mem::DEFAULT_CHUNK_SIZE);
				mem_ptr = chunks_[num_chunks_-1]->allocate();
			}
			return mem_ptr;
		}

		inline void
		deallocate(char* addr)
		{
			for(unsigned int i=0; i < num_chunks_; i++)
			{
				if(chunks_[i]->contains(addr))
				{
					chunks_[i]->deallocate(addr);
					return;
				}
			}
			std::cerr << "err; cpool::free "
				"tried to free an address not in any chunk!\n";
		}

	private:
		warthog::mem::cchunk** chunks_;
		size_t num_chunks_;
		size_t max_chunks_;
		size_t obj_size_;

		// no copy
		cpool(const warthog::mem::cpool& other) { } 
		warthog::mem::cpool&
		operator=(const warthog::mem::cpool& other) { return *this; }

};

}

}

#endif

