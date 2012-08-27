#ifndef WARTHOG_CPOOL_H
#define WARTHOG_CPOOL_H

// cpool.h
//
// A pool of pre-allocated memory specialised for the construction of
// single structs of a fixed size.
// To achieve efficient re-allocation each pre-allocated
// chunk of memory has associated with it a stack of memory offsets
// which have been previously freed. 
// This introduces a 12.5% overhead to total memory consumption.
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
			if(pool_size_ < obj_size_)
			{
				std::cerr << "warthog::mem::cchunk object size < pool size; "
					<< "setting pool size to object size"<<std::endl;
				pool_size_ = obj_size_;
			}

			mem_ = new char[pool_size_];
			next_addr_ = mem_;
			max_addr_ = mem_ + pool_size_;

			freed_stack_ = new int[(pool_size_/obj_size)];
			stack_size_ = 0;
		}

		~cchunk() 
		{
			delete [] mem_;
			delete [] freed_stack_;
		}

		inline char*
		allocate()
		{
			if(stack_size_ > 0)
			{
				char* retval = &mem_[freed_stack_[stack_size_-1]];
				--stack_size_;
				return retval;
			}

			if((next_addr_ + obj_size_) <= max_addr_)
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
			if(addr < mem_ || (addr+obj_size_) > max_addr_)
			{
				std::cerr << "err; warthog::mem::cchunk; freeing memory outside"
					" range of the chunk at addr: "<<&mem_ << "\n";
			}

			if((addr + obj_size_) > next_addr_)
			{
				std::cerr << "err; warthog::mem::cchunk freeing "
					<< "unallocated memory in chunk at addr "<<&mem_<<"\n";
			}

			freed_stack_[stack_size_] = addr - mem_;
			stack_size_++;
			if(stack_size_*obj_size_ == pool_size_)
			{
				// mark the entire chunk as free
				next_addr_ = mem_;
				stack_size_ = 0;
			}
		}

		inline bool
		contains(char* addr)
		{
			if(addr >= mem_ && (addr + obj_size_) <= max_addr_)
			{
				return true;
			}
			return false;
		}

		char* 
		first_addr()
		{
			return mem_;
		}

		size_t
		pool_size()
		{
			return pool_size_;
		}
		
		size_t
		mem()
		{
			size_t bytes = sizeof(*this);
			bytes += sizeof(char)*pool_size_;
			bytes += sizeof(int)*(pool_size_/obj_size_);
			return bytes;
		}

		void
		print(std::ostream& out)
		{
			out << "warthog::mem::cchunk pool_size: "<<pool_size_ 
				<< " obj_size: "<<obj_size_<< " freed_stack_ size: "<<stack_size_;
		}

	private:
		char* mem_;
		char* next_addr_;
		char* max_addr_;

		size_t obj_size_;  
		size_t pool_size_; 

		// keep a stack of freed objects
		int* freed_stack_;
		size_t stack_size_;
};

class cpool
{
	public:
		cpool(size_t obj_size) :
		   	num_chunks_(0), max_chunks_(1), obj_size_(obj_size)
		{
			chunks_ = new cchunk*[max_chunks_];
			chunk_faddr_ = new char*[max_chunks_];
			chunk_size_ = new size_t[max_chunks_];

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
			delete [] chunk_faddr_;
			delete [] chunk_size_;
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
				// TODO: this loop involves jumping through memory in 
				// poolsize steps and doing comparisons against address. 
				// it would be better if the values of these address were 
				// part of a header struct which could all be initialised 
				// and stored separately.
				if(addr >= chunk_faddr_[i] && (addr+obj_size_) <= (chunk_faddr_[i]+chunk_size_[i]))
				{
					chunks_[i]->deallocate(addr);
					return;
				}
			}
			std::cerr << "err; cpool::free "
				"tried to free an address not in any chunk!\n";
		}

		size_t
		mem()
		{
			size_t bytes = 0;
			for(unsigned int i=0; i < num_chunks_; i++)
			{
				bytes += chunks_[i]->mem();
			}
			bytes += sizeof(warthog::mem::cchunk*)*max_chunks_;
			bytes += sizeof(chunk_faddr_)*max_chunks_;
			bytes += sizeof(chunk_size_)*max_chunks_;
			bytes += sizeof(*this);
			return bytes;
		}

		void
		print(std::ostream& out)
		{
			out << "warthog::mem::cpool #chunks: "<<num_chunks_ 
			<<	" #max_chunks "<<max_chunks_ << " obj_size: "<<obj_size_;
			out << std::endl;
			for(unsigned int i = 0; i < num_chunks_; i++)
			{
				chunks_[i]->print(out);
				out << std::endl;
			}
		}

	private:
		warthog::mem::cchunk** chunks_;
		char** chunk_faddr_; // first addr of each chunk
		size_t* chunk_size_; // size of each chunk

		size_t num_chunks_;
		size_t max_chunks_;
		size_t obj_size_;

		// no copy
		cpool(const warthog::mem::cpool& other) { } 
		warthog::mem::cpool&
		operator=(const warthog::mem::cpool& other) { return *this; }

		void
		add_chunk(size_t pool_size)
		{
			if(num_chunks_ < max_chunks_)
			{
				chunks_[num_chunks_] = new cchunk(obj_size_, pool_size);
				chunk_faddr_[num_chunks_] = chunks_[num_chunks_]->first_addr();
				chunk_size_[num_chunks_] = chunks_[num_chunks_]->pool_size();
				num_chunks_++;
			}
			else
			{
				// make room for a new chunk
				size_t big_max= max_chunks_*2;
				cchunk** big_chunks = new cchunk*[big_max];
				char** big_chunk_faddr = new char*[big_max];
				size_t* big_chunk_size = new size_t[big_max];
				for(unsigned int i = 0; i < max_chunks_; i++)
				{
					big_chunks[i] = chunks_[i];
					big_chunk_faddr[i] = chunk_faddr_[i];
					big_chunk_size[i] = chunk_size_[i];
				}
				delete [] chunks_;
				delete [] chunk_faddr_;
				delete [] chunk_size_;

				chunks_ = big_chunks;
				chunk_faddr_ = big_chunk_faddr;
				chunk_size_ = big_chunk_size;
				max_chunks_ = big_max;

				// finally; add a new chunk
				chunks_[num_chunks_] = new cchunk(obj_size_, pool_size);
				chunk_faddr_[num_chunks_] = chunks_[num_chunks_]->first_addr();
				chunk_size_[num_chunks_] = chunks_[num_chunks_]->pool_size();
				num_chunks_++;
			}
		}
};

}

}

#endif

