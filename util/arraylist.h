#ifndef WARTHOG_ARRAYLIST_H
#define WARTHOG_ARRAYLIST_H

// arraylist.h
//
// A dumb but dynamically-sized array for both simple and complex types.
// The array has an initial size that defaults to 100 elements. New elements are 
// added to the end of the collection. When the number of elements exceeds 
// the capacity of the array, its size increases according to the formula:
// [new size] = [old size] * ::growth_factor_ + ::growth_additive_constant_.
//
// By default ::growth_factor_ = 2 and growth_additive_constant_ = 0.
//
// @author: dharabor
// @created: 22/05/2013
//

#include <cstring>
#include <stdint.h>

namespace warthog
{
	template <typename T>
	class arraylist
	{
		public:
			typedef T* iterator;

			arraylist(size_t size, size_t gfactor, size_t gadd) 
				: max_size_(size), terminator_(0), 
				growth_factor_(gfactor), growth_additive_constant_(gadd)
			{
				collection_ = new T[max_size_];	
				next_ = 0;
			}

			arraylist(size_t size=100) : max_size_(size), terminator_(0),
				growth_factor_(2), growth_additive_constant_(0)
			{
				collection_ = new T[max_size_];	
				next_ = 0;
			}

			~arraylist()
			{
				delete [] collection_;
			}

			inline T
			at(uint32_t index)
			{
				return collection_[index];
			}

			inline T
			operator[](uint32_t index)
			{
				return collection_[index];
			}

			inline void
			push_back(T element)
			{
				if(next_ < max_size_)
				{
					collection_[next_] = element;	
					next_++;
				}
				else
				{
					grow(size()*growth_factor_ + growth_additive_constant_);
					collection_[next_] = element;
					next_++;
				}
			}

			inline void
			pop_back()
			{
				if(next_ > 0)
				{
					next_--;
				}
			}

			inline void
			clear() 
			{
				next_ = 0;
			}

			inline size_t
			size()
			{
				return next_;
			}

			size_t
			mem() { return sizeof(T)*max_size_;}


		private:
			T* collection_;
			size_t next_;
			uint32_t max_size_;
			const typename warthog::arraylist<T>::iterator terminator_;

			uint32_t growth_factor_;
			uint32_t growth_additive_constant_;

			inline void 
			grow(size_t newsize)
			{
				if(newsize <= max_size_) { return; }

				T* bigcollection = new T[newsize];
				for(size_t i = 0; i < max_size_; i++)
				{
					bigcollection[i] = collection_[i];
				}
				delete [] collection_;
				collection_ = bigcollection;
				max_size_ = newsize;
			}

	};
}

#endif

