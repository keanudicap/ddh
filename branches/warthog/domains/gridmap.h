#ifndef WARTHOG_GRIDMAP_H
#define WARTHOG_GRIDMAP_H

// gridmap.h
//
// A gridmap for simple uniform cost or weighted cost gridmaps.
// This implementation stores gridmaps in a compact matrix form.
// For the weighted case nodes are represented as single byte quantities.
// For the uniform case, nodes are represented as single bit quantities.
//
// TODO: Support arbitrary node labels (e.g. complex types). Need different 
// word sizes for this; e.g. a labels may be many bytes as well as single
// bytes or single bits. This would be much nicer than allocating complex
// node labels separately and just storing in the matrix pointers to them.
//
// TODO: Template suport in addition to the above -- rather than specialising
// just for one type of complex label. Code everything in terms of bytes
// and use sizeof(T) to compute indexes. Remember to add some specialised
// versions for bool (don't want to lose bitpacking).
//
// TODO: Compress y-dimension in addition to x when working with uniform grids
// i.e. each entry in ::db_[x][y] should contain a square block of bits of size
// DBWORD_BITS*DBWORD_BITS
//
// @author: dharabor
// @created: 08/08/2012
// 

#include "constants.h"
#include "gm_parser.h"

#include <climits>

namespace warthog
{

class gridmap
{
	public:
		gridmap(unsigned int height, unsigned int width, bool uniform);
		gridmap(const char* filename, bool uniform);
		virtual ~gridmap();

		inline warthog::dbword get_label(unsigned int x, unsigned int y)
		{
			if(x >= 0 && y >= 0 && x < this->width() && y < this->height())
			{
				if(this->uniform_)
				{
					dbword bitmask = (1 << (x & (warthog::DBWORD_BITS-1)));
					int dbx = x >> warthog::LOG2_DBWORD_BITS;
					return (this->db_[dbx][y] & bitmask) != 0;
				}
				else
				{
					return this->db_[x][y];
				}
			}
			return false;
		}

		inline void set_label(unsigned int x, unsigned int y, warthog::dbword label)
		{
			if(x >= 0 && y >= 0 && x < this->width() && y < this->height())
			{
				if(this->uniform_)
				{
					dbword bitmask = (1 << (x & (warthog::DBWORD_BITS-1)));
					int dbx = x >> warthog::LOG2_DBWORD_BITS;
					if(label)
					{
						this->db_[dbx][y] |= bitmask;
					}
					else
					{
						this->db_[dbx][y] &= ~bitmask;
					}
				}
				else
				{
					db_[x][y] = label;
				}
			}
		}

		inline unsigned int height() 
		{ 
			return this->header_.height_; 
		} 

		inline unsigned int width() 
		{ 
			return this->header_.width_;
		}

		void print(std::ostream&);

	private:
		gridmap(const warthog::gridmap& other) {}
		gridmap& operator=(const warthog::gridmap& other) { return *this; }
		unsigned int dbwidth();
		unsigned int dbheight();

		bool uniform_;
		warthog::gm_header header_;
		warthog::dbword** db_;
};

}

#endif

