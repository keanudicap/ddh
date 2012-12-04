#ifndef WARTHOG_GRIDMAP_H
#define WARTHOG_GRIDMAP_H

// gridmap.h
//
// A gridmap for simple uniform cost or weighted cost gridmaps.
// This implementation stores gridmaps in a compact matrix form.
// For the weighted case nodes are represented as single byte quantities.
// For the uniform case, nodes are represented as single bit quantities.
//
// NB: To allow efficient lookups this implementation uses a padding scheme
// that adds extra elements to the end of each row 
// and also introduces an empty row immediately before and immediately after 
// the start and end of the gridmap data.
// Padding allows us to refer to tiles and their neighbours by their indexes
// in a one dimensional array and also to avoid range checks when trying to 
// identify invalid neighbours of tiles on the edge of the map.
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
// @author: dharabor
// @created: 08/08/2012
// 

#include "constants.h"
#include "helpers.h"
#include "gm_parser.h"

#include <climits>
#include "stdint.h"

namespace warthog
{

class gridmap
{
	public:
		gridmap(uint32_t height, unsigned int width, bool uniform);
		gridmap(const char* filename, bool uniform);
		~gridmap();

		// here we convert from the coordinate space of 
		// the grid to the coordinate space of db_. 
		inline uint32_t
		to_padded_id(uint32_t node_id)
		{
			return node_id + 
				// add one full padded row
				padded_width_ +
			   	// padding from each row of data before this one
				(node_id / header_.width_) * padding_ + 
				// padding at the front of the current row
				1;
		}

		inline void
		get_horizontal_neis_from_padded_id(uint32_t padded_id, char tiles[3])
		{
			// 1. calculate the dbword offset for the node at index padded_id
			// 2. convert padded_id into a dbword index.
			uint32_t bit_offset = (padded_id & warthog::DBWORD_BITS_MASK);
			uint32_t dbindex = padded_id >> warthog::LOG2_DBWORD_BITS;

			// copy each neighbour into @param tiles
			// there are a few special cases to consider, depending
			// on whether node_id corresponds to a bit at the beginning, 
			// at the end, or in the middle of its corresponding dbword.
			switch(bit_offset)
			{
				// node_id is at the beginning of its dbword
				// some neighbours are in pos, pos2 and pos3
				// while others are in pos-1, pos2-1 and pos3-1
				case 0:
				{
					// NB: implicit bitmask: 1 << bit_offset
					tiles[0] = (db_[dbindex-1] & 128) ? 1:0;
					tiles[1] = (db_[dbindex] & 1) ? 1:0;
					tiles[2] = (db_[dbindex] & 2) ? 1:0;
					break;
				}
				// node_id is at the end of its dbword
				// some neighbours are in pos, pos2 and pos3
				// while others are in pos+1, pos2+1 and pos3+1
				case 7:
				{
					// NB: implicit bitmask: 1 << bit_offset
					tiles[0] = (db_[dbindex] & 64) ? 1:0;
					tiles[1] = (db_[dbindex] & 128) ? 1:0;
					tiles[2] = (db_[dbindex+1] & 1) ? 1:0;
					break;
				}	
				// node_id is in the middle of its dbword
				// all neighbours are in pos, pos2 and pos3
				default:
				{
					tiles[0] = (db_[dbindex] & (bit_offset-1)) ? 1:0;
					tiles[1] = (db_[dbindex] & bit_offset) ? 1:0;
					tiles[2] = (db_[dbindex] & (bit_offset+1)) ? 1:0;
					break;
				}
			}
		}

		inline void
		get_vertical_neis_from_padded_id(uint32_t padded_id, char tiles[3])
		{
			// 1. calculate the dbword offset for the node at index padded_id
			// 2. convert padded_id into a dbword index.
			uint32_t bit_offset = (padded_id & warthog::DBWORD_BITS_MASK);
			uint32_t dbindex1 = padded_id >> warthog::LOG2_DBWORD_BITS;

			// compute dbword indexes for tiles immediately above 
			// and immediately below node_id
			uint32_t dbindex0 = padded_id - dbwidth_;
			uint32_t dbindex2 = padded_id + dbwidth_;

			// copy each neighbour into @param tiles
			// there are a few special cases to consider, depending
			// on whether node_id corresponds to a bit at the beginning, 
			// at the end, or in the middle of its corresponding dbword.
			tiles[0] = (db_[dbindex0] & bit_offset) ? 1:0;
			tiles[1] = (db_[dbindex1] & bit_offset) ? 1:0;
			tiles[2] = (db_[dbindex2] & bit_offset) ? 1:0;
		}

		// get the immediately adjacent neighbours of @param node_id
		// neighbours from the row above node_id are stored in 
		// @param tiles[0], neighbours from the same row in tiles[1]
		// and neighbours from the row below in tiles[2].
		// In each case the bits for each neighbour are in the three 
		// lowest positions of the byte.
		// position :0 is the nei in direction NW, :1 is N and :2 is NE 
		inline void
		get_neighbours2(uint32_t node_id, char tiles[3])
		{
			// TODO: can we eliminate this check with an extra bit of padding
			// around the edges of the map? (i.e. map size = (w+2)*(h+2))
			if(node_id >= max_id_) 
			{
				tiles[0] = tiles[1] = tiles[2] = 0;
				return;
			}
			uint32_t padded_id = to_padded_id(node_id);

			// 1. calculate the dbword offset for the node at index padded_id
			// 2. convert padded_id into a dbword index.
			uint32_t bit_offset = (padded_id & warthog::DBWORD_BITS_MASK);
			padded_id >>= warthog::LOG2_DBWORD_BITS;

			// compute dbword indexes for tiles immediately above 
			// and immediately below node_id
			uint32_t pos1 = padded_id - dbwidth_;
			uint32_t pos2 = padded_id;
			uint32_t pos3 = padded_id + dbwidth_;

			// read from the byte just before node_id and shift down until the
			// nei adjacent to node_id is in the lowest position
			tiles[0] = (char)(*((int*)(db_+(pos1-1))) >> (bit_offset+7));
			tiles[1] = (char)(*((int*)(db_+(pos2-1))) >> (bit_offset+7));
			tiles[2] = (char)(*((int*)(db_+(pos3-1))) >> (bit_offset+7));
		}

		// get all the tiles adjacent to nodeid. this function begins at 
		// node (x-1, y-1) (relative to node_id) and steps through the
		// list of adjacent tiles in left-to-right, top-to-bottom order
		//
		// @return true if the function is given a valid node_id 
		// and finishes successfully tiles is populated with the terrain
		// types of the 3x3 square centred at node_id. 
		inline void
		get_neighbours(uint32_t node_id, char tiles[9])
		{
			// TODO: can we eliminate this check with an extra bit of padding
			// around the edges of the map? (i.e. map size = (w+2)*(h+2))
			if(node_id >= max_id_) 
			{
				tiles[0] = tiles[1] = tiles[2] = tiles[3] = tiles[4] = 
				tiles[5] = tiles[6] = tiles[7] = tiles[8] = 0;
				return;
			}

			uint32_t padded_id = to_padded_id(node_id);
			if(uniform_)
			{
				// 1. calculate the dbword offset for the node at index padded_id
				// 2. convert padded_id into a dbword index.
				uint32_t bit_offset = (padded_id & warthog::DBWORD_BITS_MASK);
				padded_id >>= warthog::LOG2_DBWORD_BITS;

				// compute dbword indexes for tiles immediately above 
				// and immediately below node_id
				uint32_t pos = padded_id - dbwidth_;
				uint32_t pos2 = padded_id;
				uint32_t pos3 = padded_id + dbwidth_;

				// copy each neighbour into @param tiles
				// there are a few special cases to consider, depending
				// on whether node_id corresponds to a bit at the beginning, 
				// at the end, or in the middle of its corresponding dbword.
				switch(bit_offset)
				{
					// node_id is at the beginning of its dbword
					// some neighbours are in pos, pos2 and pos3
					// while others are in pos-1, pos2-1 and pos3-1
					case 0:
					{
						// NB: implicit bitmask: 1 << bit_offset
						tiles[0] = (db_[pos-1] & 128) ? 1:0;
						tiles[1] = (db_[pos] & 1) ? 1:0;
						tiles[2] = (db_[pos] & 2) ? 1:0;
						tiles[3] = (db_[pos2-1] & 128) ? 1:0;
						tiles[4] = (db_[pos2] & 1) ? 1:0;
						tiles[5] = (db_[pos2] & 2) ? 1:0;
						tiles[6] = (db_[pos3-1] & 128) ? 1:0;
						tiles[7] = (db_[pos3] & 1) ? 1:0;
						tiles[8] = (db_[pos3] & 2) ? 1:0;
						break;
					}
					// node_id is at the end of its dbword
					// some neighbours are in pos, pos2 and pos3
					// while others are in pos+1, pos2+1 and pos3+1
					case 7:
					{
						// NB: implicit bitmask: 1 << bit_offset
						tiles[0] = (db_[pos] & 64) ? 1:0;
						tiles[1] = (db_[pos] & 128) ? 1:0;
						tiles[2] = (db_[pos+1] & 1) ? 1:0;
						tiles[3] = (db_[pos2] & 64) ? 1:0;
						tiles[4] = (db_[pos2] & 128) ? 1:0;
						tiles[5] = (db_[pos2+1] & 1) ? 1:0;
						tiles[6] = (db_[pos3] & 64) ? 1:0;
						tiles[7] = (db_[pos3] & 128) ? 1:0;
						tiles[8] = (db_[pos3+1] & 1) ? 1:0;
						break;
					}	
					// node_id is in the middle of its dbword
					// all neighbours are in pos, pos2 and pos3
					default:
					{
						uint32_t bitmask1 = (1 << (bit_offset-1));
						uint32_t bitmask2 = (1 << bit_offset);
						uint32_t bitmask3 = (1 << (bit_offset+1));
						tiles[0] = (db_[pos] & bitmask1) ? 1:0;
						tiles[1] = (db_[pos] & bitmask2) ? 1:0;
						tiles[2] = (db_[pos] & bitmask3) ? 1:0;
						tiles[3] = (db_[pos2] & bitmask1) ? 1:0;
						tiles[4] = (db_[pos2] & bitmask2) ? 1:0;
						tiles[5] = (db_[pos2] & bitmask3) ? 1:0;
						tiles[6] = (db_[pos3] & bitmask1) ? 1:0;
						tiles[7] = (db_[pos3] & bitmask2) ? 1:0;
						tiles[8] = (db_[pos3] & bitmask3) ? 1:0;
						break;
					}
				}
			}
			else
			{
				uint32_t pos = padded_id - dbwidth_;
				uint32_t pos2 = padded_id;
				uint32_t pos3 = padded_id + dbwidth_;
				tiles[0] = db_[pos];
				tiles[1] = db_[pos+1];
				tiles[2] = db_[pos+2];
				tiles[3] = db_[pos2];
				tiles[4] = db_[pos2+1];
				tiles[5] = db_[pos2+2];
				tiles[6] = db_[pos3];
				tiles[7] = db_[pos3+1];
				tiles[8] = db_[pos3+2];
			}
		}

		inline warthog::dbword
		get_label(uint32_t x, unsigned int y)
		{
			if(x >= header_.width_ || y >= header_.height_)
			{
				return 0;
			}
			return this->get_label(y*header_.width_+x);
		}

		inline warthog::dbword 
		get_label(uint32_t node_id)
		{
			if(node_id >= max_id_)
			{
				return 0;
			}

			// here we convert from the coordinate space of 
			// the grid to the coordinate space of db_. 
			uint32_t padded_id = node_id + 
				// add one full padded row
				padded_width_ +
			   	// padding from each row of data before this one
				(node_id / header_.width_) * padding_ + 
				// padding at the front of the current row
				1;

			// now we can fetch the label
			if(uniform_)
			{
				warthog::dbword bitmask = 1;
				bitmask <<= (padded_id & warthog::DBWORD_BITS_MASK);
				padded_id >>= warthog::LOG2_DBWORD_BITS;
				return (db_[padded_id] & bitmask) != 0;
			}
			return db_[padded_id];
		}

		inline void
		set_label(uint32_t x, unsigned int y, warthog::dbword label)
		{
			this->set_label(y*header_.width_+x, label);
		}

		inline void 
		set_label(uint32_t node_id, warthog::dbword label)
		{
			if(node_id >= max_id_)
			{
				return;
			}

			// here we convert from the coordinate space of 
			// the grid to the coordinate space of db_. 
			uint32_t padded_id = node_id + 
				// add one full padded row
				padded_width_ +
			   	// padding from each row of data before this one
				(node_id / header_.width_) * padding_ + 
				// padding at the front of the current row
				1;

			if(uniform_)
			{
				warthog::dbword bitmask =
				   	(1 << (padded_id & warthog::DBWORD_BITS_MASK));
				padded_id >>= warthog::LOG2_DBWORD_BITS;
				if(label)
				{
					db_[padded_id] |= bitmask;
				}
				else
				{
					db_[padded_id] &= ~bitmask;
				}
			}
			else
			{
				db_[padded_id] = label;
			}
		}

		inline uint32_t 
		height() const
		{ 
			return this->header_.height_; 
		} 

		inline uint32_t 
		width() const 
		{ 
			return this->header_.width_;
		}

//		inline uint32_t
//		db_width()
//		{
//			return dbwidth_;
//		}
//
//		inline uint32_t
//		db_height()
//		{
//			return dbheight_;
//		}

		void 
		print(std::ostream&);
		
		void
		printdb(std::ostream& out);

		uint32_t 
		mem()
		{
			return sizeof(*this) +
			sizeof(warthog::dbword) * db_size_;
		}


	private:
		bool uniform_;
		warthog::gm_header header_;
		warthog::dbword* db_;
		uint32_t dbwidth_;
		uint32_t dbheight_;
		uint32_t db_size_;
		uint32_t padded_width_;
		uint32_t padding_;
		uint32_t max_id_;

		gridmap(const warthog::gridmap& other) {}
		gridmap& operator=(const warthog::gridmap& other) { return *this; }
		void init_db();
};

}

#endif

