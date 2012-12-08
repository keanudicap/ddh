#ifndef WARTHOG_GRIDMAP_H
#define WARTHOG_GRIDMAP_H

// gridmap.h
//
// A uniform cost gridmap implementation.  The map is stored in
// a compact matrix form. Nodes are represented as single bit quantities.
//
// NB: To allow efficient lookups this implementation uses a padding scheme
// that adds extra elements to the end of each row and also introduces an 
// empty row immediately before and immediately after the start and end of 
// the gridmap data.
// Padding allows us to refer to tiles and their neighbours by their indexes
// in a one dimensional array and also to avoid range checks when trying to 
// identify invalid neighbours of tiles on the edge of the map.
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
		gridmap(uint32_t height, unsigned int width);
		gridmap(const char* filename);
		~gridmap();

		// here we convert from the coordinate space of 
		// the original grid to the coordinate space of db_. 
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

		// here we convert from the coordinate space of 
		// the original grid to the coordinate space of db_. 
		inline uint32_t
		to_padded_id(uint32_t x, uint32_t y)
		{
			return (y+1)*padded_width_ + (x+1);
		}

		// get the immediately adjacent neighbours of @param node_id
		// neighbours from the row above node_id are stored in 
		// @param tiles[0], neighbours from the same row in tiles[1]
		// and neighbours from the row below in tiles[2].
		// In each case the bits for each neighbour are in the three 
		// lowest positions of the byte.
		// position :0 is the nei in direction NW, :1 is N and :2 is NE 
		inline void
		get_neighbours(uint32_t padded_id, uint8_t tiles[3])
		{
			// 1. calculate the dbword offset for the node at index padded_id
			// 2. convert padded_id into a dbword index.
			uint32_t bit_offset = (padded_id & warthog::DBWORD_BITS_MASK);
			uint32_t dbindex = padded_id >> warthog::LOG2_DBWORD_BITS;
			if(dbindex >= max_id_) 
			{
				tiles[0] = tiles[1] = tiles[2] = 0;
				return;
			}

			// compute dbword indexes for tiles immediately above 
			// and immediately below node_id
			uint32_t pos1 = dbindex - dbwidth_;
			uint32_t pos2 = dbindex;
			uint32_t pos3 = dbindex + dbwidth_;

			// read from the byte just before node_id and shift down until the
			// nei adjacent to node_id is in the lowest position
			tiles[0] = (uint8_t)(*((uint32_t*)(db_+(pos1-1))) >> (bit_offset+7));
			tiles[1] = (uint8_t)(*((uint32_t*)(db_+(pos2-1))) >> (bit_offset+7));
			tiles[2] = (uint8_t)(*((uint32_t*)(db_+(pos3-1))) >> (bit_offset+7));
		}

		
		inline bool
		get_label(uint32_t x, unsigned int y)
		{
			return this->get_label(y*padded_width_+x);
		}

		inline warthog::dbword 
		get_label(uint32_t padded_id)
		{
			// now we can fetch the label
			warthog::dbword bitmask = 1;
			bitmask <<= (padded_id & warthog::DBWORD_BITS_MASK);
			uint32_t dbindex = padded_id >> warthog::LOG2_DBWORD_BITS;
			if(dbindex > max_id_) { return 0; }
			return (db_[dbindex] & bitmask) != 0;
		}

		inline void
		set_label(uint32_t x, unsigned int y, bool label)
		{
			this->set_label(y*padded_width_+x, label);
		}

		inline void 
		set_label(uint32_t padded_id, bool label)
		{
			warthog::dbword bitmask =
				(1 << (padded_id & warthog::DBWORD_BITS_MASK));
			uint32_t dbindex = padded_id >> warthog::LOG2_DBWORD_BITS;
			if(dbindex > max_id_) { return; }

			if(label)
			{
				db_[dbindex] |= bitmask;
			}
			else
			{
				db_[dbindex] &= ~bitmask;
			}
		}

		inline uint32_t 
		height() const
		{ 
			return this->padded_height_;
		} 

		inline uint32_t 
		width() const 
		{ 
			return this->padded_width_;
		}

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
		warthog::gm_header header_;
		warthog::dbword* db_;
		uint32_t dbwidth_;
		uint32_t dbheight_;
		uint32_t db_size_;
		uint32_t padded_width_;
		uint32_t padded_height_;
		uint32_t padding_;
		uint32_t max_id_;

		gridmap(const warthog::gridmap& other) {}
		gridmap& operator=(const warthog::gridmap& other) { return *this; }
		void init_db();
};

}

#endif

