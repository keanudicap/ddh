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
		~gridmap();

		// get all the tiles adjacent to nodeid. this function begins at 
		// node (x-1, y-1) (relative to node_id) and steps through the
		// list of adjacent tiles in left-to-right, top-to-bottom order
		inline void
		get_neighbours(unsigned int node_id, char tiles_[9])
		{
			if(node_id >= max_id_) { return; }
			if(uniform_)
			{
				unsigned int offset = (node_id & warthog::DBWORD_BITS_MASK) ;

				// NB: the location of words in the original w*h
				// grid must be converted to their location in
				// the dbwidth*dbheight grid
				node_id = (node_id >> warthog::LOG2_DBWORD_BITS);
				unsigned int pos = node_id+1;
				unsigned int pos2 = node_id + dbwidth_;
				unsigned int pos3 = node_id + (dbwidth_<<1) - 1;

				if(offset == 0)
				{
					// NB: implicit bitmask: 1 << offset
					tiles_[0] = (db_[pos-1] & 128) ? 1:0;
					tiles_[1] = (db_[pos] & 1) ? 1:0;
					tiles_[2] = (db_[pos] & 2) ? 1:0;
					tiles_[3] = (db_[pos2-1] & 128) ? 1:0;
					tiles_[4] = (db_[pos2] & 1) ? 1:0;
					tiles_[5] = (db_[pos2] & 2) ? 1:0;
					tiles_[6] = (db_[pos3-1] & 128) ? 1:0;
					tiles_[7] = (db_[pos3] & 1) ? 1:0;
					tiles_[8] = (db_[pos3] & 2) ? 1:0;
					return;
				}
				if(offset == 7) 
				{
					// NB: implicit bitmask: 1 << offset
					tiles_[0] = (db_[pos] & 64) ? 1:0;
					tiles_[1] = (db_[pos] & 128) ? 1:0;
					tiles_[2] = (db_[pos+1] & 1) ? 1:0;
					tiles_[3] = (db_[pos2] & 64) ? 1:0;
					tiles_[4] = (db_[pos2] & 128) ? 1:0;
					tiles_[5] = (db_[pos2+1] & 1) ? 1:0;
					tiles_[6] = (db_[pos3] & 64) ? 1:0;
					tiles_[7] = (db_[pos3] & 128) ? 1:0;
					tiles_[8] = (db_[pos3+1] & 1) ? 1:0;
					return;
				}	

				unsigned int bitmask1 = (1 << (offset-1));
				unsigned int bitmask2 = (1 << offset);
				unsigned int bitmask3 = (1 << (offset+1));
				tiles_[0] = (db_[pos] & bitmask1) ? 1:0;
				tiles_[1] = (db_[pos] & bitmask2) ? 1:0;
				tiles_[2] = (db_[pos] & bitmask3) ? 1:0;
				tiles_[3] = (db_[pos2] & bitmask1) ? 1:0;
				tiles_[4] = (db_[pos2] & bitmask2) ? 1:0;
				tiles_[5] = (db_[pos2] & bitmask3) ? 1:0;
				tiles_[6] = (db_[pos3] & bitmask1) ? 1:0;
				tiles_[7] = (db_[pos3] & bitmask2) ? 1:0;
				tiles_[8] = (db_[pos3] & bitmask3) ? 1:0;
			}
			else
			{
				// NB: the location of words in the original w*h
				// grid must be converted to their location in
				// the dbwidth*dbheight grid
				node_id = (node_id >> warthog::LOG2_DBWORD_BITS);
				unsigned int pos = node_id+1;
				unsigned int pos2 = node_id + dbwidth_;
				unsigned int pos3 = node_id + (dbwidth_<<1)-1;
				tiles_[0] = db_[pos];
				tiles_[1] = db_[pos+1];
				tiles_[2] = db_[pos+2];
				tiles_[3] = db_[pos2];
				tiles_[4] = db_[pos2+1];
				tiles_[5] = db_[pos2+2];
				tiles_[6] = db_[pos3];
				tiles_[7] = db_[pos3+1];
				tiles_[8] = db_[pos3+2];
			}
		}

		inline warthog::dbword
		get_label(unsigned int x, unsigned int y)
		{
			if(x >= header_.width_ || y >= header_.height_)
			{
				return 0;
			}
			return this->get_label(y*header_.width_+x);
		}

		inline warthog::dbword 
		get_label(unsigned int node_id)
		{
			if(node_id >= max_id_)
			{
				return 0;
			}

			if(uniform_)
			{
				warthog::dbword bitmask = 1;
//				if(node_id == 938418)
//				{
//				std::cerr << "\ndbword_bits_mask "<<warthog::DBWORD_BITS_MASK<< 
//				"; bitmask inital: "<<(int)bitmask;
//				}
				bitmask <<= (node_id & warthog::DBWORD_BITS_MASK);
//				if(node_id == 938418)
//				{
//				std::cerr << " after & "<<(int)bitmask << "; nodeid: "<<node_id;
//				}
				node_id >>= warthog::LOG2_DBWORD_BITS;
//				if(node_id == (938418 >> warthog::LOG2_DBWORD_BITS))
//				{
//					std::cerr << " shifted id: "<<node_id;
//					std::cerr << " lookup id: "<<node_id<<
//						" raw lookup value: "<< (int)db_[node_id+dbwidth_]
//						<< "; extracted bit: "<<(int)(db_[node_id+dbwidth_] & bitmask)<<std::endl;
//					std::cerr<<std::flush;
//				}
				node_id += dbwidth_;
				return (db_[node_id] & bitmask) != 0;
			}
			return db_[node_id + dbwidth_];
		}

		inline void
		set_label(unsigned int x, unsigned int y, warthog::dbword label)
		{
			this->set_label(y*header_.width_+x, label);
		}

		inline void 
		set_label(unsigned int node_id, warthog::dbword label)
		{
			if(node_id >= max_id_)
			{
				return;
			}

			if(uniform_)
			{
				warthog::dbword bitmask =
				   	(1 << (node_id & warthog::DBWORD_BITS_MASK));
				node_id >>= warthog::LOG2_DBWORD_BITS;
				node_id += dbwidth_;
				if(label)
				{
					db_[node_id] |= bitmask;
				}
				else
				{
					db_[node_id] &= ~bitmask;
				}
			}
			else
			{
				db_[node_id + dbwidth_] = label;
			}
		}

		inline unsigned int 
		height() const
		{ 
			return this->header_.height_; 
		} 

		inline unsigned int 
		width() const 
		{ 
			return this->header_.width_;
		}

		inline unsigned int
		db_width()
		{
			return dbwidth_;
		}

		inline unsigned int
		db_height()
		{
			return dbheight_;
		}

		void 
		print(std::ostream&);

		unsigned int 
		mem()
		{
			return sizeof(*this) +
			sizeof(warthog::dbword) * dbsize_;
		}


	private:
		bool uniform_;
		warthog::gm_header header_;
		warthog::dbword* db_;
		unsigned int dbsize_;
		unsigned int dbheight_;
		unsigned int dbwidth_;
		unsigned int max_id_;

		gridmap(const warthog::gridmap& other) {}
		gridmap& operator=(const warthog::gridmap& other) { return *this; }
		void init_db();
};

}

#endif

