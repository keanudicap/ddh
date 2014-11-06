#ifndef WARTHOG_WEIGHTED_GRIDMAP_RLE_H
#define WARTHOG_WEIGHTED_GRIDMAP_RLE_H

// weighted_gridmap_rle.h
//
// A weighted gridmap implementation. Individual cells are
// represented using single byte quantities. The entire map
// is stored as a run-length-encoded array.
//
// We add some padding around the map to speed up access operations:
//  - a terminator character is added to indicate end-of-row.
//  - a line of terminator characters are added before the first row.
//  - a line of terminator characters are added after the last row.
//
// @author: dharabor
// @created: 2014-09-10
// 

#include "arraylist.h"
#include "constants.h"
#include "helpers.h"
#include "gm_parser.h"
#include "rle.h"

#include <climits>
#include <stdint.h>
#include <cassert>
#include <cstring>

namespace warthog
{

class weighted_gridmap_rle
{
    typedef warthog::arraylist< warthog::rle::rle_run > rle_row;
	public:
		weighted_gridmap_rle(uint32_t height, unsigned int width);
		weighted_gridmap_rle(const char* filename);
		~weighted_gridmap_rle();
        
		// here we convert from the coordinate space of 
		// the original grid to the coordinate space of db_. 
		inline uint32_t
		to_padded_id(uint32_t node_id)
		{
			return node_id + 
				// padded rows before the actual map data starts
				padded_rows_before_first_row_*padded_width_ +
			   	// padding from each row of data before this one
				(node_id / header_.width_) * padding_per_row_;
		}

		inline uint32_t
		to_padded_id(uint32_t x, uint32_t y)
		{
			return to_padded_id(y * this->header_width() + x);
		}

		inline void
		to_unpadded_xy(uint32_t padded_id, uint32_t& x, uint32_t& y)
		{
			padded_id -= padded_rows_before_first_row_* padded_width_;
			y = padded_id / padded_width_;
			x = padded_id % padded_width_;
		}

        // read all tiles in the 3x3 square centred on @param db_id
        // @param costs: stores the terrain cost of all tiles
        // in the 3x3 square
		inline void
		get_neighbours(uint32_t db_id, warthog::dbword costs[9])
		{
            costs[0] = get_label_raw((db_id-1) - this->padded_width_)->get_run_label(); // northwest
            costs[1] = get_label_raw(db_id - this->padded_width_)->get_run_label(); // north
            costs[2] = get_label_raw((db_id+1) - this->padded_width_)->get_run_label(); // northeast

            costs[3] = get_label_raw(db_id-1)->get_run_label(); // west
            costs[4] = get_label_raw(db_id)->get_run_label();
            costs[5] = get_label_raw(db_id+1)->get_run_label(); // east

            costs[6] = get_label_raw((db_id-1) + this->padded_width_)->get_run_label(); // southwest
            costs[7] = get_label_raw(db_id + this->padded_width_)->get_run_label(); // south
            costs[8] = get_label_raw((db_id+1) + this->padded_width_)->get_run_label(); // southeast
		}

		// get the label associated with the padded coordinate pair (x, y)
		inline warthog::dbword
		get_label(uint32_t x, unsigned int y)
		{
			return this->get_label(y*padded_width_+x);
		}

		inline warthog::dbword
		get_label(uint32_t padded_id)
		{
            return get_label_raw(padded_id)->get_run_label();
        }

		inline warthog::rle::rle_run*
		get_label_raw(uint32_t padded_id)
		{
            // binary search to find the run that contains
            // @param padded_id
            uint32_t target = padded_id % padded_width_;
            rle_row* comp_row = map_->at(padded_id / padded_width_);
            assert(comp_row->at(0).get_run_index() == 0);

            uint32_t first = 0; 
            uint32_t last = comp_row->size() - 1;
            if(first == last) { return &comp_row->at(0); }

            uint32_t i = first + ((last - first) >> 1);
            while(last - first > 1)
            {
                if(target <= comp_row->at(i).get_run_index())
                {
                    last = i;
                    i = first + ((last - first) >> 1);
                }
                else
                {
                    first = i;
                    i = first + ((last - first) >> 1);
                }
            }

            if(target < comp_row->at(i+1).get_run_index())
            {
                return &comp_row->at(i);
            }
            return &comp_row->at(i+1);
		}

		// set the label associated with the padded coordinate pair (x, y)
		inline void
		set_label(uint32_t x, unsigned int y, warthog::dbword label)
		{
			this->set_label(y*padded_width_+x, label);
		}

		inline void 
		set_label(uint32_t padded_id, warthog::dbword label)
		{
            uint32_t rowid = padded_id / padded_width_;
            rle_row* oldrow = map_->at(rowid);
            warthog::rle::rle_run* first = oldrow->begin();
            warthog::rle::rle_run* current = get_label_raw(padded_id);

            if(current->get_run_label() == label) { return; }

            rle_row* newrow = new rle_row();
            if(first != current)
            {
               newrow->append(first, current);
            }

            // split the run containing padded_id
            uint32_t first_seg_len  =  padded_id - current->get_run_index();
            if(first_seg_len > 0)
            {
                newrow->push_back(warthog::rle::rle_run(current->get_run_index(), first_seg_len));
            }
           
            // insert a new run or possibly merge padded_id with the next 
            // run in the row  (depending on its label value)
            if((current+1)->get_run_label() == label)
            {
                (current+1)->update(padded_id, label);    

            }
            else
            {
                newrow->push_back(warthog::rle::rle_run(padded_id, label));
                uint32_t last_seg_len = (current+1)->get_run_index() - (padded_id + 1);
                if(last_seg_len > 0)
                {
                    newrow->push_back(warthog::rle::rle_run(padded_id+1, last_seg_len));
                }
            }
            map_->at(rowid) = newrow;
            delete oldrow;
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

		inline uint32_t 
		header_height()
		{
			return this->header_.height_;
		}

		inline uint32_t 
		header_width()
		{
			return this->header_.width_;
		}

		inline const char*
		filename()
		{
			return this->filename_;
		}

		void 
		print(std::ostream&);
		
		uint32_t 
		mem()
		{
			uint32_t sz = sizeof(*this) + map_->size() * sizeof(rle_row*);
            for(uint32_t i = 0; i < map_->size(); i++)
            {
                sz += map_->at(i)->size() * sizeof(warthog::rle::rle_run);
            }
            return sz;
		}


	private:
		char filename_[256];
		warthog::gm_header header_;
        warthog::arraylist< rle_row* >* map_;

		uint32_t padded_width_;
		uint32_t padded_height_;
		uint32_t padding_per_row_;
		uint32_t padded_rows_before_first_row_;
		uint32_t padded_rows_after_last_row_;
        bool padded_width_is_odd_;

		weighted_gridmap_rle(const warthog::weighted_gridmap_rle& other) {}
		weighted_gridmap_rle& 
        operator=(const warthog::weighted_gridmap_rle& other) { return *this; }
		void init_db();
};

}

#endif

