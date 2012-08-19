#ifndef WARTHOG_BIT_TABLE_H
#define WARTHOG_BIT_TABLE_H

#include "gridmap.h"

// bit_table.h
//
// A space efficient closed list for explicit graphs.
// One bit is kept for every node in the graph. 
// Initially false, setting a bit to true indicates the
// corresponding node has been expanded.
//
// @author: dharabor
// 19/08/2012
//

namespace warthog
{

class bit_table
{
	public:
		bit_table(int size) 
		{
			unsigned int w = ceil(sqrt(size));
			table_ = new gridmap(w, w, true);
		}
		~bit_table()
		{
			delete table_;
		}

		inline void 
		set_mark(unsigned int id, bool value)
		{
			unsigned int x, y;	
			x = id & (warthog::DBWORD_BITS-1);
			y = id / table_->width();

			// don't trust value to be only 1 or 0
			table_->set_label(x, y, value?true:false);
		}

		inline bool
		get_mark(unsigned int id)
		{
			unsigned int x, y;	
			x = id & (warthog::DBWORD_BITS-1);
			y = id / table_->width() ;
			return table_->get_label(x, y);
		}

		void
		clear()
		{
			for(unsigned int x = 0; x < table_->width(); x++)
			{
				for(unsigned int y=0; y < table_->height(); y++)
				{
					table_->set_label(x, y, false);
				}
			}
		}

	private:
		gridmap* table_;
};

}

#endif

