#include "gm_parser.h"
#include "gridmap.h"

#include <cassert>

warthog::gridmap::gridmap(unsigned int h, unsigned int w, bool uniform)
	: uniform_(uniform), header_(h, w, "octile")
{	
	this->init_db();
}

warthog::gridmap::gridmap(const char* filename, bool uniform)
	: uniform_(uniform)
{
	warthog::gm_parser parser(filename);
	this->header_ = parser.get_header();

	init_db();
	// populate matrix
	for(unsigned int i = 0; i < parser.get_num_tiles(); i++)
	{
		char c = parser.get_tile_at(i);
		switch(c)
		{
			case 'S':
			case 'W': 
			case 'T':
			case '@':
			case 'O': // these terrain types are obstacles
				this->set_label(i, 0); 
				assert(this->get_label(i) == 0);
				break;
			default: // everything else is traversable
				if(uniform_)
				{
					this->set_label(i, 1); 
					assert(this->get_label(i) == 1);
				}
				else
				{
					this->set_label(i, c);
					assert(this->get_label(i) == c);
				}
				break;
		}
	}
}

void
warthog::gridmap::init_db()
{
	// notice that db_ is larger than w*h; by padding the edges of the
	// map with zeroes we can quickly compute the neighbours of any
	// grid location without converting an id to (x, y) values; we also
	// correctly return 0 for all neighbours which are at invalid
	// locations (e.g the node left of (0, 0))
	
	dbheight_ = header_.height_ + 2;
	dbwidth_ = (uniform_ ? ((header_.width_ >> warthog::LOG2_DBWORD_BITS)+1) :
			(header_.width_+1));
	db_size_ = dbwidth_*dbheight_;

	// some padding constants
	padded_width_ = (uniform_ ? 
			(dbwidth_ * warthog::DBWORD_BITS) : dbwidth_);
	padding_ = padded_width_ - header_.width_;

	this->db_ = new warthog::dbword[db_size_];
	for(unsigned int i=0; i < db_size_; i++)
	{
		db_[i] = 0;
	}

	max_id_ = header_.height_ * header_.width_;
}

warthog::gridmap::~gridmap()
{
	delete [] db_;
}

void 
warthog::gridmap::print(std::ostream& out)
{
	out << "type "<< header_.type_ << std::endl;
	out << "height "<< header_.height_ << std::endl;
	out << "width "<< header_.width_ << std::endl;
	out << "map" << std::endl;
	for(unsigned int y=0; y < this->height(); y++)
	{
		for(unsigned int x=0; x < this->width(); x++)
		{
			char c = this->get_label(y*header_.width_+x);
			out << (c ? (uniform_ ? '.' : c) : '@');
		}
		out << std::endl;
	}	
}

void 
warthog::gridmap::printdb(std::ostream& out)
{
	char tiles[9] = {'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x'};
	tiles[0] = tiles[0];
}
