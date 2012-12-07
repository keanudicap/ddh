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
				this->set_label(to_padded_id(i), 0); 
				assert(this->get_label(to_padded_id(i)) == 0);
				break;
			default: // everything else is traversable
				if(uniform_)
				{
					this->set_label(to_padded_id(i), 1); 
					assert(this->get_label(to_padded_id(i)) == 1);
				}
				else
				{
					this->set_label(to_padded_id(i), c);
					assert(this->get_label(to_padded_id(i)) == c);
				}
				break;
		}
	}
}

void
warthog::gridmap::init_db()
{
	// when storing the grid we pad the edges of the map with
	// zeroes. this eliminates the need for bounds checking when
	// fetching the neighbours of a node. 
	this->padded_height_ = this->header_.height_ + 2;
	this->padded_width_ = this->header_.width_ + 2;

	// calc how many dbwords are needed for a single grid row.
	// in the weighted-cost case, each node requires one dbword;
	// in the uniform-cost case, each node requires one bit.
	if(uniform_)
	{
		this->dbheight_ = this->padded_height_;
		this->dbwidth_  = this->padded_width_ >> warthog::LOG2_DBWORD_BITS;
		this->dbwidth_++; // round up

		// calculate # of extra/redundant padding bits required,
		// per row, to align map width with dbword size
		this->padded_width_ = (this->dbwidth_ * warthog::DBWORD_BITS);
	}
	else
	{
		dbwidth_ = this->padded_width_;
		dbheight_ = this->padded_height_;
	}
	this->padding_ = this->padded_width_ - this->header_.width_;
	this->db_size_ = this->dbwidth_ * this->dbheight_;

	// create a one dimensional dbword array to store the grid
	this->db_ = new warthog::dbword[db_size_];
	for(unsigned int i=0; i < db_size_; i++)
	{
		db_[i] = 0;
	}

	max_id_ = db_size_-1;
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
