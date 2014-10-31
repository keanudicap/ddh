#include "weighted_gridmap_rle.h"

warthog::weighted_gridmap_rle::weighted_gridmap_rle(unsigned int h, unsigned int w)
	: header_(h, w, "octile")
{	
	this->init_db();
}

warthog::weighted_gridmap_rle::weighted_gridmap_rle(const char* filename)
{
	strcpy(filename_, filename);
	warthog::gm_parser parser(filename);
	this->header_ = parser.get_header();

    init_db();
    if(padded_width_ % 2) { padded_width_is_odd_ = 1; }

    map_ = new warthog::arraylist< rle_row* >(padded_height_); 

    // add some rows of padding before the real data begins
    for(uint32_t i = 0; i < this->padded_rows_before_first_row_; i++)
    {
        rle_row* comp_row = new rle_row();
        comp_row->push_back(warthog::rle::rle_run(0, 0));
        map_->push_back(comp_row);
    }

    // read in the map, row by row
    char* row_ = new char[header_.width_];
	for(unsigned int i = 0; i < parser.get_num_tiles(); i++)
            
	{
		char c = parser.get_tile_at(i);
		switch(c)
		{
            // explicit obstacle
			case '@':  
                row_[i % header_.width_] = 0;
				break;
            // other tiles have terrain cost equal to their ascii value
			default: 
                row_[i % header_.width_] = c;
				break;
		}
        
        // (i) compress each row; 
        // (ii) pad the end with an extra obstacle element;
        // (iii) add it to the compressed map
        if(((i+1) % header_.width_) == 0)
        {
            rle_row* comp_row = warthog::rle::compress(row_, header_.width_);
            comp_row->push_back(warthog::rle::rle_run(header_.width_, 0));
            map_->push_back(comp_row);
        }
	}
    delete row_;

    // add some rows of padding after the last bit of real data
    for(uint32_t i = map_->size(); i < padded_height_; i++)
    {
        rle_row* comp_row = new rle_row();
        comp_row->push_back(warthog::rle::rle_run(0, 0));
        comp_row->push_back(warthog::rle::rle_run(0, 0));
        map_->push_back(comp_row);
    }
}

void
warthog::weighted_gridmap_rle::init_db()
{
	// when storing the grid we pad the edges of the map.
	// this eliminates the need for bounds checking when
	// fetching the neighbours of a node. 
	this->padded_rows_before_first_row_ = 2;
	this->padded_rows_after_last_row_ = 2;
	this->padding_per_row_ = 1;

	this->padded_width_ = this->header_.width_ + this->padding_per_row_;
	this->padded_height_ = this->header_.height_ + 
		this->padded_rows_after_last_row_ +
		this->padded_rows_before_first_row_;
}

warthog::weighted_gridmap_rle::~weighted_gridmap_rle()
{
    for(uint32_t i = 0; i < map_->size(); i++)
    {
        delete map_->at(i);
    }
    delete map_;
}

void 
warthog::weighted_gridmap_rle::print(std::ostream& out)
{
	out << "printing padded map" << std::endl;
	out << "-------------------" << std::endl;
	out << "type "<< header_.type_ << std::endl;
	out << "height "<< this->height() << std::endl;
	out << "width "<< this->width() << std::endl;
	out << "map" << std::endl;
	for(unsigned int y=0; y < this->height(); y++)
	{
		for(unsigned int x=0; x < this->width(); x++)
		{
            warthog::dbword c = this->get_label(y*this->width()+x);
			out << (c == 0 ? '@': (char)c);
		}
		out << std::endl;
	}	
}
