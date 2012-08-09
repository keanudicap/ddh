#include "gm_parser.h"
#include "gridmap.h"

warthog::gridmap::gridmap(const char* filename, bool uniform)
	: uniform_(uniform)
{
	warthog::gm_parser parser(filename);
	this->header_ = parser.get_header();
//	std::cout << "map (WxH): "<<this->width()<<"x"<<this->height()<<std::endl;
//	std::cout << "db (WxH): "<<this->dbwidth()<<"x"<<this->dbheight()<<std::endl;
//	std::cout << "char bits: "<<warthog::DBWORD_BITS;
//	std::cout<<"  log2(char_bits): "<<warthog::LOG2_DBWORD_BITS<<std::endl;
//	std::cout<<"  log2(char_bits): "<<warthog::LOG2_DBWORD_BITS<<std::endl;
//	std::cout << std::flush;

	// init matrix
	this->db_ = new warthog::dbword*[this->dbwidth()];
	for(unsigned int i=0; i < this->dbwidth(); i++)
	{
		this->db_[i] = new warthog::dbword[this->dbheight()];
	}

	// populate matrix
	for(unsigned int i = 0; i < parser.get_num_tiles(); i++)
	{
		char c = parser.get_tile_at(i);
		unsigned int x = i % this->width();
		unsigned int y = i / this->width();

		if(this->uniform_)
		{
			switch(c)
			{
				case 'S':
				case 'W': 
				case 'T':
				case '@':
				case 'O':
					this->set_label(x, y, false); // obstacle
					break;
				default:
					this->set_label(x, y, true); // traversable
					break;
			}
		}
		else
		{
			this->set_label(x, y, c);
		}
	}
}

warthog::gridmap::~gridmap()
{
	
	for(unsigned int i=0; i < this->dbwidth(); i++)
	{
		delete [] db_[i];
	}
	delete [] db_;
}

unsigned int 
warthog::gridmap::dbwidth()
{
	if(this->uniform_)
	{
		// +1 to compensate for truncation
		return (this->width() >> warthog::LOG2_DBWORD_BITS) + 1;
	}
	return this->width();
}

unsigned int 
warthog::gridmap::dbheight()
{
//	if(this->uniform_)
//	{
//		// +1 to compensate for truncation
//		return (this->height() >> warthog::LOG2_DBWORD_BITS) + 1;
//	}
	return this->height();
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
			if(this->uniform_)
			{
				out << (this->get_label(x, y) != 0 ? '.':'@');
			}
			else
			{
				out << this->get_label(x, y);
			}
		}
		out << std::endl;
	}	
}
