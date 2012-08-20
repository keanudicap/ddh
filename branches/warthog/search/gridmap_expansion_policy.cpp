#include "gridmap_expansion_policy.h"
#include "problem_instance.h"

warthog::gridmap_expansion_policy::gridmap_expansion_policy(
		std::shared_ptr<warthog::gridmap> map) : map_(std::move(map))
{
}

warthog::gridmap_expansion_policy::~gridmap_expansion_policy()
{
}

void 
warthog::gridmap_expansion_policy::expand(unsigned int nodeid,
		warthog::problem_instance* problem)
{
	cx_ = nodeid;
	cy_ = 0;
	unsigned int map_width = map_->width();
	unsigned int map_height = map_->height();

	for( ; cx_ >= map_width ; cx_ -= map_width)
	{
		cy_++; // sub faster than div
	}

	// get terrain type of each tile in the 3x3 square around (x, y)
	unsigned int index = 0;
	int foo = (int)cx_-1; 
	int bar = (int)cy_-1; 
	for(int i = foo; i <= foo+2; i++)
	{
		for(int j = bar; j <= bar+2; j++)
		{
			if(!(i < 0 || j < 0 || i >= (int)map_width || j >= (int)map_height))
			{
				tiles_[index] = map_->get_label(i, j);
			}
			else
			{
				tiles_[index] = 0;
			}
			index++;
		}
	}

	// calculate transition costs; from (x, y) to each adjacent tile
	// this value is just an average of terrain costs weighted by the 
	// transition type (straight or diagonal)
	for(int i=0; i < 9; i++)
	{
		costs_[i] = warthog::INF;
		switch(i)
		{
			case 0: // (x-1, y-1)
				if(tiles_[4] && tiles_[1] && tiles_[3] && tiles_[i])
				{
					costs_[i] = (tiles_[4] + tiles_[1] + tiles_[3] + tiles_[i]);
					costs_[i] *= 0.25 * warthog::ROOT_TWO;
				}
				else // neighbour cannot be reached; mark it as an obstacle
				{
					tiles_[i] = 0;
				}
				break;
			case 1: // (x-1, y)
				if(tiles_[4] && tiles_[i])
				{
					costs_[i] = (tiles_[4] + tiles_[i]) * 0.5;
				}
				else // neighbour cannot be reached; mark it as an obstacle
				{
					tiles_[i] = 0;
				}
				break;
			case 2: // (x-1, y+1)
				if(tiles_[4] && tiles_[1] && tiles_[5] && tiles_[i])
				{
					costs_[i] = (tiles_[4] + tiles_[1] + tiles_[5] + tiles_[i]);
					costs_[i] *= 0.25 * warthog::ROOT_TWO;
				}
				else // neighbour cannot be reached; mark it as an obstacle
				{
					tiles_[i] = 0;
				}
				break;
			case 3: // (x, y-1)
				if(tiles_[4] && tiles_[i])
				{
					costs_[i] = (tiles_[4] + tiles_[i])*0.5;
				}
				else // neighbour cannot be reached; mark it as an obstacle
				{
					tiles_[i] = 0;
				}
				break;
			case 4: // (x, y)
				costs_[i] = 0;
				break;
			case 5: //  (x, y+1)
				if(tiles_[4] && tiles_[i])
				{
					costs_[i] = (tiles_[4] + tiles_[i])*0.5;
				}
				else // neighbour cannot be reached; mark it as an obstacle
				{
					tiles_[i] = 0;
				}
				break;
			case 6: // (x+1, y-1)
				if(tiles_[4] && tiles_[7] && tiles_[3] && tiles_[i])
				{
					costs_[i] = (tiles_[4] + tiles_[7] + tiles_[3] + tiles_[i]);
					costs_[i] *= 0.25 * warthog::ROOT_TWO;
				}
				else // neighbour cannot be reached; mark it as an obstacle
				{
					tiles_[i] = 0;
				}
				break;
			case 7: // (x+1, y)
				if(tiles_[4] && tiles_[i])
				{
					costs_[i] = (tiles_[4] + tiles_[i])*0.5;
				}
				else // neighbour cannot be reached; mark it as an obstacle
				{
					tiles_[i] = 0;
				}
				break;
			case 8: // (x+1, y+1)
				if(tiles_[4] && tiles_[7] && tiles_[5] && tiles_[i])
				{
					costs_[i] = (tiles_[4] + tiles_[7] + tiles_[5] + tiles_[i]);
					costs_[i] *= 0.25 * warthog::ROOT_TWO;
				}
				else // neighbour cannot be reached; mark it as an obstacle
				{
					tiles_[i] = 0;
				}
				break;
			default:
				break;
		}
	}
	which_ = 0;
}

