#include "constants.h"
#include "jps.h"

uint32_t
warthog::jps::compute_forced(warthog::jps::direction d, char tiles[9])
{
	uint32_t ret = 0;
	double cost;
	double pcost;
	switch(d)
	{
		case warthog::jps::NORTH:
			if(tiles[3])
			{
				cost = (tiles[7] + 2*tiles[4] + tiles[3]) *
				   	warthog::ONE_OVER_TWO;
				pcost = tiles[6] ?
				   	((tiles[3] + tiles[6] + tiles[7] + tiles[4]) *
					warthog::ROOT_TWO_OVER_FOUR) : warthog::INF;
				if(cost < pcost) 
				{ 
					ret |= warthog::jps::WEST; 
					ret |= warthog::jps::NORTHWEST;
				}
			}
			if(tiles[5])
			{
				cost = (tiles[7] + 2*tiles[4] + tiles[5]) *
				   	warthog::ONE_OVER_TWO;
				pcost = tiles[8] ?
				   	((tiles[5] + tiles[8] + tiles[7] + tiles[4]) *
					warthog::ROOT_TWO_OVER_FOUR) : warthog::INF;
				if(cost < pcost) 
				{ 
					ret |= warthog::jps::EAST; 
					ret |= warthog::jps::NORTHEAST;
				}
			}
			break;
		case warthog::jps::SOUTH:
			if(tiles[3])
			{
				cost = (tiles[1] + 2*tiles[4] + tiles[3]) *
				   	warthog::ONE_OVER_TWO;
				pcost = tiles[0] ?
				   	((tiles[3] + tiles[0] + tiles[1] + tiles[4]) *
					warthog::ROOT_TWO_OVER_FOUR) : warthog::INF;
				if(cost < pcost) 
				{
				   	ret |= warthog::jps::WEST; 
					ret |= warthog::jps::SOUTHWEST;
				}
			}
			if(tiles[5])
			{
				cost = (tiles[1] + 2*tiles[4] + tiles[5]) *
				   	warthog::ONE_OVER_TWO;
				pcost = tiles[2] ?
				   	((tiles[5] + tiles[2] + tiles[1] + tiles[4]) *
					warthog::ROOT_TWO_OVER_FOUR) : warthog::INF;
				if(cost < pcost) 
				{ 
					ret |= warthog::jps::EAST; 
					ret |= warthog::jps::SOUTHEAST;
				}
			}
			break;
		case warthog::jps::EAST:
			if(tiles[1])
			{
				cost = (tiles[3] + 2*tiles[4] + tiles[1]) *
				   	warthog::ONE_OVER_TWO;
				pcost = tiles[0] ?
				   	((tiles[3] + tiles[0] + tiles[1] + tiles[4]) *
					warthog::ROOT_TWO_OVER_FOUR) : warthog::INF;
				if(cost < pcost) 
				{ 
					ret |= warthog::jps::NORTH; 
					ret |= warthog::jps::NORTHEAST;
				}
			}
			if(tiles[7])
			{
				cost = (tiles[3] + 2*tiles[4] + tiles[7]) *
				   	warthog::ONE_OVER_TWO;
				pcost = tiles[6] ?
				   	((tiles[3] + tiles[6] + tiles[7] + tiles[4]) * 
					warthog::ROOT_TWO_OVER_FOUR) : warthog::INF;
				if(cost < pcost) 
				{ 
					ret |= warthog::jps::SOUTH; 
					ret |= warthog::jps::SOUTHEAST;
				}

			}
			break;
		case warthog::jps::WEST:
			if(tiles[1])
			{
				cost = (tiles[5] + 2*tiles[4] + tiles[1]) *
				   	warthog::ONE_OVER_TWO;
				pcost = tiles[2] ?
				   	((tiles[5] + tiles[2] + tiles[1] + tiles[4]) *
					warthog::ROOT_TWO_OVER_FOUR) : warthog::INF;
				if(cost < pcost) 
				{
				   	ret |= warthog::jps::NORTH; 
					ret |= warthog::jps::NORTHWEST;
				}
			}
			if(tiles[7])
			{
				cost = (tiles[5] + 2*tiles[4] + tiles[7]) *
				   	warthog::ONE_OVER_TWO;
				pcost = tiles[8] ?
				   	((tiles[5] + tiles[8] + tiles[7] + tiles[4]) * 
					warthog::ROOT_TWO_OVER_FOUR) : warthog::INF;
				if(cost < pcost) 
				{ 
					ret |= warthog::jps::SOUTH; 
					ret |= warthog::jps::SOUTHWEST;
				}
			}
			break;
		default:
			break;
	}
	return ret;
}

uint32_t 
warthog::jps::compute_natural(warthog::jps::direction d, char tiles[9])
{
	uint32_t ret = d;
	switch(d)
	{
		case warthog::jps::NORTHWEST:
			ret |= warthog::jps::NORTH;
			ret |= warthog::jps::WEST;
			break;
		case warthog::jps::NORTHEAST:
			ret |= warthog::jps::NORTH;
			ret |= warthog::jps::WEST;
			break;
		case warthog::jps::SOUTHWEST:
			ret |= warthog::jps::SOUTH;
			ret |= warthog::jps::WEST;
			break;
		case warthog::jps::SOUTHEAST:
			ret |= warthog::jps::SOUTH;
			ret |= warthog::jps::EAST;
			break;
		case warthog::jps::NONE:
			ret |= warthog::jps::NORTH;
			ret |= warthog::jps::SOUTH;
			ret |= warthog::jps::EAST;
			ret |= warthog::jps::WEST;
			ret |= warthog::jps::NORTHEAST;
			ret |= warthog::jps::NORTHWEST;
			ret |= warthog::jps::SOUTHEAST;
			ret |= warthog::jps::SOUTHWEST;
			break;
		default:
			ret |= d;
	}
	return ret;
}

