#include "constants.h"
#include "jps.h"



// computes the forced neighbours of a node.
// for a neighbour to be forced we must check that 
// (a) the alt path from the parent is blocked and
// (b) the neighbour is not an obstacle.
// if the test succeeds, we set a bit to indicate 
// the direction from the current node (tiles[4])
// to the forced neighbour.
//
// @return an integer value whose lower 8 bits indicate
// the directions of forced neighbours
uint32_t
warthog::jps::compute_forced(warthog::jps::direction d, char tiles[9])
{
	// NB: to avoid branching statements, shift constants are
	// used below to match the result of the two tests with
	// the bit-values of the forced directions.
	uint32_t ret = 0;
	switch(d)
	{
		case warthog::jps::NORTH:
			ret |= (((!tiles[6] && tiles[3]) << 3) & warthog::jps::WEST);
			ret |= (((!tiles[6] && tiles[0]) << 5) & warthog::jps::NORTHWEST);
			ret |= (((!tiles[8] && tiles[5]) << 2) & warthog::jps::EAST);
			ret |= (((!tiles[8] && tiles[2]) << 4) & warthog::jps::NORTHEAST);
			break;
		case warthog::jps::SOUTH:
			ret |= (((!tiles[0] && tiles[3]) << 3) & warthog::jps::WEST);
			ret |= (((!tiles[0] && tiles[6]) << 7) & warthog::jps::SOUTHWEST);
			ret |= (((!tiles[2] && tiles[5]) << 2) & warthog::jps::EAST);
			ret |= (((!tiles[2] && tiles[8]) << 6) & warthog::jps::SOUTHEAST);
			break;
		case warthog::jps::EAST:
			ret |= (((!tiles[0] && tiles[1]) << 0) & warthog::jps::NORTH);
			ret |= (((!tiles[0] && tiles[2]) << 4) & warthog::jps::NORTHEAST);
			ret |= (((!tiles[6] && tiles[7]) << 1) & warthog::jps::SOUTH);
			ret |= (((!tiles[6] && tiles[8]) << 6) & warthog::jps::SOUTHEAST);
			break;
		case warthog::jps::WEST:
			ret |= (((!tiles[2] && tiles[1]) << 0) & warthog::jps::NORTH);
			ret |= (((!tiles[2] && tiles[0]) << 5) & warthog::jps::NORTHWEST);
			ret |= (((!tiles[8] && tiles[7]) << 1) & warthog::jps::SOUTH);
			ret |= (((!tiles[8] && tiles[6]) << 7) & warthog::jps::SOUTHWEST);
			break;
		default:
			break;
	}
	return ret;
}

uint32_t 
warthog::jps::compute_natural(warthog::jps::direction d, char tiles[9])
{
	uint32_t ret = 0;
	switch(d)
	{
		case warthog::jps::NORTH:
			ret |= (tiles[1] << 0) & warthog::jps::NORTH;
			break;
		case warthog::jps::SOUTH:
			ret |= (tiles[7] << 1) & warthog::jps::SOUTH;
			break;
		case warthog::jps::EAST: 
			ret |= (tiles[5] << 2) & warthog::jps::EAST;
			break;
		case warthog::jps::WEST:
			ret |= (tiles[3] << 3) & warthog::jps::WEST;
			break;
		case warthog::jps::NORTHWEST:
			ret |= (tiles[1] << 0) & warthog::jps::NORTH;
			ret |= (tiles[3] << 3) & warthog::jps::WEST;
			ret |= ((tiles[1] && tiles[3] && tiles[0]) << 5) & 
				warthog::jps::NORTHWEST;
			break;
		case warthog::jps::NORTHEAST:
			ret |= (tiles[1] << 0) & warthog::jps::NORTH;
			ret |= (tiles[5] << 2) & warthog::jps::EAST;
			ret |= ((tiles[1] && tiles[5] && tiles[2]) << 4) &
				warthog::jps::NORTHEAST;
			break;
		case warthog::jps::SOUTHWEST:
			ret |= (tiles[7] << 1) & warthog::jps::SOUTH;
			ret |= (tiles[3] << 3) & warthog::jps::WEST;
			ret |= ((tiles[7] && tiles[3] && tiles[6]) << 7) &
				warthog::jps::SOUTHWEST;
			break;
		case warthog::jps::SOUTHEAST:
			ret |= (tiles[7] << 1) & warthog::jps::SOUTH;
			ret |= (tiles[5] << 2) & warthog::jps::EAST;
			ret |= ((tiles[5] && tiles[7] && tiles[8]) << 6) &
				warthog::jps::SOUTHEAST;
			break;
		default:
			ret |= (tiles[1] << 0) & warthog::jps::NORTH;
			ret |= (tiles[7] << 1) & warthog::jps::SOUTH;
			ret |= (tiles[5] << 2) & warthog::jps::EAST;
			ret |= (tiles[3] << 3) & warthog::jps::WEST;
			ret |= ((tiles[1] && tiles[3] && tiles[0]) << 5) & 
				warthog::jps::NORTHWEST;
			ret |= ((tiles[1] && tiles[5] && tiles[2]) << 4) &
				warthog::jps::NORTHEAST;
			ret |= ((tiles[7] && tiles[3] && tiles[6]) << 7) &
					warthog::jps::SOUTHWEST;
			ret |= ((tiles[5] && tiles[7] && tiles[8]) << 6) &
					warthog::jps::SOUTHEAST;
			break;
	}
	return ret;
}

