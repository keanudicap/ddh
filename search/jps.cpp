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
	// NB: to avoid branching statements, shift operations are
	// used below. The values of the constants correspond to 
	// direction values.
	uint32_t ret = 0;
	switch(d)
	{
		case warthog::jps::NORTH:
			if(tiles[6] && tiles[8]) { return ret; }
			ret |= ((!tiles[6] && tiles[3]) << 3); // W
			ret |= ((!tiles[6] && tiles[0]) << 5); // NW
			ret |= ((!tiles[8] && tiles[5]) << 2); // E
			ret |= ((!tiles[8] && tiles[2]) << 4); // NE
			break;
		case warthog::jps::SOUTH:
			if(tiles[0] && tiles[2]) { return ret; }
			ret |= ((!tiles[0] && tiles[3]) << 3); // W
			ret |= ((!tiles[0] && tiles[6]) << 7); // SW
			ret |= ((!tiles[2] && tiles[5]) << 2); // E
			ret |= ((!tiles[2] && tiles[8]) << 6); // SE
			break;
		case warthog::jps::EAST:
			if(tiles[0] && tiles[6]) { return ret; }
			ret |= ((!tiles[0] && tiles[1]) << 0); // N
			ret |= ((!tiles[0] && tiles[2]) << 4); // NE
			ret |= ((!tiles[6] && tiles[7]) << 1); // S
			ret |= ((!tiles[6] && tiles[8]) << 6); // SE
			break;
		case warthog::jps::WEST:
			if(tiles[2] && tiles[8]) { return ret; }
			ret |= ((!tiles[2] && tiles[1]) << 0); // N
			ret |= ((!tiles[2] && tiles[0]) << 5); // NW
			ret |= ((!tiles[8] && tiles[7]) << 1); // S
			ret |= ((!tiles[8] && tiles[6]) << 7); // SW
			break;
		default:
			break;
	}
	return ret;
}

uint32_t 
warthog::jps::compute_natural(warthog::jps::direction d, char tiles[9])
{
	// In the shift operations below the constant values
	// correspond to bit offsets for warthog::jps::direction
	uint32_t ret = 0;
	switch(d)
	{
		case warthog::jps::NORTH:
			ret |= (tiles[1] << 0);
			break;
		case warthog::jps::SOUTH:
			ret |= (tiles[7] << 1);
			break;
		case warthog::jps::EAST: 
			ret |= (tiles[5] << 2);
			break;
		case warthog::jps::WEST:
			ret |= (tiles[3] << 3);
			break;
		case warthog::jps::NORTHWEST:
			ret |= (tiles[1] << 0);
			ret |= (tiles[3] << 3);
			ret |= ((tiles[1] && tiles[3] && tiles[0]) << 5);
			break;
		case warthog::jps::NORTHEAST:
			ret |= (tiles[1] << 0);
			ret |= (tiles[5] << 2);
			ret |= ((tiles[1] && tiles[5] && tiles[2]) << 4);
			break;
		case warthog::jps::SOUTHWEST:
			ret |= (tiles[7] << 1);
			ret |= (tiles[3] << 3);
			ret |= ((tiles[7] && tiles[3] && tiles[6]) << 7);
			break;
		case warthog::jps::SOUTHEAST:
			ret |= (tiles[7] << 1);
			ret |= (tiles[5] << 2);
			ret |= ((tiles[5] && tiles[7] && tiles[8]) << 6);
			break;
		default:
			ret |= (tiles[1] << 0);
			ret |= (tiles[7] << 1);
			ret |= (tiles[5] << 2);
			ret |= (tiles[3] << 3);
			ret |= ((tiles[1] && tiles[3] && tiles[0]) << 5);
			ret |= ((tiles[1] && tiles[5] && tiles[2]) << 4);
			ret |= ((tiles[7] && tiles[3] && tiles[6]) << 7);
			ret |= ((tiles[5] && tiles[7] && tiles[8]) << 6);
			break;
	}
	return ret;
}

