#ifndef WARTHOG_JPS_H
#define WARTHOG_JPS_H

// jps.h
//
// This file contains the namespace for common definitions
// required by the various classes that use Jump Points.
//
// @author: dharabor
// @created: 04/09/2012
//

#include "stdint.h"

namespace warthog
{

namespace jps
{

typedef enum
{
	NONE = 0,
	NORTH = 1,
	SOUTH = 2,
	EAST = 4,
	WEST = 8,
	NORTHEAST = 16,
	NORTHWEST = 32, 
	SOUTHEAST = 64,
	SOUTHWEST = 128
} direction;

// Computes the set of "forced" directions in which to search for jump points
// from a given location (x, y). 
// A neighbour is forced if it cannot be proven that there is at least one 
// alternative optimal path that does not pass through the node (x, y).
//
// @param d: the direction of travel used to reach (x, y) -- from its parent.
// @param tiles: the 3x3 square of tiles having (x, y) at its centre.
//
// @return an integer representing the set of forced directions.
// Each of the first 8 bits of the returned value, when set, correspond to a direction, 
// as defined in warthog::jps::direction
//
uint32_t
compute_forced(warthog::jps::direction d, char tiles[9]);

// Computes the set of "natural" neighbours for a given location
// (x, y).
//
// @param d: the direction of travel used to reach (x, y)
// @param tiles: the square of cells having (x, y) at its centre.
uint32_t 
compute_natural(warthog::jps::direction d, char tiles[9]);

// Computes all successors (forced \union natural) of a node (x, y)
//
// @param d: the direction of travel used to reach (x, y)
// @param tiles: the square of cells having (x, y) at its centre.
inline uint32_t
compute_successors(warthog::jps::direction d, char tiles[9])
{
	return warthog::jps::compute_forced(d, tiles) |
	   	warthog::jps::compute_natural(d, tiles);
}

}
}

#endif

