#include "jps.h"
#include "jps_wgm.h"

// The implementation here use some bitshifting to avoid
// branching. Basically: we test if the tiles corresponding 
// to natural neighbours are not null (i.e. they exist) then
// bitshift the result to obtain the corresponding direction
// of the natural neighbour. The number of shifts is log2 of 
// natural neighbour direction.
uint32_t 
warthog::jps::compute_natural_wgm(warthog::jps::direction d, 
        warthog::dbword tiles[9])
{
    uint32_t retval = 0;
    switch(d)
    {
        case warthog::jps::NORTH:
            retval |= (tiles[1] && 1);
            break;
        case warthog::jps::SOUTH:
            retval |= (tiles[7] && 1) << 1;
            break;
        case warthog::jps::EAST:
            retval |= (tiles[5] && 1) << 2;
            break;
        case warthog::jps::WEST:
            retval |= (tiles[3] && 1) << 3;
            break;
        case warthog::jps::NORTHEAST:
            retval |= (tiles[1] && 1);
            retval |= (tiles[5] && 1) << 2;
            retval |= (tiles[1] && tiles[5]) << 4;
            break;
        case warthog::jps::NORTHWEST:
            retval |= (tiles[1] && 1);
            retval |= (tiles[3] && 1) << 3;
            retval |= (tiles[1] && tiles[3]) << 5;
            break;
        case warthog::jps::SOUTHEAST:
            retval |= (tiles[7] && 1) << 1;
            retval |= (tiles[5] && 1) << 2;
            retval |= (tiles[7] && tiles[5]) << 6;
            break;
        case warthog::jps::SOUTHWEST:
            retval |= (tiles[7] && 1) << 1;
            retval |= (tiles[3] && 1) << 3;
            retval |= (tiles[7] && tiles[3]) << 7;
            break;
        default:
            break;
    }
    return retval;
}

// The implementation here uses bitshifting operations to avoid
// branching. The details are similar to ::compute_natural_wgm.
// NB: This implementation assumes corner-cutting is not allowed.
uint32_t 
warthog::jps::compute_forced_wgm(warthog::jps::direction d,
        warthog::dbword tiles[9])
{
    uint32_t retval = 0;
    switch(d)
    {
        case warthog::jps::NORTH:
            retval |= (!tiles[8] && 1) << 2; // E is forced
            retval |= (!tiles[6] && 1) << 3; // W is forced
            break;
        case warthog::jps::SOUTH:
            retval |= (!tiles[2] && 1) << 2; // E is forced
            retval |= (!tiles[0] && 1) << 3; // W is forced
            break;
        case warthog::jps::EAST:
            retval |= (!tiles[0] && 1); // N is forced
            retval |= (!tiles[6] && 1) << 1; // S is forced
            break;
        case warthog::jps::WEST:
            retval |= (!tiles[2] && 1); // N is forced
            retval |= (!tiles[8] && 1) << 1; // S is forced
            break;
        default:
            break;
    }
    return retval;
}

uint32_t 
warthog::jps::compute_forced_wgm_terrain(warthog::jps::direction d,
        warthog::dbword tiles[9])
{
    uint32_t retval = 0;

    retval |= (tiles[1] == tiles[4]); // N is forced
    retval |= (tiles[7] == tiles[4]) << 1; // S is forced
    retval |= (tiles[5] == tiles[4]) << 2; // E is forced
    retval |= (tiles[3] == tiles[4]) << 3; // W is forced

     // NE is forced 
    retval |= ((tiles[1] && tiles[5]) && (tiles[2] == tiles[4])) << 4;
    // NW is forced
    retval |= ((tiles[1] && tiles[3]) && (tiles[0] == tiles[4])) << 5;
    // SE is forced
    retval |= ((tiles[7] && tiles[5]) && (tiles[8] == tiles[4])) << 6;
    // SW is forced
    retval |= ((tiles[7] && tiles[3]) && (tiles[6] == tiles[4])) << 7;

    return retval;
}
