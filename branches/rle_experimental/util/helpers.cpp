#include "helpers.h"

uint64_t
warthog::helpers::integer_min(uint64_t first, uint64_t second)
{
    if(first <= second) { return first; }
    return second;
}

void
warthog::helpers::index_to_xy(unsigned int id, unsigned int mapwidth, 
        unsigned int& x, unsigned int& y)
{	
    y = id / mapwidth;
    x = id % mapwidth;
}

