#ifndef WARTHOG_HELPERS_H
#define WARTHOG_HELPERS_H

// helpers.h
//
// Helper functions that don't fit anywhere else.
//
// @author: dharabor
// @created: 21/08/2012
//

#include <stdint.h>

namespace warthog
{

namespace helpers
{

// convert id into x/y coordinates on a grid of width 'mapwidth'
void
index_to_xy(unsigned int id, unsigned int mapwidth, 
		unsigned int& x, unsigned int& y);

uint64_t
integer_min(uint64_t first, uint64_t second);

}

}

#endif


