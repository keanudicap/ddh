#ifndef WARTHOG_CONSTANTS_H
#define WARTHOG_CONSTANTS_H

#include <cmath>
#include <climits>

namespace warthog
{
	// gridmap constants
	typedef unsigned char dbword;
	const int DBWORD_BITS = sizeof(warthog::dbword)*8;
	const int LOG2_DBWORD_BITS = ceil(log10(warthog::DBWORD_BITS) / log10(2));

	// blockmap constants
	const unsigned int BLOCKSIZE = 256; // NB: must be a power of 2!!
	const unsigned int LOG_BLOCKSIZE = ceil(log10(BLOCKSIZE) / log10(2));

	// search and sort constants
	const double EPSILON = 0.000001;
}

#endif


