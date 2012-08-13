#ifndef WARTHOG_CONSTANTS_H
#define WARTHOG_CONSTANTS_H

#include <cmath>
#include <climits>

namespace warthog
{
	typedef unsigned char dbword;
	const int DBWORD_BITS = sizeof(warthog::dbword)*8;
	const int LOG2_DBWORD_BITS = ceil(log10(warthog::DBWORD_BITS) / log10(2));
	const double EPSILON = 0.000001;
}

#endif


