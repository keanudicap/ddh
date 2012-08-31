#ifndef WARTHOG_CONSTANTS_H
#define WARTHOG_CONSTANTS_H

// constants.h
//
// @author: dharabor
// @created: 01/08/2012
//

#include <cfloat>
#include <cmath>
#include <climits>

namespace warthog
{
	typedef unsigned char dbword;

	// gridmap constants
	static const unsigned int DBWORD_BITS = sizeof(warthog::dbword)*8;
	static const unsigned int DBWORD_BITS_MASK = (warthog::DBWORD_BITS-1);
	static const unsigned int LOG2_DBWORD_BITS = ceil(log10(warthog::DBWORD_BITS) / log10(2));

	// search and sort constants
	static const double EPSILON = 0.000001;
	static const double ONE = 1.0f;
	static const double TWO = 2.0f;
	static const double ROOT_TWO = 1.414213562f;
	static const double ONE_OVER_ROOT_TWO = 1.0/ROOT_TWO;//0.707106781f;
	//static const double INF = DBL_MAX;
	static const unsigned int INF = UINT_MAX;

	// hashing constants
	static const unsigned int FNV32_offset_basis = 2166136261;
	static const unsigned int FNV32_prime = 16777619;

}

#endif


