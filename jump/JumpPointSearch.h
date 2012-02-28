#ifndef JUMPPOINTSEARCH_H
#define JUMPPOINTSEARCH_H

// JumpPointSearch.h
//
// A convenience wrapper for constructing and managing different types of
// Jump Point Search.
//
// @author: dharabor
// @created: 28/02/2012
//

#include "searchAlgorithm.h"

class FlexibleAStar;
class Heuristic;
class mapAbstraction;
class path;
class JumpPointSearch : public searchAlgorithm
{
	public:
		JumpPointSearch(bool online, unsigned int maxdepth, 
				Heuristic* heuristic, mapAbstraction* map);
		virtual ~JumpPointSearch();

		virtual const char *getName() { return name; }
		virtual path *getPath(graphAbstraction *aMap, node *from, node *to, 
				reservationProvider *rp = 0);	
		void resetMetrics();

	private:
		void addIntermediateNodes(path* thepath);

		bool online;
		unsigned int maxdepth;
		Heuristic* heuristic;
		mapAbstraction* map;
		FlexibleAStar* astar;
		char* name;
};

#endif

