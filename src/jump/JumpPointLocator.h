#ifndef JUMPPOINTLOCATOR_H
#define JUMPPOINTLOCATOR_H

// JumpPointLocator.h
//
// An abstract base class for identifying jump point successors of nodes
// on a grid map.
//
// Known implementations:
// 	- OnlineJumpPointLocator
// 	- OfflineJumpPointLocator
//
// @author: dharabor
// @created: 

#include "Jump.h"

class node;
class mapAbstraction;
class JumpPointLocator
{
	public:
		JumpPointLocator(mapAbstraction* _map);
		virtual ~JumpPointLocator();

		virtual node* findJumpNode(Jump::Direction d, int x, int y, 
				int goalx, int goaly) = 0;

		int computeSuccessors(Jump::Direction d, int x, int y);
		int computeNatural(Jump::Direction d, int x, int y);
		int computeForced(Jump::Direction d, int x, int y);

		inline int getLimit() { return jumplimit; }
		inline void setLimit(int lim) { jumplimit = lim; }

	protected:
		mapAbstraction* map;
		int jumplimit; // max # of steps to take before giving up (default=inf)
};

#endif

