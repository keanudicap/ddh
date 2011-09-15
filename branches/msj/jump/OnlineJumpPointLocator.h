#ifndef ONLINEJUMPPOINTLOCATOR_H
#define ONLINEJUMPPOINTLOCATOR_H

// OnlineJumpPointLocator.h
//
// A class wrapper around some code that finds, online, jump point
// successors of an arbitrary nodes in a uniform-cost grid map.
//
// For theoretical details see:
// [Harabor D. and Grastien A, 2011, 
// Online Graph Pruning Pathfinding on Grid Maps, AAAI]
//
// @author: dharabor
// @created: 04/06/2011
//

#include "JumpPointLocator.h"

class mapAbstraction;
class node;

class OnlineJumpPointLocator : public JumpPointLocator
{
	public: 
		OnlineJumpPointLocator(mapAbstraction* map);
		virtual ~OnlineJumpPointLocator();

		virtual node* findJumpNode(Jump::Direction d, int x, int y, 
				int goalx, int goaly);

	private:
		void computeNextStepCoords(Jump::Direction d, int& x, int& y);
};

#endif

