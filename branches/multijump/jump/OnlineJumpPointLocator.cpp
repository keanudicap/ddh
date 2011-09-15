#include "OnlineJumpPointLocator.h"
#include "graph.h"
#include "mapAbstraction.h"

#include <climits>


OnlineJumpPointLocator::OnlineJumpPointLocator(mapAbstraction* _map) 
	: JumpPointLocator(_map)
{
	jumplimit = INT_MAX;
}

OnlineJumpPointLocator::~OnlineJumpPointLocator()
{
}

// Finds a jump point successor of node (x, y) in Direction d. There a number
// of conditions that identify a particular node in the grid as a jump point:
// 	1. The node being evaluated is the goal node.
// 	2. The node being evaluated has at least 1 forced neighbour.
// 	3. The direction of travel is diagonal and the node being evaluated has a
// 	jump point successor in a straight direction.
//
// 	The function can also terminate if an obstacle is reached before a jump
// 	point is found.
// 	 
// @param d: the direction to search in
// @param x,y: the coordinates of the node to begin the search at
// @param goalx,goaly: the coordinates of the goal node
//
// @return: a jump point successor or 0 if no such jump point exists.
node*
OnlineJumpPointLocator::findJumpNode(Jump::Direction d, int x, int y, 
		int goalx, int goaly)
{
	node* n = 0;
	int nx = x;
	int ny = y;
	for(int steps=1; steps <= jumplimit ; steps++)
	{
		// step to the next node in direction d 
		computeNextStepCoords(d, nx, ny);
		n = map->getNodeFromMap(nx, ny);

		// stop if we hit an obstacle (no jump node exists)
		if(n == 0)
			break;

		// stop if (nx, ny) is the goal
		if(nx == goalx && ny == goaly)
			break;

		// stop if (nx, ny) has a forced neighbour
		if(computeForced(d, nx, ny))
			break;
			
		// if travelling in a diagonal direction, stop if n has a jump node
		// successor reachanble by travelling vertically or horizontally
		if(d == Jump::NE)
		{
			if(findJumpNode(Jump::N, nx, ny, goalx, goaly) || 
					findJumpNode(Jump::E, nx, ny, goalx, goaly))
				break;
		}
		if(d == Jump::SE)
		{
			if(findJumpNode(Jump::S, nx, ny, goalx, goaly) || 
					findJumpNode(Jump::E, nx, ny, goalx, goaly))
				break;
		}
		if(d == Jump::NW)
		{
			if(findJumpNode(Jump::N, nx, ny, goalx, goaly) || 
					findJumpNode(Jump::W, nx, ny, goalx, goaly))
				break;
		}
		if(d == Jump::SW)
		{
			if(findJumpNode(Jump::S, nx, ny, goalx, goaly) || 
					findJumpNode(Jump::W, nx, ny, goalx, goaly))
				break;
		}
	}
	return n;
}

// Calculate the coordinates of the next node to evaluate along the way
// to identifying a jump point.
//
// @param d: the direction of travel
// @param x: in/out variable. in: the current x-coordinate; out: the
// x-coordinate of the next node to evaluate
// @param y: in/out variable. in: the current y-coordinate; out: the
// y-coordinate of the next node to evaluate
void
OnlineJumpPointLocator::computeNextStepCoords(Jump::Direction d, int& x, int& y)
{
	switch(d)
	{
		case Jump::N:
			y--;
			break;
		case Jump::S:
			y++;
			break;
		case Jump::E:
			x++;
			break;
		case Jump::W:
			x--;
			break;
		case Jump::NE:
			x++;
			y--;
			break;
		case Jump::SE:
			x++;
			y++;
			break;
		case Jump::SW:
			x--;
			y++;
			break;
		case Jump::NW:
			x--;
			y--;
			break;
		default:
			break;
	}
}
