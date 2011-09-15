#include "JumpPointsExpansionPolicy.h"

#include "JumpPointLocator.h"
#include "fpUtil.h"
#include "Heuristic.h"
#include "mapAbstraction.h"
#include "ProblemInstance.h"

#include <limits.h>

JumpPointsExpansionPolicy::JumpPointsExpansionPolicy(JumpPointLocator* _jpl)
	: ExpansionPolicy()
{
	neighbourIndex = 0;
	jpl = _jpl; 
	maxdepth = 1;
}

JumpPointsExpansionPolicy::~JumpPointsExpansionPolicy()
{
	neighbours.clear();
	delete jpl;
}


void 
JumpPointsExpansionPolicy::expand(node* t) throw(std::logic_error)
{
	ExpansionPolicy::expand(t);


	neighbours.clear();
	computeNeighbourSet();

	neighbourIndex = 0;
}

// Computes the set of jump point neighbours for the target node which is being
// expanded. If the target node has only a single successor, we do not generate
// it. Instead, we immediately jump again in a recursive fashion, each time
// stepping in the direction of the single successor until we identify a jump
// point with a branching factor >= 2. When we find such a jump point, we
// generate it as a successor of the target node.
// 
// NB: it is possible that after some number of recursive steps we eventually
// reach a node that has zero jump point successors. In this case we conclude
// that the target node is sterile -- i.e. it cannot appear on the optimal path
// to the goal.
void 
JumpPointsExpansionPolicy::computeNeighbourSet()
{
	int x = target->getLabelL(kFirstData);
	int y = target->getLabelL(kFirstData+1);
	node* parent = target->backpointer;	
	int px = x;
	int py = y;
	if(parent)
	{
		px = parent->getLabelL(kFirstData);
		py = parent->getLabelL(kFirstData+1);
	}

	Jump::Direction which = computeDirection(px, py, x, y);
	computeNeighbours(which, x, y);

	if(neighbours.size() == 1)
	{
		int goalx = problem->getGoalNode()->getLabelL(kFirstData);
		int goaly = problem->getGoalNode()->getLabelL(kFirstData+1);

		// if (x, y) only has one successor, generate it immediately and keep
		// recursing this way until we either reach a dead-end or find another 
		// successor with a branching factor > 1.
		while(neighbours.size() == 1)
		{
			px = x;
			py = y;
			node* n = neighbours.back();
			x = n->getLabelL(kFirstData);
			y = n->getLabelL(kFirstData+1);
			n->backpointer = problem->getMap()->getNodeFromMap(px, py);

			// terminate recursion early if we've jumped to the goal. 
			if(x == goalx && y == goaly)
				break;
			else
				neighbours.pop_back();

			which = computeDirection(px, py, x, y);
			computeNeighbours(which, x, y);
		}

		// found an alternative successor. add it to the list of neighbours for
		// the target node being expanded.
		// NB: when we expand this node, we'll have to rediscover its jump point
		// successors. Which is kinda shitty. Can we optimise this?
		if(neighbours.size() > 1)
		{
			neighbours.clear();
			neighbours.push_back(
					problem->getMap()->getNodeFromMap(x, y));
		}
	}
}

// Generates the set of jump point successors for node (x, y).
// When this method terminates, the vector this->neighbours contains pointers to
// every jump point reachable from (x, y).
//
// @param x: x-coordinate of the node from which we look for jump points.
// @param y: y-coordinate of the node from which we look for jump points.
// @param which: the direction in which we step, looking for jump points. 
void 
JumpPointsExpansionPolicy::computeNeighbours(Jump::Direction which, 
		int x, int y)
{
	mapAbstraction* map = problem->getMap();
	switch(which)
	{
		case Jump::S:
		{
			node* n = findJumpNode(Jump::S, x, y);
			if(n)
				neighbours.push_back(n);

			// add SE neighbour only if E neighbour is null 
			if(!map->getNodeFromMap(x+1, y))
			{
				n = findJumpNode(Jump::SE, x, y); 
				if(n)
					neighbours.push_back(n);
			}
			// add SW neighbour only if W neighbour is null
			if(!map->getNodeFromMap(x-1, y))
			{
				n = findJumpNode(Jump::SW, x, y); 
				if(n)
					neighbours.push_back(n);
			}
			break;
		}

		case Jump::SW:
		{
			node* n = findJumpNode(Jump::SW, x, y);
			if(n)
				neighbours.push_back(n);

			n = findJumpNode(Jump::S, x, y);
			if(n)
				neighbours.push_back(n);

			n = findJumpNode(Jump::W, x, y);
			if(n)
				neighbours.push_back(n);

			// add NW neighbour only if N neighbour is null
			if(!map->getNodeFromMap(x, y-1))
			{
				n = findJumpNode(Jump::NW, x, y); 
				if(n)
					neighbours.push_back(n);
			}

			// add SE neighbour only if E neighbour is null
			if(!map->getNodeFromMap(x+1, y))
			{
				n = findJumpNode(Jump::SE, x, y);
				if(n)
					neighbours.push_back(n);
			}
			break;
		}

		case Jump::W:
		{
			node* n = findJumpNode(Jump::W, x, y);
			if(n)
				neighbours.push_back(n);

		 	// add NW neighbour only if N neighbour is null	
			if(!map->getNodeFromMap(x, y-1))
			{
				n = findJumpNode(Jump::NW, x, y);
				if(n)
					neighbours.push_back(n);
			}
			// add SW neighbour only if S neighbour is null
			if(!map->getNodeFromMap(x, y+1))
			{
				n = findJumpNode(Jump::SW, x, y); 
				if(n)
					neighbours.push_back(n);
			}
			break;
		}

		case Jump::NW:
		{
			node* n = findJumpNode(Jump::NW, x, y); 
			if(n)
				neighbours.push_back(n);

			n = findJumpNode(Jump::N, x, y);
			if(n)
				neighbours.push_back(n);

			n = findJumpNode(Jump::W, x, y);
			if(n)
				neighbours.push_back(n);

			// add SW neighbour only if S neighbour is null
			if(!map->getNodeFromMap(x, y+1))
			{
				n = findJumpNode(Jump::SW, x, y); 
				if(n)
					neighbours.push_back(n);
			}

			// add NE neighbour only if E neighbour is null
			if(!map->getNodeFromMap(x+1, y))
			{
				n = findJumpNode(Jump::NE, x, y); 
				if(n)
					neighbours.push_back(n);
			}
			break;
		}

		case Jump::N:
		{
			node* n = findJumpNode(Jump::N, x, y);
			if(n)
				neighbours.push_back(n);

			// add NE neighbour only if E neighbour is null 
			if(!map->getNodeFromMap(x+1, y))
			{
				n = findJumpNode(Jump::NE, x, y); 
				if(n)
					neighbours.push_back(n);
			}
			// add NW neighbour only if W neighbour is null
			if(!map->getNodeFromMap(x-1, y))
			{
				n = findJumpNode(Jump::NW, x, y);
				if(n)
					neighbours.push_back(n);
			}
			break;
		}

		case Jump::NE:
		{
			node* n = findJumpNode(Jump::NE, x, y); 
			if(n)
				neighbours.push_back(n);

			n = findJumpNode(Jump::N, x, y);
			if(n)
				neighbours.push_back(n);

			n = findJumpNode(Jump::E, x, y);
			if(n)
				neighbours.push_back(n);

			// add SE neighbour only if S neighbour is null
			if(!map->getNodeFromMap(x, y+1))
			{
				n = findJumpNode(Jump::SE, x, y);
				if(n)
					neighbours.push_back(n);
			}

			// add NW neighbour only if W neighbour is null
			if(!map->getNodeFromMap(x-1, y))
			{
				n = findJumpNode(Jump::NW, x, y);
				if(n)
					neighbours.push_back(n);
			}
			break;
		}

		case Jump::E:
		{
			node* n = findJumpNode(Jump::E, x, y);
			if(n)
				neighbours.push_back(n);

		 	// add NE neighbour only if N neighbour is null 
			if(!map->getNodeFromMap(x, y-1))
			{
				n = findJumpNode(Jump::NE, x, y);
				if(n)
					neighbours.push_back(n);
			}
			// add SE neighbour only if S neighbour is null
			if(!map->getNodeFromMap(x, y+1))
			{
				n = findJumpNode(Jump::SE, x, y);
				if(n)
					neighbours.push_back(n);
			}
			break;
		}

		case Jump::SE:
		{
			node* n = findJumpNode(Jump::SE, x, y);
			if(n)
				neighbours.push_back(n);

			n = findJumpNode(Jump::S, x, y);
			if(n)
				neighbours.push_back(n);

			n = findJumpNode(Jump::E, x, y);
			if(n)
				neighbours.push_back(n);

			// add NE neighbour only if N neighbour is null
			if(!map->getNodeFromMap(x, y-1))
			{
				n = findJumpNode(Jump::NE, x, y); 
				if(n)
					neighbours.push_back(n);
			}

			// add SW neighbour only if W neighbour is null
			if(!map->getNodeFromMap(x-1, y))
			{
				n = findJumpNode(Jump::SW, x, y);
				if(n)
					neighbours.push_back(n);
			}
			break;
		}
		case Jump::NONE:
		{
			// when a node has no parent (usually only the start node)
			// we generate jump point successors in every traversable direction
			node* n = findJumpNode(Jump::N, x, y);
			if(n)
				neighbours.push_back(n);

			n = findJumpNode(Jump::S, x, y);
			if(n)
				neighbours.push_back(n);

			n = findJumpNode(Jump::E, x, y);
			if(n)
				neighbours.push_back(n);

			n = findJumpNode(Jump::W, x, y);
			if(n)
				neighbours.push_back(n);

			n = findJumpNode(Jump::NE, x, y);
			if(n)
				neighbours.push_back(n);

			n = findJumpNode(Jump::NW, x, y);
			if(n)
				neighbours.push_back(n);

			n = findJumpNode(Jump::SE, x, y);
			if(n)
				neighbours.push_back(n);

			n = findJumpNode(Jump::SW, x, y);
			if(n)
				neighbours.push_back(n);

		}
	}
}

node* 
JumpPointsExpansionPolicy::first()
{
	if(neighbours.size() > 0)
		return neighbours.at(0);
	return 0;
}

node* 
JumpPointsExpansionPolicy::next()
{
	node* nextnode = 0;

	if(hasNext())
	{
		neighbourIndex++;
		nextnode = n();
	}

	return nextnode;
}

node* 
JumpPointsExpansionPolicy::n()
{
	node* retVal = 0;
	unsigned int numNeighbours = neighbours.size();
	if(numNeighbours > 0 && neighbourIndex < numNeighbours)
	{
		retVal = neighbours.at(neighbourIndex);
	}
	return retVal;
}

double 
JumpPointsExpansionPolicy::cost_to_n()
{
	node* current = n();
	return problem->getHeuristic()->h(target, current);
}

bool 
JumpPointsExpansionPolicy::hasNext()
{
	if(neighbourIndex+1 < neighbours.size())
		return true;
	return false;
}


// Compute the direction of travel required to reach a node (x, y) if we begin
// jumping from a previous node (px, py).
// NB: (x, y) needs to be a jump point successor of (px, py) for this method to
// work. When this is not the case, the returned direction is meaningless.
//
// @return: the direction from (x, y) to (px, py).
Jump::Direction 
JumpPointsExpansionPolicy::computeDirection(int px, int py, int x, int y)
{
	if(x == px && y == py)
		return Jump::NONE;
	
	if(py == y)
	{
		if(px > x)
			return Jump::W;
		else
			return Jump::E;
	}

	if(py < y)
	{
		if(px < x)
			return Jump::SE;
		else if(px > x)
			return Jump::SW;
		else
			return Jump::S;
	}

	if(py > y)
	{
		if(px < x)
			return Jump::NE;
		else if(px > x)
			return Jump::NW;
		else
			return Jump::N;
	}

	throw std::logic_error("JumpPointsExpansionPolicy::computeDirection"
			" failed to determine direction to parent!");
}

// Finds the nearest jump node neighbour (if any) for the target node
// in a given direction.
//
// @return: a jump node (null  if none is found)
node*
JumpPointsExpansionPolicy::findJumpNode(Jump::Direction d, int x, int y)
{
	int goalx = problem->getGoalNode()->getLabelL(kFirstData);
	int goaly = problem->getGoalNode()->getLabelL(kFirstData+1);
	return jpl->findJumpNode(d, x, y, goalx, goaly);
}

