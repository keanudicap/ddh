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

void 
JumpPointsExpansionPolicy::computeNeighbourSet()
{
	mapAbstraction* map = problem->getMap();
	int x = target->getLabelL(kFirstData);
	int y = target->getLabelL(kFirstData+1);

	Jump::Direction which = directionToParent();
	switch(which)
	{
		case Jump::N:
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

		case Jump::NE:
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

		case Jump::E:
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

		case Jump::SE:
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

		case Jump::S:
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

		case Jump::SW:
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

		case Jump::W:
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

		case Jump::NW:
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


// direction from the target (=node being expanded) to its parent
Jump::Direction 
JumpPointsExpansionPolicy::directionToParent()
{
	node* parent = target->backpointer;	
	if(!parent)
		return Jump::NONE;

	int x = target->getLabelL(kFirstData);
	int y = target->getLabelL(kFirstData+1);
	int px = parent->getLabelL(kFirstData);
	int py = parent->getLabelL(kFirstData+1);
	
	if(py == y)
	{
		if(px > x)
			return Jump::E;
		else
			return Jump::W;
	}

	if(py < y)
	{
		if(px < x)
			return Jump::NW;
		else if(px > x)
			return Jump::NE;
		else
			return Jump::N;

	}

	if(py > y)
	{
		if(px < x)
			return Jump::SW;
		else if(px > x)
			return Jump::SE;
		else
			return Jump::S;
	}

	throw std::logic_error("JumpPointsExpansionPolicy::directionToParent"
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

