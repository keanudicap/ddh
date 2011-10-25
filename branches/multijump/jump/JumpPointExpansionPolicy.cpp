#include "JumpPointExpansionPolicy.h"

#include "JumpPointLocator.h"
#include "fpUtil.h"
#include "Heuristic.h"
#include "mapAbstraction.h"
#include "ProblemInstance.h"

#include <limits.h>

JumpPointExpansionPolicy::JumpPointExpansionPolicy(JumpPointLocator* _jpl)
	: ExpansionPolicy()
{
	neighbourIndex = 0;
	jpl = _jpl; 
}

JumpPointExpansionPolicy::~JumpPointExpansionPolicy()
{
	neighbours.clear();
	delete jpl;
}


void 
JumpPointExpansionPolicy::expand(node* t) throw(std::logic_error)
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
//
//
// Thoughts:
// 1. If only a single successor, keep jumping (same as stepping diagonally)
// 2. If forced neighbour: check if it has a jump point. 
// 		2a. If not, keep jumping in original direction.
// 		2b. Else, jump in original direction and see if there is a further jump
// 		point.
// 			2b i). If yes, stop and return the current node as a jump point.
// 			2b ii). Else, continue jumping in the direction of the forced
// 			neighbour (probably resuming from its jump point rather than
// 			rediscoverig it).
// 	3. If no forced neighbour (but # successors > 1), check straight jump point
// 	successors. 
// 		3a. If they themselves a 
//
// 	Can we avoid sticking intermediate nodes on open? Just put them on closed
// 	immediately? i.e. trying to optimise like Rabin suggests (and like BigWorld
// 	stuff?).
// 	Could we generate the successors of the jump point set neighbours? i.e.
// 	the neighbours that forced the recursion to stop?
// 	Seems so: we'd have to generate successors of nodes not-yet-expanded.
// 	Normally this is a problem: if we find a shorter path to n, we need to
// 	propagate that lower g-value to all successors generated before n was
// 	expanded. But using jump points, we can only find a shorter path by
// 	travelling in a different direction to reach n. 
// 	Is there an overlap between the neighbour sets when travelling in two
// 	directions that would require propagation? Maybe the straight neighbours
// 	when jumping diagonally?
// 	If no propagation is needed, then we can generate nodes before their parents
// 	are expanded because these guys can never be on the optimal path (their
// 	f-values must be <= and if n is on the optimal path these guys will not be.)
// 	Disadvantages include more heap ops, a larger heap and possibly exploring
// 	parts of the search space we might have ignored (maybe? check this).
// 	Main advantage is that we don't need to rediscover directions or track
// 	directions when jumping more than once.
// 	There is some overlap. Need to explicitly re-identify straight jump point
// 	neighbours when expanding a node reached by travelling diagonally.
//
// 		
//
void 
JumpPointExpansionPolicy::computeNeighbourSet()
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
JumpPointExpansionPolicy::first()
{
	if(neighbours.size() > 0)
		return neighbours.at(0);
	return 0;
}

node* 
JumpPointExpansionPolicy::next()
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
JumpPointExpansionPolicy::n()
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
JumpPointExpansionPolicy::cost_to_n()
{
	node* current = n();
	return problem->getHeuristic()->h(target, current);
}

bool 
JumpPointExpansionPolicy::hasNext()
{
	if(neighbourIndex+1 < neighbours.size())
		return true;
	return false;
}


// direction from the target (=node being expanded) to its parent
Jump::Direction 
JumpPointExpansionPolicy::directionToParent()
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

	throw std::logic_error("JumpPointExpansionPolicy::directionToParent"
			" failed to determine direction to parent!");
}

// Finds the nearest jump node neighbour (if any) for the target node
// in a given direction.
//
// @return: a jump node (null  if none is found)
node*
JumpPointExpansionPolicy::findJumpNode(Jump::Direction d, int x, int y)
{
	int goalx = problem->getGoalNode()->getLabelL(kFirstData);
	int goaly = problem->getGoalNode()->getLabelL(kFirstData+1);
	return jpl->findJumpNode(d, x, y, goalx, goaly);
}

