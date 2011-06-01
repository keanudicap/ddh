#include "JPAExpansionPolicy.h"

#include "graph.h"
#include "Heuristic.h"
#include "mapAbstraction.h"
#include "ProblemInstance.h"

#include <cstdlib>
#include <limits.h>

JPAExpansionPolicy::JPAExpansionPolicy()
	: ExpansionPolicy()
{
	lastcost = 0; // cost from target to last node returned by ::n
}


JPAExpansionPolicy::~JPAExpansionPolicy()
{
}

node* 
JPAExpansionPolicy::first()
{
	computeNeighbourSet();
	if(hasNext()) 
	{
		return n();
	}
	return 0;
}

node* 
JPAExpansionPolicy::next()
{
	node* nextnode = 0;

	neighbours.pop_back();
	if(hasNext())
	{
		nextnode = n();
	}

	return nextnode;
}

// Returns the current neighbour of the target node being expanded.
// Usually the neighbour is a destination for one of the outgoing edges.
// However, there is one important exception:
// if the edge (target, neighbour) is a transition which involves jumping
// over the row or column of the goal node we return instead the node
// on the row or column of the goal node. 
//
// This procedure is necssary to ensure we do not miss the goal node
// while searching on a map of type JumpPointAbstraction.
node*
JPAExpansionPolicy::n()
{
	int offset = calculateEdgeIndex(neighbours.back());
	edge_iterator iter = target->getOutgoingEdgeIter() + offset;
	edge* e = target->edgeIterNextOutgoing(iter);

	graph* g = problem->getMap()->getAbstractGraph(0);
	node* n = g->getNode(e->getTo());

	int tx = target->getLabelL(kFirstData);
	int ty = target->getLabelL(kFirstData+1);

	int nx = n->getLabelL(kFirstData);
	int ny = n->getLabelL(kFirstData+1);

	int gx = problem->getGoalNode()->getLabelL(kFirstData);
	int gy = problem->getGoalNode()->getLabelL(kFirstData+1);

	int deltay = abs(gy - ty);
	int deltax = abs(gx - tx);

	// the edge (target, n) crosses both row and column of the goal node
	if(((tx < gx && gx <= nx) || (tx > gx && gx >= nx)) && 
			((ty < gy && gy <= ny) || (ty > gy && gy >= ny)))
	{
		// pick from the two possible n nodes the one closer to g
		if(deltax < deltay) 
		{
			nx = gx;
			ny = (ny > ty)?(ty + deltax):(ty - deltax);
		}
		else
		{
			ny = gy;
			nx = (nx > tx)?(tx + deltay):(tx - deltay);
		}
	}
	// the edge (target, n) crosses only the column of the goal node
	else if((tx < gx && gx <= nx) || (tx > gx && gx >= nx))
	{
		nx = gx;
		// (target, n) is a diagonal transition 
		if(ty != ny)
			ny = (ny > ty)?(ty + deltax):(ty - deltax);
	}
	// the edge (target, n) crosses only the row of the goal node
	else if((ty < gy && gy <= ny) || (ty > gy && gy >= ny))
	{
		ny = gy;

		// (target, n) is a diagonal transition 
		if(tx != nx)
			nx = (nx > tx)?(tx + deltay):(tx - deltay);
	}

	n = problem->getMap()->getNodeFromMap(nx, ny);
	lastcost = problem->getHeuristic()->h(target, n);
	return n;
}

double
JPAExpansionPolicy::cost_to_n()
{
	return lastcost;
}

bool
JPAExpansionPolicy::hasNext()
{
	if(neighbours.size() > 0)
	{
		int offset = calculateEdgeIndex(neighbours.back());
		edge_iterator iter = target->getOutgoingEdgeIter() + offset;
		edge* e = target->edgeIterNextOutgoing(iter);
		if(e->getFrom() == e->getTo())
		{
			neighbours.pop_back();
			return hasNext();
		}

		return true;
	}
	return false;
}

Jump::Direction 
JPAExpansionPolicy::directionToParent()
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

	throw std::logic_error("JPAExpansionPolicy::directionToParent"
			" failed to determine direction to parent!");
}

// given a direction for a neighbour, return the corresponding index
// for the edge to that neighbour in the array of edges stored by the 
// target node.
int
JPAExpansionPolicy::calculateEdgeIndex(Jump::Direction dir)
{
	int idx = 0;
	switch(dir)
	{
		case Jump::NONE:
			std::cerr << "JPAExpansionPolicy::n neighbour in direction Jump::NONE?!";
			exit(1);
		case Jump::N:
			break;
		case Jump::NE:
			idx++;
			break;
		case Jump::E:
			idx+=2;
			break;
		case Jump::SE:
			idx+=3;
			break;
		case Jump::S:
			idx+=4;
			break;
		case Jump::SW:
			idx+=5;
			break;
		case Jump::W:
			idx+=6;
			break;
		case Jump::NW:
			idx+=7;
			break;
	}
	return idx;
}

void 
JPAExpansionPolicy::computeNeighbourSet()
{
	mapAbstraction* map = problem->getMap();
	int x = target->getLabelL(kFirstData);
	int y = target->getLabelL(kFirstData+1);

	Jump::Direction which = directionToParent();
	switch(which)
	{
		case Jump::N:
		{
			neighbours.push_back(Jump::S);

			// check if we also need to add diagonal neighbours
			if(!map->getNodeFromMap(x+1, y))
			{
				if(map->getNodeFromMap(x+1, y+1))
					neighbours.push_back(Jump::SE);
			}
			if(!map->getNodeFromMap(x-1, y))
			{
				if(map->getNodeFromMap(x-1, y+1))
					neighbours.push_back(Jump::SW);
			}
			break;
		}

		case Jump::NE:
		{
			neighbours.push_back(Jump::SW);
			neighbours.push_back(Jump::S);
			neighbours.push_back(Jump::W);

			// add NW neighbour only if N neighbour is null
			if(!map->getNodeFromMap(x, y-1))
			{
				if(map->getNodeFromMap(x-1, y-1))
					neighbours.push_back(Jump::NW);
			}

			// add SE neighbour only if E neighbour is null
			if(!map->getNodeFromMap(x+1, y))
			{
				if(map->getNodeFromMap(x+1, y+1)) 
					neighbours.push_back(Jump::SE);
			}
			break;
		}

		case Jump::E:
		{
			neighbours.push_back(Jump::W);

			// check if we also need to add diagonal neighbours
			if(!map->getNodeFromMap(x, y-1))
			{
				if(map->getNodeFromMap(x-1, y-1))
					neighbours.push_back(Jump::NW);
			}
			if(!map->getNodeFromMap(x, y+1))
			{
				if(map->getNodeFromMap(x-1, y+1))
					neighbours.push_back(Jump::SW);
			}
			break;
		}

		case Jump::SE:
		{
			neighbours.push_back(Jump::NW);
			neighbours.push_back(Jump::W);
			neighbours.push_back(Jump::N);

			// add SW neighbour only if S neighbour is null
			if(!map->getNodeFromMap(x, y+1))
			{
				if(map->getNodeFromMap(x-1, y+1))
					neighbours.push_back(Jump::SW);
			}

			// add NE neighbour only if E neighbour is null
			if(!map->getNodeFromMap(x+1, y))
			{
				if(map->getNodeFromMap(x+1, y-1))
					neighbours.push_back(Jump::NE);
			}
			break;
		}

		case Jump::S:
		{
			neighbours.push_back(Jump::N);

			// check if we also need to add diagonal neighbours
			if(!map->getNodeFromMap(x+1, y))
			{
				if(map->getNodeFromMap(x+1, y-1))
					neighbours.push_back(Jump::NE);
			}
			if(!map->getNodeFromMap(x-1, y))
			{
				if(map->getNodeFromMap(x-1, y-1))
					neighbours.push_back(Jump::NW);
			}
			break;
		}

		case Jump::SW:
		{
			neighbours.push_back(Jump::NE);
			neighbours.push_back(Jump::N);
			neighbours.push_back(Jump::E);
		
			// add SE neighbour only if S neighbour is null
			if(!map->getNodeFromMap(x, y+1))
			{
				if(map->getNodeFromMap(x+1, y+1))
					neighbours.push_back(Jump::SE);
			}

			// add NW neighbour only if W neighbour is null
			if(!map->getNodeFromMap(x-1, y))
			{
				if(map->getNodeFromMap(x-1, y-1))
					neighbours.push_back(Jump::NW);
			}
			break;
		}

		case Jump::W:
		{

			neighbours.push_back(Jump::E);

			// check if we also need to add diagonal neighbours
			if(!map->getNodeFromMap(x, y-1))
			{
				if(map->getNodeFromMap(x+1, y-1))
					neighbours.push_back(Jump::NE);
			}
			if(!map->getNodeFromMap(x, y+1))
			{
				if(map->getNodeFromMap(x+1, y+1))
					neighbours.push_back(Jump::SE);
			}
			break;
		}

		case Jump::NW:
		{
			neighbours.push_back(Jump::SE);
			neighbours.push_back(Jump::E);
			neighbours.push_back(Jump::S);

			// add NE neighbour only if N neighbour is null
			if(!map->getNodeFromMap(x, y-1))
			{
				if(map->getNodeFromMap(x+1, y-1))
					neighbours.push_back(Jump::NE);
			}

			// add SW neighbour only if W neighbour is null
			if(!map->getNodeFromMap(x-1, y))
			{
				if(map->getNodeFromMap(x-1, y+1))
					neighbours.push_back(Jump::SW);
			}
			break;
		}
		case Jump::NONE:
		{
			neighbours.push_back(Jump::N);
			neighbours.push_back(Jump::S);
			neighbours.push_back(Jump::E);
			neighbours.push_back(Jump::W);
			neighbours.push_back(Jump::NE);
			neighbours.push_back(Jump::NW);
			neighbours.push_back(Jump::SE);
			neighbours.push_back(Jump::SW);
			break;
		}
	}
}

