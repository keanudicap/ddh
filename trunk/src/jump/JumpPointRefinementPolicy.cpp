#include "JumpPointRefinementPolicy.h"

#include "graph.h"
#include "JumpPointsExpansionPolicy.h"
#include "mapAbstraction.h"
#include "OnlineJumpPointLocator.h"
#include "OctileDistanceRefinementPolicy.h"
#include "path.h"

#include <cstdlib>

JumpPointRefinementPolicy::JumpPointRefinementPolicy(mapAbstraction* _map,
		int _maxdepth) : RefinementPolicy(_map)
{
	this->jpl = new OnlineJumpPointLocator(_map);
	this->maxdepth = _maxdepth;
}

JumpPointRefinementPolicy::~JumpPointRefinementPolicy()
{
	delete jpl;
}

path*
JumpPointRefinementPolicy::refine(path* abspath)
{
	if(!abspath && !abspath->next)
		return 0;

	if(verbose)
	{
		std::cout << "refining abstract jump point path. maxdepth: "<<
			maxdepth<<std::endl;
	}

	Jump::Direction which = Jump::NONE;
	path* retVal = recurse(&which, abspath->n, abspath->next->n, 0);
	if(retVal == 0)
	{
		std::cerr << "couldn't refine segment. cannot continue.\n";
		exit(1);
	}
	path* tail = retVal->tail();

	abspath = abspath->next;
	while(abspath->next)
	{
		if(verbose)
		{
			std::cout << "refining segment ("<<
				abspath->n->getLabelL(kFirstData)<<", "<<
				abspath->n->getLabelL(kFirstData+1)<<") (";
			std::cout << abspath->next->n->getLabelL(kFirstData)<<", "<<
				abspath->next->n->getLabelL(kFirstData+1)<<")" <<std::endl;
		}

		path *segment = recurse(&which, abspath->n, abspath->next->n, 0);
		if(segment == 0)
		{
			std::cerr << "couldn't refine segment. cannot continue.\n";
			exit(1);
		}

		// append the new segment to the refined path and delete the overlapping
		// node that appears at the end of one and beginning of the other.
		tail->next = segment->next;
		segment->next = 0;
		delete segment;

		tail = tail->tail();
		abspath = abspath->next;
	}

	// Now that we have a path of jump points that are reachanble by travelling
	// straight from one to the next, refine that path
	path* tmp = retVal;
	OctileDistanceRefinementPolicy rpol(map);
	retVal = rpol.refine(retVal);
	delete tmp;
	return retVal;
}


path*
JumpPointRefinementPolicy::recurse(Jump::Direction* d, node* current, 
		node* target, int depth)
{
	if(verbose)
	{
		std::cout << "current: ("<<current->getLabelL(kFirstData)<<", "<<
			current->getLabelL(kFirstData+1)<<")"<<std::endl;
	}

	depth++;
	if(depth == maxdepth)
	{
		if(verbose)
			std::cout << "not expanding this node; max depth reached.\n";
		return 0;
	}

	int x = current->getLabelL(kFirstData);
	int y = current->getLabelL(kFirstData+1);
	int nx = target->getLabelL(kFirstData);
	int ny = target->getLabelL(kFirstData+1);

	int successors = computeSuccessors(*d, x, y);
	for(int i=1; i <= 128; i*=2)
	{
		if(i & successors)
		{
			*d = (Jump::Direction) i;
			node* jp_next = jpl->findJumpNode(*d, x, y, nx, ny);

			if(jp_next == 0)
				continue;

			if(jp_next->getNum() == target->getNum())
			{
				if(verbose)
					std::cout << "target node found!\n"<<std::endl;
				return new path(current, new path(jp_next, 0));
			}

			path* p = recurse(d, jp_next, target, depth);
			if(p)
			{
				p = new path(current, p);
				return p;
			}
		}
	}

	if(verbose)
	{
		std::cout << "node ("<<current->getLabelL(kFirstData)<<", "<<
			current->getLabelL(kFirstData+1)<<")"<<std::endl;
		std::cout << "is a dead end."<<std::endl;
	}
	return 0;
}

int
JumpPointRefinementPolicy::computeSuccessors(Jump::Direction d, int x, int y)
{
	int retVal = 0;
	switch(d)
	{
		case Jump::S:
		{
			retVal |= Jump::S;
			if(!map->getNodeFromMap(x+1, y))
				retVal |= Jump::SE;
			if(!map->getNodeFromMap(x-1, y))
				retVal |= Jump::SW;
			break;
		}

		case Jump::SW:
		{
			retVal |= Jump::SW;
			retVal |= Jump::S;
			retVal |= Jump::W;

			if(!map->getNodeFromMap(x, y-1))
				retVal |= Jump::NW;
			if(!map->getNodeFromMap(x+1, y))
				retVal |= Jump::SE;
			break;
		}

		case Jump::W:
		{
			retVal |= Jump::W;
			if(!map->getNodeFromMap(x, y-1))
				retVal |= Jump::NW;
			if(!map->getNodeFromMap(x, y+1))
				retVal |= Jump::SW;
			break;
		}

		case Jump::NW:
		{
			retVal |= Jump::NW;
			retVal |= Jump::N;
			retVal |= Jump::W;

			if(!map->getNodeFromMap(x, y+1))
				retVal |= Jump::SW;
			if(!map->getNodeFromMap(x+1, y))
				retVal |= Jump::NE;
			break;
		}

		case Jump::N:
		{
			retVal |= Jump::N;

			if(!map->getNodeFromMap(x+1, y))
				retVal |= Jump::NE;
			if(!map->getNodeFromMap(x-1, y))
				retVal |= Jump::NW;
			break;
		}

		case Jump::NE:
		{
			retVal |= Jump::NE;
			retVal |= Jump::E;
			retVal |= Jump::N;

			if(!map->getNodeFromMap(x, y+1))
				retVal |= Jump::SE;
			if(!map->getNodeFromMap(x-1, y))
				retVal |= Jump::NW;
			break;
		}

		case Jump::E:
		{
			retVal |= Jump::E;
			retVal |= Jump::SE;

			if(!map->getNodeFromMap(x, y-1))
				retVal |= Jump::NE;
			if(!map->getNodeFromMap(x, y+1))
				retVal |= Jump::SE;
			break;
		}

		case Jump::SE:
		{
			retVal |= Jump::SE;
			retVal |= Jump::S;
			retVal |= Jump::E;

			if(!map->getNodeFromMap(x, y-1))
				retVal |= Jump::NE;
			if(!map->getNodeFromMap(x-1, y))
				retVal |= Jump::SW;
			break;
		}
		case Jump::NONE:
		{
			// when a node has no parent (usually only the start node)
			// we generate jump point successors in every traversable direction
			retVal = 0xFF;
		}
	}
	return retVal;
}

