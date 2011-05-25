#include "OctileExpansionPolicy.h"

#include "mapAbstraction.h"
#include "OctileHeuristic.h"
#include "ProblemInstance.h"

OctileExpansionPolicy::OctileExpansionPolicy()
	: GridMapExpansionPolicy(8)
{
}

OctileExpansionPolicy::~OctileExpansionPolicy()
{

}

node* OctileExpansionPolicy::n()
{
	node* n = 0;
	mapAbstraction* map = problem->getMap();
	switch(which)
	{
		case 0: // n
			n = map->getNodeFromMap(
					target->getLabelL(kFirstData),
					target->getLabelL(kFirstData+1)-1);
			break;

		case 1: // e
			n = map->getNodeFromMap(
					target->getLabelL(kFirstData)+1,
					target->getLabelL(kFirstData+1));
			break;
		
		case 2: // s
			n = map->getNodeFromMap(
					target->getLabelL(kFirstData),
					target->getLabelL(kFirstData+1)+1);
			break;

		case 3: // w
			n = map->getNodeFromMap(
					target->getLabelL(kFirstData)-1,
					target->getLabelL(kFirstData+1));
			break;

		case 4: // nw
			n = map->getNodeFromMap(
					target->getLabelL(kFirstData)-1,
					target->getLabelL(kFirstData+1)-1);
			break;

		case 5: // ne
			n = map->getNodeFromMap(
					target->getLabelL(kFirstData)+1,
					target->getLabelL(kFirstData+1)-1);
			break;

		case 6: // se
			n = map->getNodeFromMap(
					target->getLabelL(kFirstData)+1,
					target->getLabelL(kFirstData+1)+1);
			break;

		case 7: // sw
			n = map->getNodeFromMap(
					target->getLabelL(kFirstData)-1,
					target->getLabelL(kFirstData+1)+1);
			break;
	}
	return n;
}
