#include "OctileExpansionPolicy.h"

#include "mapAbstraction.h"
#include "OctileHeuristic.h"

OctileExpansionPolicy::OctileExpansionPolicy(mapAbstraction* map)
	: GridMapExpansionPolicy(map, new OctileHeuristic(), 8)
{
}

OctileExpansionPolicy::~OctileExpansionPolicy()
{

}

node* OctileExpansionPolicy::n()
{
	node* n = 0;
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
