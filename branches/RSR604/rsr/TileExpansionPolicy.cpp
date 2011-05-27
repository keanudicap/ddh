#include "TileExpansionPolicy.h"
#include "ManhattanHeuristic.h"
#include "mapAbstraction.h"

TileExpansionPolicy::TileExpansionPolicy(mapAbstraction* map) 
	: GridMapExpansionPolicy(map, new ManhattanHeuristic(), 4)
{
}

TileExpansionPolicy::~TileExpansionPolicy()
{
}

node* TileExpansionPolicy::n()
{
	node* n = 0;
	switch(which)
	{
		case 0:
			n = map->getNodeFromMap(
					target->getLabelL(kFirstData),
					target->getLabelL(kFirstData+1)-1);
			break;

		case 1:
			n = map->getNodeFromMap(
					target->getLabelL(kFirstData)+1,
					target->getLabelL(kFirstData+1));
			break;
		
		case 2:
			n = map->getNodeFromMap(
					target->getLabelL(kFirstData),
					target->getLabelL(kFirstData+1)+1);
			break;
		case 3:
			n = map->getNodeFromMap(
					target->getLabelL(kFirstData)-1,
					target->getLabelL(kFirstData+1));
			break;
	}
	return n;
}



