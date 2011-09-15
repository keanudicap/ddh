#include "RefinementPolicy.h"

#include "mapAbstraction.h"

RefinementPolicy::RefinementPolicy(mapAbstraction* _map)
{
	map = _map;
	verbose = false;
}

RefinementPolicy::~RefinementPolicy()
{
}

void
RefinementPolicy::resetMetrics()
{
	searchTime = 0;
	nodesExpanded = nodesGenerated = nodesTouched = 0;
}
