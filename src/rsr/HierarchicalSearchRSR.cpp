#include "HierarchicalSearchRSR.h"

#include "EmptyClusterAbstraction.h"
#include "EmptyClusterInsertionPolicy.h"
#include "FlexibleAStar.h"
#include "IncidentEdgesExpansionPolicy.h"
#include "ManhattanHeuristic.h"
#include "OctileDistanceRefinementPolicy.h"
#include "OctileHeuristic.h"


HierarchicalSearchRSR::HierarchicalSearchRSR(EmptyClusterAbstraction* _map, 
		FlexibleAStar* alg)	
	: HierarchicalSearch(
			new EmptyClusterInsertionPolicy(_map), 
			alg,
			new OctileDistanceRefinementPolicy(_map))
{

}

HierarchicalSearchRSR::~HierarchicalSearchRSR()
{
}

path*
HierarchicalSearchRSR::getClusterPath(node* _from, node* _to)
{
	ClusterNode* from = dynamic_cast<ClusterNode*>(_from);
	ClusterNode* to = dynamic_cast<ClusterNode*>(_to);

	assert(from);
	assert(to);

	path *p = 0;
	if(from->getParentClusterId() == to->getParentClusterId())
	{
		to->backpointer = from;
		p = new path(from, new path(to, 0));
	}
	return p;
}

path*
HierarchicalSearchRSR::getPath(graphAbstraction* aMap, node* from, node* to, 
		reservationProvider *rp)
{
	path* retVal = getClusterPath(from, to);
	if(retVal == 0)
	{
		retVal = HierarchicalSearch::getPath(aMap, from, to, rp);
	}
	else
	{
		resetMetrics();
		path* tmp = retVal;
		retVal = refinePolicy->refine(retVal);
		delete tmp;
		
		searchTime = refinePolicy->getSearchTime();
		nodesTouched = refinePolicy->getNodesTouched();
		nodesExpanded = refinePolicy->getNodesExpanded();
		nodesGenerated = refinePolicy->getNodesGenerated();
	}
	
	return retVal;
}
