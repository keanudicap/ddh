#ifndef HIERARCHICALSEARCHRSR_H
#define HIERARCHICALSEARCHRSR_H

// HierarchicalSearchRSR.h
// 
// Almost identical to the standard HierarchicalSearch class,
// this variant is hardcoded to use:
//  - a FlexibleAStar search algorithm
// 	- a EmptyClusterInsertionPolicy
//  - a OctileDistanceRefinementPolicy.
//
// There is also a small optimisation where the optimal path
// is returned without search if the start and goal are
// located in the same cluster.
// 
// @author: dharabor
// @created: 03/06/2011
//

#include "HierarchicalSearch.h"

class EmptyClusterAbstraction;
class FlexibleAStar;
class node;
class path;
class reservationProvider;

class HierarchicalSearchRSR : public HierarchicalSearch
{
	public: 
		HierarchicalSearchRSR(EmptyClusterAbstraction* _map, 
				FlexibleAStar* alg);
		virtual ~HierarchicalSearchRSR();

		virtual path *getPath(graphAbstraction *aMap, node *from, node *to, 
				reservationProvider *rp = 0);	

	private:
		path* getClusterPath(node* from, node* to);
};

#endif

