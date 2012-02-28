#include "RecursiveJumpPointRefinementPolicy.h"

#include "OctileDistanceRefinementPolicy.h"
#include "ProblemInstance.h"
#include "RecursiveJumpPointExpansionPolicy.h"

#include "mapAbstraction.h"
#include "path.h"

#include <assert.h>

RecursiveJumpPointRefinementPolicy::RecursiveJumpPointRefinementPolicy(
		const RecursiveJumpPointExpansionPolicy* _expander) : expander(_expander)

	: RefinementPolicy()
{
}

RecursiveJumpPointRefinementPolicy::~RecursiveJumpPointRefinementPolicy()
{
}

// Refine a path computed by the recursive multi-step jumping procedure.
// This operation involves inserting into the final path every intermediate 
// jump point which was skipped because its branching factor was < 2. 
path* 
RecursiveJumpPointRefinementPolicy::refine(path* abspath)
{
	path* thepath=abspath;
	while(thepath && thepath->next)
	{
		// get the intermediate nodes that were skipped on the way to
		// thepath->next->n because their branching factor equaled 1.
		JumpInfo* info = expander->getJumpInfo();
		assert(info);
		//info->print(std::cout);

		// insert each intermediate jump point into the path
		// (the last node == thepath->n, so skip it)
		for(unsigned int i=0; i < info->nodecount() - 1; i++)
		{
			path* newnext = new path(info->getNode(i), thepath->next);
			thepath->next = newnext;
			thepath = newnext;
		}
		thepath = thepath->next;
	}
}
