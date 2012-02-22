#include "RecursiveJumpPointRefinementPolicy.h"

#include "RecursiveJumpPointExpansionPolicy.h"

#include "path.h"

#include <assert.h>

RecursiveJumpPointRefinementPolicy::RecursiveJumpPointRefinementPolicy(
		JumpPointLocator* jpl) 
	: RefinementPolicy()
{
	expander = new RecursiveJumpPointExpansionPolicy(jpl);
}

RecursiveJumpPointRefinementPolicy::~RecursiveJumpPointRefinementPolicy()
{
	delete expander;
}

// Refine a path computed by the recursive multi-step jumping procedure.
// This operation involves re-expanding every node along the path and
// inserting every intermediate jump point which was jumped over 
// (i.e. those jump points with a branching factor of 1) 
path* 
RecursiveJumpPointRefinementPolicy::refine(path* abspath)
{
	path* thepath=abspath;
	int maxdepth = expander->getMaxDepth();
	node** intermediates = new node*[maxdepth];
	while(thepath && thepath->next)
	{
		// expand thepath->n 
		expander->expand(thepath->n);
		node* succ = 0;
		for(succ = expander->first(); expander->hasNext(); expander->next())
		{
			if(succ->getUniqueID() == thepath->next->n->getUniqueID())
				break;	
		}
		assert(succ);

		// get the intermediate nodes that were skipped on the way to
		// thepath->next->n because their branching factor equaled 1.
		for(int i=0; i < maxdepth; i++)
			intermediates[i] = 0;
		expander->getIntermediateNodes(intermediates, maxdepth);

		// insert each intermediate jump point into the path
		int cindex = 0;
		while(intermediates[cindex])
		{
			path* newnext = new path(intermediates[cindex], thepath->next);
			thepath->next = newnext;
			thepath = newnext;
			cindex++;
		}
	}
	delete intermediates;
	return abspath;
}
