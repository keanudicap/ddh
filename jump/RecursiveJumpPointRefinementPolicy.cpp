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

		// iterater over all neighbours until we find thepath->next->n
		for( 	node* succ = expander->first(); 
				expander->hasNext(); 
				expander->next()	)
		{
			if(&*succ == &*(thepath->next->n))
				break;	
		}
		assert(expander->n());

		// get the intermediate nodes that were skipped on the way to
		// thepath->next->n because their branching factor equaled 1.
		JumpInfo* info = expander->getJumpInfo();

		// insert each intermediate jump point into the path
		// (the last node == thepath->n, so skip it)
		for(unsigned int i=0; i < info->nodecount() - 1; i++)
		{
			path* newnext = new path(info->getNode(i), thepath->next);
			thepath->next = newnext;
			thepath = newnext;
		}
	}
	delete intermediates;
	return abspath;
}
