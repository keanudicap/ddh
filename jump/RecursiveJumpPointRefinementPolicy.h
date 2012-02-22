#ifndef RECURSIVEJUMPPOINTREFINEMENTPOLICY_H
#define RECURSIVEJUMPPOINTREFINEMENTPOLICY_H

#include "RefinementPolicy.h"

class path;
class JumpPointLocator;
class RecursiveJumpPointExpansionPolicy;
class RecursiveJumpPointRefinementPolicy : public RefinementPolicy
{
	public:
		RecursiveJumpPointRefinementPolicy(JumpPointLocator*);
		virtual ~RecursiveJumpPointRefinementPolicy();

		virtual path* refine(path* abspath);
	
	private:
		RecursiveJumpPointExpansionPolicy* expander;
};

#endif

