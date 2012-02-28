#ifndef RECURSIVEJUMPPOINTREFINEMENTPOLICY_H
#define RECURSIVEJUMPPOINTREFINEMENTPOLICY_H

#include "RefinementPolicy.h"

class path;
class OctileDistanceRefinementPolicy;
class RecursiveJumpPointExpansionPolicy;
class RecursiveJumpPointRefinementPolicy : public RefinementPolicy
{
	public:
		RecursiveJumpPointRefinementPolicy(
				const RecursiveJumpPointExpansionPolicy*);
		virtual ~RecursiveJumpPointRefinementPolicy();

		virtual path* refine(path* abspath);
	
	private:
		const RecursiveJumpPointExpansionPolicy* expander;
		OctileDistanceRefinementPolicy* again;
};

#endif

