#ifndef OCTILEEXPANSIONPOLICY_H
#define OCTILEEXPANSIONPOLICY_H

#include "GridMapExpansionPolicy.h"

class mapAbstraction;
class OctileHeuristic;
class OctileExpansionPolicy : public GridMapExpansionPolicy
{
	public:
		OctileExpansionPolicy(mapAbstraction*);
		virtual ~OctileExpansionPolicy();

		virtual node* n();
};

#endif
