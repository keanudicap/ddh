#include "RRExpansionPolicy.h"

#include "IncidentEdgesExpansionPolicy.h"
#include "EmptyCluster.h"
#include "EmptyClusterAbstraction.h"
#include "MacroNode.h"
#include "graphAbstraction.h"
#include "graph.h"

RRExpansionPolicy::RRExpansionPolicy(graphAbstraction* map) :

	SelectiveExpansionPolicy()
{
	this->map = map;	

	primary = new IncidentEdgesExpansionPolicy(map);
	skipSecondary = false;
	whichSecondary = 0;
	numSecondary = 0;
	cost = 0;
}

RRExpansionPolicy::~RRExpansionPolicy()
{
	delete primary;
}

void RRExpansionPolicy::expand(node* target_)
{
	primary->expand(target_);

	ExpansionPolicy::expand(target_);
	this->g = map->getAbstractGraph(target_->getLabelL(kAbstractionLevel));
	MacroNode* mnTarget = dynamic_cast<MacroNode*>(target_);
	assert(mnTarget);
	MacroNode* mnBackpointer = dynamic_cast<MacroNode*>(target_->backpointer);
	if(mnTarget && mnBackpointer && 
		mnTarget->getParentClusterId() == mnBackpointer->getParentClusterId())
	{
		skipSecondary = true;
	}
	else
	{
		skipSecondary = false;
	}

	whichSecondary = 0;
	numSecondary = mnTarget->numSecondaryEdges();
}

node* RRExpansionPolicy::first_impl()
{
	whichSecondary = 0;
	node* retVal = primary->first();
	cost = primary->cost_to_n();
	return retVal;
}

node* RRExpansionPolicy::n_impl()
{
	node* retVal = primary->n();
	if(retVal)
		cost = primary->cost_to_n();
	else
	{
		assert(primary->hasNext() == false);
		if(skipSecondary == false && whichSecondary < numSecondary)
		{
			MacroNode* mnTarget = dynamic_cast<MacroNode*>(target);
			edge* e = mnTarget->getSecondaryEdge(whichSecondary);
			assert(e);
			int neighbourid = e->getFrom()==mnTarget->getNum()?e->getTo():e->getFrom();
			retVal = g->getNode(neighbourid);
			assert(retVal);
			cost = e->getWeight();
		}
	}
	return retVal;
}

node* RRExpansionPolicy::next_impl()
{
	node* retVal = 0;
	if(primary->hasNext())
	{
		retVal = primary->next();
		cost = primary->cost_to_n();
	}
	else if(skipSecondary == false)
	{
		if(whichSecondary < numSecondary)
		{
			MacroNode* mnTarget = dynamic_cast<MacroNode*>(target);
			edge* e = mnTarget->getSecondaryEdge(whichSecondary);
			assert(e);
			int neighbourid = e->getFrom()==mnTarget->getNum()?e->getTo():e->getFrom();
			retVal = g->getNode(neighbourid);
			assert(retVal);
			cost = e->getWeight();
			whichSecondary++;
		}
	}

	return retVal;
}

bool RRExpansionPolicy::hasNext()
{
	if(primary->hasNext())
		return true;
	if(0 <= whichSecondary < numSecondary)
		return true;
	return false;
}

double RRExpansionPolicy::cost_to_n()
{
	return cost;	
}
